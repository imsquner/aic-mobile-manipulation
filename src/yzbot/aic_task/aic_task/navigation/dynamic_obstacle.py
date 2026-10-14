#!/usr/bin/env python3
"""Bounded dynamic-obstacle controller for path and sine trajectories."""

import math
import time
import json
import threading
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .obstacle_motion import load_obstacle_motions
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates



def select_obstacle_names(selector, available_names):
    """Resolve `enabled_obstacles` to a concrete set of obstacle names.

    Only the explicit literals ``""``/``"all"`` select everything. A blank or
    whitespace-only selector is *not* silently widened to "all": that used to
    turn an upstream wiring mistake (for example a world with no entry in
    DYNAMIC_OBSTACLE_NAMES_BY_WORLD) into a downstream "No dynamic_obstacles
    configured" RuntimeError that pointed at the wrong file.
    """
    available = tuple(str(name) for name in available_names)
    text = str(selector).strip()
    if text.lower() == "all":
        return available
    if not text:
        raise ValueError(
            "enabled_obstacles is empty; pass 'all' explicitly to select every "
            f"obstacle. available={list(available)}"
        )
    requested = tuple(
        dict.fromkeys(name.strip() for name in text.split(",") if name.strip())
    )
    unknown = sorted(set(requested) - set(available))
    if not requested or unknown:
        raise ValueError(
            f"Invalid enabled_obstacles={selector!r}; unknown={unknown}, "
            f"available={list(available)}"
        )
    return requested


class DynamicObstacleController(Node):
    """Advance by ROS simulation time; reset requires fresh physical proof."""
    def __init__(self):
        super().__init__('dynamic_obstacle_controller')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('world2 dynamic obstacles require use_sim_time=true')
        self.declare_parameter('points_file',str(Path(get_package_share_directory('aic_task'))/'config/scenes/world2/obstacles.yaml'))
        self.declare_parameter('enabled_obstacles','all')
        self.declare_parameter('set_entity_service','/set_entity_state')
        self._cbg=ReentrantCallbackGroup();self._lock=threading.RLock()
        self.cli=self.create_client(SetEntityState,str(self.get_parameter('set_entity_service').value),callback_group=self._cbg)
        if not self.cli.wait_for_service(timeout_sec=10.):raise RuntimeError('Gazebo state service unavailable')
        all_motions=load_obstacle_motions(str(self.get_parameter('points_file').value))
        names=select_obstacle_names(self.get_parameter('enabled_obstacles').value,all_motions)
        self.obstacles={n:all_motions[n] for n in names};self._pending={};self._actual={};self._model_sequence=0
        self._robot_xy=None;self._model_receipt=0.;self._last_sim_ns=None;self._last_status=0.;self._resetting=False;self._failure='';self._enabled=False
        self.create_subscription(ModelStates,'/gazebo/model_states',self._on_model_states,
                                 QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=self._cbg)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/competition/current_state',self._on_system_state,retained,callback_group=self._cbg)
        self.pub_status=self.create_publisher(String,'/competition/dynamic_obstacles_status',retained)
        self.create_service(Trigger,'/competition/start_dynamic_obstacles',self._enable,callback_group=self._cbg)
        self.create_service(Trigger,'/competition/reset_dynamic_obstacles',self._reset,callback_group=self._cbg)
        self.create_timer(.1,self._update,callback_group=self._cbg)
        self.get_logger().info('Dynamic obstacle controller configured; awaiting READY: '+str(list(self.obstacles)))

    def _on_system_state(self,message):
        with self._lock:
            for motion in self.obstacles.values():motion.observe_state(message.data)

    def _on_model_states(self,message):
        now=time.monotonic()
        with self._lock:
            self._model_sequence+=1;self._model_receipt=now
            if 'six_arm' in message.name:
                p=message.pose[message.name.index('six_arm')].position;self._robot_xy=(p.x,p.y)
            else:self._robot_xy=None
            self._actual={}
            for name in self.obstacles:
                if name not in message.name:continue
                pose=message.pose[message.name.index(name)];p,q=pose.position,pose.orientation
                self._actual[name]=(p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def _status(self):
        payload={'enabled':self._enabled,'resetting':self._resetting,'failure':self._failure,'sim_ns':self.get_clock().now().nanoseconds,
                 'obstacles':{n:{'target':(*m.position,m.z,m.yaw),'actual':self._actual.get(n),'elapsed_sim_sec':m.elapsed,
                                 'state':m.state,'ready':m.ready,'clock_fault':m.clock_fault} for n,m in self.obstacles.items()}}
        self.pub_status.publish(String(data=json.dumps(payload)))

    def _update(self):
        with self._lock:
            current=self.get_clock().now().nanoseconds
            dt=0. if self._last_sim_ns is None else (current-self._last_sim_ns)/1e9
            self._last_sim_ns=current
            if self._enabled and not self._resetting and not self._failure:
                for name,motion in self.obstacles.items():
                    if name in self._pending:
                        future=self._pending[name]
                        if not future.done():continue
                        if future.exception() is not None or future.result() is None or not future.result().success:
                            self._failure='Gazebo obstacle update failed';break
                    if name not in self._actual:continue
                    before=motion.position
                    target=motion.advance(dt,self._robot_xy,time.monotonic()-self._model_receipt<1.)
                    if target!=before:self._set_position(name,motion)
            if time.monotonic()-self._last_status>=.5:
                self._last_status=time.monotonic();self._status()

    def _set_position(self,name,motion):
        req=SetEntityState.Request();req.state=EntityState();req.state.name=name;req.state.reference_frame='world'
        req.state.pose.position.x,req.state.pose.position.y=motion.position;req.state.pose.position.z=motion.z
        req.state.pose.orientation.z=math.sin(motion.yaw/2);req.state.pose.orientation.w=math.cos(motion.yaw/2)
        future=self.cli.call_async(req);self._pending[name]=future
        return future

    def _enable(self,request,response):
        with self._lock:
            response.success=all(m.ready for m in self.obstacles.values()) and not self._resetting and not self._failure
            if response.success:
                self._enabled=True;self._last_sim_ns=self.get_clock().now().nanoseconds
            response.message='enabled after READY' if response.success else 'system is not ready'
        return response

    def _reset(self,request,response):
        with self._lock:
            if self._resetting:
                response.success=False;response.message='reset already in progress';return response
            self._resetting=True
        deadline=time.monotonic()+3.;evidence={'verified':False,'obstacles':{}};issued=False;reply_baseline=None
        try:
            while time.monotonic()<deadline:
                with self._lock:
                    if (not issued and all(f.done() for f in self._pending.values())
                            and self._robot_xy is not None and time.monotonic()-self._model_receipt<1.):
                        for m in self.obstacles.values():
                            if math.hypot(m.center[0]-self._robot_xy[0],m.center[1]-self._robot_xy[1])<m.radius:
                                raise RuntimeError('reset center is too close to robot')
                        for name,m in self.obstacles.items():m.reset();self._set_position(name,m)
                        baseline=self._model_sequence;issued=True
                    replies_ok=issued and all(f.done() and f.exception() is None and f.result() is not None and f.result().success for f in self._pending.values())
                    if replies_ok and reply_baseline is None:
                        reply_baseline=self._model_sequence
                    if replies_ok and self._model_sequence>reply_baseline:
                        evidence['obstacles']={n:self._actual.get(n) for n in self.obstacles}
                        ok=all(n in self._actual and math.dist(self._actual[n][:3],(*m.position,m.z))<.03 and
                               abs(math.atan2(math.sin(self._actual[n][3]-m.yaw),math.cos(self._actual[n][3]-m.yaw)))<.03 for n,m in self.obstacles.items())
                        if ok:
                            evidence.update(verified=True,model_sequence=self._model_sequence,sim_ns=self.get_clock().now().nanoseconds)
                            self._failure='';self._last_sim_ns=self.get_clock().now().nanoseconds;break
                time.sleep(.02)
            response.success=evidence['verified']
            if not response.success:evidence['error']='dynamic reset physical verification timeout'
        except Exception as exc:
            response.success=False;evidence['error']=str(exc)
        finally:
            with self._lock:
                self._resetting=False
                if not response.success:self._failure=evidence.get('error','reset failed')
                self._status()
        response.message=json.dumps(evidence);return response


def main():
    rclpy.init();node=None;executor=MultiThreadedExecutor(num_threads=3)
    try:
        node=DynamicObstacleController();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown()
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':main()
