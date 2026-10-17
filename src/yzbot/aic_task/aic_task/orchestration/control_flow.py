"""One owner for operator cleanup; motion requests remain tracked until terminal."""
import json
from datetime import datetime
from zoneinfo import ZoneInfo
import math
import threading
import time
from concurrent.futures import Future

from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from .constants import POSE_PRE_GRASP

ARM_JOINTS = [f'joint{i}' for i in range(1, 7)]
from .states import TaskState


class ControlInterrupted(RuntimeError):
    pass


class ControlFlowMixin:
    def _init_control_flow(self, config):
        self._control_lock = threading.RLock()
        self._motion_command_lock = threading.RLock()
        self._control_mode = None
        self._control_generation = 0
        self._control_thread = None
        self._control_local = threading.local()
        self._pending_action_sends = []
        self._tracked_goals = []
        self._pending_waits = []
        self._abandoned_sends = set()
        self._motion_tracking_failed = False
        self._expected_round_id = None
        self._plan_revision = -1
        self._attachment_state = 'UNKNOWN'
        self._plan_ready = False
        self._latest_joint_positions = {}
        self._latest_joint_time = 0.0
        self._latest_odom_time = 0.0
        self._latest_base_speed = None
        self._reset_scene_verified = False
        self._step_text = ''
        self._parser_state = 'EMPTY'
        self._control_timeouts = {
            'stop': max(1.0, float(config.get('stop_timeout_sec', 15.0))),
            'release': max(1.0, float(config.get('release_timeout_sec', 10.0))),
            'pose': max(1.0, float(config.get('pose_timeout_sec', 15.0))),
        }

    def _is_control_worker(self):
        return threading.current_thread() is getattr(self, '_control_thread', None)

    def _control_checkpoint(self):
        if self._is_control_worker():
            generation = getattr(self._control_local, 'generation', None)
            if generation != self._control_generation:
                raise ControlInterrupted('operator changed cleanup request')

    def _set_step(self, text):
        if getattr(self, '_control_mode', None) and not self._is_control_worker():
            return
        self._step_text = str(text)
        if self._control_mode == 'reset' and self.completed_count:
            self._step_text += ' | 计数待清'
        self._publish_operator_view()

    def _publish_operator_view(self):
        if not hasattr(self, 'pub_step'):
            return
        self.pub_step.publish(String(data=self._step_text))
        self.pub_attachment.publish(String(data=(
            self._attachment_state
        )))
        self.pub_updated.publish(String(data=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S')))

    def _on_control_joint_state(self, message):
        self._latest_joint_positions = dict(zip(message.name, message.position))
        self._latest_joint_time = time.monotonic()

    def _on_control_odom(self, message):
        velocity = message.twist.twist
        self._latest_base_speed = (math.hypot(velocity.linear.x, velocity.linear.y), abs(velocity.angular.z))
        self._latest_odom_time = time.monotonic()

    def _confirm_base_stopped(self):
        deadline = time.monotonic() + self._control_timeouts['stop']
        stable_since = None
        sample_time = None
        while time.monotonic() < deadline:
            self._control_checkpoint()
            now = time.monotonic()
            if self._latest_odom_time != sample_time and now - self._latest_odom_time < 1.0:
                sample_time = self._latest_odom_time
                speed = self._latest_base_speed
                if speed is not None and speed[0] <= .01 and speed[1] <= .02:
                    stable_since = stable_since or now
                    if now - stable_since >= .3:
                        return self._wait_for_base_still(frame_id='odom')
                else:
                    stable_since = None
            if now - self._latest_odom_time >= 1.0:
                stable_since = None
            self._publish_precision_stop()
            time.sleep(.05)
        return False

    def _on_parser_state(self, message):
        with self._control_lock:
            self._parser_state = message.data
            self._plan_ready = (message.data == 'PLAN_READY' and self.state == TaskState.READY
                                and self.current_task is not None and not self._task_plan_frozen
                                and not self._control_mode)

    def _publish_motion_command(self, command):
        with self._motion_command_lock:
            if (command.linear.x or command.angular.z) and self._stop_exec:
                return
            self.pub_precision_cmd.publish(command)

    def _send_motion_goal(self, client, goal):
        stopped = self._should_stop()
        with self._motion_command_lock:
            recovery = self._is_control_worker()
            stale = recovery and getattr(self._control_local, 'generation', None) != self._control_generation
            if stopped or stale or (self._stop_exec and not recovery):
                future = Future()
                future.set_result(None)
                return future
            return self._track_action_send(client.send_goal_async(goal))

    def _on_grasp_execution(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        self._attachment_state = ('UNKNOWN' if payload.get('attachment_unknown', True)
                                  else 'ATTACHED' if payload.get('attached') else 'DETACHED')
        if payload.get('attached'):
            self._attached_cube = str(payload.get('cube_model', '')) or self._attached_cube
        elif self._attachment_state == 'DETACHED':
            self._attached_cube = None
        self._publish_operator_view()

    def _on_pick_status(self, message):
        if self._control_mode or self.state != TaskState.GRASPING:
            return
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if payload.get('state') == 'stage_started':
            labels = {'latch_pregrasp': 'IK校验', 'plan_pick': '抓取规划',
                      'execute_pregrasp': '预抓取', 'execute_observation': '观察位',
                      'execute_descend': '下降抓取', 'attach_target': '吸附',
                      'execute_lift': '抬升', 'return_arm': '回识别位'}
            stage = payload.get('stage', '')
            self.pub_manipulation.publish(String(data=stage.upper()))
            self._set_step(labels.get(stage, stage))

    def _track_action_send(self, future):
        """Track late handles, including when the caller has already stopped waiting."""
        recovery = self._is_control_worker()
        generation = self._control_generation
        with self._motion_command_lock:
            self._pending_action_sends.append(future)
        def accepted(done):
            try:
                handle = done.result()
                if handle is None or not handle.accepted:
                    return
                result = handle.get_result_async()
                with self._motion_command_lock:
                    self._tracked_goals.append((handle, result))
                    cancel = ((self._stop_exec and not recovery) or id(done) in self._abandoned_sends
                              or (recovery and generation != self._control_generation))
                if cancel:
                    handle.cancel_goal_async()
            except Exception as exc:
                self._motion_tracking_failed = True
                self._event('motion_tracking_error', error=str(exc))
            finally:
                with self._motion_command_lock:
                    if done in self._pending_action_sends:
                        self._pending_action_sends.remove(done)
        future.add_done_callback(accepted)
        return future

    def _cancel_tracked_goals(self):
        with self._motion_command_lock:
            for handle, result in self._tracked_goals:
                if not result.done():
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        self._motion_tracking_failed = True

    def _motion_settled(self):
        with self._motion_command_lock:
            for _handle, result in self._tracked_goals:
                if result.done() and result.exception() is not None:
                    self._motion_tracking_failed = True
            self._tracked_goals[:] = [(h, f) for h, f in self._tracked_goals if not f.done()]
            self._pending_waits[:] = [f for f in self._pending_waits if not f.done()]
            return not (self._motion_tracking_failed or self._pending_action_sends or self._tracked_goals or self._pending_waits)

    def _control_service(self, client, timeout):
        self._control_checkpoint()
        if not client.wait_for_service(timeout_sec=min(.3, timeout)):
            return None
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            return None
        return future.result()

    def _freeze_parser_task(self):
        reply = self._control_service(self._parser_freeze_client, 3.0)
        if reply is None or not reply.success:
            self.pub_last_error.publish(String(data='题目已变更或冻结未确认，请复位后重试'))
            return False
        try:
            payload = json.loads(reply.message)
        except (ValueError, TypeError):
            return False
        return (payload.get('round_id') == self._execution_round_id
                and payload.get('generation') == self._execution_revision)

    def _request_control(self, mode):
        with self._control_lock:
            if self._control_mode == mode:
                self._event('control_merged', operation=mode)
                return
            if mode == 'cancel' and self.state == TaskState.CANCELLED:
                return
            self._control_generation += 1
            self._reset_scene_verified = False
            self._control_mode = mode
            self._stop_exec = True
            self._deadline_grace_active = False
            self._plan_ready = False
            if self.start_time is not None:
                self._frozen_elapsed = max(0.0, time.monotonic() - self.start_time)
                self.start_time = None
            self._transition(TaskState.RESETTING if mode == 'reset' else TaskState.CANCELLING,
                             control=True)
            self._step_text = '停止全部动作'
            self.pub_navigation.publish(String(data='STOPPING'))
            self.pub_manipulation.publish(String(data='STOPPING'))
            self._cancel_navigation()
            self._cancel_trajectories()
            self._publish_precision_stop()
            # Remote stops are nonblocking and never wait behind the operation lock.
            for client in (self._pick_stop_client, self._grasp_stop_client):
                if client.service_is_ready():
                    self._pending_waits.append(client.call_async(Trigger.Request()))
            if self._control_thread is None or not self._control_thread.is_alive():
                self._control_thread = threading.Thread(target=self._control_worker,
                                                        name='operator-control', daemon=True)
                self._control_thread.start()
            self._publish_operator_view()

    def _wait_control_stopped(self):
        deadline = time.monotonic() + self._control_timeouts['stop']
        while time.monotonic() < deadline:
            self._control_checkpoint()
            self._publish_precision_stop()
            self._cancel_tracked_goals()
            sequence = self._control_service(self._pick_stop_client, .5)
            grasp = self._control_service(self._grasp_stop_client, .5)
            workers = [self._exec_thread, getattr(self, '_startup_pose_thread', None)]
            local_done = all(t is None or not t.is_alive() for t in workers)
            if sequence and sequence.success and grasp and grasp.success and local_done and self._motion_settled():
                return self._confirm_base_stopped() and self._wait_arm_still()
            time.sleep(.05)
        return False

    def _wait_arm_still(self):
        deadline = time.monotonic() + self._control_timeouts['pose']
        previous = None
        sample_time = None
        stable_since = None
        while time.monotonic() < deadline:
            self._control_checkpoint()
            now = time.monotonic()
            current = self._latest_joint_positions
            if (self._latest_joint_time != sample_time and now - self._latest_joint_time < 1.0) and all(j in current for j in ARM_JOINTS):
                sample_time = self._latest_joint_time
                positions = tuple(float(current[j]) for j in ARM_JOINTS)
                if previous is not None and max(abs(a-b) for a,b in zip(previous, positions)) < .003:
                    stable_since = stable_since or now
                    if now - stable_since >= .3:
                        return True
                else:
                    stable_since = None
                previous = positions
            elif now - self._latest_joint_time >= 1.0:
                previous = stable_since = None
            time.sleep(.05)
        return False

    def _recover_recognition_pose(self):
        self._set_step('机械臂回识别位')
        self.pub_manipulation.publish(String(data='RETURNING_RECOGNITION_POSE'))
        if not self._arm(POSE_PRE_GRASP, 1.2):
            return False
        if not self._wait_arm_still():
            return False
        if max(abs(float(self._latest_joint_positions[j])-target)
               for j,target in zip(ARM_JOINTS, POSE_PRE_GRASP)) > .08:
            return False
        self.pub_manipulation.publish(String(data='RECOGNITION_POSE'))
        return True

    def _clear_current_target(self):
        for publisher in (self.pub_current_color, self.pub_current_zone):
            publisher.publish(String(data=''))
        self.pub_current_item.publish(Int32(data=0))
        self.pub_current_group_count.publish(Int32(data=0))

    def _control_worker(self):
        while True:
            with self._control_lock:
                mode = self._control_mode
                generation = self._control_generation
                self._control_local.generation = generation
            try:
                clear_client = self._parser_clear_client if mode == 'reset' else self._parser_cancel_client
                reply = self._control_service(clear_client, 3.0) if self._parser_enabled else None
                if self._parser_enabled and (reply is None or not reply.success):
                    raise RuntimeError('题目清理未确认')
                if reply:
                    self._expected_round_id = reply.message
                    self._plan_revision = -1
                if mode == 'reset':
                    with self._data_lock:
                        self.current_task = None
                        self.expanded_items = []
                    self._clear_current_target()
                    self.pub_total.publish(Int32(data=0))
                self._set_step('等待全部动作停止')
                if not self._wait_control_stopped():
                    raise RuntimeError('动作停止未确认')
                self.pub_navigation.publish(String(data='STOPPED'))
                self.pub_manipulation.publish(String(data='STOPPED'))
                self._set_step('释放当前物块')
                self.pub_attachment.publish(String(data='DETACHING'))
                reply = self._control_service(self._grasp_abort_client, self._control_timeouts['release'])
                if reply is None or not reply.success:
                    raise RuntimeError('释放结果未确认')
                self._attached_cube = None
                self._attachment_state = 'DETACHED'
                if mode == 'reset':
                    if not self.reset_scene_on_reset:
                        raise RuntimeError('完整复位要求 reset_scene_on_reset=True')
                    scene_ok, detail = self._reset_scene()
                    if not scene_ok:
                        raise RuntimeError('归位未完成: ' + str(detail))
                    self._reset_scene_verified = True
                if not self._recover_recognition_pose():
                    raise RuntimeError('识别姿态未确认')
                self._set_step('恢复导航环境')
                if not self._set_navigation_profile('transit'):
                    raise RuntimeError('导航参数恢复失败')
                if mode == 'reset':
                    self._publish_placed_cargo_obstacles()
                if mode == 'reset' and not self._clear_navigation_costmaps():
                    raise RuntimeError('导航地图清理失败')
                with self._control_lock:
                    self._control_checkpoint()
                    if mode == 'reset':
                        self._reset_run_counters()
                        self._publish_placed_cargo_obstacles()
                        self._task_plan_frozen = False
                        self._active_cube_id = None
                        self._frozen_elapsed = None
                        self._stop_exec = False
                        self._last_reset_finished_at = time.monotonic()
                        self.pub_last_error.publish(String(data=''))
                    self._set_step('复位完成，请出题' if mode == 'reset' else '取消完成，请复位')
                    self._transition(TaskState.READY if mode == 'reset' else TaskState.CANCELLED,
                                     control=True)
                    self._control_mode = None
                    self._control_thread = None
                    self._publish_operator_view()
                    return
            except ControlInterrupted:
                continue
            except Exception as exc:
                with self._control_lock:
                    if generation != self._control_generation:
                        continue
                    self._stop_exec = True
                    self._publish_precision_stop()
                    self.pub_last_error.publish(String(data=str(exc)))
                    self._set_step(str(exc))
                    self._transition(TaskState.RESET_FAILED if mode == 'reset' else TaskState.CANCEL_FAILED,
                                     control=True)
                    self._control_mode = None
                    self._control_thread = None
                    self._event('control_failed', operation=mode, error=str(exc))
                    return
