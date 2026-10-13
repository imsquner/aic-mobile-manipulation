#!/usr/bin/env python3
"""Stage-gated MoveIt Cartesian pick executor."""

from copy import deepcopy
import json
import math
import re
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from linkattacher_msgs.srv import AttachLink, DetachLink
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MOVEIT_SUCCESS = MoveItErrorCodes.SUCCESS


class GraspExecutor(Node):
    """Plan once, then expose each potentially dangerous stage separately."""

    def __init__(self):
        super().__init__("grasp_executor")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("allow_attach", False)
        self.declare_parameter("allow_detach", False)
        self.declare_parameter("cube_model", "")
        self.declare_parameter("detach_timeout_sec", 5.0)
        self.declare_parameter("release_settle_sec", 0.25)
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("planning_frame", "arm_base_link")
        self.declare_parameter("ik_link", "grasp_center_frame")
        self.declare_parameter("max_status_age_sec", 20.0)
        self.declare_parameter("max_plan_age_sec", 120.0)
        self.declare_parameter("cartesian_step_m", 0.005)
        self.declare_parameter("cartesian_revolute_jump_rad", 0.15)
        self.declare_parameter("cartesian_fraction_min", 0.98)
        self.declare_parameter("cartesian_speed_mps", 0.03)
        self.declare_parameter("cartesian_min_duration_sec", 1.0)
        self.declare_parameter("pregrasp_velocity_scaling_factor", 0.25)
        self.declare_parameter("pregrasp_acceleration_scaling_factor", 0.25)
        self.declare_parameter("return_velocity_scaling_factor", 0.15)
        self.declare_parameter("return_acceleration_scaling_factor", 0.15)
        self.declare_parameter("observation_clearance_m", 0.02)
        self.declare_parameter("pregrasp_joint_tolerance_rad", 0.08)
        self.declare_parameter(
            "arm_trajectory_action", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("controller_timeout_scale", 2.0)
        self.declare_parameter("controller_timeout_margin_sec", 2.0)
        self.declare_parameter("return_joint_positions", [0.0] * 6)
        self.declare_parameter("return_duration_sec", 3.0)

        self.allow_motion = bool(self.get_parameter("allow_motion").value)
        self.allow_attach = bool(self.get_parameter("allow_attach").value)
        self.allow_detach = bool(self.get_parameter("allow_detach").value)
        self.cube_model = str(self.get_parameter("cube_model").value)
        self.detach_timeout = float(
            self.get_parameter("detach_timeout_sec").value
        )
        self.release_settle = float(
            self.get_parameter("release_settle_sec").value
        )
        self.group = str(self.get_parameter("planning_group").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.max_status_age = float(self.get_parameter("max_status_age_sec").value)
        self.max_plan_age = float(self.get_parameter("max_plan_age_sec").value)
        self.cartesian_step = float(self.get_parameter("cartesian_step_m").value)
        self.revolute_jump = float(
            self.get_parameter("cartesian_revolute_jump_rad").value
        )
        self.min_fraction = float(
            self.get_parameter("cartesian_fraction_min").value
        )
        self.cartesian_speed = float(
            self.get_parameter("cartesian_speed_mps").value
        )
        self.cartesian_min_duration = float(
            self.get_parameter("cartesian_min_duration_sec").value
        )
        self.pregrasp_velocity_scaling = float(
            self.get_parameter("pregrasp_velocity_scaling_factor").value
        )
        self.pregrasp_acceleration_scaling = float(
            self.get_parameter("pregrasp_acceleration_scaling_factor").value
        )
        self.return_velocity_scaling = float(
            self.get_parameter("return_velocity_scaling_factor").value
        )
        self.return_acceleration_scaling = float(
            self.get_parameter("return_acceleration_scaling_factor").value
        )
        self.observation_clearance = float(
            self.get_parameter("observation_clearance_m").value
        )
        self.joint_tolerance = float(
            self.get_parameter("pregrasp_joint_tolerance_rad").value
        )
        self.arm_trajectory_action = str(
            self.get_parameter("arm_trajectory_action").value
        )
        self.controller_timeout_scale = float(
            self.get_parameter("controller_timeout_scale").value
        )
        self.controller_timeout_margin = float(
            self.get_parameter("controller_timeout_margin_sec").value
        )
        self.return_positions = [
            float(value)
            for value in self.get_parameter("return_joint_positions").value
        ]
        self.return_duration = float(
            self.get_parameter("return_duration_sec").value
        )

        self._cbg = ReentrantCallbackGroup()
        self._move_group = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self._cbg
        )
        self._arm_trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            self.arm_trajectory_action,
            callback_group=self._cbg,
        )
        self._cartesian = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self._cbg
        )
        self._attach = self.create_client(
            AttachLink, "/ATTACHLINK", callback_group=self._cbg
        )
        self._detach = self.create_client(
            DetachLink, "/DETACHLINK", callback_group=self._cbg
        )

        self.create_subscription(
            String,
            "/competition/pregrasp_status",
            self._on_pregrasp_status,
            10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            PoseStamped,
            "/competition/pregrasp_pose",
            self._on_pregrasp_pose,
            10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            PoseStamped,
            "/competition/contact_pose_preview",
            self._on_contact_pose,
            10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
            callback_group=self._cbg,
        )
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(
            String, "/competition/grasp_execution_status", status_qos
        )

        self.create_service(
            Trigger, "/competition/plan_pick", self._on_plan, callback_group=self._cbg
        )
        self.create_service(
            Trigger,
            "/competition/execute_pregrasp",
            self._on_execute_pregrasp,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/execute_observation",
            self._on_execute_observation,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/execute_descend",
            self._on_execute_descend,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/attach_target",
            self._on_attach,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/execute_lift",
            self._on_execute_lift,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/return_arm",
            self._on_return,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/release_target",
            self._on_release,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/reset_pick",
            self._on_reset,
            callback_group=self._cbg,
        )
        self.create_service(
            Trigger,
            "/competition/abort_pick",
            self._on_abort,
            callback_group=self._cbg,
        )
        self.create_service(
            SetParameters,
            "/competition/set_target_model",
            self._on_set_target_model,
            callback_group=self._cbg,
        )

        self._ik_status = None
        self._ik_received = 0.0
        self._pregrasp_pose = None
        self._contact_pose = None
        self._joint_positions = {}
        self._pregrasp_trajectory = None
        self._observation_trajectory = None
        self._observation_joint_positions = None
        self._descend_trajectory = None
        self._lift_trajectory = None
        self._plan_time = 0.0
        self._at_pregrasp = False
        self._at_observation = False
        self._at_contact = False
        self._attached = False
        self._lifted = False
        self._returned = False
        self._released = False
        self._operation_lock = threading.Lock()
        self._motion_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._active_goals = []
        self._pending_sends = []
        self._pending_calls = []
        self._attachment_unknown = False
        self._tracking_failed = False
        self.create_service(Trigger, "/competition/stop_grasp", self._on_stop,
                            callback_group=self._cbg)
        self.add_on_set_parameters_callback(self._on_runtime_parameters)

        self._publish(
            "ready",
            allow_motion=self.allow_motion,
            allow_attach=self.allow_attach,
            allow_detach=self.allow_detach,
            cube_model=self.cube_model or None,
            execution_backend="follow_joint_trajectory",
        )

    def _on_runtime_parameters(self, parameters):
        updates = {}
        speed_parameters = {
            "cartesian_speed_mps": "cartesian_speed",
            "cartesian_min_duration_sec": "cartesian_min_duration",
            "pregrasp_velocity_scaling_factor": "pregrasp_velocity_scaling",
            "pregrasp_acceleration_scaling_factor": "pregrasp_acceleration_scaling",
            "return_velocity_scaling_factor": "return_velocity_scaling",
            "return_acceleration_scaling_factor": "return_acceleration_scaling",
            "return_duration_sec": "return_duration",
        }
        for parameter in parameters:
            attribute = speed_parameters.get(parameter.name)
            if attribute is None:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be numeric",
                )
            if parameter.name.endswith("scaling_factor"):
                valid = 0.0 < value <= 1.0
            else:
                valid = value > 0.0
            if not valid:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} is outside its valid range",
                )
            updates[attribute] = value
        if updates and self._operation_lock.locked():
            return SetParametersResult(
                successful=False,
                reason="cannot change motion parameters during a grasp operation",
            )
        for attribute, value in updates.items():
            setattr(self, attribute, value)
        if updates:
            self._publish("motion_parameters_updated", **updates)
        return SetParametersResult(successful=True)

    @staticmethod
    def _valid_cube_model(value):
        return re.fullmatch(r"(?:red|blue)_cube_[1-5]", str(value)) is not None

    def _on_set_target_model(self, request, response):
        result = SetParametersResult(successful=False, reason="invalid request")
        response.results = [result]
        if len(request.parameters) != 1:
            result.reason = "exactly one cube_model parameter is required"
            return response
        parameter = request.parameters[0]
        if parameter.name != "cube_model":
            result.reason = "parameter name must be cube_model"
            return response
        if parameter.value.type != ParameterType.PARAMETER_STRING:
            result.reason = "cube_model must be a string"
            return response
        cube_model = parameter.value.string_value.strip()
        if not self._valid_cube_model(cube_model):
            result.reason = "cube_model must match red_cube_1..5 or blue_cube_1..5"
            return response
        if not self._operation_lock.acquire(blocking=False):
            result.reason = "another grasp operation is active"
            return response
        try:
            if self._attached:
                result.reason = "cannot switch target while a cube is attached"
                return response
            self.cube_model = cube_model
            self._clear_plan()
            self._released = False
            result.successful = True
            result.reason = f"target model set to {cube_model}"
            self._publish("target_model_set", cube_model=cube_model)
            return response
        finally:
            self._operation_lock.release()

    def _on_pregrasp_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if status.get("state") == "ik_valid":
            self._ik_status = status
            self._ik_received = time.monotonic()
            self._clear_plan()

    def _on_pregrasp_pose(self, message):
        self._pregrasp_pose = message

    def _on_contact_pose(self, message):
        self._contact_pose = message

    def _on_joint_state(self, message):
        self._joint_positions = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }

    def _publish(self, state, **details):
        payload = {
            "state": state,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "motion_enabled": self.allow_motion,
            "attach_enabled": self.allow_attach,
            "detach_enabled": self.allow_detach,
            "attached": self._attached,
            "attachment_unknown": self._attachment_unknown,
            "cube_model": self.cube_model,
        }
        payload.update(details)
        encoded = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(String(data=encoded))
        self.get_logger().info(encoded)

    def _clear_plan(self):
        self._pregrasp_trajectory = None
        self._observation_trajectory = None
        self._observation_joint_positions = None
        self._descend_trajectory = None
        self._lift_trajectory = None
        self._plan_time = 0.0
        self._at_pregrasp = False
        self._at_observation = False
        self._at_contact = False

    @staticmethod
    def _finish(response, success, message):
        response.success = bool(success)
        response.message = str(message)
        return response

    def _wait_future(self, future, timeout, allow_stopped=False):
        if future is None:
            return None
        self._pending_calls.append(future)
        deadline = time.monotonic() + timeout
        while not future.done():
            if (self._stop_event.is_set() and not allow_stopped) or time.monotonic() >= deadline:
                return None
            time.sleep(.02)
        try:
            return future.result()
        except Exception:
            return None

    def _send_tracked_goal(self, client, goal):
        with self._motion_lock:
            if self._stop_event.is_set():
                return None
            future = client.send_goal_async(goal)
            self._pending_sends.append(future)
        def accepted(done):
            try:
                handle = done.result()
                if handle is not None and handle.accepted:
                    result = handle.get_result_async()
                    with self._motion_lock:
                        self._active_goals.append((handle, result))
                        if self._stop_event.is_set():
                            handle.cancel_goal_async()
            except Exception:
                self._tracking_failed = True
            finally:
                with self._motion_lock:
                    if done in self._pending_sends:
                        self._pending_sends.remove(done)
        future.add_done_callback(accepted)
        return future

    def _on_stop(self, request, response):
        # Never acquire _operation_lock: its owner may be waiting on a trajectory.
        with self._motion_lock:
            self._stop_event.set()
            for handle, result in self._active_goals:
                if not result.done():
                    handle.cancel_goal_async()
            for _handle, result in self._active_goals:
                if result.done() and result.exception() is not None:
                    self._tracking_failed = True
            self._active_goals[:] = [(h, f) for h, f in self._active_goals if not f.done()]
            self._pending_calls[:] = [f for f in self._pending_calls if not f.done()]
            stopped = not (self._operation_lock.locked() or self._active_goals
                           or self._pending_sends or self._pending_calls or self._attachment_unknown
                           or getattr(self, '_tracking_failed', False))
        self._publish("stopped" if stopped else "stopping")
        return self._finish(response, stopped, "stopped" if stopped else "waiting for execution termination")

    def _acquire_operation(self, response, allow_stopped=False):
        if self._stop_event.is_set() and not allow_stopped:
            return self._finish(response, False, "grasp stopped; reset required")
        if not self._operation_lock.acquire(blocking=False):
            return self._finish(response, False, "another grasp operation is active")
        return None

    def _validated_input(self):
        if self._ik_status is None:
            return None, "no ik_valid pregrasp status received"
        age = time.monotonic() - self._ik_received
        if age > self.max_status_age:
            return None, f"pregrasp status is stale ({age:.2f}s)"
        if self._pregrasp_pose is None or self._contact_pose is None:
            return None, "pregrasp/contact poses have not been received"
        if self._pregrasp_pose.header.frame_id != self.planning_frame:
            return None, "pregrasp pose frame mismatch"
        if self._contact_pose.header.frame_id != self.planning_frame:
            return None, "contact pose frame mismatch"
        solution = self._ik_status.get("joint_solution", {})
        if not all(joint in solution for joint in ARM_JOINTS):
            return None, "IK status has no complete arm joint solution"
        return [float(solution[joint]) for joint in ARM_JOINTS], None

    def _on_plan(self, request, response):
        del request
        if self._attached:
            return self._finish(
                response, False, "cannot plan a new pick while target is attached"
            )
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            positions, error = self._validated_input()
            if error:
                return self._finish(response, False, error)
            self._publish("planning")
            pregrasp, error = self._plan_pregrasp(positions)
            if error:
                self._publish("plan_failed", reason=error)
                return self._finish(response, False, error)
            observation, observation_positions, observation_fraction, error = (
                self._plan_observation(positions)
            )
            if error:
                self._publish(
                    "plan_failed", reason=error, fraction=observation_fraction
                )
                return self._finish(response, False, error)
            descend, descend_fraction, error = self._plan_cartesian(
                observation_positions,
                self._contact_pose,
                self.observation_clearance
                - float(self._ik_status.get("contact_clearance_m", 0.003)),
                "final descend",
            )
            if error:
                self._publish("plan_failed", reason=error, fraction=descend_fraction)
                return self._finish(response, False, error)

            self._pregrasp_trajectory = pregrasp
            self._observation_trajectory = observation
            self._observation_joint_positions = observation_positions
            self._descend_trajectory = descend
            self._lift_trajectory = self._reverse_trajectories(
                [observation, descend],
                self._pose_distance(
                    self._pregrasp_pose.pose, self._contact_pose.pose
                ),
            )
            self._plan_time = time.monotonic()
            details = {
                "pregrasp_points": len(pregrasp.joint_trajectory.points),
                "observation_points": len(observation.joint_trajectory.points),
                "descend_points": len(descend.joint_trajectory.points),
                "observation_clearance_m": self.observation_clearance,
                "observation_fraction": observation_fraction,
                "descend_fraction": descend_fraction,
                "cartesian_fraction": min(
                    observation_fraction, descend_fraction
                ),
            }
            self._publish("plan_ready", **details)
            return self._finish(response, True, json.dumps(details, sort_keys=True))
        finally:
            self._operation_lock.release()

    def _plan_pregrasp(self, positions):
        if not self._move_group.wait_for_server(timeout_sec=3.0):
            return None, "/move_action unavailable"
        constraints = Constraints(name="vision_pregrasp")
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=position,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for joint, position in zip(ARM_JOINTS, positions)
        ]
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.pregrasp_velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.pregrasp_acceleration_scaling
        )
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False

        goal_handle = self._wait_future(
            self._send_tracked_goal(self._move_group, goal), 8.0
        )
        if goal_handle is None or not goal_handle.accepted:
            return None, "pregrasp planning goal rejected"
        wrapped = self._wait_future(goal_handle.get_result_async(), 15.0)
        if wrapped is None:
            return None, "pregrasp planning timed out"
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MOVEIT_SUCCESS
        ):
            return None, (
                "pregrasp planning failed: "
                f"status={wrapped.status}, code={wrapped.result.error_code.val}"
            )
        trajectory = wrapped.result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            return None, "pregrasp planner returned an empty trajectory"
        return trajectory, None

    def _plan_observation(self, positions):
        contact_clearance = float(
            self._ik_status.get("contact_clearance_m", 0.003)
        )
        if self.observation_clearance <= contact_clearance:
            return None, None, 0.0, (
                "observation_clearance_m must exceed contact clearance "
                f"({contact_clearance:.3f}m)"
            )
        observation_pose = deepcopy(self._contact_pose)
        observation_pose.pose.position.z += (
            self.observation_clearance - contact_clearance
        )
        distance = self._pose_distance(
            self._pregrasp_pose.pose, observation_pose.pose
        )
        trajectory, fraction, error = self._plan_cartesian(
            positions, observation_pose, distance, "observation descend"
        )
        if error:
            return None, None, fraction, error
        end_positions = self._trajectory_end_positions(trajectory)
        if end_positions is None:
            return None, None, fraction, (
                "observation trajectory has incomplete joint positions"
            )
        return trajectory, end_positions, fraction, None

    def _plan_cartesian(self, positions, target_pose, distance, label):
        if not self._cartesian.wait_for_service(timeout_sec=3.0):
            return None, 0.0, "/compute_cartesian_path unavailable"
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.group_name = self.group
        request.link_name = self.ik_link
        request.start_state = RobotState()
        request.start_state.joint_state.name = list(ARM_JOINTS)
        request.start_state.joint_state.position = list(positions)
        request.start_state.is_diff = False
        request.waypoints = [deepcopy(target_pose.pose)]
        request.max_step = self.cartesian_step
        request.jump_threshold = 0.0
        request.revolute_jump_threshold = self.revolute_jump
        request.avoid_collisions = True

        result = self._wait_future(self._cartesian.call_async(request), 10.0)
        if result is None:
            return None, 0.0, f"{label} Cartesian planning timed out"
        fraction = float(result.fraction)
        if result.error_code.val != MOVEIT_SUCCESS:
            return None, fraction, (
                f"{label} Cartesian planning failed: code={result.error_code.val}"
            )
        if fraction < self.min_fraction:
            return None, fraction, (
                f"{label} Cartesian path fraction "
                f"{fraction:.3f} < {self.min_fraction:.3f}"
            )
        trajectory = result.solution
        if len(trajectory.joint_trajectory.points) < 2:
            return None, fraction, f"{label} Cartesian planner returned too few points"
        self._retime(
            trajectory,
            max(distance / self.cartesian_speed, self.cartesian_min_duration),
        )
        return trajectory, fraction, None

    @staticmethod
    def _trajectory_end_positions(trajectory):
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if not points or len(names) != len(points[-1].positions):
            return None
        positions = dict(zip(names, points[-1].positions))
        if not all(joint in positions for joint in ARM_JOINTS):
            return None
        return [float(positions[joint]) for joint in ARM_JOINTS]

    @staticmethod
    def _pose_distance(first, second):
        return math.sqrt(
            (first.position.x - second.position.x) ** 2
            + (first.position.y - second.position.y) ** 2
            + (first.position.z - second.position.z) ** 2
        )

    @staticmethod
    def _set_duration(duration, seconds):
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1e9)

    def _retime(self, trajectory, total_seconds):
        points = trajectory.joint_trajectory.points
        count = len(points)
        for index, point in enumerate(points):
            seconds = total_seconds * (index + 1) / count
            point.velocities = []
            point.accelerations = []
            self._set_duration(point.time_from_start, seconds)

    def _reverse_trajectories(self, trajectories, distance):
        reversed_trajectory = deepcopy(trajectories[0])
        reversed_trajectory.joint_trajectory.points = []
        for trajectory in reversed(trajectories):
            reversed_trajectory.joint_trajectory.points.extend(
                deepcopy(list(reversed(trajectory.joint_trajectory.points)))
            )
        total = max(distance / self.cartesian_speed, self.cartesian_min_duration)
        self._retime(reversed_trajectory, total)
        return reversed_trajectory

    def _plan_is_current(self):
        return (
            self._pregrasp_trajectory is not None
            and time.monotonic() - self._plan_time <= self.max_plan_age
        )

    @staticmethod
    def _controller_execution_timeout(
        trajectory, timeout_scale, timeout_margin
    ):
        points = trajectory.joint_trajectory.points
        if not points:
            return max(float(timeout_margin), 1.0)
        duration = points[-1].time_from_start
        planned_seconds = float(duration.sec) + float(duration.nanosec) / 1e9
        return max(
            planned_seconds * float(timeout_scale) + float(timeout_margin),
            1.0,
        )

    def _execute_trajectory(self, trajectory, label):
        if not trajectory.joint_trajectory.points:
            return False, f"{label} trajectory is empty"
        if not self._arm_trajectory.wait_for_server(timeout_sec=3.0):
            return False, f"{self.arm_trajectory_action} unavailable"

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = deepcopy(trajectory.joint_trajectory)
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        goal_handle = self._wait_future(
            self._send_tracked_goal(self._arm_trajectory, goal), 5.0
        )
        if goal_handle is None or not goal_handle.accepted:
            return False, f"{label} execution goal rejected"
        timeout = self._controller_execution_timeout(
            trajectory,
            self.controller_timeout_scale,
            self.controller_timeout_margin,
        )
        wrapped = self._wait_future(goal_handle.get_result_async(), timeout)
        if wrapped is None:
            goal_handle.cancel_goal_async()
            return False, f"{label} controller execution timed out ({timeout:.2f}s)"
        success = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not success:
            error_text = wrapped.result.error_string or "controller rejected trajectory"
            return False, (
                f"{label} execution failed: status={wrapped.status}, "
                f"code={wrapped.result.error_code}, reason={error_text}"
            )
        return True, "ok"

    def _motion_guard(self, response):
        if not self.allow_motion:
            return self._finish(response, False, "motion disabled by parameter")
        if not self._plan_is_current():
            return self._finish(response, False, "no current plan; run plan_pick again")
        return None

    def _on_execute_pregrasp(self, request, response):
        del request
        blocked = self._motion_guard(response)
        if blocked:
            return blocked
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            self._publish("executing_pregrasp")
            ok, message = self._execute_trajectory(
                self._pregrasp_trajectory, "pregrasp"
            )
            self._at_pregrasp = ok
            self._publish("at_pregrasp" if ok else "execution_failed", reason=message)
            return self._finish(response, ok, message)
        finally:
            self._operation_lock.release()

    def _joint_error(self, target_positions):
        if not all(joint in self._joint_positions for joint in ARM_JOINTS):
            return float("inf")
        return max(
            abs(
                math.atan2(
                    math.sin(self._joint_positions[joint] - target),
                    math.cos(self._joint_positions[joint] - target),
                )
            )
            for joint, target in zip(ARM_JOINTS, target_positions)
        )

    def _joint_error_from_pregrasp(self):
        solution = self._ik_status.get("joint_solution", {})
        return self._joint_error(
            [float(solution[joint]) for joint in ARM_JOINTS]
        )

    def _on_execute_observation(self, request, response):
        del request
        blocked = self._motion_guard(response)
        if blocked:
            return blocked
        if not self._at_pregrasp:
            return self._finish(response, False, "pregrasp stage not completed")
        error = self._joint_error_from_pregrasp()
        if error > self.joint_tolerance:
            return self._finish(
                response, False, f"arm is not at pregrasp; joint error={error:.3f}rad"
            )
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            self._publish(
                "executing_observation",
                observation_clearance_m=self.observation_clearance,
            )
            ok, message = self._execute_trajectory(
                self._observation_trajectory, "MoveL observation descend"
            )
            self._at_observation = ok
            self._publish(
                "at_observation" if ok else "execution_failed",
                reason=message,
                observation_clearance_m=self.observation_clearance,
            )
            return self._finish(response, ok, message)
        finally:
            self._operation_lock.release()

    def _on_execute_descend(self, request, response):
        del request
        blocked = self._motion_guard(response)
        if blocked:
            return blocked
        if not self._at_observation:
            return self._finish(response, False, "observation stage not completed")
        error = self._joint_error(self._observation_joint_positions)
        if error > self.joint_tolerance:
            return self._finish(
                response,
                False,
                f"arm is not at observation point; joint error={error:.3f}rad",
            )
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            self._publish("executing_descend")
            ok, message = self._execute_trajectory(
                self._descend_trajectory, "MoveL descend"
            )
            self._at_contact = ok
            self._publish("at_contact" if ok else "execution_failed", reason=message)
            return self._finish(response, ok, message)
        finally:
            self._operation_lock.release()

    def _on_attach(self, request, response):
        del request
        if not self.allow_attach:
            return self._finish(response, False, "attach disabled by parameter")
        if not self._at_contact:
            return self._finish(response, False, "contact stage not completed")
        if not self.cube_model:
            return self._finish(response, False, "cube_model parameter is empty")
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            if not self._attach.wait_for_service(timeout_sec=3.0):
                return self._finish(response, False, "/ATTACHLINK unavailable")
            attach = AttachLink.Request()
            attach.model1_name = "six_arm"
            attach.link1_name = "link6"
            attach.model2_name = self.cube_model
            attach.link2_name = "link"
            with self._motion_lock:
                if self._stop_event.is_set():
                    return self._finish(response, False, "attach cancelled")
                pending = self._attach.call_async(attach)
                self._pending_calls.append(pending)
                self._attachment_unknown = True
            def attached(done):
                try:
                    result = done.result()
                    with self._motion_lock:
                        self._attached = bool(result is not None and result.success)
                        self._attachment_unknown = result is None
                except Exception:
                    self._attachment_unknown = True
            pending.add_done_callback(attached)
            result = self._wait_future(pending, 5.0)
            if result is not None:
                self._attached = bool(result.success)
                self._attachment_unknown = False
            if self._attached:
                self._lifted = False
                self._returned = False
                self._released = False
            message = result.message if result is not None else "attach timed out"
            self._publish(
                "attached" if self._attached else "attach_failed",
                cube_model=self.cube_model,
                reason=message,
            )
            return self._finish(response, self._attached, message)
        finally:
            self._operation_lock.release()

    def _on_execute_lift(self, request, response):
        del request
        if not self.allow_motion:
            return self._finish(response, False, "motion disabled by parameter")
        if self._lift_trajectory is None:
            return self._finish(response, False, "no lift trajectory is available")
        if not self._at_contact:
            return self._finish(response, False, "contact stage not completed")
        if self.allow_attach and not self._attached:
            return self._finish(response, False, "target is not attached")
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            self._publish("executing_lift")
            ok, message = self._execute_trajectory(
                self._lift_trajectory, "MoveL lift"
            )
            self._lifted = ok
            if ok:
                self._returned = False
            self._publish("lifted" if ok else "execution_failed", reason=message)
            return self._finish(response, ok, message)
        finally:
            self._operation_lock.release()

    def _on_return(self, request, response):
        del request
        if not self.allow_motion:
            return self._finish(response, False, "motion disabled by parameter")
        if len(self.return_positions) != len(ARM_JOINTS):
            return self._finish(response, False, "return_joint_positions must have 6 values")
        if not self._lifted:
            return self._finish(response, False, "lift stage not completed")
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            trajectory, error = self._plan_return()
            if error:
                self._publish("return_failed", reason=error)
                return self._finish(response, False, error)
            error = self._joint_path_error(
                trajectory, self.return_positions, max_extra_travel=0.25
            )
            if error:
                self._publish("return_failed", reason=error)
                return self._finish(response, False, error)
            self._publish("executing_return")
            ok, message = self._execute_trajectory(
                trajectory, "recognition-pose return"
            )
            self._returned = ok
            self._publish(
                "returned" if ok else "return_failed", reason=message
            )
            return self._finish(response, ok, message)
        finally:
            self._operation_lock.release()

    def _on_release(self, request, response):
        """Release the carried cube after the task layer confirms region arrival."""
        del request
        blocked = self._release_block_reason(
            self.allow_detach,
            self.cube_model,
            self._attached,
            self._lifted,
            self._returned,
        )
        if blocked:
            return self._finish(response, False, blocked)
        busy = self._acquire_operation(response)
        if busy:
            return busy
        try:
            if not self._detach.wait_for_service(timeout_sec=self.detach_timeout):
                return self._finish(response, False, "/DETACHLINK unavailable")
            detach = self._make_detach_request(self.cube_model)
            result = self._wait_future(
                self._detach.call_async(detach), self.detach_timeout
            )
            success = bool(result is not None and result.success)
            message = result.message if result is not None else "detach timed out"
            if success:
                self._attached = False
                self._released = True
                if self.release_settle > 0.0:
                    time.sleep(self.release_settle)
            self._publish(
                "released" if success else "release_failed",
                cube_model=self.cube_model,
                settle_sec=self.release_settle if success else 0.0,
                reason=message,
            )
            return self._finish(response, success, message)
        finally:
            self._operation_lock.release()

    @staticmethod
    def _release_block_reason(allow_detach, cube_model, attached, lifted, returned):
        if not allow_detach:
            return "detach disabled by parameter"
        if not cube_model:
            return "cube_model parameter is empty"
        if not attached:
            return "no target is attached"
        if not lifted:
            return "lift stage not completed"
        if not returned:
            return "recognition-pose return not completed"
        return None

    @staticmethod
    def _make_detach_request(cube_model):
        detach = DetachLink.Request()
        detach.model1_name = "six_arm"
        detach.link1_name = "link6"
        detach.model2_name = cube_model
        detach.link2_name = "link"
        return detach

    def _plan_return(self):
        if not self._move_group.wait_for_server(timeout_sec=3.0):
            return None, "/move_action unavailable"
        if not all(joint in self._joint_positions for joint in ARM_JOINTS):
            return None, "current joint state unavailable"
        constraints = Constraints(name="recognition_pose")
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=position,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for joint, position in zip(ARM_JOINTS, self.return_positions)
        ]
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group
        goal.request.start_state.joint_state.name = list(ARM_JOINTS)
        goal.request.start_state.joint_state.position = [
            self._joint_positions[joint] for joint in ARM_JOINTS
        ]
        goal.request.start_state.is_diff = False
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.return_velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.return_acceleration_scaling
        )
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal_handle = self._wait_future(
            self._send_tracked_goal(self._move_group, goal), 8.0
        )
        if goal_handle is None or not goal_handle.accepted:
            return None, "recognition-pose planning goal rejected"
        wrapped = self._wait_future(goal_handle.get_result_async(), 15.0)
        if wrapped is None:
            return None, "recognition-pose planning timed out"
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MOVEIT_SUCCESS
        ):
            return None, (
                "recognition-pose planning failed: "
                f"status={wrapped.status}, code={wrapped.result.error_code.val}"
            )
        trajectory = wrapped.result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            return None, "recognition-pose planner returned an empty trajectory"
        self._retime(trajectory, self.return_duration)
        return trajectory, None

    def _joint_path_error(self, trajectory, targets, max_extra_travel):
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if not points or not all(joint in names for joint in ARM_JOINTS):
            return "return trajectory has incomplete joints"
        for joint, target in zip(ARM_JOINTS, targets):
            index = names.index(joint)
            values = [self._joint_positions[joint]] + [
                float(point.positions[index]) for point in points
            ]
            steps = [
                abs(math.atan2(math.sin(b - a), math.cos(b - a)))
                for a, b in zip(values, values[1:])
            ]
            direct = abs(
                math.atan2(
                    math.sin(float(target) - values[0]),
                    math.cos(float(target) - values[0]),
                )
            )
            endpoint_error = abs(
                math.atan2(
                    math.sin(values[-1] - float(target)),
                    math.cos(values[-1] - float(target)),
                )
            )
            if endpoint_error > 0.03:
                return f"return endpoint error for {joint} is {endpoint_error:.3f}rad"
            if steps and max(steps) > 0.50:
                return f"return path has a {max(steps):.3f}rad jump on {joint}"
            total = sum(steps)
            if total > direct + max_extra_travel:
                return (
                    f"return path detours on {joint}: "
                    f"travel={total:.3f}rad direct={direct:.3f}rad"
                )
        return None

    def _on_reset(self, request, response):
        del request
        if not self._operation_lock.acquire(blocking=False):
            return self._finish(response, False, "grasp operation is busy")
        try:
            if (self._attached or self._attachment_unknown or getattr(self, '_tracking_failed', False)
                    or self._pending_sends or any(not f.done() for f in self._pending_calls)):
                return self._finish(response, False, "previous grasp is not settled or cargo remains attached")
            self._ik_status = None
            self._pregrasp_pose = None
            self._contact_pose = None
            self._clear_plan()
            self._lifted = False
            self._returned = False
            self._released = False
            self._stop_event.clear()
            self._publish("reset")
            return self._finish(response, True, "pick state reset")
        finally:
            self._operation_lock.release()

    def _on_abort(self, request, response):
        """Best-effort state cleanup after a failed multi-stage sequence."""
        del request
        busy = self._acquire_operation(response, allow_stopped=True)
        if busy:
            return busy
        try:
            if self._attachment_unknown or self._pending_sends or any(not f.done() for f in self._pending_calls):
                return self._finish(response, False, "attachment or previous requests remain unknown")
            if self._attached:
                if not self.allow_detach:
                    return self._finish(response, False, "detach disabled by parameter")
                if not self._detach.wait_for_service(timeout_sec=self.detach_timeout):
                    return self._finish(response, False, "/DETACHLINK unavailable")
                result = self._wait_future(
                    self._detach.call_async(self._make_detach_request(self.cube_model)),
                    self.detach_timeout, allow_stopped=True,
                )
                if result is None or not result.success:
                    message = result.message if result is not None else "detach timed out"
                    self._publish("abort_failed", cube_model=self.cube_model, reason=message)
                    return self._finish(response, False, message)
            self._attached = False
            self._ik_status = None
            self._pregrasp_pose = None
            self._contact_pose = None
            self._clear_plan()
            self._lifted = False
            self._returned = False
            self._released = False
            self._publish("aborted", cube_model=self.cube_model)
            return self._finish(response, True, "pick state aborted and cleared")
        finally:
            self._operation_lock.release()


def main():
    rclpy.init()
    node = GraspExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
