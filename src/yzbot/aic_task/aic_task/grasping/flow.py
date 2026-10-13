"""Grasp execution, arm control, and visual recovery flows."""

import json
import math
import statistics
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from aic_task.motion_geometry import (
    bearing_to_point,
    gazebo_entity_pose_to_planar,
    grasp_alignment_errors,
    signed_object_yaw_error,
    visual_reposition_face_target,
    visual_reposition_goal,
)
from aic_task.navigation.operations import RELATIVE_BASE_MOTION_TOLERANCE_M
from aic_task.orchestration.constants import POSE_PRE_GRASP
from aic_task.orchestration.states import TaskState
from aic_task.visual_targeting import select_map_detection


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

GRIPPER_JOINTS = ["finger_joint1"]

VISUAL_HANDOFF_YAW_RAD = math.radians(25.0)

GRASP_ORIENTATIONS_ARM_BASE = (
    ("vertical", (0.0, 1.0, 0.0, 0.0)),
    ("tilted_165deg", (0.0, 0.9914448614, 0.0, 0.1305261922)),
    ("tilted_150deg", (0.0, 0.9659258263, 0.0, 0.2588190451)),
    ("tilted_135deg", (0.0, 0.9238795325, 0.0, 0.3826834324)),
    ("tilted_120deg", (0.0, 0.8660254038, 0.0, 0.5)),
)


def ordered_grasp_orientations(target_arm_x):
    """Try the orientation most likely to solve at the measured reach first."""
    if float(target_arm_x) > 0.40:
        return (
            GRASP_ORIENTATIONS_ARM_BASE[2],
            GRASP_ORIENTATIONS_ARM_BASE[1],
            GRASP_ORIENTATIONS_ARM_BASE[3],
            GRASP_ORIENTATIONS_ARM_BASE[4],
            GRASP_ORIENTATIONS_ARM_BASE[0],
        )
    return GRASP_ORIENTATIONS_ARM_BASE


def should_attempt_visual_ik_reposition(failure_reason, target_arm_x, target_arm_y):
    """Allow camera-guided base correction whenever a valid target has no IK."""
    return (
        failure_reason == "no_solution"
        and math.isfinite(float(target_arm_x))
        and math.isfinite(float(target_arm_y))
        and float(target_arm_x) > 0.0
    )


def grasp_alignment_rotation_required(yaw_error, yaw_tolerance):
    """Rotate only when the visual target is outside the final grasp gate."""
    return abs(float(yaw_error)) > abs(float(yaw_tolerance))


class GraspingFlowMixin:
    def _cancel_trajectories(self):
        self._cancel_tracked_goals()
        if self._active_move_group_goal is not None:
            try:
                self._active_move_group_goal.cancel_goal_async()
            except Exception:
                pass
        for goal_handle in list(self._active_trajectory_goals):
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

    def _send_trajectory(self, client, joint_names, positions, duration, timeout):
        trajectory = JointTrajectory(joint_names=list(joint_names))
        point = JointTrajectoryPoint(positions=[float(value) for value in positions])
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)

        send_future = self._send_motion_goal(client, goal)
        if not self._wait_future(send_future, 5.0):
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        self._active_trajectory_goals.append(goal_handle)
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout):
            goal_handle.cancel_goal_async()
            if goal_handle in self._active_trajectory_goals:
                self._active_trajectory_goals.remove(goal_handle)
            return False
        wrapped_result = result_future.result()
        if goal_handle in self._active_trajectory_goals:
            self._active_trajectory_goals.remove(goal_handle)
        return (
            wrapped_result is not None
            and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped_result.result.error_code == 0
        )

    def _arm(self, pose, duration=1.2):
        return self._send_trajectory(
            self._arm_client, ARM_JOINTS, pose, duration, duration + 5.0
        )

    def _move_group_to_joints(self, positions, timeout=30.0):
        constraint = Constraints(name="opencv_grasp")
        constraint.joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=float(position),
                tolerance_above=0.02,
                tolerance_below=0.02,
                weight=1.0,
            )
            for joint, position in zip(ARM_JOINTS, positions)
        ]
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.move_group_velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.move_group_acceleration_scaling
        )
        goal.request.goal_constraints = [constraint]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        send_future = self._send_motion_goal(self._move_group_client, goal)
        if not self._wait_future(send_future, 10.0):
            self._event("moveit_grasp_result", success=False, reason="send timeout")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._event("moveit_grasp_result", success=False, reason="goal rejected")
            return False
        self._active_move_group_goal = goal_handle
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout):
            goal_handle.cancel_goal_async()
            self._active_move_group_goal = None
            self._event("moveit_grasp_result", success=False, reason="result timeout")
            return False
        wrapped = result_future.result()
        self._active_move_group_goal = None
        result = wrapped.result if wrapped is not None else None
        error_code = int(result.error_code.val) if result is not None else None
        planned_points = (
            len(result.planned_trajectory.joint_trajectory.points)
            if result is not None
            else 0
        )
        success = (
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and error_code == MoveItErrorCodes.SUCCESS
            and planned_points > 0
        )
        self._event(
            "moveit_grasp_result",
            success=success,
            status=wrapped.status if wrapped is not None else None,
            error_code=error_code,
            planned_points=planned_points,
        )
        return success

    def _moveit_to_visual_target(self, detection, cube_id, expected_position):
        self._last_ik_target_arm = None
        self._last_ik_failure_reason = None
        target_map = PointStamped()
        target_map.header.frame_id = "map"
        target_map.point.x = float(detection["world_x"])
        target_map.point.y = float(detection["world_y"])
        expected_surface_z = float(expected_position["z"]) + float(
            self.points["approach"].get("cube_half_height_m", 0.015)
        )
        detected_surface_z = float(detection["world_z"])
        surface_z = max(detected_surface_z, expected_surface_z)
        target_map.point.z = surface_z + self.grasp_tool_offset
        try:
            transform = self._tf_buffer.lookup_transform(
                "arm_base_link",
                "map",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            target_arm = do_transform_point(target_map, transform)
        except Exception as exc:
            self._last_ik_failure_reason = "transform"
            self._event("opencv_target_transform", success=False, error=str(exc))
            return False

        self._last_ik_target_arm = {
            "x": float(target_arm.point.x),
            "y": float(target_arm.point.y),
            "z": float(target_arm.point.z),
            "map_x": float(target_map.point.x),
            "map_y": float(target_map.point.y),
        }

        self._event(
            "opencv_ik_target",
            cube=cube_id,
            map_x=target_map.point.x,
            map_y=target_map.point.y,
            map_z=target_map.point.z,
            arm_x=target_arm.point.x,
            arm_y=target_arm.point.y,
            arm_z=target_arm.point.z,
            detected_surface_z=detected_surface_z,
            clamped_surface_z=surface_z,
            tool_offset_m=self.grasp_tool_offset,
        )

        response = None
        selected_orientation = None
        error_code = None
        for orientation_name, orientation_values in ordered_grasp_orientations(
            target_arm.point.x
        ):
            request = GetPositionIK.Request()
            request.ik_request.group_name = "arm"
            request.ik_request.robot_state.is_diff = True
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = "grasping_frame"
            request.ik_request.pose_stamped.header.frame_id = "arm_base_link"
            request.ik_request.pose_stamped.pose.position = target_arm.point
            orientation = request.ik_request.pose_stamped.pose.orientation
            orientation.x, orientation.y, orientation.z, orientation.w = orientation_values
            request.ik_request.timeout = Duration(seconds=0.6).to_msg()

            future = self._ik_client.call_async(request)
            if not self._wait_future(future, 1.5):
                self._event(
                    "opencv_ik_attempt",
                    cube=cube_id,
                    orientation=orientation_name,
                    success=False,
                    reason="timeout",
                )
                continue
            candidate = future.result()
            error_code = (
                int(candidate.error_code.val) if candidate is not None else None
            )
            success = (
                candidate is not None and error_code == MoveItErrorCodes.SUCCESS
            )
            self._event(
                "opencv_ik_attempt",
                cube=cube_id,
                orientation=orientation_name,
                success=success,
                error_code=error_code,
            )
            if success:
                response = candidate
                selected_orientation = orientation_name
                break
        if response is None:
            self._last_ik_failure_reason = "no_solution"
            self._event(
                "opencv_ik_solution",
                cube=cube_id,
                success=False,
                error_code=error_code,
            )
            return False
        solution = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(joint in solution for joint in ARM_JOINTS):
            self._last_ik_failure_reason = "invalid_solution"
            self._event(
                "opencv_ik_solution",
                cube=cube_id,
                success=False,
                reason="arm joints missing from IK response",
            )
            return False
        joint_positions = [solution[joint] for joint in ARM_JOINTS]
        self._event(
            "opencv_ik_solution",
            cube=cube_id,
            success=True,
            orientation=selected_orientation,
            joint_positions=joint_positions,
        )
        if not self._move_group_to_joints(joint_positions):
            self._last_ik_failure_reason = "move_group"
            return False

        reached = None
        error = math.inf
        for attempt in range(1, 3):
            try:
                reached = self._tf_buffer.lookup_transform(
                    "map",
                    "grasping_frame",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                ).transform.translation
                error = math.sqrt(
                    (reached.x - target_map.point.x) ** 2
                    + (reached.y - target_map.point.y) ** 2
                    + (reached.z - target_map.point.z) ** 2
                )
            except Exception as exc:
                self._last_ik_failure_reason = "reached_transform"
                self._event(
                    "opencv_grasp_reached",
                    cube=cube_id,
                    success=False,
                    error=str(exc),
                )
                return False
            if error <= self.grasp_pose_tolerance or attempt == 2:
                break
            self._event(
                "moveit_grasp_reach_retry",
                cube=cube_id,
                attempt=attempt + 1,
                error_m=error,
                tolerance_m=self.grasp_pose_tolerance,
            )
            if not self._move_group_to_joints(joint_positions):
                self._last_ik_failure_reason = "move_group"
                return False
        success = error <= self.grasp_pose_tolerance
        if not success:
            self._last_ik_failure_reason = "reach_error"
        self._event(
            "opencv_grasp_reached",
            cube=cube_id,
            success=success,
            error_m=error,
            tolerance_m=self.grasp_pose_tolerance,
            reached_x=reached.x,
            reached_y=reached.y,
            reached_z=reached.z,
        )
        if success:
            self._active_grasp_joint_positions = list(joint_positions)
            self._active_grasp_orientation = selected_orientation
        return success

    def _visual_target_in_arm_frame(self, detection, cube_id):
        target_frame = str(detection.get("target_frame") or "")
        if target_frame and bool(detection.get("target_transform_ok")):
            direct_values = (
                detection.get("target_x"),
                detection.get("target_y"),
                detection.get("target_z"),
            )
            if all(
                value is not None and math.isfinite(float(value))
                for value in direct_values
            ):
                target_local = PointStamped()
                target_local.header.frame_id = target_frame
                target_local.point.x = float(direct_values[0])
                target_local.point.y = float(direct_values[1])
                target_local.point.z = float(direct_values[2])
                try:
                    if target_frame == "arm_base_link":
                        target_arm = target_local
                    else:
                        local_transform = self._tf_buffer.lookup_transform(
                            "arm_base_link",
                            target_frame,
                            rclpy.time.Time(),
                            timeout=Duration(seconds=1.0),
                        )
                        target_arm = do_transform_point(target_local, local_transform)
                except Exception as exc:
                    self._event(
                        "teammate_grasp_alignment_transform",
                        cube=cube_id,
                        success=False,
                        source=f"{target_frame}_to_arm_base_link",
                        error=str(exc),
                    )
                else:
                    arm_values = (
                        target_arm.point.x,
                        target_arm.point.y,
                        target_arm.point.z,
                    )
                    if all(math.isfinite(float(value)) for value in arm_values):
                        self._event(
                            "teammate_grasp_alignment_transform",
                            cube=cube_id,
                            success=True,
                            source=(
                                "camera_to_arm_base_link"
                                if target_frame == "arm_base_link"
                                else f"camera_to_{target_frame}_to_arm_base_link"
                            ),
                        )
                        return {
                            "x": float(arm_values[0]),
                            "y": float(arm_values[1]),
                            "z": float(arm_values[2]),
                            "map_x": float(detection["world_x"]),
                            "map_y": float(detection["world_y"]),
                        }
        target_map = PointStamped()
        target_map.header.frame_id = "map"
        target_map.point.x = float(detection["world_x"])
        target_map.point.y = float(detection["world_y"])
        target_map.point.z = float(detection["world_z"])
        try:
            transform = self._tf_buffer.lookup_transform(
                "arm_base_link",
                "map",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            target_arm = do_transform_point(target_map, transform)
        except Exception as exc:
            self._event(
                "teammate_grasp_alignment_transform",
                cube=cube_id,
                success=False,
                error=str(exc),
            )
            return None
        result = {
            "x": float(target_arm.point.x),
            "y": float(target_arm.point.y),
            "z": float(target_arm.point.z),
            "map_x": float(target_map.point.x),
            "map_y": float(target_map.point.y),
        }
        if not all(math.isfinite(value) for value in result.values()):
            self._event(
                "teammate_grasp_alignment_transform",
                cube=cube_id,
                success=False,
                error="non-finite transformed target",
            )
            return None
        return result

    def _align_for_teammate_grasp(
        self,
        detection,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
        force_correction=False,
    ):
        desired_x = float(getattr(self, "grasp_desired_arm_x", 0.395))
        forward_tolerance = float(
            getattr(self, "grasp_forward_tolerance", 0.03)
        )
        yaw_tolerance = float(
            getattr(self, "grasp_yaw_tolerance", math.radians(3.0))
        )
        correction_count = int(getattr(self, "_grasp_base_correction_count", 0))
        correction_max_attempts = int(
            getattr(self, "grasp_base_correction_max_attempts", 1)
        )
        alignment_match_tolerance = max(
            float(vision_match_tolerance),
            float(detection.get("recovery_match_tolerance_m", 0.0)),
        )
        target = self._visual_target_in_arm_frame(detection, cube_id)
        if target is None:
            return None
        try:
            forward_error, yaw_error = grasp_alignment_errors(
                (target["x"], target["y"]), desired_x
            )
        except ValueError as exc:
            self._event(
                "teammate_grasp_alignment",
                cube=cube_id,
                success=False,
                error=str(exc),
            )
            return None
        correction_required = (
            abs(forward_error) > forward_tolerance
            or grasp_alignment_rotation_required(yaw_error, yaw_tolerance)
        )
        self._event(
            "teammate_grasp_alignment",
            cube=cube_id,
            success=True,
            authority=(
                "ik_decides_graspability"
                if not force_correction
                else "single_post_ik_base_correction"
            ),
            arm_x=target["x"],
            arm_y=target["y"],
            forward_error_m=forward_error,
            yaw_error_deg=math.degrees(yaw_error),
            visual_handoff_yaw_deg=math.degrees(VISUAL_HANDOFF_YAW_RAD),
            correction_required=correction_required,
            correction_count=correction_count,
        )
        if not correction_required:
            if force_correction:
                self._event(
                    "teammate_grasp_base_correction_skipped",
                    cube=cube_id,
                    reason="IK failed but visual geometry is already inside correction tolerances",
                )
                return None
            return detection
        if correction_count >= correction_max_attempts:
            self._event(
                "teammate_grasp_base_correction_skipped",
                cube=cube_id,
                reason="single correction budget exhausted",
                correction_count=correction_count,
            )
            return None

        measured_range = math.hypot(target["x"], target["y"])
        range_correction = measured_range - desired_x
        self._grasp_base_correction_count = correction_count + 1
        correction_reason = (
            "teammate IK validation failed"
            if force_correction
            else "visual geometry outside grasp comfort window"
        )
        self._event(
            "teammate_grasp_base_correction",
            cube=cube_id,
            attempt=self._grasp_base_correction_count,
            success=None,
            reason=correction_reason,
            yaw_correction_deg=math.degrees(yaw_error),
            range_correction_m=range_correction,
        )
        rotated = True
        rotation_required = grasp_alignment_rotation_required(
            yaw_error, yaw_tolerance
        )
        if rotation_required:
            rotated = self._direct_rotate_relative(
                yaw_error,
                timeout=10.0,
                yaw_tolerance=yaw_tolerance,
            )
        translation_required = abs(range_correction) > forward_tolerance
        moved = rotated
        if moved and translation_required:
            moved = self._direct_drive_relative(
                range_correction,
                timeout=8.0,
                distance_tolerance=RELATIVE_BASE_MOTION_TOLERANCE_M,
                max_speed=float(getattr(self, "grasp_base_correction_speed", 0.15)),
            )
        self._event(
            "teammate_grasp_base_correction_result",
            cube=cube_id,
            attempt=self._grasp_base_correction_count,
            success=moved,
            rotated=rotated,
            rotation_required=rotation_required,
            translation_required=translation_required,
            max_linear_speed_mps=float(
                getattr(self, "grasp_base_correction_speed", 0.15)
            ),
        )
        if not moved or not self._wait_for_base_still():
            return None
        detection, _ = self._reacquire_after_alignment_correction(
            cube_id,
            expected_color,
            expected_position,
            object_parking,
            alignment_match_tolerance,
        )
        if detection is not None:
            self._invalidate_localization_reference(
                "post-IK visual base correction completed"
            )
        return detection

    def _reacquire_after_alignment_correction(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        match_tolerance,
    ):
        effective_tolerance = float(match_tolerance)
        detection = self._confirm_visual_target(
            expected_color,
            expected_position,
            timeout=float(getattr(self, "vision_attempt_timeout", 2.5)),
            match_tolerance=effective_tolerance,
        )
        scan_used = False
        if detection is None:
            scan_used = True
            detection = self._scan_camera_for_expected_target(
                cube_id,
                expected_color,
                expected_position,
                object_parking,
                effective_tolerance,
            )
        self._event(
            "teammate_grasp_visual_reacquire",
            cube=cube_id,
            recovery_used=False,
            scan_used=scan_used,
            match_tolerance_m=effective_tolerance,
            success=detection is not None,
        )
        return detection, effective_tolerance

    def _reposition_for_visual_ik(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
    ):
        target = self._last_ik_target_arm or {}
        target_x = float(target.get("x", 0.0))
        target_y = float(target.get("y", 0.0))
        if not should_attempt_visual_ik_reposition(
            self._last_ik_failure_reason, target_x, target_y
        ):
            return None
        pose = self._base_pose()
        if pose is None:
            return None

        # Use both visual axes. A forward-only correction can move a cube with
        # a large arm-frame Y offset out of the camera before the IK retry.
        try:
            (
                corrected_x,
                corrected_y,
                corrected_yaw,
                forward_m,
                lateral_m,
                correction_m,
            ) = visual_reposition_goal(
                pose,
                (target_x, target_y),
                face_target_position=visual_reposition_face_target(
                    target, expected_position
                ),
            )
        except ValueError:
            return None
        corrected = {
            "x": corrected_x,
            "y": corrected_y,
            "yaw": corrected_yaw,
            "xy_tolerance_m": 0.10,
            "yaw_tolerance_rad": float(object_parking["yaw_tolerance_rad"]),
        }
        self._event(
            "visual_ik_reposition",
            cube=cube_id,
            initial_arm_x=target_x,
            initial_arm_y=target_y,
            forward_m=forward_m,
            lateral_m=lateral_m,
            correction_m=correction_m,
            target_x=corrected["x"],
            target_y=corrected["y"],
            target_yaw=corrected["yaw"],
        )
        if not self._arm(POSE_PRE_GRASP, 0.6):
            return None
        repositioned = self._direct_precision_to_parking(
            corrected, timeout=16.0, position_tolerance=0.10
        )
        if not repositioned:
            self._event(
                "visual_ik_pose_gate_bypassed",
                cube=cube_id,
                reason="vision will validate target after approximate base correction",
            )
        camera_reoriented = repositioned
        if not repositioned:
            camera_reoriented = self._reorient_camera_to_expected_position(
                cube_id, expected_position, object_parking
            )
        self._event(
            "visual_ik_camera_reorientation",
            cube=cube_id,
            success=camera_reoriented,
            combined_with_reposition=repositioned,
            fallback_used=not repositioned,
        )
        if not self._arm(POSE_PRE_GRASP, 0.6) or not self._wait_for_base_still():
            return None
        return self._confirm_visual_target(
            expected_color,
            expected_position,
            match_tolerance=vision_match_tolerance,
        )

    def _gripper(self, pose, state):
        self.pub_gripper.publish(String(data=state))
        return self._send_trajectory(
            self._gripper_client, GRIPPER_JOINTS, pose, 0.6, 5.0
        )

    def _reorient_camera_to_expected_position(
        self, cube_id, expected_position, object_parking
    ):
        robot_pose = self._entity_pose("six_arm")
        cube_pose = self._entity_pose(cube_id)
        if robot_pose is not None and cube_pose is not None:
            robot_planar_pose = gazebo_entity_pose_to_planar(robot_pose)
            yaw_delta = signed_object_yaw_error(
                robot_planar_pose,
                (cube_pose[0], cube_pose[1]),
            )
            self._event(
                "visual_camera_reorientation",
                cube=cube_id,
                authority="gazebo recovery yaw only; RGB-D and IK remain mandatory",
                start_yaw=robot_planar_pose[2],
                target_yaw=self._normalize_angle(
                    robot_planar_pose[2] + yaw_delta
                ),
                yaw_error_deg=math.degrees(abs(yaw_delta)),
            )
            return self._direct_rotate_relative(
                yaw_delta,
                timeout=6.0,
                yaw_tolerance=self.direct_precision_yaw_tolerance,
            )
        pose = self._base_pose("map")
        if pose is None:
            return False
        target_yaw = bearing_to_point(
            pose,
            (expected_position["x"], expected_position["y"]),
        )
        reorientation = {
            "x": pose[0],
            "y": pose[1],
            "yaw": target_yaw,
            "xy_tolerance_m": 0.05,
            "yaw_tolerance_rad": float(object_parking["yaw_tolerance_rad"]),
        }
        self._event(
            "visual_camera_reorientation",
            cube=cube_id,
            base_x=pose[0],
            base_y=pose[1],
            start_yaw=pose[2],
            target_yaw=target_yaw,
            yaw_error_deg=math.degrees(
                abs(self._normalize_angle(target_yaw - pose[2]))
            ),
        )
        return self._direct_precision_to_parking(
            reorientation,
            timeout=6.0,
            position_tolerance=0.05,
        )

    def _scan_camera_for_expected_target(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
    ):
        scan_span = math.radians(25.0)
        for scan_index, yaw_offset in enumerate((-scan_span, scan_span), start=1):
            robot_pose = self._entity_pose("six_arm")
            cube_pose = self._entity_pose(cube_id)
            physical_scan = robot_pose is not None and cube_pose is not None
            if physical_scan:
                robot_planar_pose = gazebo_entity_pose_to_planar(robot_pose)
                yaw_delta = self._normalize_angle(
                    signed_object_yaw_error(
                        robot_planar_pose,
                        (cube_pose[0], cube_pose[1]),
                    )
                    + yaw_offset
                )
                target_yaw = self._normalize_angle(
                    robot_planar_pose[2] + yaw_delta
                )
            else:
                pose = self._base_pose("map")
                if pose is None:
                    return None
                center_yaw = bearing_to_point(
                    pose,
                    (expected_position["x"], expected_position["y"]),
                )
                target_yaw = self._normalize_angle(center_yaw + yaw_offset)
                scan_pose = {
                    "x": pose[0],
                    "y": pose[1],
                    "yaw": target_yaw,
                    "xy_tolerance_m": 0.05,
                    "yaw_tolerance_rad": float(object_parking["yaw_tolerance_rad"]),
                }
            self._event(
                "visual_camera_scan",
                cube=cube_id,
                scan_index=scan_index,
                yaw_offset_deg=math.degrees(yaw_offset),
                target_yaw=target_yaw,
                authority=(
                    "gazebo recovery yaw only; RGB-D remains mandatory"
                    if physical_scan
                    else "map fallback"
                ),
            )
            if physical_scan:
                scan_motion_succeeded = self._direct_rotate_relative(
                    yaw_delta,
                    timeout=6.0,
                    yaw_tolerance=self.direct_precision_yaw_tolerance,
                    verification_phase="gazebo_relative_visual_scan",
                )
            else:
                scan_motion_succeeded = self._direct_precision_to_parking(
                    scan_pose,
                    timeout=6.0,
                    position_tolerance=0.05,
                )
            if not scan_motion_succeeded:
                self._event(
                    "visual_camera_scan_motion_failed",
                    cube=cube_id,
                    scan_index=scan_index,
                )
                continue
            if not self._wait_for_base_still():
                return None
            detection = self._confirm_visual_target(
                expected_color,
                expected_position,
                timeout=min(2.5, self.vision_attempt_timeout),
                match_tolerance=vision_match_tolerance,
            )
            if detection is not None:
                self._event(
                    "visual_camera_scan_result",
                    cube=cube_id,
                    scan_index=scan_index,
                    success=True,
                )
                return detection
        self._event(
            "visual_camera_scan_result",
            cube=cube_id,
            scan_index=2,
            success=False,
        )
        return None

    def _call_grasp_trigger(self, client, service_name, timeout):
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
            self._last_grasp_service_reason = "unavailable"
            self._event("grasp_service_result", service=service_name, success=False, reason="unavailable")
            return False
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            self._last_grasp_service_reason = "timeout"
            self._event("grasp_service_result", service=service_name, success=False, reason="timeout")
            return False
        response = future.result()
        success = response is not None and bool(response.success)
        self._last_grasp_service_reason = str(
            getattr(response, "message", "no response")
        )
        self._event(
            "grasp_service_result",
            service=service_name,
            success=success,
            reason=self._last_grasp_service_reason,
        )
        return success

    def _quiesce_failed_pick(self):
        deadline = time.monotonic() + self._control_timeouts['stop']
        stopped = False
        while time.monotonic() < deadline and not self._should_stop():
            sequence = self._call_grasp_trigger(self._pick_stop_client,
                                                "/competition/stop_pick_sequence", .5)
            grasp = self._call_grasp_trigger(self._grasp_stop_client,
                                             "/competition/stop_grasp", .5)
            if sequence and grasp:
                stopped = True
                break
        if stopped and self._call_grasp_trigger(self._grasp_abort_client,
                                                "/competition/abort_pick", 10.0):
            self._attached_cube = None
            return True
        if not self._control_mode:
            self._request_control('cancel')
        return False

    def _set_grasp_target_model(self, cube_id):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="cube_model",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=str(cube_id),
                ),
            )
        ]
        future = self._grasp_set_target_client.call_async(request)
        if not self._wait_future(future, 5.0):
            self._event("grasp_target_model", cube=cube_id, success=False, reason="timeout")
            return False
        response = future.result()
        result = response.results[0] if response is not None and response.results else None
        success = result is not None and bool(result.successful)
        self._event(
            "grasp_target_model",
            cube=cube_id,
            success=success,
            reason=getattr(result, "reason", "no response"),
        )
        return success

    def _visual_recovery_pose(self, expected_position, object_parking):
        pose = self._base_pose("map")
        cube_x = float(expected_position["x"])
        cube_y = float(expected_position["y"])
        if pose is None:
            pose = (
                float(object_parking["x"]),
                float(object_parking["y"]),
                float(object_parking["yaw"]),
            )
        dx = float(pose[0]) - cube_x
        dy = float(pose[1]) - cube_y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            dx = float(object_parking["x"]) - cube_x
            dy = float(object_parking["y"]) - cube_y
            distance = max(math.hypot(dx, dy), 1e-6)
        approach = self.points.get("approach", {})
        radius = float(
            approach.get(
                "object_approach_target_distance_m",
                approach.get("object_approach_distance_m", 0.45),
            )
        )
        recovery_x = cube_x + radius * dx / distance
        recovery_y = cube_y + radius * dy / distance
        return {
            "x": recovery_x,
            "y": recovery_y,
            "yaw": math.atan2(cube_y - recovery_y, cube_x - recovery_x),
            "xy_tolerance_m": 0.10,
            "yaw_tolerance_rad": float(object_parking["yaw_tolerance_rad"]),
        }

    def _recover_visual_replan_approach(
        self, cube_id, expected_position, recovery_pose, timeout
    ):
        """Use the one unused ranked candidate without starting another retry chain."""
        navigation_arrived = self._navigate_to(
            recovery_pose["x"],
            recovery_pose["y"],
            recovery_pose["yaw"],
            timeout=timeout,
        )
        stationary = self._wait_for_base_still()
        safe_neighborhood = self._object_approach_within_gate(
            expected_position,
            self.object_approach_safety_min_distance,
            self.object_approach_safety_max_distance,
            math.pi,
        )
        self._event(
            "visual_replan_approach_result",
            cube=cube_id,
            navigation_arrived=navigation_arrived,
            base_stationary=stationary,
            safe_visual_neighborhood_reached=safe_neighborhood,
            success=stationary and safe_neighborhood,
        )
        return stationary and safe_neighborhood

    def _confirm_replanned_visual_target(
        self,
        cube_id,
        expected_color,
        expected_position,
        recovery_pose,
        vision_match_tolerance,
    ):
        """Confirm the centered target before any clearance-changing yaw scan."""
        # Never widen same-colour association beyond the configured physical
        # match gate; a fresh camera-to-arm transform and IK are still required.
        recovery_match_tolerance = float(vision_match_tolerance)
        detection = self._confirm_visual_target(
            expected_color,
            expected_position,
            timeout=self.vision_attempt_timeout,
            match_tolerance=recovery_match_tolerance,
        )
        centered_success = detection is not None
        scan_used = False
        if detection is None:
            scan_used = True
            detection = self._scan_camera_for_expected_target(
                cube_id,
                expected_color,
                expected_position,
                recovery_pose,
                recovery_match_tolerance,
            )
        self._event(
            "visual_replan_confirmation",
            cube=cube_id,
            centered_success=centered_success,
            scan_used=scan_used,
            success=detection is not None,
        )
        if detection is not None:
            detection["recovery_match_tolerance_m"] = recovery_match_tolerance
        return detection

    def _grasp(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
        visual_recovery_pose=None,
        object_navigation_deadline=None,
        direct_reorientation_on_failure=False,
        visual_scan_first=False,
    ):
        del direct_reorientation_on_failure
        self._active_grasp_joint_positions = None
        self._active_grasp_orientation = None
        self._grasp_base_correction_count = 0
        self._transition(TaskState.GRASPING)
        self.pub_manipulation.publish(String(data="PRE_GRASP"))
        if not self._arm(POSE_PRE_GRASP, 0.8):
            return False
        if not self._wait_for_base_still():
            return False
        scan_attempted = bool(visual_scan_first)
        if scan_attempted:
            detection = self._scan_camera_for_expected_target(
                cube_id,
                expected_color,
                expected_position,
                object_parking,
                vision_match_tolerance,
            )
        else:
            detection = self._confirm_visual_target(
                expected_color,
                expected_position,
                timeout=self.vision_attempt_timeout,
                match_tolerance=vision_match_tolerance,
            )
        if detection is None and not scan_attempted:
            self._event(
                "vision_recovery",
                cube=cube_id,
                action="bounded_yaw_scan",
            )
            detection = self._scan_camera_for_expected_target(
                cube_id,
                expected_color,
                expected_position,
                object_parking,
                vision_match_tolerance,
            )
            scan_attempted = True
        if detection is None:
            recovery_pose = (
                dict(visual_recovery_pose)
                if visual_recovery_pose is not None
                else None
            )
            remaining_budget = (
                float(object_navigation_deadline) - time.monotonic()
                if object_navigation_deadline is not None
                else 0.0
            )
            if recovery_pose is None or remaining_budget < 3.0:
                self._event(
                    "vision_recovery",
                    cube=cube_id,
                    action="no_ranked_backup_available",
                    remaining_budget_sec=max(0.0, remaining_budget),
                )
                return False
            self._event(
                "vision_recovery",
                cube=cube_id,
                action="single_ranked_backup",
                x=recovery_pose["x"],
                y=recovery_pose["y"],
                yaw=recovery_pose["yaw"],
                remaining_budget_sec=remaining_budget,
            )
            if not self._recover_visual_replan_approach(
                cube_id,
                expected_position,
                recovery_pose,
                min(self.object_candidate_navigation_timeout, remaining_budget),
            ):
                return False
            detection = self._confirm_replanned_visual_target(
                cube_id,
                expected_color,
                expected_position,
                recovery_pose,
                vision_match_tolerance,
            )
            if detection is None:
                return False

        self._event(
            "opencv_grasp_goal",
            cube=cube_id,
            color=expected_color,
            pixel_x=detection.get("px"),
            pixel_y=detection.get("py"),
            detected_x=float(detection["world_x"]),
            detected_y=float(detection["world_y"]),
            detected_z=float(detection["world_z"]),
        )
        detection = self._align_for_teammate_grasp(
            detection,
            cube_id,
            expected_color,
            expected_position,
            object_parking,
            vision_match_tolerance,
        )
        if detection is None:
            self._event(
                "teammate_grasp_alignment_failed",
                cube=cube_id,
                reason="visual target could not be transformed for IK",
            )
            return False
        if not self._call_grasp_trigger(
            self._pick_reset_client, "/competition/reset_pick_sequence", 5.0
        ):
            return False
        if not self._call_grasp_trigger(
            self._grasp_reset_client, "/competition/reset_pick", 5.0
        ):
            return False
        if not self._set_grasp_target_model(cube_id):
            return False
        self.pub_manipulation.publish(String(data="EXECUTING_PICK_SEQUENCE"))
        pick_succeeded = self._call_grasp_trigger(
            self._grasp_execute_client,
            "/competition/execute_pick_sequence",
            90.0,
        )
        if not pick_succeeded:
            first_failure_reason = self._last_grasp_service_reason
            if not self._quiesce_failed_pick():
                return False
            ik_failed = any(
                marker in first_failure_reason.lower()
                for marker in ("ik validation failed", "no_ik_solution", "no ik solution")
            )
            self._event(
                "teammate_pick_retry_decision",
                cube=cube_id,
                retry=ik_failed,
                reason=first_failure_reason,
            )
            if not ik_failed:
                return False
            detection = self._align_for_teammate_grasp(
                detection,
                cube_id,
                expected_color,
                expected_position,
                object_parking,
                vision_match_tolerance,
                force_correction=True,
            )
            if detection is None:
                return False
            if not self._call_grasp_trigger(
                self._pick_reset_client, "/competition/reset_pick_sequence", 5.0
            ) or not self._call_grasp_trigger(
                self._grasp_reset_client, "/competition/reset_pick", 5.0
            ) or not self._set_grasp_target_model(cube_id):
                return False
            self.pub_manipulation.publish(String(data="EXECUTING_PICK_SEQUENCE_RETRY"))
            pick_succeeded = self._call_grasp_trigger(
                self._grasp_execute_client,
                "/competition/execute_pick_sequence",
                90.0,
            )
            if not pick_succeeded:
                self._quiesce_failed_pick()
                return False
        self._attached_cube = cube_id
        if not self._verify_near(
            cube_id,
            "six_arm::link6",
            float(self.points["approach"]["attached_tolerance_m"]),
            xy_only=False,
        ):
            self._quiesce_failed_pick()
            return False
        self._event("carry_constraint", cube=cube_id, link="link6")
        return True

    def _publish_arm_grasp_target(
        self, cube_id, expected_color, detection, expected_position
    ):
        detected_map = PointStamped()
        detected_map.header.frame_id = "map"
        detected_map.point.x = float(detection["world_x"])
        detected_map.point.y = float(detection["world_y"])
        detected_map.point.z = float(detection["world_z"])
        expected_map = PointStamped()
        expected_map.header.frame_id = "map"
        expected_map.point.x = float(expected_position["x"])
        expected_map.point.y = float(expected_position["y"])
        expected_map.point.z = float(expected_position["z"]) + float(
            self.points["approach"].get("cube_half_height_m", 0.015)
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                "arm_base_link",
                "map",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            detected_arm = do_transform_point(detected_map, transform)
            expected_arm = do_transform_point(expected_map, transform)
        except Exception as exc:
            self._event(
                "grasp_target_arm",
                object_id=cube_id,
                success=False,
                error=str(exc),
            )
            return False
        payload = {
            "object_id": cube_id,
            "frame_id": "arm_base_link",
            "x": float(detected_arm.point.x),
            "y": float(detected_arm.point.y),
            "z": float(detected_arm.point.z),
            "color": expected_color,
            "delta_x": float(detected_arm.point.x - expected_arm.point.x),
            "delta_y": float(detected_arm.point.y - expected_arm.point.y),
            "map_x": float(detection["world_x"]),
            "map_y": float(detection["world_y"]),
            "map_z": float(detection["world_z"]),
            "stable_frames": int(detection.get("stable_frames", 1)),
            "xy_spread_m": float(detection.get("xy_spread_m", 0.0)),
            "z_spread_m": float(detection.get("z_spread_m", 0.0)),
        }
        self.pub_grasp_target_arm.publish(
            String(data=json.dumps(payload, sort_keys=True))
        )
        self._event("grasp_target_arm", success=True, **payload)
        return True

    def _confirm_visual_target(
        self,
        expected_color,
        expected_position,
        timeout=None,
        match_tolerance=None,
    ):
        attempt_timeout = self.vision_timeout if timeout is None else float(timeout)
        tolerance = (
            self.vision_match_tolerance
            if match_tolerance is None
            else float(match_tolerance)
        )
        deadline = time.monotonic() + attempt_timeout
        samples = []
        last_generation = -1
        last_rejection = {"reason": "no fresh detection frame"}
        while time.monotonic() < deadline and not self._should_stop():
            with self._data_lock:
                detections = list(self._latest_detections)
                age = time.monotonic() - self._latest_detection_time
                generation = self._detection_generation
            if age > 1.0 or generation == last_generation:
                time.sleep(0.05)
                continue
            last_generation = generation
            best, error = select_map_detection(
                detections,
                expected_color,
                expected_position,
                world_z_bounds=self.vision_world_z_bounds,
            )
            if best is None or error > tolerance:
                last_rejection = {
                    "reason": "no valid expected-color map detection"
                    if best is None
                    else "nearest detection outside map tolerance",
                    "detection_count": len(detections),
                    "nearest_error_m": None if best is None else float(error),
                }
                samples.clear()
                time.sleep(0.05)
                continue
            samples.append(dict(best))
            samples = samples[-self.vision_stable_frames :]
            if len(samples) < self.vision_stable_frames:
                self.pub_vision.publish(
                    String(
                        data=f"STABILIZING_{len(samples)}_OF_{self.vision_stable_frames}"
                    )
                )
                continue

            median_x = statistics.median(float(item["world_x"]) for item in samples)
            median_y = statistics.median(float(item["world_y"]) for item in samples)
            median_z = statistics.median(float(item["world_z"]) for item in samples)
            xy_spread = max(
                math.hypot(
                    float(item["world_x"]) - median_x,
                    float(item["world_y"]) - median_y,
                )
                for item in samples
            )
            z_spread = max(
                abs(float(item["world_z"]) - median_z) for item in samples
            )
            if (
                xy_spread > self.vision_stability_xy
                or z_spread > self.vision_stability_z
            ):
                samples.pop(0)
                self._event(
                    "vision_target_stability",
                    color=expected_color,
                    success=False,
                    sample_count=len(samples) + 1,
                    xy_spread_m=xy_spread,
                    z_spread_m=z_spread,
                )
                continue

            aggregated = dict(samples[-1])
            aggregated["world_x"] = float(median_x)
            aggregated["world_y"] = float(median_y)
            aggregated["world_z"] = float(median_z)
            aggregated["stable_frames"] = len(samples)
            aggregated["xy_spread_m"] = float(xy_spread)
            aggregated["z_spread_m"] = float(z_spread)
            expected_x = float(expected_position["x"])
            expected_y = float(expected_position["y"])
            error = math.hypot(median_x - expected_x, median_y - expected_y)
            self._event(
                "vision_target_confirmation",
                color=expected_color,
                expected_x=expected_x,
                expected_y=expected_y,
                detected_x=median_x,
                detected_y=median_y,
                detected_z=median_z,
                error_m=error,
                tolerance_m=tolerance,
                stable_frames=len(samples),
                xy_spread_m=xy_spread,
                z_spread_m=z_spread,
                success=True,
            )
            self.pub_last_error.publish(String(data=""))
            self.pub_vision.publish(String(data="CONFIRMED_STABLE"))
            return aggregated
        self._event(
            "vision_target_confirmation",
            color=expected_color,
            success=False,
            stable_samples=len(samples),
            reason="target not detected or not stable in map frame",
            last_rejection=last_rejection,
        )
        self.pub_vision.publish(String(data="TARGET_NOT_FOUND"))
        self.pub_last_error.publish(
            String(data=f"Vision target not found: {expected_color}")
        )
        return None

