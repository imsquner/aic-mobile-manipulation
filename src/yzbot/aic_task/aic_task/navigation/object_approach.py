"""Gazebo and visual gates for object approach recovery."""

import math
import time

from geometry_msgs.msg import Twist

from aic_task.motion_geometry import (
    gazebo_entity_pose_to_planar,
    object_approach_errors,
    object_approach_within_limits,
    signed_object_yaw_error,
)


class ObjectApproachMixin:
    def _object_pose_within_visual_gate(
        self, parking, xy_tolerance=None, yaw_tolerance=None
    ):
        pose = self._base_pose()
        if pose is None:
            return False
        xy_error = math.hypot(pose[0] - parking["x"], pose[1] - parking["y"])
        yaw_error = abs(self._normalize_angle(pose[2] - parking["yaw"]))
        xy_limit = (
            self.object_visual_gate_xy_tolerance
            if xy_tolerance is None
            else float(xy_tolerance)
        )
        yaw_limit = (
            self.object_visual_gate_yaw_tolerance
            if yaw_tolerance is None
            else float(yaw_tolerance)
        )
        success = (
            xy_error <= xy_limit
            and yaw_error <= yaw_limit
        )
        self._event(
            "object_visual_gate_pose",
            success=success,
            xy_error_m=xy_error,
            xy_tolerance_m=xy_limit,
            yaw_error_deg=math.degrees(yaw_error),
            yaw_tolerance_deg=math.degrees(
                yaw_limit
            ),
        )
        return success

    def _object_approach_within_gate(
        self,
        object_position,
        minimum_distance,
        maximum_distance,
        yaw_tolerance,
    ):
        pose = self._base_pose("map")
        if pose is None:
            return False
        target_position = (
            float(object_position["x"]),
            float(object_position["y"]),
        )
        distance, yaw_error = object_approach_errors(pose, target_position)
        success = object_approach_within_limits(
            pose,
            target_position,
            minimum_distance,
            maximum_distance,
            yaw_tolerance,
        )
        self._event(
            "object_approach_gate",
            success=success,
            distance_m=distance,
            minimum_distance_m=float(minimum_distance),
            maximum_distance_m=float(maximum_distance),
            yaw_error_deg=math.degrees(yaw_error),
            yaw_tolerance_deg=math.degrees(float(yaw_tolerance)),
        )
        return success

    def _gazebo_object_approach_gate(self, cube_id):
        """Use simulation truth only to recover from a degraded map transform."""
        robot_pose = self._entity_pose("six_arm")
        cube_pose = self._entity_pose(cube_id)
        if robot_pose is None or cube_pose is None:
            self._event(
                "gazebo_object_approach_gate",
                cube=cube_id,
                success=False,
                reason="entity pose unavailable",
            )
            return False
        approach = self.points["approach"]
        target_distance = float(
            approach.get(
                "object_approach_target_distance_m",
                approach.get("object_approach_distance_m", 0.40),
            )
        )
        minimum_distance = float(
            approach.get("object_approach_min_distance_m", target_distance)
        )
        maximum_distance = float(
            approach.get("object_approach_max_distance_m", target_distance)
        )
        yaw_tolerance = float(
            approach.get(
                "object_approach_yaw_tolerance_rad", math.radians(15.0)
            )
        )
        robot_planar_pose = gazebo_entity_pose_to_planar(robot_pose)
        distance, yaw_error = object_approach_errors(
            robot_planar_pose, (cube_pose[0], cube_pose[1])
        )
        success = object_approach_within_limits(
            robot_planar_pose,
            (cube_pose[0], cube_pose[1]),
            minimum_distance,
            maximum_distance,
            yaw_tolerance,
        )
        self._event(
            "gazebo_object_approach_gate",
            cube=cube_id,
            success=success,
            distance_m=distance,
            minimum_distance_m=minimum_distance,
            maximum_distance_m=maximum_distance,
            yaw_error_deg=math.degrees(yaw_error),
            yaw_tolerance_deg=math.degrees(yaw_tolerance),
            authority="recovery_only; RGB-D and IK remain mandatory",
        )
        return success

    def _gazebo_recover_object_approach(self, cube_id, timeout=40.0):
        """Recover range and heading after map localization becomes unreliable."""
        approach = self.points["approach"]
        target_distance = float(
            approach.get(
                "object_approach_target_distance_m",
                approach.get("object_approach_distance_m", 0.40),
            )
        )
        distance_tolerance = max(
            0.01,
            float(
                approach.get(
                    "object_recovery_distance_tolerance_m",
                    min(
                        0.04,
                        0.5
                        * (
                            float(
                                approach.get(
                                    "object_approach_max_distance_m", target_distance
                                )
                            )
                            - float(
                                approach.get(
                                    "object_approach_min_distance_m", target_distance
                                )
                            )
                        ),
                    ),
                )
            ),
        )
        yaw_tolerance = min(
            float(
                approach.get(
                    "object_approach_yaw_tolerance_rad", math.radians(25.0)
                )
            ),
            float(
                approach.get(
                    "object_recovery_yaw_tolerance_rad", math.radians(15.0)
                )
            ),
        )
        maximum_displacement = max(
            0.10,
            float(
                approach.get("object_recovery_max_displacement_m", 0.80)
            ),
        )
        first_robot_pose = self._entity_pose("six_arm")
        if first_robot_pose is None:
            return False
        deadline = time.monotonic() + float(timeout)
        sample_count = 0
        last_distance = None
        last_yaw_error = None
        failure_reason = "timeout or stop requested"
        while time.monotonic() < deadline and not self._should_stop():
            robot_pose = self._entity_pose("six_arm")
            cube_pose = self._entity_pose(cube_id)
            if robot_pose is None or cube_pose is None:
                failure_reason = "entity pose unavailable"
                break
            values = tuple(robot_pose) + tuple(cube_pose)
            if not all(math.isfinite(float(value)) for value in values):
                failure_reason = "non-finite entity pose"
                break
            displacement = math.hypot(
                robot_pose[0] - first_robot_pose[0],
                robot_pose[1] - first_robot_pose[1],
            )
            if displacement > maximum_displacement:
                failure_reason = (
                    "recovery displacement exceeded "
                    f"{maximum_displacement:.2f} m"
                )
                break
            distance = math.hypot(
                cube_pose[0] - robot_pose[0],
                cube_pose[1] - robot_pose[1],
            )
            robot_planar_pose = gazebo_entity_pose_to_planar(robot_pose)
            yaw_error = signed_object_yaw_error(
                robot_planar_pose,
                (cube_pose[0], cube_pose[1]),
            )
            last_distance = distance
            last_yaw_error = yaw_error
            sample_count += 1
            if (
                abs(distance - target_distance) <= distance_tolerance
                and abs(yaw_error) <= yaw_tolerance
            ):
                self._publish_precision_stop()
                stationary = self._wait_for_base_still(frame_id="odom")
                localization_synced = (
                    stationary
                    and self._synchronize_localization_to_gazebo(robot_pose)
                )
                success = stationary and localization_synced
                self._event(
                    "gazebo_object_approach_recovery",
                    cube=cube_id,
                    success=success,
                    sample_count=sample_count,
                    distance_m=distance,
                    target_distance_m=target_distance,
                    yaw_error_deg=math.degrees(abs(yaw_error)),
                    displacement_m=displacement,
                    maximum_displacement_m=maximum_displacement,
                    distance_tolerance_m=distance_tolerance,
                    yaw_tolerance_deg=math.degrees(yaw_tolerance),
                    base_stationary=stationary,
                    localization_synced=localization_synced,
                    authority="recovery only; RGB-D and IK remain mandatory",
                )
                return success
            command = Twist()
            command.angular.z = max(-0.25, min(0.25, 1.2 * yaw_error))
            if abs(yaw_error) <= math.radians(10.0):
                range_error = distance - target_distance
                command.linear.x = max(-0.08, min(0.08, 0.8 * range_error))
            self.pub_precision_cmd.publish(command)
            time.sleep(0.08)
        self._publish_precision_stop()
        self._event(
            "gazebo_object_approach_recovery",
            cube=cube_id,
            success=False,
            sample_count=sample_count,
            distance_m=last_distance,
            target_distance_m=target_distance,
            yaw_error_deg=(
                math.degrees(abs(last_yaw_error))
                if last_yaw_error is not None
                else None
            ),
            reason=failure_reason,
            maximum_displacement_m=maximum_displacement,
            distance_tolerance_m=distance_tolerance,
            yaw_tolerance_deg=math.degrees(yaw_tolerance),
            authority="recovery only; RGB-D and IK remain mandatory",
        )
        return False

    def _runtime_cube_for_approach(self, cube):
        """Snapshot the current simulated cube position for this attempt."""
        runtime_cube = dict(cube)
        static_position = dict(cube["position"])
        runtime_cube["position"] = static_position
        entity_pose = self._entity_pose(str(cube["id"]))
        source = "static_config"
        displacement = None
        if entity_pose is not None and len(entity_pose) >= 3:
            live_position = tuple(float(value) for value in entity_pose[:3])
            if all(math.isfinite(value) for value in live_position):
                displacement = math.hypot(
                    live_position[0] - float(static_position["x"]),
                    live_position[1] - float(static_position["y"]),
                )
                runtime_cube["position"] = {
                    **static_position,
                    "x": live_position[0],
                    "y": live_position[1],
                    "z": live_position[2],
                }
                source = "gazebo_runtime_recovery"
        selected_position = runtime_cube["position"]
        self._event(
            "object_runtime_position",
            cube=str(cube["id"]),
            source=source,
            authority="approach recovery only; RGB-D, alignment, and IK remain mandatory",
            static_position={
                key: float(static_position[key])
                for key in ("x", "y", "z")
                if key in static_position
            },
            runtime_position={
                key: float(selected_position[key])
                for key in ("x", "y", "z")
                if key in selected_position
            },
            displacement_m=displacement,
        )
        return runtime_cube
