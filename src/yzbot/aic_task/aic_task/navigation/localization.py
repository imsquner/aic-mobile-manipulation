"""AMCL/Gazebo localization reconciliation and base-pose lookup."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration

from aic_task.motion_geometry import (
    expected_map_pose,
    gazebo_entity_pose_to_planar,
)


class LocalizationMixin:
    def _localization_consistency(self):
        map_pose = self._base_pose("map")
        odom_pose = self._base_pose("odom")
        current_reference = self._base_pose("map", "odom")
        if map_pose is None or odom_pose is None or current_reference is None:
            return None
        if self._map_from_odom_reference is None:
            self._map_from_odom_reference = current_reference
            self._event(
                "localization_reference",
                map_x=map_pose[0],
                map_y=map_pose[1],
                map_yaw=map_pose[2],
                odom_x=odom_pose[0],
                odom_y=odom_pose[1],
                odom_yaw=odom_pose[2],
                map_from_odom_x=current_reference[0],
                map_from_odom_y=current_reference[1],
                map_from_odom_yaw=current_reference[2],
            )
        expected = expected_map_pose(
            self._map_from_odom_reference, odom_pose
        )
        xy_error = math.hypot(
            current_reference[0] - self._map_from_odom_reference[0],
            current_reference[1] - self._map_from_odom_reference[1],
        )
        yaw_error = abs(
            self._normalize_angle(
                current_reference[2] - self._map_from_odom_reference[2]
            )
        )
        drifted = (
            xy_error > self.localization_drift_xy
            or yaw_error > self.localization_drift_yaw
        )
        # AMCL is expected to refine map->odom continuously. Compare each
        # sample with the immediately preceding transform so normal gradual
        # corrections cannot accumulate into a false localization reset.
        if not drifted:
            self._map_from_odom_reference = current_reference
        return {
            "drifted": drifted,
            "map_pose": map_pose,
            "odom_pose": odom_pose,
            "map_from_odom": current_reference,
            "expected_map_pose": expected,
            "xy_error_m": xy_error,
            "yaw_error_rad": yaw_error,
        }

    def _accept_amcl_update(self, localization, phase):
        """Accept AMCL's authoritative map->odom correction without stopping Nav2."""
        self._map_from_odom_reference = localization["map_from_odom"]
        self._event(
            "localization_update_accepted",
            phase=str(phase),
            xy_delta_m=localization["xy_error_m"],
            yaw_delta_deg=math.degrees(localization["yaw_error_rad"]),
            map_from_odom_x=localization["map_from_odom"][0],
            map_from_odom_y=localization["map_from_odom"][1],
            map_from_odom_yaw=localization["map_from_odom"][2],
        )

    def _recover_localization(self, localization):
        self._publish_precision_stop()
        stationary = self._wait_for_base_still(frame_id="odom")
        current_map_pose = self._base_pose("map") or localization["map_pose"]
        odom_pose = self._base_pose("odom")
        current_reference = self._base_pose("map", "odom")
        if current_map_pose is None or odom_pose is None or current_reference is None:
            self._event(
                "localization_recovery_result",
                success=False,
                costmaps_cleared=False,
                method="trust_amcl_rebase",
                reason="map or odom pose unavailable",
            )
            return False
        expected = localization["expected_map_pose"]
        self._event(
            "localization_drift_detected",
            xy_error_m=localization["xy_error_m"],
            yaw_error_deg=math.degrees(localization["yaw_error_rad"]),
            odom_stationary=stationary,
            expected_map_x=expected[0],
            expected_map_y=expected[1],
            expected_map_yaw=expected[2],
            actual_map_x=current_map_pose[0],
            actual_map_y=current_map_pose[1],
            actual_map_yaw=current_map_pose[2],
            recovery_method="trust_amcl_rebase",
        )
        if not stationary:
            self._navigation_observation_quarantined = True
            self._stop_exec = True
            self._event(
                "localization_recovery_result",
                success=False,
                costmaps_cleared=False,
                method="trust_amcl_rebase",
                odom_stationary=False,
                reason="base remained in motion; navigation stopped for safety",
            )
            return False
        # AMCL owns map->odom. Odometry is relative and must never overwrite a
        # valid AMCL correction, especially while the base is still settling.
        self._map_from_odom_reference = current_reference
        costmaps_cleared = self._clear_navigation_costmaps()
        self._event(
            "localization_recovery_result",
            success=costmaps_cleared,
            costmaps_cleared=costmaps_cleared,
            method="trust_amcl_rebase",
            odom_stationary=stationary,
            map_x=current_map_pose[0],
            map_y=current_map_pose[1],
            map_yaw=current_map_pose[2],
        )
        return costmaps_cleared

    def _invalidate_localization_reference(self, reason):
        previous = self._map_from_odom_reference
        self._map_from_odom_reference = None
        self._event(
            "localization_reference_invalidated",
            reason=str(reason),
            had_reference=previous is not None,
        )

    def _synchronize_localization_to_gazebo(self, entity_pose, timeout=5.0):
        """Re-anchor AMCL after the bounded simulation-only base recovery."""
        try:
            x, y, yaw = gazebo_entity_pose_to_planar(entity_pose)
        except ValueError as exc:
            self._event(
                "gazebo_localization_sync",
                success=False,
                reason=str(exc),
            )
            return False

        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01

        deadline = time.monotonic() + float(timeout)
        publish_count = 0
        last_xy_error = None
        last_yaw_error = None
        next_publish = 0.0
        while time.monotonic() < deadline and not self._should_stop():
            now = time.monotonic()
            if now >= next_publish:
                message.header.stamp = self.get_clock().now().to_msg()
                self.pub_initial_pose.publish(message)
                publish_count += 1
                next_publish = now + 0.25
            map_pose = self._base_pose("map")
            if map_pose is not None:
                last_xy_error = math.hypot(map_pose[0] - x, map_pose[1] - y)
                last_yaw_error = abs(self._normalize_angle(map_pose[2] - yaw))
                if (
                    last_xy_error <= 0.12
                    and last_yaw_error <= math.radians(8.0)
                ):
                    current_reference = self._base_pose("map", "odom")
                    if current_reference is not None:
                        self._map_from_odom_reference = current_reference
                        costmaps_cleared = self._clear_navigation_costmaps()
                        self._event(
                            "gazebo_localization_sync",
                            success=costmaps_cleared,
                            publish_count=publish_count,
                            xy_error_m=last_xy_error,
                            yaw_error_deg=math.degrees(last_yaw_error),
                            authority="recovery only; RGB-D and IK remain mandatory",
                            costmaps_cleared=costmaps_cleared,
                        )
                        return costmaps_cleared
            time.sleep(0.05)
        self._event(
            "gazebo_localization_sync",
            success=False,
            publish_count=publish_count,
            xy_error_m=last_xy_error,
            yaw_error_deg=(
                math.degrees(last_yaw_error)
                if last_yaw_error is not None
                else None
            ),
            reason="AMCL did not converge to the recovered physical pose",
            authority="recovery only; RGB-D and IK remain mandatory",
        )
        return False

    def _ensure_gazebo_localization_before_navigation(self):
        """Reject gross AMCL drift before Nav2 plans from an invalid map pose."""
        entity_pose = self._entity_pose("six_arm")
        map_pose = self._base_pose("map")
        if entity_pose is None or map_pose is None:
            self._event(
                "pre_navigation_localization_gate",
                success=True,
                checked=False,
                reason="Gazebo entity pose or map transform unavailable",
            )
            return True
        try:
            gazebo_x, gazebo_y, gazebo_yaw = gazebo_entity_pose_to_planar(entity_pose)
        except ValueError as exc:
            self._event(
                "pre_navigation_localization_gate",
                success=False,
                checked=True,
                reason=str(exc),
            )
            return False
        xy_error = math.hypot(map_pose[0] - gazebo_x, map_pose[1] - gazebo_y)
        yaw_error = abs(self._normalize_angle(map_pose[2] - gazebo_yaw))
        xy_limit = min(float(self.localization_drift_xy), 0.35)
        yaw_limit = min(float(self.localization_drift_yaw), math.radians(15.0))
        drifted = xy_error > xy_limit or yaw_error > yaw_limit
        if not drifted:
            self._event(
                "pre_navigation_localization_gate",
                success=True,
                checked=True,
                corrected=False,
                xy_error_m=xy_error,
                xy_tolerance_m=xy_limit,
                yaw_error_deg=math.degrees(yaw_error),
                yaw_tolerance_deg=math.degrees(yaw_limit),
            )
            return True

        self._publish_precision_stop()
        stationary = self._wait_for_base_still(frame_id="odom")
        corrected = stationary and self._synchronize_localization_to_gazebo(
            entity_pose
        )
        self._event(
            "pre_navigation_localization_gate",
            success=corrected,
            checked=True,
            corrected=corrected,
            xy_error_m=xy_error,
            xy_tolerance_m=xy_limit,
            yaw_error_deg=math.degrees(yaw_error),
            yaw_tolerance_deg=math.degrees(yaw_limit),
            base_stationary=stationary,
            authority="simulation recovery only",
        )
        return corrected

    def _base_pose(self, frame_id="map", child_frame_id="base_footprint"):
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                child_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            ).transform
        except Exception as exc:
            self._event("base_pose", success=False, error=str(exc))
            return None
        translation = transform.translation
        rotation = transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return translation.x, translation.y, math.atan2(siny_cosp, cosy_cosp)

