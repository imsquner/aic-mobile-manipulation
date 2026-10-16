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


class ResetLocalizationGate:
    """Reset-epoch evidence gate; normal navigation drift limits are unrelated."""
    def __init__(self, target, started_ns, started_mono, amcl_sequence,
                 stable_sec=.5, min_samples=5, max_age_sec=1.):
        self.target = target
        self.started_ns, self.started_mono = started_ns, started_mono
        self.amcl_sequence = amcl_sequence
        self.stable_sec, self.min_samples, self.max_age_sec = stable_sec, min_samples, max_age_sec
        self.last_ros_ns = started_ns
        self.invalid_clock = False
        self.stable_since = None
        self.samples = 0
        self.previous = None
        self.last_stamp = None

    def reject(self, reason, **detail):
        self.stable_since, self.samples, self.previous, self.last_stamp = None, 0, None, None
        return {"success": False, "reason": reason, **detail}

    def observe(self, snapshot, amcl, *, now_mono, ros_now_ns, base_speed, odom_age_sec):
        if ros_now_ns < self.last_ros_ns:
            self.invalid_clock = True
        self.last_ros_ns = ros_now_ns
        if self.invalid_clock:
            return self.reject("LOC_CLOCK_ROLLBACK")
        if (not amcl or amcl.get("sequence", 0) <= self.amcl_sequence
                or amcl.get("receipt_mono", -1) < self.started_mono
                or amcl.get("stamp_ns", 0) < self.started_ns
                or amcl.get("frame_id") != "map"):
            return self.reject("LOC_NO_NEW_AMCL")
        if not snapshot or snapshot.get("reason"):
            return self.reject((snapshot or {}).get("reason", "LOC_TF_UNAVAILABLE"))
        try:
            poses = [snapshot[key] for key in ("map_base", "odom_base", "map_odom")]
            if any(len(pose) != 3 or not all(math.isfinite(v) for v in pose) for pose in poses):
                return self.reject("LOC_TF_UNAVAILABLE")
            stamp = snapshot["stamp_ns"]
        except (KeyError, TypeError, ValueError):
            return self.reject("LOC_TF_UNAVAILABLE")
        age = (ros_now_ns - stamp) / 1e9
        if stamp < max(self.started_ns, amcl["stamp_ns"]) or age > self.max_age_sec or age < -.15:
            return self.reject("LOC_TF_STALE_OR_EPOCH", age_sec=age)
        if (base_speed is None or odom_age_sec > 1. or base_speed[0] > .01 or base_speed[1] > .02):
            return self.reject("BASE_NOT_STILL")
        amcl_pose = amcl.get("pose")
        if not amcl_pose or len(amcl_pose) != 3 or not all(math.isfinite(v) for v in amcl_pose):
            return self.reject("LOC_NO_NEW_AMCL")
        if math.hypot(amcl_pose[0]-self.target[0], amcl_pose[1]-self.target[1]) > .12:
            return self.reject("LOC_TARGET_XY", source="AMCL")
        if abs(math.atan2(math.sin(amcl_pose[2]-self.target[2]), math.cos(amcl_pose[2]-self.target[2]))) > math.radians(8):
            return self.reject("LOC_TARGET_YAW", source="AMCL")
        pose = snapshot["map_base"]
        xy = math.hypot(pose[0]-self.target[0], pose[1]-self.target[1])
        angle = abs(math.atan2(math.sin(pose[2]-self.target[2]), math.cos(pose[2]-self.target[2])))
        if xy > .12:
            return self.reject("LOC_TARGET_XY", xy_error_m=xy)
        if angle > math.radians(8):
            return self.reject("LOC_TARGET_YAW", yaw_error_rad=angle)
        if self.previous is not None:
            for key in ("map_base", "map_odom"):
                previous, current = self.previous[key], snapshot[key]
                if (math.hypot(current[0]-previous[0], current[1]-previous[1]) > .03
                        or abs(math.atan2(math.sin(current[2]-previous[2]), math.cos(current[2]-previous[2]))) > .05):
                    self.stable_since, self.samples, self.last_stamp = None, 0, None
                    break
        if stamp != self.last_stamp:
            if self.stable_since is None:
                self.stable_since = now_mono
            self.samples += 1
            self.last_stamp = stamp
            self.previous = dict(snapshot)
        success = self.samples >= self.min_samples and now_mono-self.stable_since >= self.stable_sec
        return {"success": success, "verified": success, "reason": "" if success else "LOC_NOT_STABLE",
                "samples": self.samples, "stable_duration_sec": now_mono-self.stable_since,
                "xy_error_m": xy, "yaw_error_rad": angle, "snapshot": dict(snapshot),
                "amcl_sequence": amcl["sequence"], "amcl_stamp_ns": amcl["stamp_ns"]}


class LocalizationMixin:
    def _new_reset_localization_gate(self, target, started_ns, started_mono, amcl_sequence, **kwargs):
        return ResetLocalizationGate(target, started_ns, started_mono, amcl_sequence, **kwargs)

    def _on_reset_amcl(self, message):
        pose = message.pose.pose
        q = pose.orientation
        planar = (pose.position.x, pose.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(math.isfinite(v) for v in planar):
            return
        with self._reset_amcl_lock:
            self._reset_amcl_sequence += 1
            self._latest_reset_amcl = {"sequence": self._reset_amcl_sequence,
                "stamp_ns": message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                "receipt_mono": time.monotonic(), "frame_id": message.header.frame_id, "pose": planar}

    def _reset_localization_snapshot(self, at_time, target_pose):
        try:
            if at_time is None:
                latest = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                latest_ns = latest.header.stamp.sec*10**9+latest.header.stamp.nanosec
                at_time = rclpy.time.Time(nanoseconds=min(latest_ns, self.get_clock().now().nanoseconds))
            result = {"stamp_ns": at_time.nanoseconds, "lookup_stamps_ns": {}}
            for key, parent, child in (("map_base", "map", "base_footprint"),
                                       ("odom_base", "odom", "base_footprint"),
                                       ("map_odom", "map", "odom")):
                stamped = self._tf_buffer.lookup_transform(parent, child, at_time)
                t, q = stamped.transform.translation, stamped.transform.rotation
                result[key] = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                result["lookup_stamps_ns"][key] = stamped.header.stamp.sec*10**9+stamped.header.stamp.nanosec
            return result
        except Exception as exc:
            return {"reason": "LOC_TF_UNAVAILABLE", "exception": type(exc).__name__}

    def _verify_reset_localization(self, reset_id, target_pose, *, timeout_sec=None,
                                   stable_sec=None, min_samples=None, publish_initial=True):
        timeout_sec = self._reset_localization_timeout if timeout_sec is None else timeout_sec
        with self._reset_amcl_lock:
            sequence = self._reset_amcl_sequence
        now = time.monotonic()
        ros_start = self.get_clock().now().nanoseconds
        if publish_initial:
            self._reset_localization_epoch = {"reset_id": reset_id, "target": target_pose,
                "started_ns": ros_start, "started_mono": now, "amcl_sequence": sequence}
        epoch = self._reset_localization_epoch
        if not epoch or epoch["reset_id"] != reset_id:
            return False, {"reason": "LOC_TF_STALE_OR_EPOCH"}
        gate = self._new_reset_localization_gate(target_pose, epoch["started_ns"], epoch["started_mono"],
            epoch["amcl_sequence"], stable_sec=self._reset_localization_stable if stable_sec is None else stable_sec,
            min_samples=self._reset_localization_samples if min_samples is None else min_samples)
        deadline = now+timeout_sec
        gate.last_ros_ns = epoch.get("last_ros_ns", epoch["started_ns"])
        next_publish = 0.
        first_publish = True
        evidence = {"reason": "LOC_NO_NEW_AMCL"}
        while time.monotonic() < deadline:
            self._control_checkpoint()
            now = time.monotonic()
            with self._reset_amcl_lock:
                amcl = dict(self._latest_reset_amcl) if self._latest_reset_amcl else None
            if publish_initial and now >= next_publish and (not amcl or amcl["sequence"] <= sequence):
                message = PoseWithCovarianceStamped()
                message.header.frame_id = "map"
                # Plan G fix 2 (2026-10-16): stamping this with "now" makes AMCL look up a transform
                # the TF buffer has not produced yet ("Failed to transform initial pose in time ...
                # extrapolation into the future"), so the reset's initial pose was silently dropped
                # and the gate then timed out with LOC_NO_NEW_AMCL (5 of the 10 reset failures in the
                # 54-round run). A zero stamp means "latest available" for tf2, which is what AMCL
                # should use; the freshness epoch below still uses the real publish time.
                publish_ns = self.get_clock().now().nanoseconds
                message.header.stamp.sec = 0
                message.header.stamp.nanosec = 0
                message.pose.pose.position.x, message.pose.pose.position.y = target_pose[:2]
                message.pose.pose.orientation.z = math.sin(target_pose[2]/2)
                message.pose.pose.orientation.w = math.cos(target_pose[2]/2)
                message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[35] = .01
                if first_publish:
                    with self._reset_amcl_lock:
                        sequence = self._reset_amcl_sequence
                    epoch.update(started_ns=publish_ns,
                                 started_mono=time.monotonic(), amcl_sequence=sequence)
                    gate = self._new_reset_localization_gate(target_pose, epoch["started_ns"], epoch["started_mono"],
                        sequence, stable_sec=self._reset_localization_stable if stable_sec is None else stable_sec,
                        min_samples=self._reset_localization_samples if min_samples is None else min_samples)
                    first_publish = False
                self.pub_initial_pose.publish(message)
                next_publish = now+.25
            snapshot = self._reset_localization_snapshot(None, target_pose)
            evidence = gate.observe(snapshot, amcl, now_mono=now,
                ros_now_ns=self.get_clock().now().nanoseconds,
                base_speed=self._latest_base_speed, odom_age_sec=now-self._latest_odom_time)
            evidence["reset_id"] = reset_id
            epoch["last_ros_ns"] = gate.last_ros_ns
            if evidence["success"]:
                self._event("reset_localization_verified", **evidence)
                return True, evidence
            if evidence["reason"] == "LOC_CLOCK_ROLLBACK":
                break
            time.sleep(.05)
        self._event("reset_localization_failed", **evidence)
        return False, evidence

    def _finalize_reset_localization(self, reset_id, evidence=None):
        epoch = self._reset_localization_epoch
        if not epoch or epoch["reset_id"] != reset_id:
            return False, {"reason": "LOC_TF_STALE_OR_EPOCH"}
        ok, result = self._verify_reset_localization(reset_id, epoch["target"], publish_initial=False)
        if ok:
            self._control_checkpoint()
            self._map_from_odom_reference = tuple(result["snapshot"]["map_odom"])
            self._reset_localization_evidence = result
        return ok, result

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

