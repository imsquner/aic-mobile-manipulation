"""Precision turn, approach, and retreat control for a reserved slot."""
import math, time
from geometry_msgs.msg import Twist
from .geometry import (
    SLOT_RELEASE_YAW_TOLERANCE_RAD,
    normalize_angle,
    reverse_progress_m,
    safe_prefix_distance,
)
from .models import SlotStatus

class ZoneSlotMotionMixin:
    'Slot motion controllers'

    def _rotate_to_slot_yaw(self, target_yaw, cube_id):
        initial = self._base_pose("map")
        if initial is None:
            return False, "map-frame base pose unavailable"
        # deadline 必须与"转向进度"用同一时钟：底盘角速度是在**仿真时间**里
        # 积分的，而本 VM 的仿真实时因子实测只有 ~0.46，用 time.monotonic()
        # （墙钟）计 deadline 会让超时提前约 2.2 倍触发 —— 2026-09-26 实测
        # world2 槽位转向因此在 32.9 s 报 "rotation timed out"。
        # 同时把转速上限提到高于估算基准（0.20 -> 0.35 rad/s），并与批准的
        # 停车门统一使用 20°，不再要求额外的 5° 精调。
        sim_now = self.get_clock().now().nanoseconds / 1e9
        deadline_sim = sim_now + max(
            8.0,
            abs(normalize_angle(float(target_yaw) - initial[2])) / 0.20 + 3.0,
        )
        yaw_tolerance = SLOT_RELEASE_YAW_TOLERANCE_RAD
        while (
            self.get_clock().now().nanoseconds / 1e9 < deadline_sim
            and not self._should_stop()
        ):
            pose = self._base_pose("map")
            if pose is None:
                self._publish_precision_stop()
                return False, "map-frame pose lost during rotation"
            yaw_error = normalize_angle(float(target_yaw) - pose[2])
            if abs(yaw_error) <= yaw_tolerance:
                self._publish_precision_stop()
                if not self._wait_for_base_still(frame_id="odom"):
                    return False, "base did not settle after rotation"
                return True, ""
            clear, reason = self._slot_rotation_clear(
                pose, pose[2], target_yaw, cube_id
            )
            if not clear:
                self._publish_precision_stop()
                return False, reason
            command = Twist()
            command.angular.z = max(-0.35, min(0.35, 1.2 * yaw_error))
            self.pub_precision_cmd.publish(command)
            time.sleep(0.05)
        self._publish_precision_stop()
        return False, "rotation timed out or execution stopped"

    def _retreat_from_slot(self, slot, cube_id):
        map_pose = self._base_pose("map")
        odom_start = self._base_pose("odom")
        odom_heading = float(odom_start[2]) if odom_start is not None else None
        requested_distance = 0.0
        last_known_signed_progress = 0.0
        motion_commanded = False

        def finish(success, reason):
            self._publish_precision_stop()
            settled = True
            if motion_commanded:
                try:
                    settled = bool(self._wait_for_base_still(frame_id="odom"))
                except Exception:
                    settled = False
            current_map = self._base_pose("map")
            current_odom = self._base_pose("odom")
            can_continue = bool(
                success
                and settled
                and current_map is not None
                and current_odom is not None
            )
            if success and not can_continue:
                if not settled:
                    reason = reason or "base did not settle after retreat"
                else:
                    reason = reason or (
                        "retreat reached its distance but the actual stop pose is unavailable"
                    )
            if current_map is None:
                self._planner_location = "__live_pose__"
            else:
                self._planner_location = self._slot_route_location_for_pose(
                    current_map
                )
            signed_progress = 0.0
            if odom_start is not None and current_odom is not None:
                try:
                    signed_progress = reverse_progress_m(
                        odom_start, current_odom, odom_heading
                    )
                except (TypeError, ValueError, IndexError):
                    signed_progress = last_known_signed_progress
            elif odom_start is not None:
                signed_progress = last_known_signed_progress
            actual_distance = max(0.0, float(signed_progress))
            fields = {
                "slot": slot.slot_id,
                "cube": str(cube_id),
                "success": can_continue,
                "safe_to_continue": can_continue,
                "base_settled": bool(settled),
                "requested_distance_m": float(requested_distance),
                "actual_distance_m": actual_distance,
                "signed_odom_progress_m": float(signed_progress),
                "next_location": self._planner_location,
                "reason": str(reason or ""),
            }
            if current_map is not None:
                fields.update({
                    "stop_map_x": float(current_map[0]),
                    "stop_map_y": float(current_map[1]),
                    "stop_map_yaw": float(current_map[2]),
                })
            else:
                fields["stop_map_pose_known"] = False
            if current_odom is not None:
                fields.update({
                    "stop_odom_x": float(current_odom[0]),
                    "stop_odom_y": float(current_odom[1]),
                    "stop_odom_yaw": float(current_odom[2]),
                })
            else:
                fields["stop_odom_pose_known"] = False
            self._event("slot_retreat_stop", **fields)
            return can_continue, actual_distance, str(reason or "")

        if map_pose is None:
            return finish(False, "map-frame pose unavailable for retreat")
        if odom_start is None:
            return finish(False, "odometry pose unavailable for retreat")
        slot_status = self._placement_slot_states.get(
            slot.slot_id, SlotStatus.AVAILABLE
        )
        if slot_status not in (
            SlotStatus.PHYSICALLY_RELEASED,
            SlotStatus.VERIFIED,
        ):
            return finish(False, "retreat requires a confirmed release in this slot")
        released_position = self._placed_cube_obstacle_positions.get(str(cube_id))
        if released_position is None or len(released_position) < 2:
            return finish(
                False,
                "confirmed released-cube obstacle position is unavailable",
            )
        max_distance = min(
            1.0,
            float(slot.exit.max_distance_m),
            float(slot.exit.nominal_distance_m),
        )
        if max_distance <= 0.0:
            return finish(
                False,
                "configured retreat distance is zero; execution must stop",
            )
        requested_distance = float(max_distance)
        start = (float(map_pose[0]), float(map_pose[1]))
        map_heading = float(map_pose[2])

        def clear_segment(first, second):
            clear, _ = self._slot_segment_clear(
                first,
                second,
                map_heading,
                active_cube_id=cube_id,
                carrying=False,
                departing_cube_id=cube_id,
            )
            return clear

        distance = safe_prefix_distance(
            start,
            normalize_angle(map_heading + math.pi),
            max_distance,
            clear_segment,
            spacing_m=0.05,
            stopping_margin_m=0.12,
        )
        requested_distance = float(distance)
        scan_points, sensor_error = self._slot_scan_points_map()
        if scan_points is None or self._latest_map is None:
            return finish(
                False,
                sensor_error or "static occupancy map unavailable",
            )
        if distance <= 0.0:
            return finish(
                False,
                "no checked reverse prefix from release pose; stopped without planning onward",
            )

        sim_now = self.get_clock().now().nanoseconds / 1e9
        sim_deadline = sim_now + max(45.0, distance / 0.04 * 3.0 + 15.0)
        wall_deadline = time.monotonic() + max(
            120.0,
            distance / 0.02 * 2.5 + 30.0,
        )
        last_progress = 0.0
        last_progress_wall = time.monotonic()
        no_progress_timeout = 15.0
        while (
            self.get_clock().now().nanoseconds / 1e9 < sim_deadline
            and time.monotonic() < wall_deadline
            and not self._should_stop()
        ):
            pose = self._base_pose("map")
            current_odom = self._base_pose("odom")
            if pose is None:
                return finish(False, "map-frame pose lost during retreat")
            if current_odom is None:
                return finish(False, "odometry pose lost during retreat")
            try:
                raw_progress = reverse_progress_m(
                    odom_start, current_odom, odom_heading
                )
            except (TypeError, ValueError, IndexError) as exc:
                return finish(False, f"invalid odometry progress during retreat: {exc}")
            last_known_signed_progress = float(raw_progress)
            progress = max(0.0, float(raw_progress))
            if progress >= distance - 0.015:
                return finish(True, "")
            if progress >= last_progress + 0.005:
                last_progress = progress
                last_progress_wall = time.monotonic()
            elif time.monotonic() - last_progress_wall > no_progress_timeout:
                return finish(False, "retreat made no odometry progress")

            remaining = max(0.0, distance - progress)
            lookahead = min(distance, progress + 0.12)
            clearance_target = (
                start[0] - lookahead * math.cos(map_heading),
                start[1] - lookahead * math.sin(map_heading),
            )
            clear, reason = self._slot_segment_clear(
                (float(pose[0]), float(pose[1])),
                clearance_target,
                float(pose[2]),
                active_cube_id=cube_id,
                carrying=False,
                departing_cube_id=cube_id,
            )
            if not clear:
                self._event(
                    "slot_retreat_early_stop",
                    slot=slot.slot_id,
                    cube=str(cube_id),
                    actual_distance_m=progress,
                    obstacle_reason=reason,
                )
                return finish(False, reason or "retreat clearance check blocked")
            yaw_error = normalize_angle(map_heading - float(pose[2]))
            command = Twist()
            # 2026-09-27 老板指示：原实现最大只有 0.04 m/s（比正常行驶 1.2 m/s 慢 30 倍），
            # 退 1 m 要 25 s 仿真时间，严重拖慢整轮。这里提到与行驶同量级（上限 1.0 m/s），
            # 同时把转向权限提上来以维持在直线走廊内。净空检查仍是每周期执行（20 Hz，
            # 单周期行程 ≤0.05 m，与 safe_prefix_distance 的 spacing_m 一致）。
            command.linear.x = -min(1.0, max(0.15, 1.5 * remaining))
            command.angular.z = max(-0.5, min(0.5, 1.2 * yaw_error))
            motion_commanded = True
            self.pub_precision_cmd.publish(command)
            time.sleep(0.05)
        if self._should_stop():
            return finish(False, "execution stopped during retreat")
        if time.monotonic() >= wall_deadline:
            reason = "retreat wall-clock watchdog expired"
        else:
            reason = "retreat simulation-time deadline expired"
        return finish(False, reason)
