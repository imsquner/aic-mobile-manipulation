"""Release, verification, occupancy, and UNKNOWN transaction."""
import math, time
from std_msgs.msg import String
from aic_task.placement_geometry import check_region_placement
from .geometry import (
    SLOT_RELEASE_RECOVERY_RADIUS_M,
    SLOT_RELEASE_YAW_TOLERANCE_RAD,
    classify_slot_parking,
    normalize_angle,
)
from .models import SlotStatus
from .selection import matrix_node_name, transition_slot_state

class ZoneSlotReleaseMixin:
    'Slot release transaction'

    def _slot_staging_route_available(self, source_node, slot):
        """Require both matrix legs before using staging as a Nav2 waypoint."""
        matrix = getattr(self, "_route_matrix", None)
        if matrix is None:
            return False, "path matrix is unavailable"
        staging_node = matrix_node_name(slot.zone, slot.number, "staging")
        release_node = matrix_node_name(slot.zone, slot.number, "release")
        nodes = set(getattr(matrix, "node_names", ()))
        if not source_node or source_node not in nodes:
            return False, f"source node {source_node!r} is not in the path matrix"
        if staging_node not in nodes or release_node not in nodes:
            return False, "staging or release node is not in the path matrix"
        try:
            matrix.edge(str(source_node), staging_node)
            matrix.edge(staging_node, release_node)
        except Exception as exc:
            return False, f"matrix route via staging is not fully reachable: {exc}"
        return True, ""

    def _record_slot_navigation_edge(
        self, source, target, started_at, success, phase
    ):
        matrix = getattr(self, "_route_matrix", None)
        nodes = set(getattr(matrix, "node_names", ())) if matrix is not None else set()
        if source in nodes and target in nodes:
            self._record_navigation_edge(
                source, target, started_at, success, phase
            )
            return
        self._event(
            "navigation_edge_observation_skipped",
            source=source,
            target=target,
            success=bool(success),
            phase=phase,
            reason="actual route endpoint is not represented in the path matrix",
        )

    def _navigate_slot_via_staging(self, source_node, slot):
        available, reason = self._slot_staging_route_available(source_node, slot)
        if not available:
            self._event(
                "slot_staging_route_rejected",
                slot=slot.slot_id,
                source=source_node,
                reason=reason,
            )
            return False, reason

        staging_node = matrix_node_name(slot.zone, slot.number, "staging")
        release_node = matrix_node_name(slot.zone, slot.number, "release")
        started = time.monotonic()
        staging_reported = self._navigate_to(
            slot.staging.x,
            slot.staging.y,
            slot.yaw,
            timeout=self.carry_navigation_timeout,
        )
        stage_pose = self._base_pose("map")
        stage_error = (
            math.hypot(stage_pose[0] - slot.staging.x,
                       stage_pose[1] - slot.staging.y)
            if stage_pose is not None else math.inf
        )
        stage_yaw_error = (
            abs(normalize_angle(stage_pose[2] - slot.yaw))
            if stage_pose is not None else math.inf
        )
        stage_arrived = (
            stage_pose is not None
            and classify_slot_parking(stage_error, stage_yaw_error) == "normal"
            and self._wait_for_base_still(frame_id="odom")
        )
        self._record_slot_navigation_edge(
            source_node,
            staging_node,
            started,
            stage_arrived,
            "carry_to_slot_staging_waypoint",
        )
        self._event(
            "slot_staging_waypoint",
            slot=slot.slot_id,
            nav_reported=bool(staging_reported),
            position_error_m=stage_error,
            yaw_error_rad=stage_yaw_error,
            final_release_goal=False,
            arrived=bool(stage_arrived),
        )
        if not stage_arrived:
            return False, "Nav2 did not reach the optional staging waypoint"

        release_started = time.monotonic()
        release_reported = self._navigate_to(
            slot.release.x,
            slot.release.y,
            slot.yaw,
            timeout=self.carry_navigation_timeout,
        )
        self._record_slot_navigation_edge(
            staging_node,
            release_node,
            release_started,
            release_reported,
            "carry_from_staging_to_slot_release",
        )
        self._event(
            "slot_release_navigation",
            slot=slot.slot_id,
            route="staging_then_release",
            success=bool(release_reported),
        )
        return bool(release_reported), (
            "" if release_reported else "Nav2 failed from staging to slot release"
        )

    def _execute_world2_slot(
        self, item, cube, zone, slot, *, has_more_items,
        route_via_staging=False,
    ):
        slot_id = slot.slot_id
        status = self._placement_slot_states.get(slot_id, SlotStatus.AVAILABLE)
        if status is not SlotStatus.AVAILABLE:
            self._event("slot_state_blocked", zone=item.zone, slot=slot_id,
                        reason=f"slot is {status.value}")
            return False, f"slot {slot_id} is not available"
        if route_via_staging:
            source_node = str(getattr(self, "_planner_location", ""))
            route_available, route_reason = self._slot_staging_route_available(
                source_node, slot
            )
            if not route_available:
                self._event(
                    "slot_staging_route_rejected",
                    slot=slot_id,
                    source=source_node,
                    reason=route_reason,
                )
                return False, route_reason
        self._placement_slot_states = transition_slot_state(
            self._placement_slot_states, slot_id, SlotStatus.RESERVED
        )
        release_confirmed = False
        release_attempted = False
        strict_inside = False
        within_nearby = False
        detail = ""
        try:
            if self._attached_cube != str(cube["id"]):
                return False, "slot release blocked: target cube is not confirmed attached"
            # Fail before Nav2 starts if critical safety inputs are unavailable.
            if self._latest_map is None:
                return False, "slot maneuver blocked: static occupancy map unavailable"
            scan_points, scan_error = self._slot_scan_points_map()
            if scan_points is None:
                return False, f"slot maneuver blocked: {scan_error}"
            self._transition(self.TaskState.NAVIGATING_TO_ZONE)
            self.pub_navigation.publish(String(data="NAVIGATING_TO_ZONE"))
            source_node = str(getattr(self, "_planner_location", ""))
            if route_via_staging:
                nav_reported, navigation_reason = self._navigate_slot_via_staging(
                    source_node, slot
                )
            else:
                nav_started = time.monotonic()
                nav_reported = self._navigate_to(
                    slot.release.x,
                    slot.release.y,
                    slot.yaw,
                    timeout=self.carry_navigation_timeout,
                )
                release_node = matrix_node_name(slot.zone, slot.number, "release")
                self._record_slot_navigation_edge(
                    source_node,
                    release_node,
                    nav_started,
                    nav_reported,
                    "carry_direct_to_slot_release",
                )
                navigation_reason = (
                    "" if nav_reported else "Nav2 failed to reach slot release"
                )
                self._event(
                    "slot_release_navigation",
                    slot=slot_id,
                    route="direct_release",
                    success=bool(nav_reported),
                )

            if not self._wait_for_base_still(frame_id="odom"):
                return False, "base did not settle after slot navigation"
            pose = self._base_pose("map")
            if pose is None:
                return False, "map-frame base pose unavailable after slot navigation"
            position_error = math.hypot(
                pose[0] - slot.release.x, pose[1] - slot.release.y
            )
            yaw_error = abs(normalize_angle(pose[2] - slot.yaw))
            parking_mode = classify_slot_parking(position_error, yaw_error)

            # A failed direct Nav2 path can be retried once via staging only if
            # both matrix edges are reachable. The staging pose is never a
            # release pose and is followed by a second Nav2 goal to release.
            if (
                parking_mode == "blocked"
                and not nav_reported
                and not route_via_staging
                and position_error > SLOT_RELEASE_RECOVERY_RADIUS_M
            ):
                nav_reported, navigation_reason = self._navigate_slot_via_staging(
                    source_node, slot
                )
                if not self._wait_for_base_still(frame_id="odom"):
                    return False, "base did not settle after staging fallback"
                pose = self._base_pose("map")
                if pose is None:
                    return False, "map-frame pose unavailable after staging fallback"
                position_error = math.hypot(
                    pose[0] - slot.release.x, pose[1] - slot.release.y
                )
                yaw_error = abs(normalize_angle(pose[2] - slot.yaw))
                parking_mode = classify_slot_parking(position_error, yaw_error)

            if parking_mode == "blocked":
                return False, (
                    "slot release parking blocked outside the 0.60 m recovery range; "
                    f"{navigation_reason}"
                )

            rotation_attempted = False
            if parking_mode == "recovery":
                # The 0.25–0.60 m envelope is permitted when the live attached
                # cube, stationary base, region, and clearance checks below
                # pass. A successful Nav2 result with an already-acceptable
                # heading needs no extra rotation, but remains a recovery pose.
                if yaw_error > SLOT_RELEASE_YAW_TOLERANCE_RAD:
                    clear, reason = self._slot_rotation_clear(
                        pose, pose[2], slot.yaw, cube["id"]
                    )
                    if not clear:
                        self._publish_precision_stop()
                        return False, f"slot rotation blocked: {reason}"
                    rotated, reason = self._rotate_to_slot_yaw(
                        slot.yaw, cube["id"]
                    )
                    if not rotated:
                        return False, f"slot rotation blocked: {reason}"
                    rotation_attempted = True
                    if not self._wait_for_base_still(frame_id="odom"):
                        return False, "base did not settle after recovery rotation"
                    pose = self._base_pose("map")
                    if pose is None:
                        return False, "map-frame pose lost after recovery rotation"
                    position_error = math.hypot(
                        pose[0] - slot.release.x, pose[1] - slot.release.y
                    )
                    yaw_error = abs(normalize_angle(pose[2] - slot.yaw))
                parking_mode = classify_slot_parking(
                    position_error,
                    yaw_error,
                    recovery_attempted=True,
                )
                if parking_mode == "blocked":
                    return False, "slot release posture remains outside recovery tolerance"

            if not self._wait_for_base_still(frame_id="odom"):
                return False, "base did not settle before release gate"
            pose = self._base_pose("map")
            if pose is None:
                return False, "map-frame base pose unavailable before release gate"
            position_error = math.hypot(
                pose[0] - slot.release.x, pose[1] - slot.release.y
            )
            yaw_error = abs(normalize_angle(pose[2] - slot.yaw))
            final_mode = classify_slot_parking(
                position_error,
                yaw_error,
                recovery_attempted=(parking_mode == "recovery"),
            )
            if final_mode == "blocked":
                return False, "slot release pose left its approved parking range"

            model_positions, model_reason = self._slot_current_model_positions_map()
            attached_position = (
                model_positions.get(str(cube["id"]))
                if model_positions is not None
                else None
            )
            if pose is None or attached_position is None:
                return False, (
                    "release gate lacks base or attached-cube map pose: "
                    f"{model_reason or 'pose unavailable'}"
                )
            base_error = position_error
            cube_error = math.hypot(
                attached_position[0] - slot.drop.x,
                attached_position[1] - slot.drop.y,
            )
            nearby_tolerance = float(
                self.points.get("approach", {}).get(
                    "region_nearby_tolerance_m", 0.0
                )
            )
            pre_release_placement = check_region_placement(
                attached_position,
                zone["position"],
                float(self.points["approach"].get("cube_half_height_m", 0.015)),
                nearby_tolerance,
            )
            terminal_clear, terminal_reason = self._slot_segment_clear(
                (pose[0], pose[1]),
                (pose[0], pose[1]),
                pose[2],
                active_cube_id=cube["id"],
                carrying=True,
                terminal_cargo=True,
            )
            final_model_positions, final_model_reason = (
                self._slot_current_model_positions_map()
            )
            final_attached_position = (
                final_model_positions.get(str(cube["id"]))
                if final_model_positions is not None
                else None
            )
            if final_attached_position is None:
                terminal_clear = False
                terminal_reason = (
                    "fresh attached-cube map pose unavailable at release: "
                    f"{final_model_reason or 'entity missing'}"
                )
            else:
                attached_position = final_attached_position
                cube_error = math.hypot(
                    attached_position[0] - slot.drop.x,
                    attached_position[1] - slot.drop.y,
                )
                pre_release_placement = check_region_placement(
                    attached_position,
                    zone["position"],
                    float(self.points["approach"].get("cube_half_height_m", 0.015)),
                    nearby_tolerance,
                )
            if (
                self._attached_cube != str(cube["id"])
                or final_mode == "blocked"
                or pre_release_placement.get("success") is not True
                or not terminal_clear
            ):
                self._event(
                    "slot_terminal_release_gate",
                    slot=slot_id,
                    success=False,
                    parking_mode=final_mode,
                    base_error_m=base_error,
                    yaw_error_rad=yaw_error,
                    attached_cube_error_m=cube_error,
                    attached_cube_inside_region=(
                        pre_release_placement.get("inside_region") is True
                    ),
                    attached_cube_within_nearby=(
                        pre_release_placement.get("within_nearby") is True
                    ),
                    attached_cube_nearby_only=(
                        pre_release_placement.get("success") is True
                        and pre_release_placement.get("inside_region") is not True
                    ),
                    nearby_tolerance_m=nearby_tolerance,
                    attached_cube_region_margin_x_m=(
                        pre_release_placement.get("margin_x_m")
                    ),
                    attached_cube_region_margin_y_m=(
                        pre_release_placement.get("margin_y_m")
                    ),
                    safety_clear=terminal_clear,
                    safety_reason=terminal_reason,
                    rotation_attempted=rotation_attempted,
                )
                return False, "slot release hard safety or region gate failed"
            self._event(
                "slot_terminal_release_gate",
                slot=slot_id,
                success=True,
                parking_mode=final_mode,
                base_error_m=base_error,
                yaw_error_rad=yaw_error,
                attached_cube_error_m=cube_error,
                attached_cube_inside_region=(
                    pre_release_placement.get("inside_region") is True
                ),
                attached_cube_within_nearby=(
                    pre_release_placement.get("within_nearby") is True
                ),
                attached_cube_nearby_only=(
                    pre_release_placement.get("success") is True
                    and pre_release_placement.get("inside_region") is not True
                ),
                nearby_tolerance_m=nearby_tolerance,
                attached_cube_region_margin_x_m=pre_release_placement.get("margin_x_m"),
                attached_cube_region_margin_y_m=pre_release_placement.get("margin_y_m"),
                safety_clear=True,
                rotation_attempted=rotation_attempted,
            )
            self._transition(self.TaskState.PLACING)
            self.pub_manipulation.publish(String(data="PRE_RELEASE"))
            release_attempted = True
            release_ok = self._call_grasp_trigger(
                self._grasp_release_client, "/competition/release_target", 10.0
            )
            if not release_ok:
                self._placement_slot_states = transition_slot_state(
                    self._placement_slot_states, slot_id, SlotStatus.UNKNOWN
                )
                self._stop_exec = True
                self._event(
                    "slot_release_outcome_unknown",
                    slot=slot_id,
                    cube=cube["id"],
                    action="stop_without_reuse",
                )
                return False, "release outcome unknown; stopped without reusing this slot"
            release_confirmed = True
            self._attached_cube = None
            self._placement_slot_states = transition_slot_state(
                self._placement_slot_states, slot_id, SlotStatus.PHYSICALLY_RELEASED
            )
            self._zone_placement_counts[item.zone] = (
                self._zone_placement_counts.get(item.zone, 0) + 1
            )
            self._transition(self.TaskState.VERIFYING)
            stable_position = self._wait_for_cube_stable(str(cube["id"]))
            released_positions, released_reason = self._slot_current_model_positions_map()
            cube_position = (
                released_positions.get(str(cube["id"]))
                if released_positions is not None
                else None
            )
            cube_pose_known = stable_position is not None and cube_position is not None
            if cube_position is None:
                self._stop_exec = True
                detail = (
                    "release confirmed but released cube map pose is unavailable; "
                    f"stopped ({released_reason or 'entity missing'})"
                )
            elif stable_position is None:
                self._placed_cube_obstacle_positions[str(cube["id"])] = (
                    float(cube_position[0]), float(cube_position[1]), 0.30
                )
                self._publish_placed_cargo_obstacles()
                self._stop_exec = True
                detail = "release confirmed but stable placement was not observed; stopped"
            else:
                self._placed_cube_obstacle_positions[str(cube["id"])] = (
                    float(cube_position[0]), float(cube_position[1]), 0.30
                )
                self._publish_placed_cargo_obstacles()
                placement = check_region_placement(
                    cube_position,
                    zone["position"],
                    float(self.points["approach"].get("cube_half_height_m", 0.015)),
                    nearby_tolerance,
                )
                strict_inside = placement.get("inside_region") is True
                within_nearby = placement.get("success") is True
                self._event(
                    "place_verification",
                    cube=cube["id"],
                    method="world2_slot_region_tolerance",
                    slot=slot_id,
                    strict_inside_region=strict_inside,
                    nearby_only=within_nearby and not strict_inside,
                    **placement,
                )
                if strict_inside:
                    self._placement_slot_states = transition_slot_state(
                        self._placement_slot_states, slot_id, SlotStatus.VERIFIED
                    )
                    detail = f"released and verified inside region at slot {slot_id}"
                elif within_nearby:
                    detail = (
                        f"released within nearby tolerance at {slot_id}; "
                        "slot remains physically occupied"
                    )
                else:
                    detail = (
                        f"release confirmed outside strict region at {slot_id}; "
                        "slot remains physically occupied"
                    )
                if strict_inside or within_nearby:
                    # Keep nearby-only cargo in the final integrity audit while
                    # the slot state above continues to distinguish strict hits.
                    self._placed_cube_regions[str(cube["id"])] = dict(zone["position"])
            self._event(
                "slot_release_committed",
                slot=slot_id,
                cube=cube["id"],
                release_confirmed=release_confirmed,
                strict_inside_region=strict_inside,
                within_nearby=within_nearby,
                nearby_only=within_nearby and not strict_inside,
                slot_state=self._placement_slot_states[slot_id].value,
            )

            # A retry after a confirmed but misplaced release must leave the
            # object area safely before the next same-ordinal attempt.
            if not cube_pose_known:
                self._publish_precision_stop()
                self._planner_location = "__live_pose__"
                self._event(
                    "slot_retreat",
                    slot=slot_id,
                    cube=cube["id"],
                    success=False,
                    distance_m=0.0,
                    reason="released cube pose unknown; fail-closed without retreat",
                    next_location=self._planner_location,
                )
            elif has_more_items or not strict_inside:
                retreated, distance, retreat_reason = self._retreat_from_slot(
                    slot, cube["id"]
                )
                self._event(
                    "slot_retreat",
                    slot=slot_id,
                    cube=cube["id"],
                    success=retreated,
                    distance_m=distance,
                    reason=retreat_reason,
                    next_location=self._planner_location,
                )
                if not retreated:
                    self._stop_exec = True
                    if strict_inside:
                        detail += "; placement committed, execution stopped because retreat could not be verified"
                    else:
                        detail += "; execution stopped because retreat could not be verified"
            else:
                self._planner_location = self._slot_route_location_for_pose(
                    self._base_pose("map") or pose
                )
            profile_restored = self._set_navigation_profile("transit")
            if not profile_restored:
                self._stop_exec = True
                detail += "; execution stopped because transit profile could not be restored"
            if within_nearby:
                return True, detail
            return False, detail or "released cube failed strict region verification"
        except Exception as exc:
            self._stop_exec = True
            self._publish_precision_stop()
            if (
                release_attempted
                and not release_confirmed
                and self._placement_slot_states.get(slot_id) is SlotStatus.RESERVED
            ):
                self._placement_slot_states = transition_slot_state(
                    self._placement_slot_states, slot_id, SlotStatus.UNKNOWN
                )
            self._event(
                "slot_maneuver_exception",
                slot=slot_id,
                cube=cube["id"],
                release_confirmed=release_confirmed,
                error=str(exc),
            )
            return False, f"slot maneuver stopped after exception: {exc}"
        finally:
            if self._placement_slot_states.get(slot_id) is SlotStatus.RESERVED:
                self._placement_slot_states = transition_slot_state(
                    self._placement_slot_states,
                    slot_id,
                    SlotStatus.UNKNOWN if release_attempted else SlotStatus.AVAILABLE,
                )
