"""Single-item approach, grasp, transport, and placement orchestration."""

import math
import time

from std_msgs.msg import String

from aic_task.motion_geometry import backoff_pose, object_approach_pose
from aic_task.orchestration.constants import POSE_PRE_GRASP
from aic_task.orchestration.states import TaskState
from aic_task.task_route_planner import NoFeasiblePlan
from aic_task.zone_slots import (
    PlacementSlotError,
    load_placement_slots,
    next_available_slot,
)


def source_aware_route_key(routes, target, source):
    """Prefer a configured target/source route without changing other sources."""
    candidate = f"{target}_{source}"
    return candidate if candidate in routes else str(target)


def slot_route_via_staging_for_cube(route_plan, cube_id):
    """Use the route flag only when the selected first step is this cube."""
    steps = getattr(route_plan, "steps", ()) if route_plan is not None else ()
    if not steps:
        return False, "no matching route-plan step; direct release navigation selected"
    step = steps[0]
    planned_cube = str(getattr(step, "cube_id", ""))
    if planned_cube != str(cube_id):
        return False, (
            f"route-plan cube {planned_cube or '<missing>'} does not match "
            f"current cube {cube_id}; direct release navigation selected"
        )
    return bool(getattr(step, "route_via_staging", False)), ""


def finalized_place_outcome(placed, retreated, profile_restored):
    """Keep a physically verified release committed despite later recovery issues."""
    if not placed:
        return False, "place verification failed", ()
    warnings = []
    if not retreated:
        warnings.append("post-place retreat incomplete")
    if not profile_restored:
        warnings.append("transit navigation profile restore deferred")
    detail = "verified" if not warnings else f"verified with warning: {'; '.join(warnings)}"
    return True, detail, tuple(warnings)


class ItemFlowMixin:
    def _execute_single_item(self, item, cube, has_more_items=True):
        cube = self._runtime_cube_for_approach(cube)
        reference_parking = self._parking_pose(
            cube, cube["position"], "object_yaw"
        )
        approach_config = self.points["approach"]
        approach_target_distance = float(
            approach_config.get(
                "object_approach_target_distance_m",
                approach_config.get("object_approach_distance_m", 0.40),
            )
        )
        approach_min_distance = float(
            approach_config.get(
                "object_approach_min_distance_m", approach_target_distance
            )
        )
        approach_max_distance = float(
            approach_config.get(
                "object_approach_max_distance_m", approach_target_distance
            )
        )
        approach_yaw_tolerance = float(
            approach_config.get(
                "object_approach_yaw_tolerance_rad", math.radians(15.0)
            )
        )
        approach_x, approach_y, approach_yaw = object_approach_pose(
            (cube["position"]["x"], cube["position"]["y"]),
            (
                reference_parking["x"],
                reference_parking["y"],
                reference_parking["yaw"],
            ),
            approach_target_distance,
        )
        radial_margin = min(
            approach_target_distance - approach_min_distance,
            approach_max_distance - approach_target_distance,
        )
        object_parking = {
            "x": approach_x,
            "y": approach_y,
            "yaw": approach_yaw,
            "xy_tolerance_m": min(
                float(reference_parking["xy_tolerance_m"]),
                max(0.02, radial_margin),
            ),
            "yaw_tolerance_rad": approach_yaw_tolerance,
        }
        circle_candidates = self._object_circle_candidates(
            cube["position"], object_parking, approach_target_distance
        )
        object_parking = dict(circle_candidates[0])
        staging_backoff = float(
            cube.get(
                "object_staging_backoff_m",
                self.points["approach"].get("object_staging_backoff_m", 0.0),
            )
        )
        staging_x, staging_y, staging_yaw = backoff_pose(
            object_parking["x"],
            object_parking["y"],
            object_parking["yaw"],
            staging_backoff,
        )
        object_staging = {
            "x": staging_x,
            "y": staging_y,
            "yaw": staging_yaw,
            "xy_tolerance_m": self.navigation_profiles["transit"][
                "xy_goal_tolerance"
            ],
            "yaw_tolerance_rad": self.navigation_profiles["transit"][
                "yaw_goal_tolerance"
            ],
        }
        zone = self.points["zones"].get(item.zone)
        if zone is None:
            return False, f"unknown zone {item.zone}"
        slot = None
        if load_placement_slots(self.points, item.zone):
            try:
                slot = next_available_slot(
                    self.points,
                    item.zone,
                    self._slot_states_for_zone(item.zone),
                )
            except PlacementSlotError as exc:
                self._event(
                    "slot_selection_blocked",
                    zone=item.zone,
                    reason=str(exc),
                )
                return False, f"world2 placement slot unavailable: {exc}"
            placement_position = {
                "x": slot.drop.x,
                "y": slot.drop.y,
                "z": slot.drop.z,
            }
            zone_parking = None
        else:
            placement_position = self._next_placement_position(
                item.zone, zone["position"]
            )
            zone_parking = self._parking_pose(zone, placement_position, "zone_yaw")

        self._transition(TaskState.NAVIGATING_TO_OBJECT)
        self.pub_navigation.publish(String(data="NAVIGATING_TO_OBJECT"))
        if not self._set_navigation_profile("transit"):
            return False, "transit navigation profile failed"
        # Nav2 only needs to bring the robot into the camera/arm workspace.
        # RGB-D confirmation and IK, rather than a centimetre-level map gate,
        # decide whether the cube is safe and reachable for suction.
        self._event(
            "visual_first_object_approach",
            cube=cube["id"],
            x=object_staging["x"],
            y=object_staging["y"],
            yaw=object_staging["yaw"],
            final_parking_x=object_parking["x"],
            final_parking_y=object_parking["y"],
            final_parking_yaw=object_parking["yaw"],
            reference_parking_x=reference_parking["x"],
            reference_parking_y=reference_parking["y"],
            target_object_distance_m=approach_target_distance,
            minimum_object_distance_m=approach_min_distance,
            maximum_object_distance_m=approach_max_distance,
            object_yaw_tolerance_deg=math.degrees(approach_yaw_tolerance),
            staging_backoff_m=staging_backoff,
            xy_goal_tolerance_m=self.navigation_profiles["transit"][
                "xy_goal_tolerance"
            ],
            yaw_goal_tolerance_rad=self.navigation_profiles["transit"][
                "yaw_goal_tolerance"
            ],
        )
        object_nav_source = self._planner_location
        object_nav_started = time.monotonic()
        edge_failure_count = 0
        if self._route_matrix is not None:
            try:
                edge_failure_count = self._route_matrix.edge(
                    object_nav_source, cube["id"]
                ).failure_count
            except NoFeasiblePlan:
                edge_failure_count = 0
        object_route_key = source_aware_route_key(
            self.points.get("navigation_fallback", {}).get("routes", {}),
            cube["id"],
            object_nav_source,
        )
        if edge_failure_count > 0 and object_route_key == cube["id"]:
            self._event(
                "source_route_failure_fallback",
                source=object_nav_source,
                target=cube["id"],
                failure_count=edge_failure_count,
                route=object_route_key,
            )
        navigation_deadline = object_nav_started + self.object_navigation_total_budget
        navigation_succeeded = False
        ideal_approach_reached = False
        safe_visual_neighborhood_reached = False
        # `selected_candidate_index` is a 0-based index into `circle_candidates`
        # (it is used to index that list below). `attempt` values reported in
        # events stay 1-based for readability. Keeping both roles distinct is
        # what the previous 1-based/0-based mix got wrong.
        selected_candidate_index = None
        for candidate_index, candidate in enumerate(circle_candidates, start=0):
            attempt = candidate_index + 1
            remaining_budget = navigation_deadline - time.monotonic()
            if remaining_budget < 3.0:
                self._event(
                    "object_navigation_budget_exhausted",
                    cube=cube["id"],
                    attempt=attempt,
                    remaining_sec=max(0.0, remaining_budget),
                )
                break
            if candidate_index > 0:
                base_stationary = self._wait_for_base_still()
                costmaps_cleared = self._clear_navigation_costmaps()
                self._event(
                    "object_backup_candidate_preparation",
                    cube=cube["id"],
                    attempt=attempt,
                    base_stationary=base_stationary,
                    costmaps_cleared=costmaps_cleared,
                )
            object_parking = dict(candidate)
            staging_x, staging_y, staging_yaw = backoff_pose(
                object_parking["x"],
                object_parking["y"],
                object_parking["yaw"],
                staging_backoff,
            )
            object_staging = {
                "x": staging_x,
                "y": staging_y,
                "yaw": staging_yaw,
                "xy_tolerance_m": self.navigation_profiles["transit"][
                    "xy_goal_tolerance"
                ],
                "yaw_tolerance_rad": self.navigation_profiles["transit"][
                    "yaw_goal_tolerance"
                ],
            }
            self._event(
                "object_circle_candidate",
                cube=cube["id"],
                attempt=attempt,
                attempts=len(circle_candidates),
                x=object_staging["x"],
                y=object_staging["y"],
                yaw=object_staging["yaw"],
                circle_offset_deg=math.degrees(
                    float(candidate.get("circle_offset_rad", 0.0))
                ),
                path_cube_clearance_m=candidate.get("path_cube_clearance_m"),
                planned_path_length_m=candidate.get("planned_path_length_m"),
                planned_path_estimated_sec=candidate.get(
                    "planned_path_estimated_sec"
                ),
                remaining_budget_sec=remaining_budget,
            )
            # --- 2026-09-28 World2 物块软交接 + 闭环 docking ---
            # 仅当场景配置开启 object_docking（World2 points.yaml 带这一段）才走新路径；
            # v1/world3 没有该配置 → 完全走下面的原逻辑，行为不变。
            # 新路径的放行依据是 docking 的**真值终停门**，不再使用 `yaw=pi` 的
            # 安全视觉邻域放行（那会放过朝向完全错的停车）。
            if getattr(self, "object_docking_enabled", False):
                docking_outcome = self._world2_dock_candidate(
                    cube, object_staging, remaining_budget, navigation_deadline
                )
                self._event(
                    "object_circle_candidate_result",
                    cube=cube["id"],
                    attempt=attempt,
                    path="object_docking",
                    ideal_approach_reached=bool(docking_outcome.success),
                    safe_visual_neighborhood_reached=bool(docking_outcome.success),
                    docking_status=docking_outcome.status,
                    docking_distance_m=docking_outcome.distance_m,
                    docking_yaw_error_rad=docking_outcome.yaw_error_rad,
                    docking_attempts=docking_outcome.attempts,
                )
                if docking_outcome.success:
                    # 循环后的收尾逻辑看这两个标志，新路径成功时必须一起置真，
                    # 否则会在 docking 成功后仍被判"navigation ... failed"。
                    ideal_approach_reached = True
                    safe_visual_neighborhood_reached = True
                    selected_candidate_index = candidate_index
                    break
                continue
            navigation_succeeded = self._navigate_to(
                object_staging["x"],
                object_staging["y"],
                object_staging["yaw"],
                timeout=min(self.object_candidate_navigation_timeout, remaining_budget),
                # 2026-09-27 老板指示：删除"提前取消"补丁，让 Nav2 按 transit 容差
                # (xy=0.10) 真正收敛到候选点，再交给原地旋转拧正。
                # 原补丁（early_position_tolerance=0.12 / early_object_gate=[0.25,0.55]）
                # 会在车刚进距离带时立刻掐断导航，导致停车点漂在 0.49~0.62、yaw 差
                # 28~91°（run_20260927_045111、run_20260927_044011 实测），主门
                # ideal_approach_reached 几乎从不通过、全靠 safety 兜底。
                # 回退：恢复被注释掉的两个 kwarg 并把 AIC_EARLY_CANCEL 置 1（默认）。
                # 备份：Deepseek/gpt-6/测试/world2_release_dsh_20260927/
                #       rollback_before_early_cancel_removal/item_flow.py.bak
            )
            ideal_approach_reached = self._object_approach_within_gate(
                cube["position"],
                approach_min_distance,
                approach_max_distance,
                approach_yaw_tolerance,
            ) or self._gazebo_object_approach_gate(cube["id"])
            safe_visual_neighborhood_reached = (
                ideal_approach_reached
                or self._object_approach_within_gate(
                    cube["position"],
                    self.object_approach_safety_min_distance,
                    self.object_approach_safety_max_distance,
                    math.pi,
                )
            )
            # 细级补偿：没进理想门且"太远"时（安全门可能已过），用确定性控制器
            # 把车从 0.50~0.55 拉回 ~0.35，让停车不再漂在安全门边界。
            if not ideal_approach_reached:
                if self._fine_stop_compensation(
                    cube["position"],
                    object_staging,
                    approach_max_distance,
                    approach_yaw_tolerance,
                ):
                    ideal_approach_reached = self._object_approach_within_gate(
                        cube["position"],
                        approach_min_distance,
                        approach_max_distance,
                        approach_yaw_tolerance,
                    ) or self._gazebo_object_approach_gate(cube["id"])
                    safe_visual_neighborhood_reached = (
                        ideal_approach_reached
                        or self._object_approach_within_gate(
                            cube["position"],
                            self.object_approach_safety_min_distance,
                            self.object_approach_safety_max_distance,
                            math.pi,
                        )
                    )
            self._event(
                "object_circle_candidate_result",
                cube=cube["id"],
                attempt=attempt,
                navigation_succeeded=navigation_succeeded,
                ideal_approach_reached=ideal_approach_reached,
                safe_visual_neighborhood_reached=safe_visual_neighborhood_reached,
            )
            if safe_visual_neighborhood_reached:
                selected_candidate_index = candidate_index
                break
        if not safe_visual_neighborhood_reached:
            self._record_navigation_edge(
                object_nav_source,
                cube["id"],
                object_nav_started,
                False,
                "object_approach",
            )
            return False, "navigation to visual grasp neighborhood failed"
        if not self._wait_for_base_still():
            self._record_navigation_edge(
                object_nav_source,
                cube["id"],
                object_nav_started,
                False,
                "object_approach",
            )
            return False, "base did not become stationary before visual grasp"
        self._planner_location = str(cube["id"])
        self._event(
            "visual_first_object_handoff",
            cube=cube["id"],
            candidate_index=selected_candidate_index,
            attempt=selected_candidate_index + 1,
            ideal_approach_reached=ideal_approach_reached,
            safe_visual_neighborhood_reached=safe_visual_neighborhood_reached,
            reason="RGB-D and arm-frame geometry own final grasp alignment",
        )
        self._record_navigation_edge(
            object_nav_source,
            cube["id"],
            object_nav_started,
            True,
            "object_approach",
        )
        self._event(
            "visual_grasp_authority",
            cube=cube["id"],
            reason="rough navigation complete; RGB-D and IK decide graspability",
        )

        # Fallback pose for a failed grasp is the *next* ranked candidate, i.e.
        # a genuinely different approach. `selected_candidate_index` is the
        # 0-based index that succeeded, so the next candidate is +1. When the
        # last candidate succeeded there is no next one and this stays None,
        # which `_grasp` reports as `no_ranked_backup_available`.
        visual_recovery_pose = None
        next_candidate_index = selected_candidate_index + 1
        if next_candidate_index < len(circle_candidates):
            visual_recovery_pose = dict(circle_candidates[next_candidate_index])

        self._transition(TaskState.PRE_GRASP)
        if not self._grasp(
            cube["id"],
            item.color,
            cube["position"],
            object_parking,
            visual_recovery_pose=visual_recovery_pose,
            object_navigation_deadline=navigation_deadline,
            vision_match_tolerance=float(
                cube.get("vision_match_tolerance_m", self.vision_match_tolerance)
            ),
            direct_reorientation_on_failure=bool(
                cube.get("direct_visual_reorientation_on_failure", False)
            ),
            visual_scan_first=bool(cube.get("visual_scan_first", False)),
        ):
            retracted = self._arm(POSE_PRE_GRASP, 0.6)
            self._event(
                "grasp_failure_retract",
                cube=cube["id"],
                success=retracted,
            )
            return False, "grasp verification failed"

        if not self._set_navigation_profile("carry"):
            if slot is None:
                self._call_grasp_trigger(
                    self._grasp_release_client, "/competition/release_target", 10.0
                )
                self._attached_cube = None
            return False, "carry navigation profile failed"
        if slot is not None:
            route_plan = (
                getattr(self, "_last_route_plan", None)
                if self._route_matrix is not None
                else None
            )
            route_via_staging, route_reason = slot_route_via_staging_for_cube(
                route_plan, cube["id"]
            )
            if route_reason:
                self._event(
                    "slot_route_plan_not_matched",
                    cube=cube["id"],
                    reason=route_reason,
                    route_via_staging=False,
                )
            return self._execute_world2_slot(
                item,
                cube,
                zone,
                slot,
                has_more_items=has_more_items,
                route_via_staging=route_via_staging,
            )
        self._transition(TaskState.NAVIGATING_TO_ZONE)
        self.pub_navigation.publish(String(data="NAVIGATING_TO_ZONE"))
        zone_nav_source = self._planner_location
        zone_nav_started = time.monotonic()
        zone_arrived = self._navigate_to_parking(
            zone_parking,
            timeout=self.carry_navigation_timeout,
            route_key=self._carry_route_key(item.zone, cube["id"]),
        )
        if not zone_arrived:
            pose = self._base_pose()
            position_error = (
                math.hypot(
                    pose[0] - zone_parking["x"],
                    pose[1] - zone_parking["y"],
                )
                if pose is not None
                else math.inf
            )
            nearby_tolerance = float(
                self.points["approach"].get(
                    "zone_base_nearby_tolerance_m", 0.0
                )
            )
            zone_arrived = (
                not self._should_stop()
                and position_error <= nearby_tolerance
                and self._wait_for_base_still()
            )
            self._event(
                "zone_position_only_fallback",
                success=zone_arrived,
                position_error_m=position_error,
                tolerance_m=nearby_tolerance,
            )
            if zone_arrived:
                orientation_aligned = self._direct_precision_to_parking(
                    zone_parking,
                    timeout=8.0,
                    position_tolerance=nearby_tolerance,
                    verification_phase="zone_orientation_recovery",
                )
                self._event(
                    "zone_orientation_recovery",
                    success=orientation_aligned,
                    reason="position was acceptable but placement still requires a fixed heading",
                )
                zone_arrived = orientation_aligned
        self._record_navigation_edge(
            zone_nav_source,
            f"zone_{item.zone}",
            zone_nav_started,
            zone_arrived,
            "carry_to_zone",
        )
        if not zone_arrived:
            self._call_grasp_trigger(
                self._grasp_release_client, "/competition/release_target", 10.0
            )
            self._attached_cube = None
            self._set_navigation_profile("transit")
            return False, "navigation or parking at zone failed"
        self._planner_location = f"zone_{item.zone}"

        self._transition(TaskState.PLACING)
        placed = self._place(cube["id"], placement_position, zone["position"])
        if placed:
            self._zone_placement_counts[item.zone] = (
                self._zone_placement_counts.get(item.zone, 0) + 1
            )
        if placed and has_more_items:
            retreated = self._retreat_after_placement(cube["id"], item.zone)
        else:
            retreated = placed
            if placed:
                self._event(
                    "post_place_retreat",
                    cube=cube["id"],
                    zone=item.zone,
                    success=True,
                    skipped=True,
                    reason="final item has no following navigation",
                )
        profile_restored = self._set_navigation_profile("transit")
        success, detail, recovery_warnings = finalized_place_outcome(
            placed, retreated, profile_restored
        )
        if not success:
            return success, detail
        if recovery_warnings:
            self._event(
                "post_place_recovery_degraded",
                cube=cube["id"],
                zone=item.zone,
                issues=list(recovery_warnings),
                reason="physical release is already verified and remains committed",
            )

        self.pub_navigation.publish(String(data="IDLE"))
        self.pub_manipulation.publish(String(data="IDLE"))
        return True, detail
