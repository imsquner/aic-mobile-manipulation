"""Task demand allocation and object approach candidate planning."""

import math
import time
from dataclasses import replace
from pathlib import Path

import yaml

from nav2_msgs.action import ComputePathToPose

from aic_task.motion_geometry import (
    ordered_object_approach_candidates,
    segment_point_clearance,
)
from aic_task.task_route_planner import NoFeasiblePlan, plan_ordered_route
from aic_task.zone_slots import PlacementSlotError


class AllocationMixin:
    def _available_inventory(self):
        unavailable = self._used_cube_ids | self._failed_cube_ids
        return {
            color: [
                str(cube["id"])
                for cube in self.points.get(f"{color}_cubes", [])
                if str(cube["id"]) not in unavailable
            ]
            for color in ("red", "blue")
        }

    def _allocate_cube(
        self,
        color,
        *,
        item=None,
        remaining_items=None,
        ordinal=1,
    ):
        if self._slot_mode_enabled():
            try:
                self._validate_slot_state_prefixes()
            except PlacementSlotError as exc:
                self._event(
                    "slot_state_blocked",
                    reason=str(exc),
                    slot_states={
                        key: value.value
                        for key, value in self._placement_slot_states.items()
                    },
                )
                self._stop_exec = True
                return None
        inventory = self._available_inventory()
        candidates = inventory.get(color, [])
        if not candidates:
            return None

        # 2026-09-27 全组合测试：任务显式指定 cube_id 时优先选用，用于逐条边（物块→区域）回归。
        # 注意 candidates 是 cube id 的 str 列表（见 _available_inventory），返回需转成 cube dict。
        if item is not None and getattr(item, "cube_id", None):
            requested_id = str(item.cube_id)
            if requested_id in candidates:
                for candidate_cube in self.points.get(f"{color}_cubes", []):
                    if str(candidate_cube["id"]) == requested_id:
                        self._event(
                            "cube_forced_by_task",
                            color=color,
                            cube=requested_id,
                            reason="explicit cube_id in task",
                        )
                        return candidate_cube

        if self.mode == "development" and self.attempted_count == 0:
            configured = self.points.get(f"{color}_cubes", [])
            target_index = max(
                0,
                int(self.get_parameter("development_cube_start_index").value),
            )
            if target_index < len(configured):
                target = configured[target_index]
                target_id = str(target["id"])
                if target_id in candidates:
                    self._event(
                        "development_cube_forced",
                        color=color,
                        cube=target_id,
                        index=target_index,
                        reason="single-cube physical acceptance target",
                    )
                    return target

        if self._route_matrix is not None and item is not None and remaining_items:
            demands = self._remaining_demands(remaining_items, ordinal)
            elapsed = (
                time.monotonic() - self.start_time
                if self.start_time is not None
                else 0.0
            )
            penalties = {}
            for configured_cube in (
                list(self.points.get("red_cubes", []))
                + list(self.points.get("blue_cubes", []))
            ):
                cube_id = str(configured_cube["id"])
                configured_risk = max(
                    0.0,
                    float(configured_cube.get("route_risk_penalty_sec", 0.0)),
                )
                failure_risk = (
                    self._cube_failure_counts.get(cube_id, 0)
                    * self._planner_cube_failure_penalty
                )
                if configured_risk + failure_risk > 0.0:
                    penalties[cube_id] = configured_risk + failure_risk
            route_location = self._planner_location
            route_slot_args = {}
            if self._slot_mode_enabled():
                live_pose = self._base_pose("map")
                if live_pose is None:
                    self._event(
                        "route_plan_unavailable",
                        ordinal=ordinal,
                        color=color,
                        reason="slot-aware planning requires the actual map-frame pose",
                    )
                    self._stop_exec = True
                    return None
                route_location = self._slot_route_location_for_pose(live_pose)
                self._planner_location = route_location
                route_slot_args = {
                    "zone_slot_counts": self._zone_slot_counts(),
                }
                if route_location == "__live_pose__":
                    first_demand = demands[0] if demands else None
                    initial_transit = (
                        self._online_first_leg_estimates(
                            first_demand, inventory, live_pose
                        )
                        if first_demand is not None
                        else {}
                    )
                    if not initial_transit:
                        self._event(
                            "route_plan_unavailable",
                            ordinal=ordinal,
                            color=color,
                            reason=(
                                "no online Nav2 first-leg estimate from the "
                                "actual pose"
                            ),
                        )
                        self._stop_exec = True
                        return None
                    route_slot_args["initial_transit_times_sec"] = initial_transit
            degraded_reason = ""
            try:
                plan = plan_ordered_route(
                    self._route_matrix,
                    current_location=route_location,
                    remaining_demands=demands,
                    inventory_by_color=inventory,
                    **route_slot_args,
                    failed_cubes=self._failed_cube_ids,
                    elapsed_time_sec=elapsed,
                    failure_penalties_sec=penalties,
                    cost_model=replace(
                        self._route_cost_model,
                        runtime_scale=self._route_runtime_scale,
                    ),
                )
            except NoFeasiblePlan as full_error:
                degraded_reason = str(full_error)
                try:
                    plan = plan_ordered_route(
                        self._route_matrix,
                        current_location=route_location,
                        remaining_demands=demands[:1],
                        inventory_by_color=inventory,
                        **route_slot_args,
                        failed_cubes=self._failed_cube_ids,
                        elapsed_time_sec=elapsed,
                        failure_penalties_sec=penalties,
                        cost_model=replace(
                            self._route_cost_model,
                            runtime_scale=self._route_runtime_scale,
                        ),
                    )
                except NoFeasiblePlan as current_error:
                    self._event(
                        "route_plan_unavailable",
                        ordinal=ordinal,
                        color=color,
                        full_error=degraded_reason,
                        current_error=str(current_error),
                    )
                    return None

            self._last_route_plan = plan
            first = plan.steps[0]
            self._event(
                "route_plan_updated",
                ordinal=ordinal,
                location=route_location,
                selected_cube=first.cube_id,
                cube_order=[step.cube_id for step in plan.steps],
                estimated_remaining_sec=plan.estimated_remaining_sec,
                estimated_total_sec=plan.estimated_total_sec,
                within_deadline=plan.within_deadline,
                source_hash=plan.source_hash,
                degraded=bool(degraded_reason),
                degraded_reason=degraded_reason,
            )
            for cube in self.points.get(f"{color}_cubes", []):
                if str(cube["id"]) == first.cube_id:
                    return cube
            self._event(
                "route_plan_unavailable",
                ordinal=ordinal,
                color=color,
                current_error=f"planned cube {first.cube_id} missing from points",
            )
            return None

        for cube in self.points.get(f"{color}_cubes", []):
            if str(cube["id"]) in candidates:
                self._event(
                    "route_plan_fallback",
                    ordinal=ordinal,
                    color=color,
                    selected_cube=cube["id"],
                    reason="path matrix is not configured",
                )
                return cube
        return None

    def _refresh_planner_location(self):
        if self._slot_mode_enabled():
            pose = self._base_pose()
            if pose is None:
                self._planner_location = "__live_pose__"
                return self._planner_location
            self._planner_location = self._slot_route_location_for_pose(pose)
            return self._planner_location
        pose = self._base_pose()
        if pose is None:
            return self._planner_location
        candidates = []
        initial = self.points.get("robot_initial_pose", {})
        candidates.append(
            (
                "start",
                float(initial.get("x", 0.0)),
                float(initial.get("y", 0.0)),
            )
        )
        for color in ("red", "blue"):
            for cube in self.points.get(f"{color}_cubes", []):
                parking = cube["parking"]
                candidates.append(
                    (str(cube["id"]), float(parking["x"]), float(parking["y"]))
                )
        for zone_name, zone in self.points.get("zones", {}).items():
            parking = zone["parking"]
            candidates.append(
                (
                    f"zone_{str(zone_name).upper()}",
                    float(parking["x"]),
                    float(parking["y"]),
                )
            )
        nearest = min(
            candidates,
            key=lambda candidate: math.hypot(
                pose[0] - candidate[1], pose[1] - candidate[2]
            ),
        )
        previous = self._planner_location
        self._planner_location = nearest[0]
        self._event(
            "route_location_refreshed",
            previous=previous,
            current=self._planner_location,
            distance_m=math.hypot(pose[0] - nearest[1], pose[1] - nearest[2]),
        )
        return self._planner_location

    def _approach_pose(self, position, yaw_key):
        approach = self.points["approach"]
        yaw = float(
            position.get(yaw_key, approach.get(yaw_key, approach.get("yaw", 0.0)))
        )
        distance_key = (
            "object_approach_distance_m"
            if yaw_key == "object_yaw"
            else "end_effector_forward_m"
        )
        distance = float(
            position.get(
                "approach_distance_m",
                approach.get(distance_key, approach["end_effector_forward_m"]),
            )
        )
        return (
            float(position["x"]) - distance * math.cos(yaw),
            float(position["y"]) - distance * math.sin(yaw),
            yaw,
        )

    def _map_point_has_clearance(self, x, y, clearance_m):
        """Check static-map occupancy around a candidate; missing maps degrade safely."""
        grid = self._latest_map
        if grid is None:
            return True
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return False
        origin = grid.info.origin
        orientation = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        dx = float(x) - float(origin.position.x)
        dy = float(y) - float(origin.position.y)
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        map_x = cosine * dx + sine * dy
        map_y = -sine * dx + cosine * dy
        center_col = int(math.floor(map_x / resolution))
        center_row = int(math.floor(map_y / resolution))
        radius_cells = int(math.ceil(float(clearance_m) / resolution))
        for row in range(center_row - radius_cells, center_row + radius_cells + 1):
            for col in range(center_col - radius_cells, center_col + radius_cells + 1):
                if math.hypot(col - center_col, row - center_row) * resolution > float(
                    clearance_m
                ):
                    continue
                if row < 0 or row >= height or col < 0 or col >= width:
                    return False
                occupancy = int(grid.data[row * width + col])
                if occupancy < 0 or occupancy >= 50:
                    return False
        return True

    def _compute_path_length_to_candidate(self, candidate, timeout):
        if timeout <= 0.0:
            return None
        deadline = time.monotonic() + float(timeout)
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(candidate["x"])
        goal.goal.pose.position.y = float(candidate["y"])
        goal.goal.pose.orientation.z = math.sin(float(candidate["yaw"]) / 2.0)
        goal.goal.pose.orientation.w = math.cos(float(candidate["yaw"]) / 2.0)
        goal.use_start = False

        send_future = self._compute_path_client.send_goal_async(goal)
        if not self._wait_future(send_future, max(0.0, deadline - time.monotonic())):
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, max(0.0, deadline - time.monotonic())):
            goal_handle.cancel_goal_async()
            return None
        response = result_future.result()
        path = response.result.path if response is not None else None
        poses = path.poses if path is not None else []
        if len(poses) < 2:
            return None
        return sum(
            math.hypot(
                float(current.pose.position.x) - float(previous.pose.position.x),
                float(current.pose.position.y) - float(previous.pose.position.y),
            )
            for previous, current in zip(poses, poses[1:])
        )

    def _rank_object_circle_candidates(self, candidates):
        shortlist = list(candidates[: self.object_candidate_shortlist_count])
        if len(shortlist) <= 1 or self.object_path_prefilter_budget <= 0.0:
            return shortlist[: self.object_circle_max_attempts]
        if not self._compute_path_client.wait_for_server(timeout_sec=0.15):
            self._event(
                "object_candidate_path_prefilter",
                available=False,
                candidate_count=len(shortlist),
                selected_count=min(len(shortlist), self.object_circle_max_attempts),
            )
            return shortlist[: self.object_circle_max_attempts]

        deadline = time.monotonic() + self.object_path_prefilter_budget
        scored = []
        unscored = []
        current_pose = self._base_pose("map")
        transit_linear_speed = max(
            0.05, float(self.navigation_profiles["transit"]["max_vel_x"])
        )
        transit_angular_speed = max(
            0.05, float(self.navigation_profiles["transit"]["max_vel_theta"])
        )
        for candidate_index, candidate in enumerate(shortlist, start=1):
            remaining = deadline - time.monotonic()
            if remaining <= 0.05:
                unscored.extend(shortlist[candidate_index - 1:])
                break
            path_length = self._compute_path_length_to_candidate(
                candidate, min(0.45, remaining)
            )
            if path_length is None:
                unscored.append(candidate)
                continue
            ranked = dict(candidate)
            ranked["planned_path_length_m"] = path_length
            final_turn = (
                abs(
                    self._normalize_angle(
                        float(candidate["yaw"]) - float(current_pose[2])
                    )
                )
                if current_pose is not None
                else 0.0
            )
            ranked["planned_final_turn_rad"] = final_turn
            ranked["planned_path_estimated_sec"] = (
                path_length / transit_linear_speed
                + final_turn / transit_angular_speed
            )
            scored.append(ranked)

        scored.sort(
            key=lambda value: (
                float(value["planned_path_estimated_sec"]),
                float(value["planned_path_length_m"]),
            )
        )
        selected = (scored + unscored)[: self.object_circle_max_attempts]
        self._event(
            "object_candidate_path_prefilter",
            available=True,
            candidate_count=len(shortlist),
            planned_count=len(scored),
            selected_count=len(selected),
            selected_path_lengths_m=[
                value.get("planned_path_length_m") for value in selected
            ],
            selected_estimated_sec=[
                value.get("planned_path_estimated_sec") for value in selected
            ],
            elapsed_sec=self.object_path_prefilter_budget
            - max(0.0, deadline - time.monotonic()),
        )
        return selected

    # 2026-10-16 老板要求：把移动障碍物的行驶路径当"墙"，不允许把停车点选在路径上。
    # 禁止带半宽 = 障碍安全半径(obstacles.yaml) + 机器人半径 + 余量。
    MOVING_OBSTACLE_ROBOT_RADIUS_M = 0.30
    MOVING_OBSTACLE_MARGIN_M = 0.05

    def _moving_obstacle_corridors(self):
        """`[(x_min, x_max, y, safety_radius_m), ...]` for the scene's `sine_x` obstacles.

        Read from `obstacles.yaml` next to the active scene `points_file`. Any failure is reported
        and then degrades to "no corridor" (fail-open) so a missing/renamed config can never block
        picking; the event makes the degradation visible instead of silent.
        """
        cached = getattr(self, "_moving_corridors", None)
        if cached is not None:
            return cached
        corridors = []
        try:
            points_file = getattr(self, "points_file", None)
            if not points_file:
                raise RuntimeError("points_file unavailable")
            path = Path(points_file).parent / "obstacles.yaml"
            data = yaml.safe_load(Path(path).read_text()) or {}
            for obstacle in data.get("dynamic_obstacles", []) or []:
                motion = obstacle.get("motion") or {}
                if motion.get("type") != "sine_x":
                    continue
                center_x = float(motion["center_x"])
                amplitude = abs(float(motion["amplitude_m"]))
                corridors.append((
                    center_x - amplitude,
                    center_x + amplitude,
                    float(motion["center_y"]),
                    float(obstacle.get("safety_radius_m", 0.0)),
                ))
        except Exception as exc:  # noqa: BLE001 - never block picking on a config problem
            self._event("moving_obstacle_corridor_unavailable", reason=str(exc))
            corridors = []
        self._moving_corridors = corridors
        return corridors

    def _moving_obstacle_clearance(self, x, y):
        """False when (x, y) is inside a moving obstacle's swept corridor.

        A `sine_x` obstacle sweeps one straight segment; parking within
        `safety_radius + robot_radius + margin` of that segment means the box can reach the robot.
        Distance is measured to the segment (not an infinite band) so only the range the box
        actually travels is refused.
        """
        for x_min, x_max, corridor_y, safety in self._moving_obstacle_corridors():
            if x_min <= x <= x_max:
                distance = abs(y - corridor_y)
            else:
                nearest = x_min if x < x_min else x_max
                distance = math.hypot(x - nearest, y - corridor_y)
            if distance < (safety + self.MOVING_OBSTACLE_ROBOT_RADIUS_M
                           + self.MOVING_OBSTACLE_MARGIN_M):
                return False
        return True

    def _object_circle_candidates(self, object_position, fallback_parking, radius_m):
        """Return two path-ranked circle goals without approaching through cargo."""
        current_pose = self._base_pose("map")
        if current_pose is None:
            return [dict(fallback_parking)]
        object_xy = (float(object_position["x"]), float(object_position["y"]))
        generated = ordered_object_approach_candidates(
            object_xy,
            current_pose,
            radius=radius_m,
            candidate_count=self.object_circle_candidate_count,
        )
        accepted = []
        corridor_rejected = 0
        for candidate in generated:
            path_clearance = segment_point_clearance(
                current_pose, (candidate["x"], candidate["y"]), object_xy
            )
            if path_clearance < self.object_path_cube_clearance:
                continue
            if not self._map_point_has_clearance(
                candidate["x"], candidate["y"], self.object_circle_map_clearance
            ):
                continue
            if not self._moving_obstacle_clearance(candidate["x"], candidate["y"]):
                corridor_rejected += 1
                continue
            parking = dict(candidate)
            parking["xy_tolerance_m"] = 0.10
            parking["yaw_tolerance_rad"] = float(
                fallback_parking["yaw_tolerance_rad"]
            )
            parking["path_cube_clearance_m"] = path_clearance
            accepted.append(parking)
        if corridor_rejected:
            # 绝不静默：把"被移动障碍走廊排除掉的候选数"写进事件，便于复盘与验收。
            self._event(
                "object_circle_candidates_forbidden",
                reason="moving obstacle corridor excluded candidate parking poses",
                object_x=float(object_position["x"]),
                object_y=float(object_position["y"]),
                candidate_count=len(generated),
                corridor_rejected=corridor_rejected,
                accepted_count=len(accepted),
            )
        if accepted:
            return self._rank_object_circle_candidates(accepted)
        self._event(
            "object_circle_candidates_degraded",
            reason="no map-clear circle candidate; using configured parking",
            candidate_count=len(generated),
            corridor_rejected=corridor_rejected,
        )
        return [dict(fallback_parking)]
