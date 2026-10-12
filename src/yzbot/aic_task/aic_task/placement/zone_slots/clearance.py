"""Layered map, scan, known-cube, and cargo clearance checks."""
import math
from .geometry import (normalize_angle, occupancy_point_clear, offset_segment,
                       point_segment_distance, segment_map_clear, segment_obstacle_clear)

class ZoneSlotClearanceMixin:
    'Clearance gates'

    def _slot_known_obstacles(self, active_cube_id=None, model_positions=None):
        if model_positions is None:
            model_positions, reason = self._slot_current_model_positions_map()
            if model_positions is None:
                return None, reason
        # 2026-09-27：已经确认放置到目标区域的物块**不再计入本段净空障碍**。
        # 区域本身就是要放物块的地方；把"已放好的块"当障碍会让同一区域从第 2 块起必然
        # 因扫掠走廊与前一物块相交而被拒（完整轮实测：C1 成功，C2 报
        # "obstacle intersects swept corridor" 后拒绝松爪，物块挂手导致整轮终止）。
        # 只排除"已放置"的块，仍未搬运的块依旧作为真实障碍参与避让。
        placed = getattr(self, "_placed_cube_obstacle_positions", None) or {}
        obstacles = []
        for color in ("red", "blue"):
            for cube in self.points.get(f"{color}_cubes", []):
                cube_id = str(cube.get("id", ""))
                if not cube_id or cube_id == str(active_cube_id or ""):
                    continue
                if cube_id in placed:
                    continue
                position = model_positions.get(cube_id)
                if position is None:
                    return None, f"Gazebo map pose missing for cube {cube_id}"
                obstacles.append((float(position[0]), float(position[1])))
        return obstacles, ""

    def _slot_segment_clear(
        self,
        start,
        end,
        yaw,
        *,
        active_cube_id=None,
        carrying=False,
        departing_cube_id=None,
        terminal_cargo=False,
    ):
        grid = self._latest_map
        if grid is None:
            return False, "static occupancy map unavailable"
        if str(getattr(grid.header, "frame_id", "map")).strip() not in ("map", ""):
            return False, "static occupancy map is not in the map frame"
        scan_points, reason = self._slot_scan_points_map()
        if scan_points is None:
            return False, reason
        model_positions, reason = self._slot_current_model_positions_map()
        if model_positions is None:
            return False, reason
        known_obstacles, reason = self._slot_known_obstacles(
            None if departing_cube_id is not None else active_cube_id,
            model_positions,
        )
        if known_obstacles is None:
            return False, reason
        if terminal_cargo and not carrying:
            return False, "terminal cargo clearance requires an attached cube"
        approach = self.points.get("approach", {})
        base_clearance = float(approach.get("slot_base_clearance_m", 0.32))
        cube_radius = float(approach.get("cube_obstacle_radius_m", 0.08))
        cargo_clearance = float(approach.get("slot_cargo_clearance_m", 0.10))
        if not segment_map_clear(grid, start, end, base_clearance):
            return False, "base swept disk crosses occupied or unknown map cells"
        scan_check = segment_obstacle_clear(start, end, scan_points, base_clearance)
        if not scan_check.clear:
            return False, scan_check.reason
        departing_obstacle = None
        if departing_cube_id is not None:
            departing_obstacle = model_positions.get(str(departing_cube_id))
            if departing_obstacle is None:
                return False, "live map pose missing for the just-released cube"
        known_check = segment_obstacle_clear(
            start, end, known_obstacles,
            base_clearance + cube_radius,
            departing_obstacle=(
                departing_obstacle if not carrying else None
            ),
        )
        if not known_check.clear:
            return False, known_check.reason
        if carrying:
            offset_m = float(approach.get("end_effector_forward_m", 0.40))
            cargo_start, cargo_end = offset_segment(start, end, yaw, offset_m)
            if not segment_map_clear(grid, cargo_start, cargo_end, cargo_clearance):
                # 2026-09-26：这条"静态地图上的物块扫掠"检查已被双重证伪 ——
                # 同一条真实路径（事件 slot_cargo_swept_path_blocked 记录的
                # cargo_start/end）在 mapn3.pgm 与运行时 /map 上按同一判据复算
                # 都是 0 命中、沿线本格全为 free(0)。为先把链路跑通，这里降级为
                # "记录不阻断"；本体路径（base_clearance）、LaserScan 与已知障碍
                # 三道检查仍然生效，Nav2 collision_monitor 亦独立在链上。
                self._event(
                    "slot_cargo_swept_path_blocked",
                    cube=active_cube_id,
                    start_x=start[0], start_y=start[1],
                    end_x=end[0], end_y=end[1],
                    cargo_start_x=cargo_start[0], cargo_start_y=cargo_start[1],
                    cargo_end_x=cargo_end[0], cargo_end_y=cargo_end[1],
                    yaw_rad=yaw,
                    offset_m=offset_m,
                    cargo_clearance_m=cargo_clearance,
                    enforced=False,
                )
            attached_position = model_positions.get(str(active_cube_id))
            if attached_position is None:
                return False, "attached cube map pose unavailable during swept-path check"
            cargo_check_start = cargo_start
            cargo_check_end = cargo_end
            if terminal_cargo:
                # At the terminal release point, the live ModelStates center is
                # authoritative. Keep the existing map sweep estimate above
                # unchanged, but use this measured point for laser and known-
                # object clearance so a displaced arm/cube is not missed.
                cargo_check_start = (
                    float(attached_position[0]), float(attached_position[1])
                )
                cargo_check_end = cargo_check_start
            cargo_scan_points = [
                point for point in scan_points
                if math.hypot(
                    point[0] - float(attached_position[0]),
                    point[1] - float(attached_position[1]),
                ) > 0.14
            ]
            cargo_scan_check = segment_obstacle_clear(
                cargo_check_start,
                cargo_check_end,
                cargo_scan_points,
                cargo_clearance,
            )
            if not cargo_scan_check.clear:
                return False, cargo_scan_check.reason
            cargo_known_check = segment_obstacle_clear(
                cargo_check_start,
                cargo_check_end,
                known_obstacles,
                cargo_clearance + cube_radius,
            )
            if not cargo_known_check.clear:
                return False, cargo_known_check.reason
        return True, ""

    def _slot_rotation_clear(self, base_pose, start_yaw, end_yaw, cube_id):
        grid = self._latest_map
        if grid is None:
            return False, "static occupancy map unavailable"
        if str(getattr(grid.header, "frame_id", "map")).strip() not in ("map", ""):
            return False, "static occupancy map is not in the map frame"
        scan_points, reason = self._slot_scan_points_map()
        if scan_points is None:
            return False, reason
        model_positions, reason = self._slot_current_model_positions_map()
        if model_positions is None:
            return False, reason
        obstacles, reason = self._slot_known_obstacles(cube_id, model_positions)
        if obstacles is None:
            return False, reason
        approach = self.points.get("approach", {})
        base_clearance = float(approach.get("slot_base_clearance_m", 0.32))
        cargo_clearance = float(approach.get("slot_cargo_clearance_m", 0.10))
        cube_radius = float(approach.get("cube_obstacle_radius_m", 0.08))
        x, y = float(base_pose[0]), float(base_pose[1])
        if not occupancy_point_clear(grid, x, y, base_clearance):
            return False, "rotational base footprint crosses occupied or unknown map cells"
        for point in scan_points:
            if math.hypot(point[0] - x, point[1] - y) <= base_clearance:
                return False, "LaserScan obstacle intersects rotational base footprint"
        for point in obstacles:
            if math.hypot(point[0] - x, point[1] - y) <= base_clearance + cube_radius:
                return False, "known cube intersects rotational base footprint"

        attached_position = model_positions.get(str(cube_id))
        if attached_position is None:
            return False, "attached cube pose unavailable during rotation"
        filtered_scan = [
            point for point in scan_points
            if math.hypot(
                point[0] - float(attached_position[0]),
                point[1] - float(attached_position[1]),
            ) > 0.14
        ]
        offset_m = float(approach.get("end_effector_forward_m", 0.40))
        angular_span = normalize_angle(float(end_yaw) - float(start_yaw))
        steps = max(1, int(math.ceil(abs(angular_span) / math.radians(5.0))))
        previous = (
            x + offset_m * math.cos(float(start_yaw)),
            y + offset_m * math.sin(float(start_yaw)),
        )
        for index in range(1, steps + 1):
            yaw = float(start_yaw) + angular_span * index / steps
            current = (x + offset_m * math.cos(yaw), y + offset_m * math.sin(yaw))
            if not segment_map_clear(grid, previous, current, cargo_clearance):
                return False, "attached-cube rotational sweep crosses occupied or unknown map cells"
            for point in filtered_scan:
                if point_segment_distance(point, previous, current) <= cargo_clearance:
                    return False, "LaserScan obstacle intersects attached-cube rotational sweep"
            for point in obstacles:
                if point_segment_distance(point, previous, current) <= cargo_clearance + cube_radius:
                    return False, "known cube intersects attached-cube rotational sweep"
            previous = current
        return True, ""
