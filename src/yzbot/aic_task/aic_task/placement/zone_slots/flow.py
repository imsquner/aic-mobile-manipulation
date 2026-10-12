"""Compose slot behavior mixins without duplicating mutable node state."""
import math
from .clearance import ZoneSlotClearanceMixin
from .geometry import normalize_angle
from .models import SlotCapacityError, SlotStatus, load_placement_slots
from .motion import ZoneSlotMotionMixin
from .release import ZoneSlotReleaseMixin
from .sensing import ZoneSlotSensingMixin
from .selection import matrix_node_name, next_available_slot

class ZoneSlotRoutingMixin:
    'Slot inventory and route entry helpers'

    def _slot_mode_enabled(self):
        return bool(load_placement_slots(self.points))

    def _slot_states_for_zone(self, zone_name):
        known_slot_ids = {
            slot.slot_id for slot in load_placement_slots(self.points, str(zone_name))
        }
        return {
            slot_id: status
            for slot_id, status in self._placement_slot_states.items()
            if str(slot_id) in known_slot_ids
        }

    def _zone_slot_counts(self):
        counts = {}
        slots = load_placement_slots(self.points)
        for zone_name in self.points.get("zones", {}):
            zone_slots = [slot for slot in slots if slot.zone == str(zone_name).upper()]
            counts[str(zone_name).upper()] = sum(
                self._placement_slot_states.get(slot.slot_id, SlotStatus.AVAILABLE)
                is not SlotStatus.AVAILABLE
                for slot in zone_slots
            )
        return counts

    def _validate_slot_state_prefixes(self):
        for zone_name in self.points.get("zones", {}):
            try:
                next_available_slot(
                    self.points,
                    str(zone_name),
                    self._slot_states_for_zone(zone_name),
                )
            except SlotCapacityError:
                continue

    def _online_first_leg_estimates(self, demand, inventory, live_pose):
        """Estimate actual-pose-to-cube Nav2 time for the first planned demand."""
        if demand is None:
            return {}
        candidate_ids = set(inventory.get(str(demand.color), ()))
        if not candidate_ids:
            return {}
        if not self._compute_path_client.wait_for_server(timeout_sec=0.2):
            self._event(
                "slot_first_leg_estimates",
                success=False,
                reason="ComputePathToPose is unavailable",
                current_x=float(live_pose[0]),
                current_y=float(live_pose[1]),
            )
            return {}
        profile = self.navigation_profiles["transit"]
        linear_speed = max(0.05, float(profile["max_vel_x"]))
        angular_speed = max(0.05, float(profile["max_vel_theta"]))
        turn_overhead = float(self._route_cost_model.transit.turn_overhead_sec)
        estimates = {}
        for color in ("red", "blue"):
            if color != str(demand.color):
                continue
            for cube in self.points.get(f"{color}_cubes", []):
                cube_id = str(cube.get("id", ""))
                if cube_id not in candidate_ids:
                    continue
                parking = cube.get("parking") or {}
                goal = {
                    "x": float(parking["x"]),
                    "y": float(parking["y"]),
                    "yaw": float(parking.get("yaw", 0.0)),
                }
                path_length = self._compute_path_length_to_candidate(goal, 0.65)
                if path_length is None or not math.isfinite(float(path_length)):
                    self._event(
                        "slot_first_leg_candidate",
                        cube=cube_id,
                        reachable=False,
                        reason="online Nav2 path estimate unavailable",
                    )
                    continue
                turn = abs(normalize_angle(goal["yaw"] - float(live_pose[2])))
                estimate = (
                    float(path_length) / linear_speed
                    + turn / angular_speed
                    + turn_overhead
                )
                estimates[cube_id] = estimate
                self._event(
                    "slot_first_leg_candidate",
                    cube=cube_id,
                    reachable=True,
                    path_length_m=float(path_length),
                    heading_change_rad=turn,
                    estimated_time_sec=estimate,
                )
        self._event(
            "slot_first_leg_estimates",
            success=bool(estimates),
            cube_count=len(estimates),
            current_x=float(live_pose[0]),
            current_y=float(live_pose[1]),
        )
        return estimates

    def _slot_route_location_for_pose(self, pose):
        """Use only a verified matrix node, otherwise require online first-leg costs."""
        candidates = []
        initial = self.points.get("robot_initial_pose", {})
        candidates.append(("start", initial))
        for color in ("red", "blue"):
            for cube in self.points.get(f"{color}_cubes", []):
                candidates.append((str(cube["id"]), cube.get("parking", {})))
        for slot in load_placement_slots(self.points):
            candidates.append((matrix_node_name(slot.zone, slot.number, "release"), {
                "x": slot.release.x, "y": slot.release.y, "yaw": slot.yaw,
            }))
            candidates.append((matrix_node_name(slot.zone, slot.number, "staging"), {
                "x": slot.staging.x, "y": slot.staging.y, "yaw": slot.yaw,
            }))
            candidates.append((matrix_node_name(slot.zone, slot.number, "exit"), {
                "x": slot.exit.endpoint.x,
                "y": slot.exit.endpoint.y,
                "yaw": slot.yaw,
            }))
        matches = []
        available_nodes = set(getattr(self._route_matrix, "node_names", ()))
        for name, target in candidates:
            if name not in available_nodes or not target or "x" not in target or "y" not in target:
                continue
            xy_error = math.hypot(float(pose[0]) - float(target["x"]),
                                  float(pose[1]) - float(target["y"]))
            yaw_error = abs(normalize_angle(
                float(pose[2]) - float(target.get("yaw", 0.0))
            ))
            if xy_error <= 0.08 and yaw_error <= math.radians(5.0):
                matches.append((xy_error + 0.05 * yaw_error, name))
        location = min(matches)[1] if matches else "__live_pose__"
        self._event(
            "slot_route_location",
            location=location,
            matched_matrix_node=bool(matches),
            pose_x=float(pose[0]),
            pose_y=float(pose[1]),
            pose_yaw=float(pose[2]),
        )
        return location

class ZoneSlotFlowMixin(ZoneSlotRoutingMixin, ZoneSlotSensingMixin, ZoneSlotClearanceMixin,
                        ZoneSlotMotionMixin, ZoneSlotReleaseMixin):
    """Compatibility composition; mutable state remains on the host node."""
    pass
