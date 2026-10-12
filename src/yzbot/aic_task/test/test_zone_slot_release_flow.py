import math
import sys
import types
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    from std_msgs.msg import String
except ModuleNotFoundError:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self, data=""):
            self.data = data

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

try:
    from geometry_msgs.msg import Twist
except ModuleNotFoundError:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

if "rclpy" not in sys.modules:
    sys.modules["rclpy"] = types.ModuleType("rclpy")

if "gazebo_msgs.srv" not in sys.modules:
    gazebo_msgs = types.ModuleType("gazebo_msgs")
    gazebo_msgs_srv = types.ModuleType("gazebo_msgs.srv")
    gazebo_msgs_srv.GetEntityState = type("GetEntityState", (), {})
    gazebo_msgs.srv = gazebo_msgs_srv
    sys.modules["gazebo_msgs"] = gazebo_msgs
    sys.modules["gazebo_msgs.srv"] = gazebo_msgs_srv

if "linkattacher_msgs.srv" not in sys.modules:
    linkattacher_msgs = types.ModuleType("linkattacher_msgs")
    linkattacher_srv = types.ModuleType("linkattacher_msgs.srv")
    linkattacher_srv.AttachLink = type("AttachLink", (), {})
    linkattacher_srv.DetachLink = type("DetachLink", (), {})
    linkattacher_msgs.srv = linkattacher_srv
    sys.modules["linkattacher_msgs"] = linkattacher_msgs
    sys.modules["linkattacher_msgs.srv"] = linkattacher_srv

if "sensor_msgs.msg" not in sys.modules:
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.PointCloud2 = type("PointCloud2", (), {})
    sensor_msgs_msg.PointField = type("PointField", (), {})
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

from aic_task.placement.zone_slots import geometry as slot_geometry  # noqa: E402
from aic_task.placement.zone_slots.geometry import validate_placement_slots  # noqa: E402
from aic_task.placement.zone_slots.release import ZoneSlotReleaseMixin  # noqa: E402
from aic_task.placement.zone_slots.selection import matrix_node_name  # noqa: E402
from aic_task.placement.zone_slots.flow import ZoneSlotRoutingMixin  # noqa: E402
from aic_task.placement.zone_slots.clearance import ZoneSlotClearanceMixin  # noqa: E402
from aic_task.placement.zone_slots.motion import ZoneSlotMotionMixin  # noqa: E402
from aic_task.placement.zone_slots.models import SlotStatus  # noqa: E402
from aic_task.placement.legacy_flow import PlacementLegacyFlowMixin  # noqa: E402
from aic_task.orchestration import item_flow as item_flow_module  # noqa: E402


POINTS_PATH = PACKAGE_ROOT / "config" / "scenes" / "world2" / "points.yaml"


class MatrixHarness:
    def __init__(self, edges):
        self.node_names = {
            "start",
            "red_cube_1",
            "zone_A_slot_3_staging",
            "zone_A_slot_3_release",
        }
        self.edges = set(edges)

    def edge(self, source, target):
        if (source, target) not in self.edges:
            raise LookupError(f"unreachable edge {source}->{target}")
        return object()


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SlotReleaseHarness(ZoneSlotReleaseMixin, PlacementLegacyFlowMixin):
    TaskState = SimpleNamespace(
        NAVIGATING_TO_ZONE="NAVIGATING_TO_ZONE",
        PLACING="PLACING",
        VERIFYING="VERIFYING",
    )

    def __init__(self, slot, *, nav_results=(True,), pose=None, cube_position=None,
                 matrix_edges=(), preserve_pose_on_navigation_success=False,
                 release_succeeds=True, stable_position=None):
        self.points = yaml.safe_load(POINTS_PATH.read_text(encoding="utf-8"))
        self.slot = slot
        self.base_pose = pose or (slot.release.x, slot.release.y, slot.yaw)
        self.cube_position = cube_position or (slot.drop.x, slot.drop.y, slot.drop.z)
        self.stable_position = stable_position or self.cube_position
        self.nav_results = list(nav_results)
        self.preserve_pose_on_navigation_success = bool(
            preserve_pose_on_navigation_success
        )
        self.nav_calls = []
        self.rotation_calls = []
        self.segment_clear_calls = []
        self.release_calls = 0
        self.release_succeeds = bool(release_succeeds)
        self._attached_cube = "red_cube_1"
        self._placement_slot_states = {}
        self._zone_placement_counts = {}
        self._placed_cube_regions = {}
        self._placed_cube_obstacle_positions = {}
        self.entity_positions = {"red_cube_1": self.stable_position}
        self._route_matrix = MatrixHarness(matrix_edges)
        self._planner_location = "red_cube_1"
        self._latest_map = object()
        self._stop_exec = False
        self._grasp_release_client = object()
        self.carry_navigation_timeout = 30.0
        self.navigation_profiles = {"transit": {}}
        self.pub_navigation = Publisher()
        self.pub_manipulation = Publisher()
        self.events = []

    def _event(self, name, **fields):
        self.events.append((name, fields))

    def _record_navigation_edge(self, source, target, _started, success, phase):
        self.events.append(("navigation_edge", {
            "source": source,
            "target": target,
            "success": bool(success),
            "phase": phase,
        }))

    def _transition(self, _state):
        pass

    def _should_stop(self):
        return self._stop_exec

    def _slot_scan_points_map(self):
        return [], ""

    def _navigate_to(self, x, y, yaw, **kwargs):
        self.nav_calls.append((float(x), float(y), float(yaw), kwargs))
        result = self.nav_results.pop(0) if self.nav_results else True
        if result and not self.preserve_pose_on_navigation_success:
            self.base_pose = (float(x), float(y), float(yaw))
        return result

    def _base_pose(self, _frame="map"):
        return self.base_pose

    def _wait_for_base_still(self, **_kwargs):
        return True

    def _slot_current_model_positions_map(self):
        position = self.stable_position if self.release_calls else self.cube_position
        return {"red_cube_1": position}, ""

    def _slot_mode_enabled(self):
        return True

    def _entity_position(self, cube_id):
        return self.entity_positions.get(str(cube_id))

    def _slot_segment_clear(self, *_args, **_kwargs):
        self.segment_clear_calls.append((_args, _kwargs))
        return True, ""

    def _slot_rotation_clear(self, *_args, **_kwargs):
        return True, ""

    def _rotate_to_slot_yaw(self, target_yaw, _cube_id):
        self.rotation_calls.append(float(target_yaw))
        self.base_pose = (self.base_pose[0], self.base_pose[1], float(target_yaw))
        return True, ""

    def _drive_to_slot_release(self, *_args, **_kwargs):
        raise AssertionError("custom straight-line release drive must not be called")

    def _call_grasp_trigger(self, *_args, **_kwargs):
        self.release_calls += 1
        return self.release_succeeds

    def _wait_for_cube_stable(self, _cube_id):
        return self.stable_position

    def _publish_placed_cargo_obstacles(self):
        pass

    def _publish_precision_stop(self):
        pass

    def _slot_route_location_for_pose(self, _pose):
        return "__live_pose__"

    def _set_navigation_profile(self, _profile):
        return True

    def _retreat_from_slot(self, *_args, **_kwargs):
        return True, 0.5, ""


class SlotRoutingHarness(ZoneSlotRoutingMixin):
    def __init__(self, points, matrix_nodes):
        self.points = points
        self._route_matrix = SimpleNamespace(node_names=set(matrix_nodes))
        self.events = []

    def _event(self, name, **fields):
        self.events.append((name, fields))


class SimClock:
    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        self.nanoseconds += 100_000_000
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class RetreatPublisher(Publisher):
    def __init__(self, host):
        super().__init__()
        self.host = host

    def publish(self, message):
        super().publish(message)
        velocity = float(message.linear.x)
        if velocity < 0.0:
            x, y, yaw = self.host.map_pose
            odom_x, odom_y, odom_yaw = self.host.odom_pose
            dt = 0.1
            dx = velocity * dt * math.cos(yaw)
            dy = velocity * dt * math.sin(yaw)
            self.host.map_pose = (x + dx, y + dy, yaw)
            self.host.odom_pose = (odom_x + dx, odom_y + dy, odom_yaw)


class SlotRetreatHarness(ZoneSlotMotionMixin):
    def __init__(self, slot, *, block_after_first=True, safe_prefix_m=0.50):
        self.slot = slot
        self.block_after_first = bool(block_after_first)
        self.safe_prefix_m = float(safe_prefix_m)
        self.points = yaml.safe_load(POINTS_PATH.read_text(encoding="utf-8"))
        self.map_pose = (slot.release.x, slot.release.y, slot.yaw)
        self.odom_pose = (10.0, -4.0, slot.yaw)
        self._placement_slot_states = {
            slot.slot_id: SlotStatus.PHYSICALLY_RELEASED
        }
        self._placed_cube_obstacle_positions = {
            "red_cube_1": (slot.drop.x, slot.drop.y, 0.30)
        }
        self._latest_map = SimpleNamespace(
            header=SimpleNamespace(frame_id="map")
        )
        self.pub_precision_cmd = RetreatPublisher(self)
        self.events = []
        self._planner_location = "zone_A_slot_3_release"
        self._clock = SimClock()

    def get_clock(self):
        return self._clock

    def _base_pose(self, frame_id="map"):
        return self.odom_pose if frame_id == "odom" else self.map_pose

    def _slot_scan_points_map(self):
        return [], ""

    def _slot_segment_clear(self, start, end, _yaw, **_kwargs):
        has_moved = any(
            command.linear.x < 0.0 for command in self.pub_precision_cmd.messages
        )
        if not has_moved:
            reverse_distance = math.hypot(end[0] - start[0], end[1] - start[1])
            return (
                reverse_distance <= self.safe_prefix_m,
                "static or scan obstacle beyond checked prefix",
            )
        if self.block_after_first:
            return False, "LaserScan obstacle blocks the next retreat segment"
        return True, ""

    def _should_stop(self):
        return False

    def _wait_for_base_still(self, **_kwargs):
        return True

    def _slot_route_location_for_pose(self, pose):
        self.last_route_pose = tuple(pose)
        endpoint = self.slot.exit.endpoint
        if (
            math.hypot(float(pose[0]) - endpoint.x, float(pose[1]) - endpoint.y)
            <= 0.08
            and abs(slot_geometry.normalize_angle(pose[2] - self.slot.yaw))
            <= math.radians(5.0)
        ):
            return matrix_node_name(self.slot.zone, self.slot.number, "exit")
        return "__live_pose__"

    def _event(self, name, **fields):
        self.events.append((name, fields))

    def _publish_precision_stop(self):
        self.pub_precision_cmd.publish(Twist())


class SlotRotationHarness(ZoneSlotMotionMixin):
    def __init__(self, pose):
        self.pose = tuple(pose)
        self._clock = SimClock()
        self.pub_precision_cmd = Publisher()

    def get_clock(self):
        return self._clock

    def _base_pose(self, _frame="map"):
        return self.pose

    def _should_stop(self):
        return False

    def _slot_rotation_clear(self, *_args, **_kwargs):
        return True, ""

    def _wait_for_base_still(self, **_kwargs):
        return True

    def _publish_precision_stop(self):
        self.pub_precision_cmd.publish(Twist())


class SlotClearanceHarness(ZoneSlotClearanceMixin):
    def __init__(self, model_positions, scan_points=()):
        self.points = {
            "approach": {
                "slot_base_clearance_m": 0.32,
                "cube_obstacle_radius_m": 0.08,
                "slot_cargo_clearance_m": 0.10,
                "end_effector_forward_m": 0.40,
            },
            "red_cubes": [{"id": "red_cube_1"}],
            "blue_cubes": [{"id": "blue_cube_1"}]
            if "blue_cube_1" in model_positions else [],
        }
        self._latest_map = SimpleNamespace(
            header=SimpleNamespace(frame_id="map")
        )
        self.model_positions = dict(model_positions)
        self.scan_points = list(scan_points)
        self.events = []

    def _slot_scan_points_map(self):
        return self.scan_points, ""

    def _slot_current_model_positions_map(self):
        return self.model_positions, ""

    def _event(self, name, **fields):
        self.events.append((name, fields))


class World2SlotReleaseFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points = yaml.safe_load(POINTS_PATH.read_text(encoding="utf-8"))
        cls.slot = next(
            slot for slot in __import__(
                "aic_task.placement.zone_slots.models",
                fromlist=["load_placement_slots"],
            ).load_placement_slots(cls.points, "A")
            if slot.number == 3
        )

    def _run(self, harness, *, route_via_staging=False):
        return harness._execute_world2_slot(
            SimpleNamespace(zone="A"),
            {"id": "red_cube_1"},
            self.points["zones"]["A"],
            self.slot,
            has_more_items=False,
            route_via_staging=route_via_staging,
        )

    def test_matrix_node_name_accepts_release(self):
        self.assertEqual(
            matrix_node_name("A", 3, "release"),
            "zone_A_slot_3_release",
        )

    def test_parking_policy_has_normal_recovery_and_hard_outer_limit(self):
        self.assertEqual(slot_geometry.classify_slot_parking(0.24, math.radians(19.0)), "normal")
        self.assertEqual(slot_geometry.classify_slot_parking(0.40, math.radians(10.0)), "recovery")
        self.assertEqual(slot_geometry.classify_slot_parking(0.60, math.radians(20.0)), "recovery")
        self.assertEqual(slot_geometry.classify_slot_parking(0.601, 0.0), "blocked")

    def test_nav2_targets_release_without_custom_straight_drive(self):
        host = SlotReleaseHarness(self.slot)

        success, _ = self._run(host)

        self.assertTrue(success)
        self.assertEqual(
            [(call[0], call[1]) for call in host.nav_calls],
            [(self.slot.release.x, self.slot.release.y)],
        )
        self.assertEqual(host.release_calls, 1)

    def test_plan_route_flag_only_applies_to_the_matching_cube(self):
        plan = SimpleNamespace(
            steps=[SimpleNamespace(cube_id="red_cube_1", route_via_staging=True)]
        )

        self.assertEqual(
            item_flow_module.slot_route_via_staging_for_cube(plan, "red_cube_1"),
            (True, ""),
        )
        route, reason = item_flow_module.slot_route_via_staging_for_cube(plan, "red_cube_2")
        self.assertFalse(route)
        self.assertIn("does not match", reason)

    def test_staging_fallback_requires_both_matrix_edges_and_still_nav2s_to_release(self):
        matrix_edges = {
            ("red_cube_1", "zone_A_slot_3_staging"),
            ("zone_A_slot_3_staging", "zone_A_slot_3_release"),
        }
        host = SlotReleaseHarness(
            self.slot,
            nav_results=(False, True, True),
            pose=(self.slot.release.x + 0.8, self.slot.release.y, self.slot.yaw),
            matrix_edges=matrix_edges,
        )

        success, _ = self._run(host)

        self.assertTrue(success)
        self.assertEqual(
            [(call[0], call[1]) for call in host.nav_calls],
            [
                (self.slot.release.x, self.slot.release.y),
                (self.slot.staging.x, self.slot.staging.y),
                (self.slot.release.x, self.slot.release.y),
            ],
        )

    def test_planned_staging_route_uses_nav2_for_both_waypoints(self):
        matrix_edges = {
            ("red_cube_1", "zone_A_slot_3_staging"),
            ("zone_A_slot_3_staging", "zone_A_slot_3_release"),
        }
        host = SlotReleaseHarness(self.slot, matrix_edges=matrix_edges)

        success, _ = self._run(host, route_via_staging=True)

        self.assertTrue(success)
        self.assertEqual(
            [(call[0], call[1]) for call in host.nav_calls],
            [
                (self.slot.staging.x, self.slot.staging.y),
                (self.slot.release.x, self.slot.release.y),
            ],
        )
        self.assertEqual(host.release_calls, 1)

    def test_planned_staging_route_with_unrepresented_source_fails_closed(self):
        matrix_edges = {
            ("red_cube_1", "zone_A_slot_3_staging"),
            ("zone_A_slot_3_staging", "zone_A_slot_3_release"),
        }
        host = SlotReleaseHarness(self.slot, matrix_edges=matrix_edges)
        host._planner_location = "unknown_source"

        success, reason = self._run(host, route_via_staging=True)

        self.assertFalse(success)
        self.assertEqual(host.nav_calls, [])
        self.assertEqual(host.release_calls, 0)
        self.assertIn("path matrix", reason.lower())

    def test_unreachable_staging_release_edge_blocks_fallback(self):
        host = SlotReleaseHarness(
            self.slot,
            nav_results=(False,),
            pose=(self.slot.release.x + 0.8, self.slot.release.y, self.slot.yaw),
            matrix_edges={("red_cube_1", "zone_A_slot_3_staging")},
        )

        success, reason = self._run(host)

        self.assertFalse(success)
        self.assertEqual(len(host.nav_calls), 1)
        self.assertEqual(host.release_calls, 0)
        self.assertIn("release", reason.lower())

    def test_recovery_rotates_and_checks_attached_cube_region_before_release(self):
        yaw = self.slot.yaw + math.radians(35.0)
        host = SlotReleaseHarness(
            self.slot,
            nav_results=(False,),
            pose=(self.slot.release.x + 0.40, self.slot.release.y, yaw),
            cube_position=(self.slot.drop.x, self.slot.drop.y, self.slot.drop.z),
        )

        success, _ = self._run(host)

        self.assertTrue(success)
        self.assertEqual(host.rotation_calls, [self.slot.yaw])
        self.assertEqual(host.release_calls, 1)

    def test_reported_success_within_recovery_range_can_release_without_unneeded_turn(self):
        host = SlotReleaseHarness(
            self.slot,
            nav_results=(True,),
            pose=(self.slot.release.x + 0.40, self.slot.release.y,
                  self.slot.yaw + math.radians(10.0)),
            cube_position=(self.slot.drop.x, self.slot.drop.y, self.slot.drop.z),
            preserve_pose_on_navigation_success=True,
        )

        success, _ = self._run(host)

        self.assertTrue(success)
        self.assertEqual(host.rotation_calls, [])
        self.assertEqual(host.release_calls, 1)
        gate_events = [fields for name, fields in host.events if name == "slot_terminal_release_gate"]
        self.assertEqual(gate_events[-1]["parking_mode"], "recovery")

    def test_terminal_cargo_clearance_uses_actual_base_yaw_within_tolerance(self):
        actual_yaw = self.slot.yaw + math.radians(15.0)
        host = SlotReleaseHarness(
            self.slot,
            pose=(self.slot.release.x, self.slot.release.y, actual_yaw),
            cube_position=(self.slot.drop.x, self.slot.drop.y, self.slot.drop.z),
            preserve_pose_on_navigation_success=True,
        )

        success, _ = self._run(host)

        self.assertTrue(success)
        carrying_checks = [
            (args, kwargs)
            for args, kwargs in host.segment_clear_calls
            if kwargs.get("carrying")
        ]
        self.assertTrue(carrying_checks)
        self.assertAlmostEqual(carrying_checks[-1][0][2], actual_yaw)
        self.assertTrue(carrying_checks[-1][1].get("terminal_cargo"))

    def test_terminal_known_obstacle_check_uses_live_attached_cube_center(self):
        host = SlotClearanceHarness({
            "red_cube_1": (1.0, 0.0, 0.0),
            "blue_cube_1": (1.16, 0.0, 0.0),
        })

        with patch(
            "aic_task.placement.zone_slots.clearance.segment_map_clear",
            return_value=True,
        ):
            assumed_clear, _ = host._slot_segment_clear(
                (0.0, 0.0),
                (0.0, 0.0),
                0.0,
                active_cube_id="red_cube_1",
                carrying=True,
            )
            clear, reason = host._slot_segment_clear(
                (0.0, 0.0),
                (0.0, 0.0),
                0.0,
                active_cube_id="red_cube_1",
                carrying=True,
                terminal_cargo=True,
            )

        self.assertTrue(assumed_clear)
        self.assertFalse(clear)
        self.assertIn("obstacle", reason.lower())

    def test_terminal_cargo_map_check_keeps_existing_nonblocking_boundary(self):
        host = SlotClearanceHarness({"red_cube_1": (1.0, 0.0, 0.0)})

        with patch(
            "aic_task.placement.zone_slots.clearance.segment_map_clear",
            side_effect=(True, False),
        ):
            clear, reason = host._slot_segment_clear(
                (0.0, 0.0),
                (0.0, 0.0),
                0.0,
                active_cube_id="red_cube_1",
                carrying=True,
                terminal_cargo=True,
            )

        self.assertTrue(clear, reason)
        cargo_map_events = [
            fields for name, fields in host.events
            if name == "slot_cargo_swept_path_blocked"
        ]
        self.assertEqual(len(cargo_map_events), 1)
        self.assertFalse(cargo_map_events[0]["enforced"])

    def test_terminal_cargo_scan_still_filters_self_return_at_live_cube_center(self):
        host = SlotClearanceHarness(
            {"red_cube_1": (1.0, 0.0, 0.0)}, scan_points=((1.0, 0.0),)
        )

        with patch(
            "aic_task.placement.zone_slots.clearance.segment_map_clear",
            return_value=True,
        ):
            clear, reason = host._slot_segment_clear(
                (0.0, 0.0),
                (0.0, 0.0),
                0.0,
                active_cube_id="red_cube_1",
                carrying=True,
                terminal_cargo=True,
            )

        self.assertTrue(clear, reason)

    def test_recovery_refuses_live_attached_cube_outside_zone(self):
        zone = self.points["zones"]["A"]["position"]
        outside = (zone["x"] + 2.0, zone["y"], self.slot.drop.z)
        host = SlotReleaseHarness(
            self.slot,
            nav_results=(False,),
            pose=(self.slot.release.x + 0.40, self.slot.release.y,
                  self.slot.yaw + math.radians(35.0)),
            cube_position=outside,
        )

        success, _ = self._run(host)

        self.assertFalse(success)
        self.assertEqual(host.release_calls, 0)

    def test_attached_cube_target_error_over_eight_cm_is_not_a_release_gate(self):
        offset = 0.09
        host = SlotReleaseHarness(
            self.slot,
            cube_position=(self.slot.drop.x + offset, self.slot.drop.y, self.slot.drop.z),
        )

        success, _ = self._run(host)

        self.assertTrue(success)
        gate_events = [fields for name, fields in host.events if name == "slot_terminal_release_gate"]
        self.assertTrue(gate_events)
        self.assertGreater(gate_events[-1]["attached_cube_error_m"], 0.08)

    def _outside_strict_but_nearby(self, extra_outside_m=0.10):
        region = self.points["zones"]["A"]["position"]
        half_width = float(self.points["approach"].get("cube_half_height_m", 0.015))
        local_x = float(region["size_x_m"]) / 2.0 - half_width + extra_outside_m
        yaw = float(region.get("region_yaw", 0.0))
        return (
            float(region["x"]) + math.cos(yaw) * local_x,
            float(region["y"]) + math.sin(yaw) * local_x,
            self.slot.drop.z,
        )

    def test_pre_release_nearby_tolerance_allows_release_and_records_both_classes(self):
        nearby_position = self._outside_strict_but_nearby()
        host = SlotReleaseHarness(self.slot, cube_position=nearby_position)

        success, reason = self._run(host)

        self.assertTrue(success, reason)
        self.assertEqual(host.release_calls, 1)
        gate = [fields for name, fields in host.events if name == "slot_terminal_release_gate"][-1]
        self.assertFalse(gate["attached_cube_inside_region"])
        self.assertTrue(gate["attached_cube_within_nearby"])
        self.assertAlmostEqual(gate["nearby_tolerance_m"], 0.35)

    def test_pre_release_rejects_cube_beyond_nearby_tolerance_boundary(self):
        outside_tolerance = self._outside_strict_but_nearby(0.36)
        host = SlotReleaseHarness(self.slot, cube_position=outside_tolerance)

        success, _reason = self._run(host)

        self.assertFalse(success)
        self.assertEqual(host.release_calls, 0)
        gate = [fields for name, fields in host.events if name == "slot_terminal_release_gate"][-1]
        self.assertFalse(gate["attached_cube_within_nearby"])
        self.assertFalse(gate["attached_cube_nearby_only"])
        self.assertAlmostEqual(gate["nearby_tolerance_m"], 0.35)
        self.assertTrue(gate["safety_clear"])

    def test_confirmed_nearby_release_is_successful_but_not_counted_as_strict(self):
        nearby_position = self._outside_strict_but_nearby()
        host = SlotReleaseHarness(
            self.slot,
            stable_position=nearby_position,
        )

        success, reason = self._run(host)

        self.assertTrue(success, reason)
        self.assertEqual(
            host._placement_slot_states[self.slot.slot_id],
            SlotStatus.PHYSICALLY_RELEASED,
        )
        verification = [fields for name, fields in host.events if name == "place_verification"][-1]
        self.assertFalse(verification["strict_inside_region"])
        self.assertTrue(verification["within_nearby"])
        self.assertTrue(verification["nearby_only"])
        committed = [fields for name, fields in host.events if name == "slot_release_committed"][-1]
        self.assertFalse(committed["strict_inside_region"])
        self.assertTrue(committed["within_nearby"])
        self.assertTrue(committed["nearby_only"])

    def test_nearby_release_is_rechecked_and_fails_if_it_drifts_beyond_tolerance(self):
        nearby_position = self._outside_strict_but_nearby()
        host = SlotReleaseHarness(
            self.slot,
            stable_position=nearby_position,
        )

        released, reason = self._run(host)

        self.assertTrue(released, reason)
        self.assertIn("red_cube_1", host._placed_cube_regions)
        self.assertTrue(host._verify_placed_cargo_integrity())
        first_audit = [
            fields for name, fields in host.events if name == "final_place_integrity"
        ][-1]
        self.assertTrue(first_audit["within_nearby"])
        self.assertAlmostEqual(first_audit["nearby_tolerance_m"], 0.35)

        host.entity_positions["red_cube_1"] = self._outside_strict_but_nearby(0.36)

        self.assertFalse(host._verify_placed_cargo_integrity())
        final_audit = [
            fields for name, fields in host.events if name == "final_place_integrity"
        ][-1]
        self.assertFalse(final_audit["success"])

    def test_release_outcome_unknown_remains_reserved_as_unknown_and_stops(self):
        host = SlotReleaseHarness(self.slot, release_succeeds=False)

        success, reason = self._run(host)

        self.assertFalse(success)
        self.assertIn("unknown", reason.lower())
        self.assertEqual(
            host._placement_slot_states[self.slot.slot_id], SlotStatus.UNKNOWN
        )
        self.assertTrue(host._stop_exec)
        self.assertEqual(host._attached_cube, "red_cube_1")

    def test_confirmed_release_outside_region_is_not_verified_or_reused(self):
        zone = self.points["zones"]["A"]["position"]
        outside = (zone["x"] + 2.0, zone["y"], self.slot.drop.z)
        host = SlotReleaseHarness(
            self.slot,
            cube_position=(self.slot.drop.x, self.slot.drop.y, self.slot.drop.z),
            stable_position=outside,
        )

        success, reason = self._run(host)

        self.assertFalse(success)
        self.assertIn("outside strict region", reason)
        self.assertEqual(
            host._placement_slot_states[self.slot.slot_id],
            SlotStatus.PHYSICALLY_RELEASED,
        )

    def test_scene_validation_no_longer_requires_staging_line_to_match_release_yaw(self):
        changed = yaml.safe_load(POINTS_PATH.read_text(encoding="utf-8"))
        changed["zones"]["A"]["placement_slots"][2]["staging"]["x"] += 0.20

        validate_placement_slots(changed)

    def test_reverse_progress_uses_odom_displacement_along_reverse_axis(self):
        start = (0.0, 0.0, math.pi / 2.0)
        current = (0.0, -0.4, math.pi / 2.0)

        self.assertAlmostEqual(slot_geometry.reverse_progress_m(start, current, start[2]), 0.4)

    def test_actual_release_pose_maps_to_release_matrix_node(self):
        release_node = matrix_node_name(self.slot.zone, self.slot.number, "release")
        host = SlotRoutingHarness(self.points, {release_node})

        location = host._slot_route_location_for_pose(
            (self.slot.release.x, self.slot.release.y, self.slot.yaw)
        )

        self.assertEqual(location, release_node)

    def test_retreat_uses_odom_and_reports_actual_stop_after_laser_early_stop(self):
        host = SlotRetreatHarness(self.slot)

        success, distance, reason = host._retreat_from_slot(self.slot, "red_cube_1")

        self.assertFalse(success)
        self.assertIn("LaserScan", reason)
        expected = slot_geometry.reverse_progress_m(
            (10.0, -4.0, self.slot.yaw), host.odom_pose, self.slot.yaw
        )
        self.assertGreater(expected, 0.0)
        self.assertAlmostEqual(distance, expected)
        self.assertEqual(host._planner_location, "__live_pose__")
        stop_events = [fields for name, fields in host.events if name == "slot_retreat_stop"]
        self.assertEqual(len(stop_events), 1)
        self.assertAlmostEqual(stop_events[0]["actual_distance_m"], expected)
        self.assertFalse(stop_events[0]["safe_to_continue"])

    def test_retreat_caps_distance_at_one_meter_and_records_reusable_exit_pose(self):
        overlong_exit = replace(
            self.slot.exit,
            max_distance_m=2.0,
            nominal_distance_m=2.0,
        )
        overlong_slot = replace(self.slot, exit=overlong_exit)
        host = SlotRetreatHarness(
            overlong_slot,
            block_after_first=False,
            safe_prefix_m=2.0,
        )

        with patch(
            "aic_task.placement.zone_slots.motion.time.sleep",
            lambda _seconds: None,
        ):
            success, distance, reason = host._retreat_from_slot(
                overlong_slot, "red_cube_1"
            )

        self.assertTrue(success, reason)
        self.assertLessEqual(distance, 1.0)
        self.assertGreaterEqual(distance, 0.985)
        self.assertEqual(
            host._planner_location,
            matrix_node_name(self.slot.zone, self.slot.number, "exit"),
        )
        stop_events = [fields for name, fields in host.events if name == "slot_retreat_stop"]
        self.assertEqual(stop_events[-1]["requested_distance_m"], 1.0)
        self.assertTrue(stop_events[-1]["safe_to_continue"])

    def test_rotation_controller_stops_inside_approved_twenty_degree_tolerance(self):
        target_yaw = self.slot.yaw
        host = SlotRotationHarness(
            (0.0, 0.0, target_yaw + math.radians(15.0))
        )

        with patch(
            "aic_task.placement.zone_slots.motion.time.sleep",
            lambda _seconds: None,
        ):
            success, reason = host._rotate_to_slot_yaw(target_yaw, "red_cube_1")

        self.assertTrue(success, reason)
        self.assertFalse(
            any(abs(message.angular.z) > 0.0 for message in host.pub_precision_cmd.messages)
        )


if __name__ == "__main__":
    unittest.main()
