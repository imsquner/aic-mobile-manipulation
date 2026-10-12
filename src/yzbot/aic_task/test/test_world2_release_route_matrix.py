import copy
import dataclasses
import json
import sys
import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from aic_task.planning.path_matrix import (  # noqa: E402
    Pose2D,
    _matrix_edge_pairs,
    _prepare_cache,
    _scene_nodes_from_points,
)
from aic_task.planning.route_planner import (  # noqa: E402
    Demand,
    NavigationProfile,
    NoFeasiblePlan,
    PathMatrix,
    RouteCostModel,
    plan_ordered_route,
)
from aic_task.scene_manager import (  # noqa: E402
    SceneError,
    load_scene,
    validate_scene,
)


SCENES = PACKAGE_ROOT / "config" / "scenes"
WORKSPACE = PACKAGE_ROOT.parents[2]
OLD_MATRIX = (
    WORKSPACE
    / "verification"
    / "path_matrix_world2_slots15_staging050_20260926_235400"
    / "static_path_matrix.json"
)
ROUTE_POLICY_VERSION = "world2_slot_release_nav2_v1"


def _matrix_entry(source, target, seconds=1.0, *, reachable=True):
    return {
        "from": source,
        "to": target,
        "complete": True,
        "source_hash": "test-source",
        "status": "ok" if reachable else "no_path",
        "reachable": reachable,
        "planning_time_sec": seconds if reachable else None,
        "path_length_m": seconds if reachable else 0.0,
        "heading_change_rad": 0.0,
        "turn_count": 0,
    }


def _route_matrix(edges):
    staging = "zone_A_slot_1_staging"
    release = "zone_A_slot_1_release"
    exit_node = "zone_A_slot_1_exit"
    entries = {
        f"{source}->{target}": _matrix_entry(source, target, seconds, reachable=reachable)
        for source, target, seconds, reachable in edges
    }
    return PathMatrix(
        {
            "schema_version": "1.0",
            "source_hash": "test-source",
            "node_names": ["start", "red_cube_1", staging, release, exit_node],
            "node_poses": {},
            "slot_geometry": {
                staging: {
                    "zone": "A",
                    "slot_number": 1,
                    "exit_direction": "reverse",
                    "exit_max_distance_m": 1.0,
                    "exit_nominal_distance_m": 1.0,
                }
            },
            "entries": entries,
        }
    )


def _route_plan(matrix):
    return plan_ordered_route(
        matrix,
        current_location="start",
        remaining_demands=(
            Demand(
                ordinal=1,
                sequence=1,
                variable="item_1",
                color="red",
                zone="A",
            ),
        ),
        inventory_by_color={"red": ("red_cube_1",), "blue": ()},
        zone_slot_counts={"A": 0, "B": 0, "C": 0},
        cost_model=RouteCostModel(
            transit=NavigationProfile(1.0, 1.0, 0.0),
            carry=NavigationProfile(2.0, 1.0, 0.0),
            vision_stable_sec=0.0,
            grasp_sec=0.0,
            attach_confirm_sec=0.0,
            place_sec=0.0,
            lift_retract_sec=0.0,
            dynamic_obstacle_margin_sec=0.0,
            zone_switch_penalty_sec=0.0,
            deadline_sec=300.0,
        ),
    )


class World2ReleaseRouteMatrixTest(unittest.TestCase):
    def test_scene_nodes_include_each_release_pose_and_keep_staging_and_exit(self):
        points = yaml.safe_load(
            (SCENES / "world2" / "points.yaml").read_text(encoding="utf-8")
        )
        nodes, slot_geometry = _scene_nodes_from_points(points)

        self.assertEqual(len(slot_geometry), 15)
        for zone in ("A", "B", "C"):
            for slot_number in range(1, 6):
                release = f"zone_{zone}_slot_{slot_number}_release"
                staging = f"zone_{zone}_slot_{slot_number}_staging"
                exit_node = f"zone_{zone}_slot_{slot_number}_exit"
                raw = points["zones"][zone]["placement_slots"][slot_number - 1]
                self.assertEqual((nodes[release].x, nodes[release].y),
                                 (raw["release"]["x"], raw["release"]["y"]))
                self.assertIn(staging, nodes)
                self.assertIn(exit_node, nodes)

    def test_slot_edge_policy_has_direct_release_and_verified_two_leg_fallback(self):
        points = yaml.safe_load(
            (SCENES / "world2" / "points.yaml").read_text(encoding="utf-8")
        )
        nodes, slot_geometry = _scene_nodes_from_points(points)
        pairs = _matrix_edge_pairs(nodes, slot_geometry)
        pair_set = set(pairs)

        for cube in ("red_cube_1", "blue_cube_1"):
            self.assertIn((cube, "zone_A_slot_1_release"), pair_set)
            self.assertIn((cube, "zone_A_slot_1_staging"), pair_set)
        self.assertIn(
            ("zone_A_slot_1_staging", "zone_A_slot_1_release"), pair_set
        )
        self.assertIn(("zone_A_slot_1_release", "red_cube_1"), pair_set)
        self.assertIn(("zone_A_slot_1_exit", "red_cube_1"), pair_set)
        self.assertEqual(len(pairs), len(pair_set))

    def test_old_edge_policy_cache_is_discarded_even_when_points_and_nodes_match(self):
        cache = {
            "source_hash": "same-source",
            "node_names": ["start", "cube"],
            "route_policy_version": "staging-only-v1",
            "entries": {"start->cube": {"complete": True, "reachable": True}},
        }

        prepared = _prepare_cache(
            cache,
            source_hash="same-source",
            node_names=["start", "cube"],
            route_policy_version=ROUTE_POLICY_VERSION,
        )

        self.assertEqual(prepared["entries"], {})
        self.assertEqual(prepared["route_policy_version"], ROUTE_POLICY_VERSION)

    def test_planner_uses_direct_release_edge_when_available(self):
        matrix = _route_matrix(
            [
                ("start", "red_cube_1", 2.0, True),
                ("red_cube_1", "zone_A_slot_1_release", 7.0, True),
                ("red_cube_1", "zone_A_slot_1_staging", 1.0, True),
                ("zone_A_slot_1_staging", "zone_A_slot_1_release", 1.0, True),
            ]
        )

        step = _route_plan(matrix).steps[0]

        self.assertEqual(step.carry_time_sec, 7.0)
        self.assertEqual(step.operation_time_sec, 0.0)
        self.assertEqual(step.release_node, "zone_A_slot_1_release")
        self.assertFalse(step.route_via_staging)
        self.assertEqual(step.slot_exit_time_sec, 0.0)

    def test_planner_uses_staging_only_when_both_fallback_edges_are_reachable(self):
        matrix = _route_matrix(
            [
                ("start", "red_cube_1", 2.0, True),
                ("red_cube_1", "zone_A_slot_1_release", 0.0, False),
                ("red_cube_1", "zone_A_slot_1_staging", 3.0, True),
                ("zone_A_slot_1_staging", "zone_A_slot_1_release", 4.0, True),
            ]
        )

        step = _route_plan(matrix).steps[0]

        self.assertEqual(step.carry_time_sec, 7.0)
        self.assertTrue(step.route_via_staging)
        self.assertEqual(step.staging_node, "zone_A_slot_1_staging")
        self.assertEqual(step.release_node, "zone_A_slot_1_release")
        self.assertTrue(_route_plan(matrix).to_dict()["steps"][0]["route_via_staging"])

    def test_planner_rejects_staging_fallback_when_either_leg_is_unreachable(self):
        matrix = _route_matrix(
            [
                ("start", "red_cube_1", 2.0, True),
                ("red_cube_1", "zone_A_slot_1_release", 0.0, False),
                ("red_cube_1", "zone_A_slot_1_staging", 3.0, True),
                ("zone_A_slot_1_staging", "zone_A_slot_1_release", 0.0, False),
            ]
        )

        with self.assertRaises(NoFeasiblePlan):
            _route_plan(matrix)

    def test_following_leg_uses_release_anchor_and_exit_cost_is_distance_only(self):
        staging1 = "zone_A_slot_1_staging"
        release1 = "zone_A_slot_1_release"
        exit1 = "zone_A_slot_1_exit"
        staging2 = "zone_A_slot_2_staging"
        release2 = "zone_A_slot_2_release"
        exit2 = "zone_A_slot_2_exit"
        edges = [
            ("start", "red_cube_1", 1.0, True),
            ("red_cube_1", release1, 1.0, True),
            (release1, "blue_cube_1", 2.0, True),
            ("blue_cube_1", release2, 3.0, True),
        ]
        entries = {
            f"{source}->{target}": _matrix_entry(
                source, target, seconds, reachable=reachable
            )
            for source, target, seconds, reachable in edges
        }
        matrix = PathMatrix(
            {
                "schema_version": "1.0",
                "source_hash": "test-source",
                "node_names": [
                    "start", "red_cube_1", "blue_cube_1",
                    staging1, release1, exit1, staging2, release2, exit2,
                ],
                "slot_geometry": {
                    staging: {
                        "zone": "A",
                        "slot_number": number,
                        "exit_direction": "reverse",
                        "exit_max_distance_m": 1.0,
                        "exit_nominal_distance_m": 1.0,
                    }
                    for staging, number in ((staging1, 1), (staging2, 2))
                },
                "entries": entries,
            }
        )
        plan = plan_ordered_route(
            matrix,
            current_location="start",
            remaining_demands=(
                Demand(1, 1, "item_1", "red", "A"),
                Demand(2, 2, "item_2", "blue", "A"),
            ),
            inventory_by_color={"red": ("red_cube_1",), "blue": ("blue_cube_1",)},
            zone_slot_counts={"A": 0, "B": 0, "C": 0},
            cost_model=RouteCostModel(
                transit=NavigationProfile(1.0, 1.0, 0.0),
                carry=NavigationProfile(2.0, 1.0, 0.0),
                vision_stable_sec=0.0,
                grasp_sec=0.0,
                attach_confirm_sec=0.0,
                place_sec=0.0,
                lift_retract_sec=0.0,
                dynamic_obstacle_margin_sec=0.0,
                zone_switch_penalty_sec=0.0,
                deadline_sec=300.0,
            ),
        )

        self.assertEqual(plan.steps[1].source, release1)
        self.assertEqual(plan.steps[0].slot_exit_time_sec, 0.5)
        self.assertEqual(plan.steps[1].slot_exit_time_sec, 0.0)

    def test_slot_scene_rejects_old_matrix_without_release_contract(self):
        scene = load_scene("world2", SCENES, WORKSPACE)
        old_matrix = json.loads(OLD_MATRIX.read_text(encoding="utf-8"))
        _pin_matrix_to_scene(old_matrix, scene.matrix_source_hash)

        with self.assertRaisesRegex(SceneError, "route policy|release"):
            validate_scene(scene, matrix=old_matrix)

    def test_slot_scene_rejects_wrong_release_pose_and_missing_release_edge(self):
        scene = load_scene("world2", SCENES, WORKSPACE)
        matrix = _complete_scene_matrix(scene)
        release = "zone_A_slot_1_release"
        matrix["node_poses"][release]["x"] += 0.01

        with self.assertRaisesRegex(SceneError, "pose metadata differs"):
            validate_scene(scene, matrix=matrix)

        matrix = _complete_scene_matrix(scene)
        del matrix["entries"]["red_cube_1->zone_A_slot_1_release"]
        with self.assertRaisesRegex(SceneError, "required slot edges missing"):
            validate_scene(scene, matrix=matrix)

    def test_slot_scene_rejects_matrix_with_wrong_source_hash(self):
        scene = load_scene("world2", SCENES, WORKSPACE)
        matrix = _complete_scene_matrix(scene)
        matrix["source_hash"] = "0" * 64

        with self.assertRaisesRegex(SceneError, "source hash differs"):
            validate_scene(scene, matrix=matrix)

    def test_runtime_scene_rejects_unmeasured_release_matrix(self):
        scene = load_scene("world2", SCENES, WORKSPACE)
        scene = dataclasses.replace(
            scene,
            world_file=WORKSPACE / "src/yzbot/mybot_description/worlds/offic_room.world",
            map_file=WORKSPACE / "src/yzbot/bot_navigation/maps/mapn3.yaml",
        )
        pending_seed = json.loads(scene.matrix_file.read_text(encoding="utf-8"))
        pending_edge = pending_seed["entries"][
            "red_cube_1->zone_A_slot_1_release"
        ]
        pending_edge.update(
            complete=False,
            status="pending",
            reachable=False,
            error="not_measured_in_test_fixture",
        )

        with self.assertRaisesRegex(SceneError, "critical slot edges unmeasured"):
            validate_scene(scene, matrix=pending_seed, require_runtime_files=True)


def _complete_scene_matrix(scene):
    points = yaml.safe_load(scene.points_file.read_text(encoding="utf-8"))
    nodes, slot_geometry = _scene_nodes_from_points(points)
    for zone, zone_data in points["zones"].items():
        for slot_number, slot in enumerate(zone_data["placement_slots"], start=1):
            nodes.setdefault(
                f"zone_{zone}_slot_{slot_number}_release",
                Pose2D(
                    float(slot["release"]["x"]),
                    float(slot["release"]["y"]),
                    float(slot["yaw"]),
                ),
            )
    base = json.loads(OLD_MATRIX.read_text(encoding="utf-8"))
    entries = copy.deepcopy(base.get("entries", {}))
    source_hash = scene.matrix_source_hash
    if not source_hash:
        raise AssertionError("scene fixture must pin its matrix source hash")
    required_pairs = set(_matrix_edge_pairs(nodes, slot_geometry))
    cubes = [
        str(cube["id"])
        for color in ("red_cubes", "blue_cubes")
        for cube in points.get(color, [])
    ]
    for zone in points["zones"]:
        for slot_number in range(1, 6):
            staging = f"zone_{zone}_slot_{slot_number}_staging"
            release = f"zone_{zone}_slot_{slot_number}_release"
            required_pairs.add((staging, release))
            required_pairs.update((cube, release) for cube in cubes)
            required_pairs.update((release, cube) for cube in cubes)
    for source, target in required_pairs:
        key = f"{source}->{target}"
        entries.setdefault(
            key,
            {
                "from": source,
                "to": target,
                "complete": False,
                "source_hash": source_hash,
                "status": "pending",
                "reachable": False,
                "error": "not_measured_in_static_matrix",
            },
        )
    for entry in entries.values():
        entry["source_hash"] = source_hash
    return {
        **base,
        "source_hash": source_hash,
        "route_policy_version": ROUTE_POLICY_VERSION,
        "node_names": list(nodes),
        "node_poses": {
            name: {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
            for name, pose in nodes.items()
        },
        "slot_geometry": slot_geometry,
        "entries": entries,
    }


def _pin_matrix_to_scene(matrix, source_hash):
    if not source_hash:
        raise AssertionError("scene fixture must pin its matrix source hash")
    matrix["source_hash"] = source_hash
    for entry in matrix.get("entries", {}).values():
        entry["source_hash"] = source_hash
    return matrix


if __name__ == "__main__":
    unittest.main()
