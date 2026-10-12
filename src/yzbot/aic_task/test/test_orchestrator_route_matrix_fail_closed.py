import ast
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from aic_task.zone_slots import load_placement_slots, matrix_node_name  # noqa: E402


ORCHESTRATOR = PACKAGE_ROOT / "aic_task" / "task_orchestrator_node.py"
POINTS_FILE = PACKAGE_ROOT / "config" / "scenes" / "world2" / "points.yaml"
ROUTE_POLICY_VERSION = "world2_slot_release_nav2_v1"


class FakePathMatrix:
    payload = None

    @classmethod
    def from_json(cls, path):
        payload = copy.deepcopy(cls.payload)
        return SimpleNamespace(
            _payload=payload,
            node_names=tuple(payload["node_names"]),
            source_hash=payload["source_hash"],
            reachable_edge_count=sum(
                entry.get("status") == "ok" and entry.get("reachable") is True
                for entry in payload["entries"].values()
            ),
        )


def _load_method():
    tree = ast.parse(ORCHESTRATOR.read_text(encoding="utf-8"))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TaskOrchestratorNode"
    )
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == "_load_route_matrix"
    )
    module = ast.Module(body=[copy.deepcopy(method)], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "PathMatrix": FakePathMatrix,
        "load_placement_slots": load_placement_slots,
        "matrix_node_name": matrix_node_name,
        "SLOT_RELEASE_ROUTE_POLICY_VERSION": ROUTE_POLICY_VERSION,
    }
    exec(compile(module, str(ORCHESTRATOR), "exec"), namespace)
    return namespace["_load_route_matrix"]


def _world2_payload():
    points = yaml.safe_load(POINTS_FILE.read_text(encoding="utf-8"))
    slots = load_placement_slots(points)
    cubes = [
        str(cube["id"])
        for color in ("red_cubes", "blue_cubes")
        for cube in points[color]
    ]
    stagings = [matrix_node_name(slot.zone, slot.number, "staging") for slot in slots]
    releases = [matrix_node_name(slot.zone, slot.number, "release") for slot in slots]
    exits = [matrix_node_name(slot.zone, slot.number, "exit") for slot in slots]
    required_pairs = {
        *((source, cube) for source in ("start", *exits, *releases) for cube in cubes),
        *((cube, staging) for cube in cubes for staging in stagings),
        *((cube, release) for cube in cubes for release in releases),
        *((staging, release) for staging, release in zip(stagings, releases)),
    }
    source_hash = "fixture-source"
    return {
        "schema_version": "1.0",
        "source_hash": source_hash,
        "route_policy_version": ROUTE_POLICY_VERSION,
        "node_names": ["start", *cubes, *stagings, *releases, *exits],
        "entries": {
            f"{source}->{target}": {
                "from": source,
                "to": target,
                "source_hash": source_hash,
                "complete": True,
                "status": "ok",
                "reachable": True,
            }
            for source, target in required_pairs
        },
    }


class OrchestratorHarness:
    def __init__(self, points, matrix_path=Path("matrix.json")):
        self.points = points
        self.path_matrix_file = matrix_path
        self.route_planning_required = False
        self.messages = []

    def get_logger(self):
        return SimpleNamespace(
            warning=lambda message: self.messages.append(("warning", message)),
            info=lambda message: self.messages.append(("info", message)),
        )


class OrchestratorRouteMatrixFailClosedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.load_route_matrix = staticmethod(_load_method())

    def setUp(self):
        self.points = yaml.safe_load(POINTS_FILE.read_text(encoding="utf-8"))
        FakePathMatrix.payload = _world2_payload()

    def test_world2_slot_scene_requires_matrix_even_when_legacy_flag_is_false(self):
        node = OrchestratorHarness(self.points, matrix_path=None)

        with self.assertRaisesRegex(RuntimeError, "required for World2 slot flow"):
            self.load_route_matrix(node)

    def test_world2_slot_scene_rejects_wrong_route_policy(self):
        FakePathMatrix.payload["route_policy_version"] = "staging_only_v0"
        node = OrchestratorHarness(self.points)

        with self.assertRaisesRegex(RuntimeError, "route policy"):
            self.load_route_matrix(node)

    def test_world2_slot_scene_requires_release_nodes(self):
        FakePathMatrix.payload["node_names"].remove("zone_A_slot_1_release")
        node = OrchestratorHarness(self.points)

        with self.assertRaisesRegex(RuntimeError, "misses required nodes.*release"):
            self.load_route_matrix(node)

    def test_world2_slot_scene_rejects_pending_critical_edge(self):
        entry = FakePathMatrix.payload["entries"][
            "red_cube_1->zone_A_slot_1_release"
        ]
        entry["complete"] = False
        entry["status"] = "pending"
        entry["reachable"] = False
        node = OrchestratorHarness(self.points)

        with self.assertRaisesRegex(RuntimeError, "critical.*pending"):
            self.load_route_matrix(node)

    def test_world2_slot_scene_accepts_complete_release_matrix(self):
        node = OrchestratorHarness(self.points)

        self.assertIsNotNone(self.load_route_matrix(node))

    def test_v1_scene_keeps_optional_matrix_fallback(self):
        v1_points = {"red_cubes": [], "blue_cubes": [], "zones": {}}
        node = OrchestratorHarness(v1_points, matrix_path=None)

        self.assertIsNone(self.load_route_matrix(node))


if __name__ == "__main__":
    unittest.main()
