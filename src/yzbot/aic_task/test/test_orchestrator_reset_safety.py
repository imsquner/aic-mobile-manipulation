import ast
import copy
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ORCHESTRATOR = PACKAGE_ROOT / "aic_task" / "task_orchestrator_node.py"


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _load_orchestrator_function(name):
    tree = ast.parse(ORCHESTRATOR.read_text(encoding="utf-8"))
    function = None
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            function = node
        elif isinstance(node, ast.ClassDef) and node.name == "TaskOrchestratorNode":
            function = next(
                (item for item in node.body
                 if isinstance(item, ast.FunctionDef) and item.name == name),
                function,
            )
    if function is None:
        raise AssertionError(f"orchestrator function not found: {name}")
    module = ast.Module(body=[copy.deepcopy(function)], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "TaskState": SimpleNamespace(TASK_FAILED="TASK_FAILED"),
        "threading": threading,
        "String": FakeString,
        "_world2_slot_reset_block_reason": _load_orchestrator_function(
            "_world2_slot_reset_block_reason"
        ) if name == "_complete_reset" else None,
    }
    exec(compile(module, str(ORCHESTRATOR), "exec"), namespace)
    return namespace[name]


class ResetHarness:
    def __init__(self, slot_states=None, attached_cube=None, slot_mode=True):
        self._reset_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._placement_slot_states = slot_states if slot_states is not None else {}
        self._placed_cube_regions = {"red_cube_1": {"x": 0.0, "y": 0.0}}
        self._placed_cube_obstacle_positions = {"red_cube_1": (0.0, 0.0)}
        self._attached_cube = attached_cube
        self.reset_scene_on_reset = False
        self._reset_pending = True
        self._stop_exec = True
        self._scene_reset_active = False
        self.current_task = object()
        self.expanded_items = [object()]
        self._task_plan_frozen = True
        self.start_time = 100.0
        self._frozen_elapsed = 4.0
        self.state = "CANCELLED"
        self.reset_run_called = False
        self.finish_reset_called = False
        self.events = []
        self.pub_navigation = FakePublisher()
        self.pub_last_error = FakePublisher()
        self._slot_mode_enabled = lambda: slot_mode
        self._event = lambda event, **fields: self.events.append((event, fields))
        self._transition = lambda state: setattr(self, "state", state)
        self._reset_run_counters = lambda: setattr(self, "reset_run_called", True)
        self._finish_reset = lambda: setattr(self, "finish_reset_called", True)


class OrchestratorResetSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reset_reason = staticmethod(
            _load_orchestrator_function("_world2_slot_reset_block_reason")
        )
        cls.complete_reset = staticmethod(
            _load_orchestrator_function("_complete_reset")
        )

    def test_every_non_available_slot_state_blocks_nonphysical_world2_reset(self):
        for status in (
            "reserved", "physically_released", "verified", "unknown", "unexpected"
        ):
            with self.subTest(status=status):
                reason = self.reset_reason(
                    True, False, {"A1": status}, attached_cube=None
                )
                self.assertIsNotNone(reason)
                self.assertIn("A1", reason)
                self.assertIn("physical scene reset is required", reason)

    def test_attached_cube_alone_blocks_nonphysical_world2_reset(self):
        reason = self.reset_reason(
            True, False, {}, attached_cube="red_cube_1"
        )
        self.assertIn("red_cube_1", reason)

    def test_cargo_region_or_obstacle_records_block_even_when_slot_map_is_empty(self):
        for has_regions, has_obstacles in ((True, False), (False, True)):
            with self.subTest(regions=has_regions, obstacles=has_obstacles):
                reason = self.reset_reason(
                    True, False, {}, None, has_regions, has_obstacles
                )
                self.assertIn("placed-cargo registrations remain", reason)

    def test_empty_world2_slots_and_legacy_scenes_keep_existing_reset_path(self):
        self.assertIsNone(self.reset_reason(True, False, {}, None))
        self.assertIsNone(
            self.reset_reason(False, False, {"A1": "verified"}, None)
        )
        self.assertIsNone(
            self.reset_reason(True, True, {"A1": "verified"}, None)
        )

    def test_blocked_reset_keeps_slot_and_obstacle_state_and_never_finishes_ready(self):
        states = {}
        node = ResetHarness(slot_states=states)
        slots_before = node._placement_slot_states.copy()
        regions_before = copy.deepcopy(node._placed_cube_regions)
        obstacles_before = copy.deepcopy(node._placed_cube_obstacle_positions)

        self.complete_reset(node)

        self.assertEqual(node.state, "TASK_FAILED")
        self.assertEqual(node._placement_slot_states, slots_before)
        self.assertEqual(node._placed_cube_regions, regions_before)
        self.assertEqual(node._placed_cube_obstacle_positions, obstacles_before)
        self.assertFalse(node.reset_run_called)
        self.assertFalse(node.finish_reset_called)
        self.assertFalse(node._reset_pending)
        self.assertIn(
            "physical scene reset is required",
            node.pub_last_error.messages[-1].data,
        )

    def test_non_slot_scene_still_uses_legacy_counter_reset(self):
        node = ResetHarness(slot_states={}, attached_cube=None, slot_mode=False)
        self.complete_reset(node)
        self.assertTrue(node.reset_run_called)
        self.assertTrue(node.finish_reset_called)
        self.assertNotEqual(node.state, "TASK_FAILED")


if __name__ == "__main__":
    unittest.main()
