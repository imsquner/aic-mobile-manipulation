"""软交接→发布门→预算守卫→零速保持→docking 的**时序**测试。

这是独立复核点出的最后一处空白：节点方法依赖 ROS、进不了单测，于是"顺序写反"
（例如门没过就调 dock、预算没查就启动 12 s 控制）只能靠人读代码发现。
`flow.handoff_then_dock` 把时序抽成纯编排、依赖全部回调注入，这里就用一个记录
调用序列的假宿主来锁顺序。
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "aic_task"
CTRL_SRC = PKG / "navigation" / "object_docking" / "controller.py"
HANDOFF_SRC = PKG / "navigation" / "object_docking" / "handoff.py"
FLOW_SRC = PKG / "navigation" / "object_docking" / "flow.py"
MOTION_SRC = PKG / "common" / "motion_geometry.py"


def _load(name, src):
    spec = importlib.util.spec_from_file_location(name, src)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


CTRL = _load("seq_ctrl", CTRL_SRC)
_load("aic_task.motion_geometry", MOTION_SRC)
_pkg = types.ModuleType("aic_task")
_pkg.__path__ = [str(PKG)]
sys.modules.setdefault("aic_task", _pkg)
_nav = types.ModuleType("aic_task.navigation")
_nav.__path__ = [str(PKG / "navigation")]
sys.modules.setdefault("aic_task.navigation", _nav)
_od = types.ModuleType("aic_task.navigation.object_docking")
_od.__path__ = [str(PKG / "navigation" / "object_docking")]
sys.modules.setdefault("aic_task.navigation.object_docking", _od)
sys.modules["aic_task.navigation.object_docking.controller"] = CTRL
HANDOFF = _load("seq_handoff", HANDOFF_SRC)
sys.modules["aic_task.navigation.object_docking.handoff"] = HANDOFF
FLOW = _load("seq_flow", FLOW_SRC)
sys.modules["aic_task.navigation.object_docking.flow"] = FLOW


READY_EVIDENCE = {
    "outcome": "handoff_reached",
    "handoff_reached": True,
    "cancel_acknowledged": True,
    "terminal_after_cancel": True,
}


class Harness:
    """记录每个副作用调用顺序的假宿主。"""

    def __init__(
        self,
        *,
        sample=None,
        nav_evidence=None,
        active_goal_evidence=None,
        base_still=True,
        velocity_chain_active=True,
        budget_left=30.0,
        stop_requested=False,
        zero_hold_sec=0.10,
        minimum_budget_sec=4.0,
        dock_outcome=None,
    ):
        self.log = []
        self.events = []
        # 默认放在交接带**外**（0.30 < 0.50），默认路径才会走 navigate；
        # 已在交接带的场景在 AlreadyInBandTest 里显式给 0.60。
        self._sample = sample if sample is not None else {
            "valid": True,
            "distance_m": 0.30,
            "yaw_error_rad": 0.0,
        }
        self._nav_evidence = (
            nav_evidence if nav_evidence is not None else dict(READY_EVIDENCE)
        )
        self._active_goal_evidence = active_goal_evidence
        self._base_still = base_still
        self._velocity_chain_active = velocity_chain_active
        self._budget_left = budget_left
        self._stop_requested = stop_requested
        self._zero_hold_sec = zero_hold_sec
        self._minimum_budget_sec = minimum_budget_sec
        self._dock_outcome = dock_outcome or CTRL.DockingOutcome(
            True, FLOW.STATUS_OK, 0.37, 0.05, 0.61, attempts=1
        )
        self.cancel_calls = 0
        self.dock_calls = 0
        self.stops = 0
        self.sleeps = 0

    # --- 回调 ---
    def navigate(self):
        self.log.append("navigate")
        return dict(self._nav_evidence)

    def cancel_active_goal(self):
        """返回 None = 没有活动 goal（不是一次真的取消）。"""
        self.log.append("cancel_active_goal")
        if self._active_goal_evidence is not None:
            self.cancel_calls += 1
        return self._active_goal_evidence

    def base_still(self):
        self.log.append("base_still")
        return self._base_still

    def publish_stop(self):
        self.log.append("stop")
        self.stops += 1

    def velocity_chain_active(self):
        self.log.append("velocity_chain")
        return self._velocity_chain_active

    def budget_left(self):
        self.log.append("budget")
        return self._budget_left

    def stop_requested(self):
        return self._stop_requested

    def sleep(self, seconds):
        self.log.append("sleep")
        self.sleeps += 1

    def dock(self):
        self.log.append("dock")
        self.dock_calls += 1
        return self._dock_outcome

    def event(self, kind, **fields):
        self.events.append((kind, fields))

    def run(self):
        return FLOW.handoff_then_dock(
            cube_id="red_cube_1",
            sample=self._sample,
            handoff_min_m=0.50,
            handoff_max_m=0.85,
            navigate=self.navigate,
            cancel_active_goal=self.cancel_active_goal,
            base_still=self.base_still,
            publish_stop=self.publish_stop,
            velocity_chain_active=self.velocity_chain_active,
            velocity_chain_states=lambda: {"velocity_smoother": 3},
            minimum_budget_sec=self._minimum_budget_sec,
            budget_left=self.budget_left,
            stop_requested=self.stop_requested,
            zero_hold_sec=self._zero_hold_sec,
            sleep=self.sleep,
            dock=self.dock,
            event=self.event,
            resample=lambda: self._sample,
        )

    def kinds(self):
        return [kind for kind, _ in self.events]


class HappyPathTest(unittest.TestCase):
    def test_normal_flow_calls_dock_once(self):
        h = Harness()
        out = h.run()
        self.assertTrue(out.success)
        self.assertEqual(h.dock_calls, 1)
        self.assertIn("navigate", h.log)
        self.assertIn("object_docking_handoff", h.kinds())

    def test_zero_speed_before_dock(self):
        """零速必须发生在 docking 之前。"""
        h = Harness()
        h.run()
        self.assertLess(h.log.index("stop"), h.log.index("dock"))

    def test_order_is_gate_then_budget_then_dock(self):
        """顺序：发布门 → 预算守卫 → docking。顺序写反会启动注定被掐断的控制。"""
        h = Harness()
        h.run()
        self.assertLess(h.log.index("velocity_chain"), h.log.index("budget"))
        self.assertLess(h.log.index("budget"), h.log.index("dock"))

    def test_zero_hold_confirms_still_before_dock(self):
        h = Harness()
        h.run()
        self.assertIn("base_still", h.log)
        self.assertIn("sleep", h.log)
        self.assertLess(h.log.index("base_still"), h.log.index("dock"))


class GateBlockTest(unittest.TestCase):
    def test_handoff_not_reached_never_docks(self):
        """没过发布门 → 绝不调 dock，且必须发零速。"""
        h = Harness(nav_evidence={"outcome": "timeout"})
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)
        self.assertIn("object_docking_release_blocked", h.kinds())

    def test_ack_without_terminal_never_docks(self):
        h = Harness(
            nav_evidence={
                "outcome": "handoff_not_terminal",
                "handoff_reached": False,
                "cancel_acknowledged": True,
                "terminal_after_cancel": False,
            }
        )
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)

    def test_inactive_velocity_chain_never_docks(self):
        h = Harness(velocity_chain_active=False)
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)

    def test_stale_target_never_docks(self):
        h = Harness(sample={"valid": False, "reason": "stale"})
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)

    def test_stop_requested_never_docks(self):
        h = Harness(stop_requested=True)
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)


class BudgetGuardTest(unittest.TestCase):
    def test_budget_exhausted_never_docks(self):
        h = Harness(budget_left=1.0, minimum_budget_sec=4.0)
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(out.status, FLOW.STATUS_BUDGET_EXHAUSTED)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)
        self.assertIn("object_docking_budget_exhausted", h.kinds())

    def test_enough_budget_still_docks(self):
        h = Harness(budget_left=10.0, minimum_budget_sec=4.0)
        out = h.run()
        self.assertTrue(out.success)
        self.assertEqual(h.dock_calls, 1)

    def test_budget_guard_runs_before_dock(self):
        h = Harness(budget_left=0.5)
        h.run()
        self.assertNotIn("dock", h.log)


class AlreadyInBandTest(unittest.TestCase):
    def in_band(self, **kw):
        kw.setdefault(
            "sample", {"valid": True, "distance_m": 0.60, "yaw_error_rad": 0.0}
        )
        return Harness(**kw)

    def test_in_band_skips_navigation(self):
        h = self.in_band()
        h.run()
        self.assertNotIn("navigate", h.log)

    def test_in_band_with_active_goal_cancels_before_dock(self):
        confirmed = HANDOFF.HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=True,
            terminal_after_cancel=True,
            reason=HANDOFF.REASON_OK,
        )
        h = self.in_band(active_goal_evidence=confirmed)
        out = h.run()
        self.assertEqual(h.cancel_calls, 1)
        self.assertTrue(out.success)
        self.assertLess(h.log.index("cancel_active_goal"), h.log.index("dock"))

    def test_in_band_active_goal_unconfirmed_never_docks(self):
        """P1 核心时序：取消未确认 → 绝不能进 docking。"""
        unconfirmed = HANDOFF.HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=True,
            terminal_after_cancel=False,
            reason=HANDOFF.REASON_TERMINAL_TIMEOUT,
        )
        h = self.in_band(active_goal_evidence=unconfirmed)
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)

    def test_in_band_no_goal_uses_base_still(self):
        h = self.in_band(active_goal_evidence=None)
        h.run()
        self.assertIn("base_still", h.log)
        # 没有活动 goal 时不该发生真实取消
        self.assertEqual(h.cancel_calls, 0)

    def test_in_band_not_still_never_docks(self):
        h = self.in_band(base_still=False)
        out = h.run()
        self.assertEqual(h.dock_calls, 0)
        self.assertFalse(out.success)


class ZeroHoldTest(unittest.TestCase):
    def test_base_not_still_at_handoff_blocks_docking(self):
        h = Harness(base_still=False)
        out = h.run()
        self.assertFalse(out.success)
        self.assertEqual(out.status, FLOW.STATUS_BASE_NOT_STILL)
        self.assertEqual(h.dock_calls, 0)
        self.assertGreaterEqual(h.stops, 1)

    def test_zero_hold_disabled_skips_still_check(self):
        h = Harness(zero_hold_sec=0.0)
        out = h.run()
        self.assertTrue(out.success)
        self.assertNotIn("sleep", h.log)


if __name__ == "__main__":
    unittest.main()
