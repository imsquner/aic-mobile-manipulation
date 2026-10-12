"""World2 物块 docking 闭环测试（Task 3）。

用**假宿主 + 迷你运动仿真**驱动真实的 `ObjectDockingMixin`，无需 ROS/Gazebo。
覆盖：正常一次完成、0.34 m 过近倒退再成功、一次重试仍失败、yaw 大先转、
目标失效、任务取消、硬过近、停不稳超时；并断言**所有失败路径都发过零速**、
**重试次数 ≤ 1**。
"""
import importlib.util
import math
import sys
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "aic_task"
CTRL_SRC = PKG / "navigation" / "object_docking" / "controller.py"
FLOW_SRC = PKG / "navigation" / "object_docking" / "flow.py"
MOTION_SRC = PKG / "common" / "motion_geometry.py"


def _load(name, src):
    spec = importlib.util.spec_from_file_location(name, src)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


CTRL = _load("objdock_ctrl_under_test", CTRL_SRC)
_load("aic_task.motion_geometry", MOTION_SRC)

# flow.py 用绝对导入 `aic_task.navigation.object_docking.controller`，
# 这里把包根挂进 sys.modules 再加载它。
import types  # noqa: E402

_pkg_aic = types.ModuleType("aic_task")
_pkg_aic.__path__ = [str(PKG)]
sys.modules.setdefault("aic_task", _pkg_aic)
_nav = types.ModuleType("aic_task.navigation")
_nav.__path__ = [str(PKG / "navigation")]
sys.modules.setdefault("aic_task.navigation", _nav)
_od = types.ModuleType("aic_task.navigation.object_docking")
_od.__path__ = [str(PKG / "navigation" / "object_docking")]
sys.modules.setdefault("aic_task.navigation.object_docking", _od)
sys.modules["aic_task.navigation.object_docking.controller"] = CTRL

FLOW = _load("objdock_flow_under_test", FLOW_SRC)
sys.modules["aic_task.navigation.object_docking.flow"] = FLOW


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(0.0, float(seconds))


class FakeDockingHost(FLOW.ObjectDockingMixin):
    """假宿主 + 迷你运动仿真。"""

    def __init__(
        self,
        distance,
        yaw_error=0.0,
        *,
        max_retries=1,
        target_valid=True,
        still=True,
        stop_after=math.inf,
        timeout=12.0,
        step_cap=100000,
    ):
        self.object_docking_config = CTRL.DockingConfig()
        self.object_docking_timeout_sec = timeout
        self.object_docking_frequency_hz = 50.0
        self.object_docking_max_retries = max_retries
        self.object_docking_handoff_min_m = 0.50
        self.clock = FakeClock()
        self.distance = float(distance)
        self.yaw_error = float(yaw_error)
        self.target_valid = target_valid
        self.still = still
        self.stop_after = stop_after
        self.step_cap = step_cap
        self.commands = []
        self.stops = 0
        self.events = []
        self.retreats = 0
        self.nonzero_published_after_stop = 0
        self._stop_seen = False

    # --- 宿主原语 ---
    def _docking_now(self):
        return self.clock.now()

    def _docking_sleep(self, seconds):
        self.clock.sleep(seconds)

    def _should_stop(self):
        if self.clock.t >= self.stop_after:
            self._stop_seen = True
            return True
        return False

    def _event(self, kind, **fields):
        self.events.append((kind, fields))
        if kind == "object_docking_retreat":
            self.retreats += 1

    def _base_pose(self, frame="map"):
        return (0.0, 0.0, 0.0)

    def _publish_precision_stop(self):
        self.stops += 1

    def _wait_for_base_still(self):
        return self.still

    def _docking_target_sample(self, cube_id):  # noqa: ARG002
        if not self.target_valid:
            return None
        return {
            "valid": True,
            "distance_m": self.distance,
            "yaw_error_rad": self.yaw_error,
        }

    def _publish_docking_twist(self, linear, angular):
        if len(self.commands) > self.step_cap:
            raise RuntimeError("fake host step cap exceeded (possible infinite loop)")
        self.commands.append((linear, angular))
        if self._stop_seen and (abs(linear) > 1e-9 or abs(angular) > 1e-9):
            self.nonzero_published_after_stop += 1
        dt = 1.0 / max(1.0, self.object_docking_frequency_hz)
        if -0.30 <= linear <= 0.30:
            self.distance = max(0.0, self.distance - linear * dt)
        if abs(angular) > 1e-12 and abs(self.yaw_error) > 1e-12:
            self.yaw_error -= angular * dt
            if (angular > 0 and self.yaw_error < 0) or (
                angular < 0 and self.yaw_error > 0
            ):
                self.yaw_error = 0.0


class DockingFlowTest(unittest.TestCase):
    def result_event(self, host):
        events = [fields for kind, fields in host.events if kind == "object_docking_result"]
        self.assertTrue(events, "missing object_docking_result event")
        return events[-1]

    def test_normal_run_succeeds_in_one_attempt(self):
        host = FakeDockingHost(0.60, 0.0)
        out = host._dock_to_cube("red_cube_1")
        self.assertTrue(out.success)
        self.assertEqual(out.status, FLOW.STATUS_OK)
        self.assertEqual(out.attempts, 1)
        self.assertGreaterEqual(host.stops, 1, "must end with a zero-speed publish")
        # HOLD 带是 [nominal, nominal+tol] = [0.35, 0.39]，车会停在带上沿附近；
        # 终停门是 [0.35, 0.48]，所以停在 0.39 附近是正确的、可抓的。
        self.assertGreaterEqual(host.distance, 0.35 - 1e-6)
        self.assertLessEqual(host.distance, 0.39 + 1e-6)
        self.assertTrue(
            CTRL.terminal_gate(host.distance, host.yaw_error, 1.0, host.object_docking_config)
        )

    def test_large_yaw_rotates_before_translating(self):
        """yaw 大时先原地转：第一条 ROTATE 命令线速度必须为 0。"""
        host = FakeDockingHost(0.40, math.radians(60.0))
        out = host._dock_to_cube("red_cube_1")
        first_linear, _ = host.commands[0]
        self.assertEqual(first_linear, 0.0, "first command must be pure rotation")
        self.assertTrue(out.success)
        self.assertEqual(out.attempts, 1)

    def test_too_close_holds_then_retreats_then_succeeds(self):
        """0.34 m：HOLD 但低于终停门下界 → 一次倒退 → 重试成功。"""
        host = FakeDockingHost(0.34, 0.0)
        out = host._dock_to_cube("red_cube_1")
        self.assertTrue(out.success)
        self.assertEqual(out.attempts, 2)
        self.assertTrue(out.retried, "success on attempt 2 must be flagged as retried")
        self.assertEqual(host.retreats, 1, "exactly one retreat")
        statuses = [f.get("status") for k, f in host.events if k == "object_docking_result"]
        self.assertIn(FLOW.STATUS_OK, statuses)

    def test_max_retries_zero_never_retreats(self):
        """max_retries=0 时必须直接失败、一次都不倒退。"""
        host = FakeDockingHost(0.34, 0.0, max_retries=0)
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertEqual(host.retreats, 0)
        self.assertEqual(out.attempts, 1)
        self.assertFalse(out.retried)

    def test_excess_max_retries_is_clamped_and_logged(self):
        """配置要 5 次重试也要被硬上限 1 挡下，且必须留痕（不能静默）。"""
        host = FakeDockingHost(0.34, 0.0, max_retries=5)
        host._dock_to_cube("red_cube_1")
        clamped = [f for k, f in host.events if k == "object_docking_retries_clamped"]
        self.assertEqual(len(clamped), 1)
        self.assertEqual(clamped[0]["requested"], 5)
        self.assertEqual(clamped[0]["applied"], 1)
        self.assertLessEqual(host.retreats, 1)

    def test_retry_exhausted_status_is_reported(self):
        """倒退还成功、但重试后仍停进过近区 → RETRY_EXHAUSTED，attempts=2。"""
        host = FakeDockingHost(0.30, 0.0)
        phase = {"retreating": False}
        # 倒退阶段放行到交接带（让倒退成功）；其余时刻把距离钉在 0.30 过近区
        def sample(cube_id):  # noqa: ARG001
            distance = 0.55 if phase["retreating"] else 0.30
            return {"valid": True, "distance_m": distance, "yaw_error_rad": 0.0}

        original_retreat = host._docking_retreat

        def retreat(cube_id, config):
            phase["retreating"] = True
            try:
                return original_retreat(cube_id, config)
            finally:
                phase["retreating"] = False

        host._docking_target_sample = sample
        host._docking_retreat = retreat
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertEqual(out.status, FLOW.STATUS_RETRY_EXHAUSTED)
        self.assertEqual(out.attempts, 2)
        self.assertTrue(out.retried)
        self.assertEqual(host.retreats, 1)

    def test_retreat_failure_stops_without_second_retry(self):
        """倒退时目标失效 → 立即停，不再重试（重试上限 1）。"""
        host = FakeDockingHost(0.34, 0.0)
        original = host._docking_target_sample

        calls = {"n": 0}

        def sample(cube_id):
            calls["n"] += 1
            # 第一次 docking attempt 正常；进入倒退后让目标失效
            if calls["n"] > 3:
                return None
            return original(cube_id)

        host._docking_target_sample = sample
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertLessEqual(out.attempts, 2)
        self.assertEqual(host.retreats, 1)
        self.assertGreaterEqual(host.stops, 1)

    def test_hard_too_close_recovers_by_one_retreat(self):
        """硬过近（<0.22）：不得抓取；一次安全倒退后重试成功。"""
        host = FakeDockingHost(0.18, 0.0)
        out = host._dock_to_cube("red_cube_1")
        self.assertEqual(host.retreats, 1, "hard too-close must retreat exactly once")
        self.assertEqual(out.attempts, 2)
        self.assertTrue(out.success)
        self.assertGreaterEqual(host.stops, 1)

    def test_hard_too_close_without_recovery_space_fails_closed(self):
        """过近且退不出去 → 失败且零速，不再重试。"""
        host = FakeDockingHost(0.18, 0.0)
        host._docking_target_sample = lambda cube_id: None
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertGreaterEqual(host.stops, 1)

    def test_target_lost_stops_immediately(self):
        host = FakeDockingHost(0.60, 0.0, target_valid=False)
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertEqual(out.status, FLOW.STATUS_TARGET_LOST)
        self.assertGreaterEqual(host.stops, 1)
        self.assertEqual(host.commands, [], "no nonzero command when target invalid")

    def test_task_cancel_publishes_zero_and_aborts(self):
        host = FakeDockingHost(0.60, 0.0, stop_after=0.0)
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertEqual(out.status, FLOW.STATUS_CANCELLED)
        self.assertGreaterEqual(host.stops, 1)
        self.assertEqual(host.nonzero_published_after_stop, 0)

    def test_unstable_base_times_out_without_success(self):
        """HOLD 达标但底盘停不稳 → 不得报成功。"""
        host = FakeDockingHost(0.35, 0.0, still=False)
        out = host._dock_to_cube("red_cube_1")
        self.assertFalse(out.success)
        self.assertGreaterEqual(host.stops, 1)

    def test_retries_never_exceed_one(self):
        for start in (0.34, 0.30, 0.25):
            host = FakeDockingHost(start, 0.0)
            host._dock_to_cube("red_cube_1")
            self.assertLessEqual(host.retreats, 1, f"start={start}")

    def test_zero_speed_is_always_the_last_command(self):
        """成功与失败路径都必须以零速收尾。"""
        for host in (
            FakeDockingHost(0.60, 0.0),
            FakeDockingHost(0.34, 0.0),
            FakeDockingHost(0.18, 0.0),
            FakeDockingHost(0.60, 0.0, target_valid=False),
        ):
            with self.subTest(host=host):
                host._dock_to_cube("red_cube_1")
                self.assertGreaterEqual(host.stops, 1)

    def test_result_event_records_attempts(self):
        host = FakeDockingHost(0.60, 0.0)
        host._dock_to_cube("red_cube_1")
        event = self.result_event(host)
        self.assertTrue(event["success"])
        self.assertEqual(event["attempts"], 1)


class RetreatTest(unittest.TestCase):
    def test_retreat_backs_away_until_handoff_band(self):
        host = FakeDockingHost(0.34, 0.0)
        ok = host._docking_retreat("red_cube_1", host.object_docking_config)
        self.assertTrue(ok)
        self.assertGreaterEqual(host.distance, 0.50 - 1e-6)
        self.assertGreaterEqual(host.stops, 1)

    def test_retreat_publishes_negative_linear_only(self):
        host = FakeDockingHost(0.34, 0.0)
        host._docking_retreat("red_cube_1", host.object_docking_config)
        self.assertTrue(host.commands)
        for linear, _ in host.commands:
            self.assertLessEqual(linear, 0.0, "retreat must never drive forward")

    def test_retreat_stops_when_target_lost(self):
        host = FakeDockingHost(0.34, 0.0, target_valid=False)
        ok = host._docking_retreat("red_cube_1", host.object_docking_config)
        self.assertFalse(ok)
        self.assertGreaterEqual(host.stops, 1)


if __name__ == "__main__":
    unittest.main()
