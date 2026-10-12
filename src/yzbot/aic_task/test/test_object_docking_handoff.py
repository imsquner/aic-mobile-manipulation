"""软交接失败路径测试（Task 2）。

用假动作句柄/假 future 覆盖：进入带、起点已在带、CancelGoal ACK 超时、
ACK 成功但终态未到、取消被拒、无句柄、速度链 inactive、任务取消/复位、
目标过期。所有失败路径都必须 `ready=False`（→ 零速、不进 docking）。
"""
import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "aic_task" / "navigation" / "object_docking"
HANDOFF_SRC = ROOT / "handoff.py"


def _load(name, src):
    spec = importlib.util.spec_from_file_location(name, src)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


H = _load("object_docking_handoff_under_test", HANDOFF_SRC)


# --- 假对象 ---

class FakeFuture:
    def __init__(self, result=None, raises=None):
        self._result = result
        self._raises = raises

    def result(self):
        if self._raises is not None:
            raise self._raises
        return self._result


class FakeGoal:
    def __init__(self, error_code=0):
        self.error_code = error_code


class FakeCancelResponse:
    def __init__(self, error_codes=(0,)):
        self.goals = [FakeGoal(code) for code in error_codes]


class FakeGoalHandle:
    def __init__(self, cancel_future=None, raises=None):
        self._cancel_future = cancel_future or FakeFuture(FakeCancelResponse())
        self._raises = raises
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self._raises is not None:
            raise self._raises
        return self._cancel_future


def wait_all(future, timeout):
    """假 wait：一律立即完成。"""
    return True


def wait_none(future, timeout):
    """假 wait：一律超时。"""
    return False


class CancelAndConfirmTest(unittest.TestCase):
    def test_ack_and_terminal_both_present_is_ready(self):
        ev = H.cancel_and_confirm(
            FakeGoalHandle(), FakeFuture(object()), wait_all
        )
        self.assertTrue(ev.ready)
        self.assertTrue(ev.cancel_acknowledged)
        self.assertTrue(ev.terminal_after_cancel)
        self.assertEqual(ev.reason, H.REASON_OK)

    def test_cancel_ack_timeout_is_not_ready(self):
        """CancelGoal ACK 超时 → 绝不能进 docking。"""
        handle = FakeGoalHandle()
        ev = H.cancel_and_confirm(handle, FakeFuture(object()), wait_none)
        self.assertFalse(ev.ready)
        self.assertFalse(ev.cancel_acknowledged)
        self.assertEqual(ev.reason, H.REASON_CANCEL_ACK_TIMEOUT)

    def test_ack_ok_but_terminal_missing_is_not_ready(self):
        """ACK 成功但动作未到终态 → 仍是 fail-closed（Review Focus #1）。"""
        calls = {"n": 0}

        def wait_first_only(future, timeout):
            calls["n"] += 1
            return calls["n"] == 1  # 第一次（ACK）成功，第二次（终态）超时

        ev = H.cancel_and_confirm(FakeGoalHandle(), FakeFuture(object()), wait_first_only)
        self.assertFalse(ev.ready)
        self.assertTrue(ev.cancel_acknowledged)
        self.assertFalse(ev.terminal_after_cancel)
        self.assertEqual(ev.reason, H.REASON_TERMINAL_TIMEOUT)

    def test_cancel_rejected_is_not_ready(self):
        handle = FakeGoalHandle(
            cancel_future=FakeFuture(FakeCancelResponse(error_codes=(1,)))
        )
        ev = H.cancel_and_confirm(handle, FakeFuture(object()), wait_all)
        self.assertFalse(ev.ready)
        self.assertFalse(ev.cancel_acknowledged)
        self.assertEqual(ev.reason, H.REASON_CANCEL_REJECTED)

    def test_cancel_response_unreadable_is_not_ready(self):
        """取响应抛异常 → fail-closed。"""
        handle = FakeGoalHandle(
            cancel_future=FakeFuture(raises=RuntimeError("boom"))
        )
        ev = H.cancel_and_confirm(handle, FakeFuture(object()), wait_all)
        self.assertFalse(ev.ready)
        self.assertEqual(ev.reason, H.REASON_CANCEL_REJECTED)

    def test_missing_goal_handle_is_not_ready(self):
        ev = H.cancel_and_confirm(None, FakeFuture(object()), wait_all)
        self.assertFalse(ev.ready)
        self.assertEqual(ev.reason, H.REASON_NO_GOAL_HANDLE)

    def test_cancel_service_raises_is_not_ready_and_zero_speed(self):
        """cancel 服务不可用（测试里的“取消/复位”）→ 不 ready。"""
        handle = FakeGoalHandle(raises=RuntimeError("service unavailable"))
        ev = H.cancel_and_confirm(handle, FakeFuture(object()), wait_all)
        self.assertFalse(ev.ready)
        self.assertTrue(ev.reason.startswith(H.REASON_CANCEL_REQUEST_FAILED))

    def test_missing_result_future_is_not_ready(self):
        ev = H.cancel_and_confirm(FakeGoalHandle(), None, wait_all)
        self.assertFalse(ev.ready)
        self.assertTrue(ev.cancel_acknowledged)
        self.assertFalse(ev.terminal_after_cancel)
        self.assertEqual(ev.reason, H.REASON_TERMINAL_TIMEOUT)

    def test_empty_cancel_goals_list_means_accepted(self):
        """Nav2 有时返回空 goals（表示无待取消 goal）→ 视为接受。"""
        handle = FakeGoalHandle(cancel_future=FakeFuture(FakeCancelResponse(())))
        ev = H.cancel_and_confirm(handle, FakeFuture(object()), wait_all)
        self.assertTrue(ev.ready)


class CancelResponseAcceptedTest(unittest.TestCase):
    """共享判定（operations._cancel_response_accepted 也走这里）。"""

    def test_error_none_is_accepted(self):
        self.assertTrue(
            H.cancel_response_accepted(FakeFuture(FakeCancelResponse((0,))))
        )

    def test_rejected_code_is_not_accepted(self):
        for code in (1, 2, 3):
            self.assertFalse(
                H.cancel_response_accepted(FakeFuture(FakeCancelResponse((code,)))),
                f"error_code={code}",
            )

    def test_empty_goals_is_accepted(self):
        self.assertTrue(
            H.cancel_response_accepted(FakeFuture(FakeCancelResponse(())))
        )

    def test_unreadable_response_is_not_accepted(self):
        self.assertFalse(
            H.cancel_response_accepted(FakeFuture(raises=RuntimeError("boom")))
        )

    def test_none_response_is_not_accepted(self):
        self.assertFalse(H.cancel_response_accepted(FakeFuture(None)))


class NavigationEvidenceTest(unittest.TestCase):
    def test_handoff_reached_with_full_evidence_is_ready(self):
        ev = H.handoff_from_navigation_evidence(
            {
                "outcome": "handoff_reached",
                "handoff_reached": True,
                "cancel_acknowledged": True,
                "terminal_after_cancel": True,
            }
        )
        self.assertTrue(ev.ready)
        self.assertEqual(ev.nav_outcome, "handoff_reached")

    def test_entering_band_is_the_ready_case(self):
        """进入带 = handoff_reached + ACK + 终态。"""
        ev = H.handoff_from_navigation_evidence(
            {
                "outcome": "handoff_reached",
                "handoff_reached": True,
                "cancel_acknowledged": True,
                "terminal_after_cancel": True,
            }
        )
        self.assertEqual(ev.reason, H.REASON_OK)

    def test_timeout_outcome_is_not_ready(self):
        ev = H.handoff_from_navigation_evidence(
            {
                "outcome": "timeout",
                "handoff_reached": False,
                "cancel_acknowledged": True,
                "terminal_after_cancel": True,
            }
        )
        self.assertFalse(ev.ready)
        self.assertEqual(ev.reason, H.REASON_NOT_REACHED)

    def test_handoff_not_terminal_outcome_is_not_ready(self):
        ev = H.handoff_from_navigation_evidence(
            {
                "outcome": "handoff_not_terminal",
                "handoff_reached": False,
                "cancel_acknowledged": True,
                "terminal_after_cancel": False,
            }
        )
        self.assertFalse(ev.ready)

    def test_missing_evidence_is_not_ready(self):
        self.assertFalse(H.handoff_from_navigation_evidence(None).ready)
        self.assertFalse(H.handoff_from_navigation_evidence({}).ready)

    def test_succeeded_navigation_without_gate_is_not_a_handoff(self):
        """正常开到点（没触发交接门）不是软交接。"""
        ev = H.handoff_from_navigation_evidence(
            {
                "outcome": "succeeded",
                "handoff_reached": False,
                "cancel_acknowledged": None,
                "terminal_after_cancel": None,
                "success": True,
            }
        )
        self.assertFalse(ev.ready)


class DockingReleaseGateTest(unittest.TestCase):
    def ready_evidence(self):
        return H.handoff_from_navigation_evidence(
            {
                "outcome": "handoff_reached",
                "handoff_reached": True,
                "cancel_acknowledged": True,
                "terminal_after_cancel": True,
            }
        )

    def test_ready_evidence_passes(self):
        self.assertIsNone(H.docking_release_checks(self.ready_evidence()))

    def test_not_ready_evidence_returns_its_reason(self):
        ev = H.handoff_from_navigation_evidence({"outcome": "timeout"})
        self.assertEqual(
            H.docking_release_checks(ev), H.REASON_NOT_REACHED
        )

    def test_stale_target_blocks_docking(self):
        """上一帧目标过期 → 零速、不进 docking（Review Focus #2）。"""
        self.assertEqual(
            H.docking_release_checks(self.ready_evidence(), target_fresh=False),
            H.REASON_TARGET_STALE,
        )

    def test_inactive_velocity_chain_blocks_docking(self):
        self.assertEqual(
            H.docking_release_checks(
                self.ready_evidence(), velocity_chain_active=False
            ),
            H.REASON_VELOCITY_CHAIN_INACTIVE,
        )

    def test_task_cancel_or_reset_blocks_docking(self):
        self.assertEqual(
            H.docking_release_checks(self.ready_evidence(), stop_requested=True),
            H.REASON_NAV_STOPPED,
        )

    def test_stop_wins_over_ready_evidence(self):
        """复位/取消优先于一切（fail-closed）。"""
        reason = H.docking_release_checks(
            self.ready_evidence(),
            target_fresh=False,
            velocity_chain_active=False,
            stop_requested=True,
        )
        self.assertEqual(reason, H.REASON_NAV_STOPPED)

    def test_unknown_evidence_type_is_not_ready(self):
        self.assertEqual(
            H.docking_release_checks(None), H.REASON_NO_EVIDENCE
        )


class AlreadyInBandEvidenceTest(unittest.TestCase):
    """P1 核心分支：已在交接带时，ack/terminal 只能来自真实取消确认。"""

    def test_active_goal_uses_real_cancel_confirmation(self):
        confirmation = H.HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=True,
            terminal_after_cancel=True,
            reason=H.REASON_OK,
        )
        evidence, mode = H.already_in_band_evidence(
            base_still=True, cancel_confirmation=confirmation
        )
        self.assertEqual(mode, "already_in_band_cancel_active")
        self.assertTrue(evidence["handoff_reached"])
        self.assertTrue(evidence["cancel_acknowledged"])
        self.assertTrue(evidence["terminal_after_cancel"])

    def test_active_goal_not_confirmed_is_fail_closed(self):
        """有活动 goal 但终态未确认 → 绝不放行（这正是 P1 要防的双下发）。"""
        confirmation = H.HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=True,
            terminal_after_cancel=False,
            reason=H.REASON_TERMINAL_TIMEOUT,
        )
        evidence, mode = H.already_in_band_evidence(
            base_still=True, cancel_confirmation=confirmation
        )
        self.assertEqual(mode, "already_in_band_cancel_active")
        self.assertFalse(evidence["handoff_reached"])
        self.assertTrue(evidence["cancel_acknowledged"])
        self.assertFalse(evidence["terminal_after_cancel"])

    def test_active_goal_unconfirmed_never_hardcodes_true(self):
        """回归守卫：有活动 goal 时不得出现"硬编码 ack/terminal=True"。"""
        confirmation = H.HandoffEvidence(
            reason=H.REASON_CANCEL_ACK_TIMEOUT
        )  # 全 False
        evidence, _ = H.already_in_band_evidence(
            base_still=True, cancel_confirmation=confirmation
        )
        self.assertFalse(evidence["cancel_acknowledged"])
        self.assertFalse(evidence["terminal_after_cancel"])
        self.assertFalse(evidence["handoff_reached"])

    def test_no_goal_and_still_is_ready(self):
        evidence, mode = H.already_in_band_evidence(base_still=True)
        self.assertEqual(mode, "already_in_band")
        self.assertTrue(evidence["handoff_reached"])
        self.assertTrue(evidence["cancel_acknowledged"])
        self.assertTrue(evidence["terminal_after_cancel"])

    def test_no_goal_but_not_still_is_not_ready(self):
        evidence, mode = H.already_in_band_evidence(base_still=False)
        self.assertEqual(mode, "already_in_band")
        self.assertFalse(evidence["handoff_reached"])

    def test_evidence_feeds_the_release_gate(self):
        """产出的 evidence 必须能直接喂给发布门（闭环验证）。"""
        confirmation = H.HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=True,
            terminal_after_cancel=True,
            reason=H.REASON_OK,
        )
        evidence, _ = H.already_in_band_evidence(
            base_still=True, cancel_confirmation=confirmation
        )
        gate = H.handoff_from_navigation_evidence(evidence)
        self.assertTrue(gate.ready)
        self.assertIsNone(H.docking_release_checks(gate))

        bad, _ = H.already_in_band_evidence(
            base_still=True,
            cancel_confirmation=H.HandoffEvidence(
                cancel_acknowledged=True, terminal_after_cancel=False
            ),
        )
        self.assertFalse(H.handoff_from_navigation_evidence(bad).ready)


class VelocityChainVerdictTest(unittest.TestCase):
    """A1：判据必须是 lifecycle 状态，且"查不到"不能被当成"不活跃"。"""

    def test_all_active_is_active(self):
        determinable, active = H.velocity_chain_verdict(
            {"velocity_smoother": 3, "collision_monitor": 3}
        )
        self.assertTrue(determinable)
        self.assertTrue(active)

    def test_any_inactive_is_inactive(self):
        for states in (
            {"velocity_smoother": 2, "collision_monitor": 3},
            {"velocity_smoother": 3, "collision_monitor": 1},
            {"velocity_smoother": 4},
        ):
            determinable, active = H.velocity_chain_verdict(states)
            self.assertTrue(determinable, states)
            self.assertFalse(active, states)

    def test_empty_states_is_not_determinable(self):
        """A1 回归守卫：查不到状态必须是"不可判定"，不能变成"不活跃"。"""
        determinable, active = H.velocity_chain_verdict({})
        self.assertFalse(determinable)
        self.assertFalse(active)

    def test_lifecycle_active_constant_matches_ros(self):
        # lifecycle_msgs: 1 unconfigured / 2 inactive / 3 active / 4 finalized
        self.assertEqual(H.LIFECYCLE_ACTIVE, 3)


if __name__ == "__main__":
    unittest.main()
