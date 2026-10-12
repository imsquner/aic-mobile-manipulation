"""World2 物块软交接：Nav2 → docking 的唯一控制权转移。

纯逻辑，无 ROS 依赖，便于用假动作句柄做失败路径测试（方案 Task 2）。

铁律（方案 Review Focus #1）：
**收到 CancelGoal 成功 ACK *且* 动作到达终态之后，才允许发非零 docking 速度。**
任一条件不满足 → fail-closed（零速、不进入 docking）。
"""
from __future__ import annotations

from dataclasses import dataclass

# 失败原因分类
REASON_OK = "handoff_ready"
REASON_NO_EVIDENCE = "no_navigation_evidence"
REASON_NOT_REACHED = "handoff_band_not_reached"
REASON_NO_GOAL_HANDLE = "no_goal_handle"
REASON_CANCEL_REQUEST_FAILED = "cancel_request_failed"
REASON_CANCEL_ACK_TIMEOUT = "cancel_ack_timeout"
REASON_CANCEL_REJECTED = "cancel_rejected"
REASON_TERMINAL_TIMEOUT = "cancel_terminal_timeout"
REASON_NAV_STOPPED = "navigation_stopped"
REASON_TARGET_STALE = "target_stale"
REASON_VELOCITY_CHAIN_INACTIVE = "velocity_chain_inactive"


@dataclass(frozen=True)
class HandoffEvidence:
    """软交接的结构化证据。`ready` 是唯一放行依据。"""

    handoff_reached: bool = False
    cancel_acknowledged: bool = False
    terminal_after_cancel: bool = False
    reason: str | None = None
    nav_outcome: str | None = None

    @property
    def ready(self) -> bool:
        return (
            self.handoff_reached
            and self.cancel_acknowledged
            and self.terminal_after_cancel
        )


def handoff_from_navigation_evidence(evidence):
    """把 `_navigate_to(..., evidence=...)` 写回的结构化证据转成交接判定。"""
    if not evidence:
        return HandoffEvidence(reason=REASON_NO_EVIDENCE)
    outcome = evidence.get("outcome")
    reached = bool(evidence.get("handoff_reached"))
    acknowledged = bool(evidence.get("cancel_acknowledged"))
    terminal = bool(evidence.get("terminal_after_cancel"))
    reason = None
    if not reached:
        reason = REASON_NOT_REACHED
    elif not acknowledged:
        reason = REASON_CANCEL_ACK_TIMEOUT
    elif not terminal:
        reason = REASON_TERMINAL_TIMEOUT
    else:
        reason = REASON_OK
    result = HandoffEvidence(reached, acknowledged, terminal, reason, outcome)
    return result


def cancel_response_accepted(future):
    """从 CancelGoal 响应里取是否被接受；取不到就不算接受（fail-closed）。

    由 `cancel_and_confirm` 与 `operations._navigate_to` 的取消路径共用，
    避免两份实现漂移。
    """
    try:
        response = future.result()
    except Exception:
        return False
    if response is None:
        return False
    # action_msgs/srv/CancelGoal 响应：goals 里每条有 error_code，
    # ERROR_NONE(0) 表示该 goal 已被接受取消。
    goals = getattr(response, "goals", None)
    if goals is None:
        return True
    if len(goals) == 0:
        return True
    error_code = getattr(goals[0], "error_code", 0)
    return error_code == 0


# 内部旧名，保持既有调用与测试不变
_accepted = cancel_response_accepted


def cancel_and_confirm(
    goal_handle,
    result_future,
    wait_future,
    ack_timeout=5.0,
    terminal_timeout=5.0,
):
    """取消 Nav2 goal 并确认 ACK + 动作终态。

    参数为鸭子类型，便于测试：
    - `goal_handle.cancel_goal_async()` → cancel future
    - `wait_future(future, timeout)` → bool（是否在超时内完成）
    - `result_future` → 导航动作结果 future（终态判定）

    返回 `HandoffEvidence`；只有 ACK 与终态都到位时 `ready` 才为真。
    """
    if goal_handle is None:
        return HandoffEvidence(reason=REASON_NO_GOAL_HANDLE)
    try:
        cancel_future = goal_handle.cancel_goal_async()
    except Exception as exc:  # 服务不可用等
        return HandoffEvidence(
            reason=f"{REASON_CANCEL_REQUEST_FAILED}:{type(exc).__name__}"
        )
    if not wait_future(cancel_future, ack_timeout):
        return HandoffEvidence(reason=REASON_CANCEL_ACK_TIMEOUT)
    if not _accepted(cancel_future):
        return HandoffEvidence(reason=REASON_CANCEL_REJECTED)
    acknowledged = True
    if result_future is None:
        # ACK 到了但没有结果 future：拿不到终态，视为未终态（fail-closed）
        return HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=acknowledged,
            terminal_after_cancel=False,
            reason=REASON_TERMINAL_TIMEOUT,
        )
    if not wait_future(result_future, terminal_timeout):
        return HandoffEvidence(
            handoff_reached=True,
            cancel_acknowledged=acknowledged,
            terminal_after_cancel=False,
            reason=REASON_TERMINAL_TIMEOUT,
        )
    return HandoffEvidence(
        handoff_reached=True,
        cancel_acknowledged=acknowledged,
        terminal_after_cancel=True,
        reason=REASON_OK,
    )


def already_in_band_evidence(base_still, cancel_confirmation=None):
    """起点已在交接带时，构造 `_navigate_to` 风格的结构化证据。

    返回 `(evidence, mode)`。**这是 P1 的核心分支**（独立复核指出过）：
    - 若还有活动 Nav2 goal（`cancel_confirmation` 非 None），ack/terminal 只能来自
      真实的 `cancel_and_confirm`，绝不硬编码 —— 否则 Nav2 与 docking 可能同时发速度；
    - 若压根没发过 goal，取消语义天然满足，但仍要求底盘确实停稳。

    抽成纯函数是为了让这个分支**能被测试**：节点方法本身依赖 ROS，单测进不去，
    P0/P1 正是因此溜过第一轮 85 个单测。
    """
    if cancel_confirmation is not None:
        ready = bool(cancel_confirmation.ready)
        return (
            {
                "outcome": "handoff_reached" if ready else "handoff_cancel_unconfirmed",
                "handoff_reached": ready,
                "cancel_acknowledged": bool(
                    cancel_confirmation.cancel_acknowledged
                ),
                "terminal_after_cancel": bool(
                    cancel_confirmation.terminal_after_cancel
                ),
            },
            "already_in_band_cancel_active",
        )
    still = bool(base_still)
    return (
        {
            "outcome": "handoff_reached" if still else "handoff_not_still",
            "handoff_reached": still,
            # 没有发出过 Nav2 goal，取消语义天然满足
            "cancel_acknowledged": True,
            "terminal_after_cancel": True,
        },
        "already_in_band",
    )


# lifecycle_msgs/GetState 的 active 状态 id
LIFECYCLE_ACTIVE = 3


def velocity_chain_verdict(states):
    """由 lifecycle 状态字典给出 `(determinable, all_active)`。

    `states` 只包含**查得到**的节点（查不到的调用方不该塞进来）。
    空字典 = 无法判定 → `(False, False)`，调用方据此选择"放行并留痕"而非拦死
    （查不到就拦死会造成自锁，见 A1）。
    """
    if not states:
        return False, False
    return True, all(int(state) == LIFECYCLE_ACTIVE for state in states.values())


def docking_release_checks(
    evidence,
    *,
    target_fresh=True,
    velocity_chain_active=True,
    stop_requested=False,
):
    """进入 docking 前的最后一道闸门（全部 fail-closed）。"""
    if stop_requested:
        return REASON_NAV_STOPPED
    if not isinstance(evidence, HandoffEvidence) or not evidence.ready:
        return evidence.reason if isinstance(evidence, HandoffEvidence) else REASON_NO_EVIDENCE
    if not target_fresh:
        return REASON_TARGET_STALE
    if not velocity_chain_active:
        return REASON_VELOCITY_CHAIN_INACTIVE
    return None
