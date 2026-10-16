"""World2 物块 docking 闭环：50 Hz「先转后走」+ 一次受限重试。

设计要点（方案 Task 3）：
- 每周期**重新读取**目标与底盘真值；目标过期/消失/非有限 → 立即零速。
- `ROTATE` 只发角速度，`APPROACH` 只发线速度（wz=0），`HOLD` 两轴零速。
- HOLD 连续 `stable_hold_sec` 且**真值终停门**通过才算成功。
- HOLD 但距离低于终停门下限（0.22~0.35 m 过近区）→ 立即判"过近"，不空等到超时。
- 首次未通过：**最多一次**安全倒退（复用现役 `_retreat_after_placement` 模式：
  `retreat_linear_speed` + 航向保持），退到 handoff 带后重试一次。
- 所有失败路径都先发零速。

本模块**不直接依赖 ROS**：所需原语全部通过宿主对象的方法/属性注入，
因此可用假宿主完整测试（`test_object_docking_flow.py`）。
"""
from __future__ import annotations

import math
from dataclasses import replace

from aic_task.motion_geometry import retreat_linear_speed

from aic_task.navigation.object_docking.controller import (
    APPROACH,
    HOLD,
    REJECT_TOO_CLOSE,
    ROTATE,
    DockingConfig,
    DockingOutcome,
    control_step,
    terminal_gate,
    terminal_gate_reason,
)
from aic_task.navigation.object_docking.handoff import (
    already_in_band_evidence,
    docking_release_checks,
    handoff_from_navigation_evidence,
)

# 结果分类
STATUS_OK = "CUBE_DOCKING_OK"
STATUS_TIMEOUT = "CUBE_DOCKING_TIMEOUT"
STATUS_TOO_CLOSE = "PRECISION_PARKING_OVERSHOOT"
STATUS_TOO_CLOSE_FOR_GATE = "CUBE_DOCKING_TOO_CLOSE_FOR_GATE"
STATUS_TARGET_LOST = "CUBE_DOCKING_TARGET_LOST"
STATUS_CANCELLED = "CUBE_DOCKING_CANCELLED"
STATUS_RETREAT_FAILED = "CUBE_DOCKING_RETREAT_FAILED"
STATUS_RETRY_EXHAUSTED = "CUBE_DOCKING_RETRY_EXHAUSTED"

# 首轮默认值（test13 生产实参）
DEFAULT_TIMEOUT_SEC = 12.0
DEFAULT_FREQUENCY_HZ = 50.0
DEFAULT_MAX_RETRIES = 1
DEFAULT_HANDOFF_MIN_M = 0.50
DEFAULT_HANDOFF_MAX_M = 0.85
DEFAULT_RETREAT_SPEED_MPS = 0.08
DEFAULT_RETREAT_MAX_M = 0.60
DEFAULT_RETREAT_YAW_MAX_RPS = 0.20

# 只有"停歪了/停远了"才值得倒退重试；取消与目标丢失是终态，重试没有意义。
RETRYABLE_STATUSES = frozenset(
    {STATUS_TOO_CLOSE_FOR_GATE, STATUS_TOO_CLOSE, STATUS_TIMEOUT}
)


class ObjectDockingMixin:
    """需要宿主提供下列接口（真实实现在 TaskOrchestratorNode）："""

    # --- 宿主必须提供的原语（测试用假宿主替换） ---
    # _should_stop() -> bool
    # _event(type, **fields) -> None
    # _docking_now() -> float（单调秒；默认 time.monotonic）
    # _docking_sleep(sec) -> None
    # _base_pose(frame="map") -> (x, y, yaw) | None
    # _docking_target_sample(cube_id) -> dict | None
    #     必须含: valid(bool), distance_m, yaw_error_rad, age_sec(可选)
    # _publish_docking_twist(linear_x, angular_z) -> None
    # _publish_precision_stop() -> None
    # _wait_for_base_still() -> bool
    # 属性: object_docking_config / object_docking_timeout_sec /
    #       object_docking_frequency_hz / object_docking_max_retries /
    #       object_docking_handoff_min_m / object_docking_handoff_max_m

    def _docking_config(self):
        return getattr(self, "object_docking_config", None) or DockingConfig()

    def _docking_now(self):  # pragma: no cover - 宿主可覆盖
        import time

        return time.monotonic()

    def _docking_sleep(self, seconds):  # pragma: no cover - 宿主可覆盖
        import time

        time.sleep(seconds)

    def _docking_zero(self):
        """零速：任何失败路径都先走这里。"""
        self._publish_precision_stop()
        return DockingOutcome(False, STATUS_CANCELLED)

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def _dock_to_cube(self, cube_id, candidate=None):
        """软交接后执行闭环 docking，必要时一次安全倒退重试。"""
        config = self._docking_config()
        requested_retries = int(
            getattr(self, "object_docking_max_retries", DEFAULT_MAX_RETRIES)
        )
        max_retries = max(0, min(requested_retries, DEFAULT_MAX_RETRIES))
        if requested_retries != max_retries:
            # 不静默：配置要了更多重试但被方案硬上限挡下，必须留痕
            self._event(
                "object_docking_retries_clamped",
                cube=cube_id,
                requested=requested_retries,
                applied=max_retries,
                hard_cap=DEFAULT_MAX_RETRIES,
                reason="plan caps safe retreats at one per item",
            )
        attempt = 0
        retried = False
        outcome = None
        while True:
            attempt += 1
            outcome = self._docking_attempt(cube_id, attempt, config)
            if outcome.success:
                final = replace(outcome, retried=retried, attempts=attempt)
                self._event(
                    "object_docking_result",
                    cube=cube_id,
                    success=True,
                    status=final.status,
                    distance_m=final.distance_m,
                    yaw_error_rad=final.yaw_error_rad,
                    stable_hold_sec=final.stable_hold_sec,
                    attempts=attempt,
                    retried=retried,
                )
                return final
            if outcome.status not in RETRYABLE_STATUSES:
                # 取消 / 目标丢失：终态，直接零速返回，绝不倒退重试
                self._publish_precision_stop()
                terminal = replace(outcome, attempts=attempt, retried=retried)
                self._event(
                    "object_docking_result",
                    cube=cube_id,
                    success=False,
                    status=terminal.status,
                    reason=terminal.reason,
                    distance_m=terminal.distance_m,
                    yaw_error_rad=terminal.yaw_error_rad,
                    attempts=attempt,
                    retried=retried,
                )
                return terminal
            if attempt > max_retries:
                # 用尽重试：再补一次零速（不依赖上一次 attempt 已经发过）
                self._publish_precision_stop()
                final = replace(
                    outcome,
                    status=(
                        outcome.status
                        if outcome.status
                        in (STATUS_CANCELLED, STATUS_TARGET_LOST, STATUS_TOO_CLOSE)
                        else STATUS_RETRY_EXHAUSTED
                    ),
                    attempts=attempt,
                    retried=retried,
                )
                self._event(
                    "object_docking_result",
                    cube=cube_id,
                    success=False,
                    status=final.status,
                    reason=final.reason,
                    distance_m=final.distance_m,
                    yaw_error_rad=final.yaw_error_rad,
                    attempts=attempt,
                    retried=retried,
                )
                return final
            # 一次受限倒退后再试
            retreat_ok = self._docking_retreat(cube_id, config)
            retried = True
            self._event(
                "object_docking_retry",
                cube=cube_id,
                attempt=attempt,
                retreat_success=retreat_ok,
                previous_status=outcome.status,
            )
            if not retreat_ok:
                self._publish_precision_stop()
                failed = replace(
                    outcome,
                    status=STATUS_RETREAT_FAILED,
                    retried=retried,
                    attempts=attempt,
                )
                self._event(
                    "object_docking_result",
                    cube=cube_id,
                    success=False,
                    status=failed.status,
                    attempts=attempt,
                    retried=retried,
                )
                return failed

    # ------------------------------------------------------------------
    # 单次 docking
    # ------------------------------------------------------------------
    def _docking_attempt(self, cube_id, attempt, config):
        timeout = float(getattr(self, "object_docking_timeout_sec", DEFAULT_TIMEOUT_SEC))
        frequency = float(
            getattr(self, "object_docking_frequency_hz", DEFAULT_FREQUENCY_HZ)
        )
        period = 1.0 / max(1.0, frequency)
        started = self._docking_now()
        deadline = started + max(0.1, timeout)
        hold_since = None
        last_distance = None
        last_yaw = None
        samples = 0

        while True:
            if self._should_stop():
                self._publish_precision_stop()
                return DockingOutcome(
                    False,
                    STATUS_CANCELLED,
                    last_distance,
                    last_yaw,
                    reason="stop requested",
                    attempts=attempt,
                )
            now = self._docking_now()
            if now >= deadline:
                self._publish_precision_stop()
                return DockingOutcome(
                    False,
                    STATUS_TIMEOUT,
                    last_distance,
                    last_yaw,
                    reason="docking timeout",
                    attempts=attempt,
                )
            sample = self._docking_target_sample(cube_id)
            if not sample or not sample.get("valid"):
                self._publish_precision_stop()
                return DockingOutcome(
                    False,
                    STATUS_TARGET_LOST,
                    last_distance,
                    last_yaw,
                    reason="target missing, stale or non-finite",
                    attempts=attempt,
                )
            distance = float(sample["distance_m"])
            yaw_error = float(sample["yaw_error_rad"])
            last_distance, last_yaw = distance, yaw_error
            samples += 1

            command = control_step(distance, yaw_error, config)
            if command.state == REJECT_TOO_CLOSE:
                self._publish_precision_stop()
                return DockingOutcome(
                    False,
                    STATUS_TOO_CLOSE,
                    distance,
                    yaw_error,
                    reason="distance below hard minimum",
                    attempts=attempt,
                )
            if command.state == HOLD:
                # 过近区（HOLD 但低于终停门下限）→ 立刻判过近，不空等到超时
                if distance < config.gate_min_distance_m:
                    self._publish_precision_stop()
                    return DockingOutcome(
                        False,
                        STATUS_TOO_CLOSE_FOR_GATE,
                        distance,
                        yaw_error,
                        reason="holding below terminal gate lower bound",
                        attempts=attempt,
                    )
                hold_since = hold_since if hold_since is not None else now
                stable = now - hold_since
                if terminal_gate(distance, yaw_error, stable, config):
                    self._publish_precision_stop()
                    still = self._wait_for_base_still()
                    if not still:
                        self._event(
                            "object_docking_unstable",
                            cube=cube_id,
                            distance_m=round(distance, 3),
                            yaw_error_rad=round(yaw_error, 3),
                        )
                        return DockingOutcome(
                            False,
                            STATUS_TIMEOUT,
                            distance,
                            yaw_error,
                            reason="base not still after hold",
                            attempts=attempt,
                        )
                    return DockingOutcome(
                        True,
                        STATUS_OK,
                        distance,
                        yaw_error,
                        stable,
                        attempts=attempt,
                    )
            else:
                hold_since = None

            self._publish_docking_twist(command.linear_x, command.angular_z)
            self._docking_sleep(period)

    # ------------------------------------------------------------------
    # 一次安全倒退（复用现役 _retreat_after_placement 模式）
    # ------------------------------------------------------------------
    def _docking_retreat(self, cube_id, config):
        handoff_min = float(
            getattr(self, "object_docking_handoff_min_m", DEFAULT_HANDOFF_MIN_M)
        )
        max_distance = float(DEFAULT_RETREAT_MAX_M)
        speed = abs(float(DEFAULT_RETREAT_SPEED_MPS))
        start = self._base_pose("map")
        if start is None or speed <= 0.0:
            self._event(
                "object_docking_retreat",
                cube=cube_id,
                success=False,
                reason="base pose unavailable",
            )
            return False
        deadline = self._docking_now() + max(6.0, max_distance / speed + 3.0)
        travelled = 0.0
        distance = None
        while True:
            if self._should_stop():
                self._publish_precision_stop()
                return False
            now = self._docking_now()
            if now >= deadline or travelled >= max_distance:
                break
            sample = self._docking_target_sample(cube_id)
            if not sample or not sample.get("valid"):
                self._publish_precision_stop()
                self._event(
                    "object_docking_retreat",
                    cube=cube_id,
                    success=False,
                    reason="target lost during retreat",
                )
                return False
            distance = float(sample["distance_m"])
            if distance >= handoff_min:
                break
            pose = self._base_pose("map")
            if pose is not None:
                travelled = math.hypot(pose[0] - start[0], pose[1] - start[1])
            yaw_error = float(sample["yaw_error_rad"])
            linear = -retreat_linear_speed(handoff_min - distance, speed)
            angular = max(-DEFAULT_RETREAT_YAW_MAX_RPS, min(DEFAULT_RETREAT_YAW_MAX_RPS, 1.2 * yaw_error))
            self._publish_docking_twist(linear, angular)
            self._docking_sleep(1.0 / 20.0)
        self._publish_precision_stop()
        still = self._wait_for_base_still()
        ok = bool(distance is not None and distance >= handoff_min and still)
        self._event(
            "object_docking_retreat",
            cube=cube_id,
            success=ok,
            travelled_m=round(travelled, 3),
            distance_m=round(distance, 3) if distance is not None else None,
            handoff_min_m=handoff_min,
        )
        return ok

# ----------------------------------------------------------------------
# 软交接 → 发布门 → 预算守卫 → 零速保持 → docking：**纯编排**
# ----------------------------------------------------------------------
# 这一段原来是节点方法里的"胶水"，依赖 ROS 所以进不了单测 —— 第一轮的 P0 与
# 第二轮的 A1 都出在这类无人测试的接线上。把时序抽出来、依赖全部用回调注入，
# 就能对"顺序"本身写测试（例如"发布门必须在预算守卫之前"、"零速必须在 docking
# 之前"、"门没过就绝不能调 dock"）。
STATUS_BUDGET_EXHAUSTED = "CUBE_DOCKING_BUDGET_EXHAUSTED"
STATUS_BASE_NOT_STILL = "CUBE_DOCKING_BASE_NOT_STILL"


def handoff_then_dock(
    *,
    cube_id,
    sample,
    handoff_min_m,
    handoff_max_m,
    navigate,
    cancel_active_goal,
    base_still,
    publish_stop,
    velocity_chain_active,
    minimum_budget_sec,
    budget_left,
    stop_requested,
    zero_hold_sec,
    sleep,
    dock,
    event,
    resample=None,
    velocity_chain_states=None,
):
    """执行软交接并（若允许）进入闭环 docking。返回 `DockingOutcome`。

    所有副作用都通过回调注入，函数本身不依赖 ROS、不读时钟。
    调用顺序即生产行为，测试锁的就是这个顺序。
    """
    def emit(kind, **fields):
        event(kind, **fields)

    already_in_band = bool(
        sample
        and sample.get("valid")
        and handoff_min_m <= float(sample["distance_m"]) <= handoff_max_m
    )
    if already_in_band:
        # 已在交接带：跳过 Nav2。**绝不硬编码 ack/terminal** —— 有活动 goal 时必须
        # 先真取消并确认 ACK + 动作终态（独立复核 P1）。
        publish_stop()
        confirmation = cancel_active_goal()
        still = None if confirmation is not None else base_still()
        evidence, mode = already_in_band_evidence(
            base_still=still, cancel_confirmation=confirmation
        )
        emit(
            "object_docking_handoff",
            cube=cube_id,
            mode=mode,
            distance_m=sample["distance_m"],
            base_still=None if still is None else bool(still),
            cancel_acknowledged=evidence["cancel_acknowledged"],
            terminal_after_cancel=evidence["terminal_after_cancel"],
            handoff_reason=(
                confirmation.reason if confirmation is not None else None
            ),
        )
    else:
        evidence = navigate()
        emit(
            "object_docking_handoff",
            cube=cube_id,
            mode="nav_gate",
            navigation_succeeded=evidence.get("navigation_succeeded"),
            nav_outcome=evidence.get("outcome"),
            cancel_acknowledged=evidence.get("cancel_acknowledged"),
            terminal_after_cancel=evidence.get("terminal_after_cancel"),
        )

    handoff = handoff_from_navigation_evidence(evidence)
    target = resample() if resample is not None else sample
    blocked = docking_release_checks(
        handoff,
        target_fresh=bool(target and target.get("valid")),
        velocity_chain_active=velocity_chain_active(),
        stop_requested=stop_requested(),
    )
    if blocked is not None:
        publish_stop()
        emit(
            "object_docking_release_blocked",
            cube=cube_id,
            reason=blocked,
            handoff_reason=handoff.reason,
            # 2026-10-16 Plan I instrument: the failing attempts never logged how far the robot
            # actually was from the cube, so "why did we not enter the handoff band" could not be
            # measured. `target` is the resampled (or original) visual sample for this attempt.
            distance_m=None if not target else target.get("distance_m"),
            sample_valid=None if not target else bool(target.get("valid")),
            handoff_min_m=handoff_min_m,
            handoff_max_m=handoff_max_m,
            velocity_chain_states=(
                velocity_chain_states() if velocity_chain_states is not None else None
            ),
        )
        return DockingOutcome(False, blocked, reason=blocked)

    # 预算守卫必须在启动 12 s 控制之前
    if budget_left is not None:
        remaining = float(budget_left())
        if remaining < float(minimum_budget_sec):
            publish_stop()
            emit(
                "object_docking_budget_exhausted",
                cube=cube_id,
                remaining_sec=round(remaining, 2),
                minimum_sec=float(minimum_budget_sec),
            )
            return DockingOutcome(
                False,
                STATUS_BUDGET_EXHAUSTED,
                reason="insufficient remaining item budget",
            )

    # 零速保持：真的确认底盘停稳，不是只 sleep
    if float(zero_hold_sec) > 0.0:
        publish_stop()
        still = base_still()
        sleep(float(zero_hold_sec))
        if not still:
            emit(
                "object_docking_release_blocked",
                cube=cube_id,
                reason="base_not_still_at_handoff",
            )
            return DockingOutcome(
                False, STATUS_BASE_NOT_STILL, reason="base not still"
            )
    return dock()
