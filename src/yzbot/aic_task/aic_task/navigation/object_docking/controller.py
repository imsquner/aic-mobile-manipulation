"""World2 物块 docking 纯控制器（无 ROS 依赖）。

语义迁移自 test13 现役 worktree 的
`2.0rl/test/test13/worktree/dev_ws/src/yzbot/aic_task/aic_task/transport_runtime/
continuous_transport_logic.py::precision_parking_control_step`，
**但实参取 test13 生产入口**（`single_transport_runner.py` / `path_edge_runner.py` 的 argparse
默认），不是函数签名默认值 —— 两者不同，见下表。

| 参数 | test13 函数签名默认 | test13 生产实参（本模块采用） |
|---|---|---|
| `radius_tolerance_m` | 0.08 | **0.04** |
| `yaw_tolerance_rad` | 0.15 | **0.20** |
| `max_linear_speed_mps` | 0.15 | **0.18** |
| `linear_gain` | 1.60 | **1.80** |
| `angular_gain` | 1.80 | **1.80** |

> 上表"函数签名默认"一列取自 `cube_docking_control_step`
> （`continuous_transport_logic.py:1088-1099`，即 test13 里真正被 docking 分支调用的那个
> 包装函数）。它内部转调的 `precision_parking_control_step`（:1056-1067）签名默认是
> `0.40 / 0.04 / 0.15 / 0.20 / 1.8 / 1.8` —— **两个函数的默认值不同**，这正是"不能照签名
> 抄参数"的原因。生产实参由 `path_edge_runner.py:1573-1585` 从 args 对象传入。
>
> 另注：test13 profile 里 `cube_handoff_max_distance_m` 的 **兜底值是 0.75**
> （`single_transport_runner.py:683` 的 `.get(..., 0.75)`），而 argparse 默认是 **0.85**
> （:1576）。本模块按生产入口的 argparse 默认取 **0.85**。

控制律（"先转后走"）：

1. `d < hard_min` → `REJECT_TOO_CLOSE`（两轴零速，过近优先于转向）
2. `|yaw_error| > yaw_tolerance` → `ROTATE`（`vx=0`，`wz=±min(max_wz, gain·|yaw|)`）
3. `d - nominal <= hold_tolerance` → `HOLD`（两轴零速）
4. 否则 → `APPROACH`（`vx=min(max_vx, gain·(d-nominal))`，**`wz=0`，纯直走**）

与旧 `fixed_direction_approach_command` 的关键差别：APPROACH 阶段 `wz=0`（旧实现边转边走）。 

World2 终停放行门由老板 2026-09-28 确认为 `[0.35, 0.48] m`（上限 0.48 = 几何计算的抓取可达上限；
不是 test13 的 0.43，也不是方案暂定的 0.50）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# 控制状态
ROTATE = "ROTATE"
APPROACH = "APPROACH"
HOLD = "HOLD"
REJECT_TOO_CLOSE = "REJECT_TOO_CLOSE"

# 终态判定
TERMINAL_OK = "TERMINAL_OK"
TERMINAL_TOO_CLOSE = "TERMINAL_TOO_CLOSE"
TERMINAL_TOO_FAR = "TERMINAL_TOO_FAR"
TERMINAL_YAW = "TERMINAL_YAW"
TERMINAL_UNSTABLE = "TERMINAL_UNSTABLE"


def _finite(value, name):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return number


@dataclass(frozen=True)
class DockingConfig:
    """首轮 docking 参数（test13 生产实参 + World2 终停放行门）。"""

    # 控制
    nominal_radius_m: float = 0.35
    hold_tolerance_m: float = 0.04
    hard_min_distance_m: float = 0.22
    yaw_tolerance_rad: float = 0.20
    max_linear_mps: float = 0.18
    max_angular_rps: float = 0.60
    linear_gain: float = 1.80
    angular_gain: float = 1.80
    stable_hold_sec: float = 0.60
    # 终停放行门（老板 2026-09-28 确认：上限 0.48 = 抓取可达上限）
    gate_min_distance_m: float = 0.35
    gate_max_distance_m: float = 0.48
    gate_yaw_tolerance_rad: float = 0.20

    def __post_init__(self):
        if not math.isfinite(self.nominal_radius_m) or self.nominal_radius_m <= 0.0:
            raise ValueError("nominal_radius_m must be finite and positive")
        if not math.isfinite(self.hold_tolerance_m) or self.hold_tolerance_m < 0.0:
            raise ValueError("hold_tolerance_m must be finite and non-negative")
        if (
            not math.isfinite(self.hard_min_distance_m)
            or self.hard_min_distance_m < 0.0
        ):
            raise ValueError("hard_min_distance_m must be finite and non-negative")
        if self.hard_min_distance_m >= self.gate_min_distance_m:
            raise ValueError(
                "hard_min_distance_m must be below gate_min_distance_m; otherwise "
                "the controller would HOLD across the whole gate window and the "
                "terminal gate could never be satisfied from a HOLD"
            )
        if self.gate_min_distance_m > self.gate_max_distance_m:
            raise ValueError("gate_min_distance_m must not exceed gate_max_distance_m")
        for name in (
            "yaw_tolerance_rad",
            "gate_yaw_tolerance_rad",
            "max_linear_mps",
            "max_angular_rps",
            "linear_gain",
            "angular_gain",
            "stable_hold_sec",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class DockingCommand:
    state: str
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class DockingOutcome:
    """docking 段的结构化结果（供 flow/orchestrator 判成败）。"""

    success: bool
    status: str
    distance_m: float | None = None
    yaw_error_rad: float | None = None
    stable_hold_sec: float | None = None
    attempts: int = 1
    retried: bool = False
    reason: str | None = None


def control_step(distance_m, yaw_error_rad, config=None):
    """算一步有界 docking 命令。纯函数，不读时钟、不发消息。"""
    cfg = config or DockingConfig()
    distance = _finite(distance_m, "distance_m")
    yaw_error = _finite(yaw_error_rad, "yaw_error_rad")
    if distance < 0.0:
        raise ValueError("distance_m must be non-negative")

    # 1) 过近优先：即使航向超差也两轴零速
    if distance < cfg.hard_min_distance_m:
        return DockingCommand(REJECT_TOO_CLOSE, 0.0, 0.0)
    # 2) 先转
    if abs(yaw_error) > cfg.yaw_tolerance_rad:
        magnitude = min(
            cfg.max_angular_rps,
            max(0.0, cfg.angular_gain * abs(yaw_error)),
        )
        return DockingCommand(ROTATE, 0.0, math.copysign(magnitude, yaw_error))
    # 3) 停稳
    if distance - cfg.nominal_radius_m <= cfg.hold_tolerance_m:
        return DockingCommand(HOLD, 0.0, 0.0)
    # 4) 后走：★ wz=0，纯直走
    linear = min(
        cfg.max_linear_mps,
        max(0.0, cfg.linear_gain * (distance - cfg.nominal_radius_m)),
    )
    return DockingCommand(APPROACH, linear, 0.0)


def terminal_gate(distance_m, yaw_error_rad, stable_sec, config=None):
    """真值终停放行门：距离、航向、稳定性三项全过才返回 True。"""
    cfg = config or DockingConfig()
    distance = _finite(distance_m, "distance_m")
    yaw_error = _finite(yaw_error_rad, "yaw_error_rad")
    stable = _finite(stable_sec, "stable_sec")
    if distance < 0.0:
        raise ValueError("distance_m must be non-negative")
    if stable < 0.0:
        raise ValueError("stable_sec must be non-negative")
    if distance < cfg.gate_min_distance_m:
        return False
    if distance > cfg.gate_max_distance_m:
        return False
    if abs(yaw_error) > cfg.gate_yaw_tolerance_rad:
        return False
    return stable >= cfg.stable_hold_sec


def terminal_gate_reason(distance_m, yaw_error_rad, stable_sec, config=None):
    """终停门未过的分类原因（供事件记录；过了返回 None）。"""
    cfg = config or DockingConfig()
    distance = _finite(distance_m, "distance_m")
    yaw_error = _finite(yaw_error_rad, "yaw_error_rad")
    stable = _finite(stable_sec, "stable_sec")
    if distance < cfg.gate_min_distance_m:
        return TERMINAL_TOO_CLOSE
    if distance > cfg.gate_max_distance_m:
        return TERMINAL_TOO_FAR
    if abs(yaw_error) > cfg.gate_yaw_tolerance_rad:
        return TERMINAL_YAW
    if stable < cfg.stable_hold_sec:
        return TERMINAL_UNSTABLE
    return None
