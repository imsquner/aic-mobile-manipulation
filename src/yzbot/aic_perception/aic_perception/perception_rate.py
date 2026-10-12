# -*- coding: utf-8 -*-
"""感知处理周期按阶段分频（纯函数，便于离线单测）。

来源：test13 `PERFORMANCE_TUNING.md` §6 —— 「无活动抓取阶段按 `0.50 s` 处理；
状态进入 `PRE_GRASP` 或 `GRASPING` 且已有活动任务时按 `0.05 s` 处理」。

动机：本交付包原先把 `processing_period_sec` 恒定为 0.05（= 20 Hz 上限），
即使车在长途导航、完全用不到视觉时也在满速跑 OpenCV + TF 查询。VM 实测
`color_detector` 在**完全空闲**时占 48.7% CPU。

注意该文档自己把这一项标为「已配置、待部署、待验证」，结论边界写着
「未验证 DDS 负载、TF 丢包和抓取稳定性改善幅度」—— 所以这是一条假设，
必须按 K=5 规则实跑验证，不能当既成事实。
"""
DEFAULT_ACTIVE_STATES = ("PRE_GRASP", "GRASPING")


def normalize_states(value):
    """把 active_states 归一化成大写元组；接受字符串或序列。"""
    if value is None:
        return DEFAULT_ACTIVE_STATES
    if isinstance(value, str):
        parts = [part for part in value.replace(",", " ").split() if part]
    else:
        parts = [str(part) for part in value]
    states = tuple(part.strip().upper() for part in parts if part.strip())
    return states or DEFAULT_ACTIVE_STATES


def is_active_state(state, active_states=None):
    if state is None:
        return False
    return str(state).strip().upper() in normalize_states(active_states)


def effective_period(
    *,
    fixed_period=0.0,
    idle_period=0.50,
    active_period=0.05,
    state=None,
    active_states=None,
):
    """返回本次该用的处理周期（秒）；0.0 表示不节流（处理每一帧）。

    优先级：
      1. `fixed_period > 0` —— 旧的单值 `processing_period_sec` 显式配置时优先，
         保证向后兼容（老配置行为完全不变）。
      2. 还没收到状态消息（state is None）—— 用 active_period。宁可多花点 CPU，
         也不能在「不知道现在什么阶段」时把抓取卡在低速档。
      3. 状态属于 active_states —— active_period。
      4. 其余 —— idle_period。
    """
    if fixed_period and float(fixed_period) > 0.0:
        return float(fixed_period)
    if state is None:
        return max(0.0, float(active_period))
    if is_active_state(state, active_states):
        return max(0.0, float(active_period))
    return max(0.0, float(idle_period))
