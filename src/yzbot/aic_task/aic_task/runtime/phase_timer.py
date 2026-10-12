# -*- coding: utf-8 -*-
"""阶段计时器：给性能调优提供可信的「时间花在哪个阶段」。

为什么不直接写在编排器里：编排器 import 需要 rclpy，Windows 本地跑不了单测。
把纯逻辑抽出来后，`test_phase_timer.py` 可以在本地与 VM 上都跑。

用法：
    timer = PhaseTimer()
    timer.reset("BOOTING")
    ...
    timer.enter("NAVIGATING_TO_OBJECT")   # 关闭上一个阶段并开始新阶段
    ...
    timer.snapshot()                      # {'BOOTING': 0.1, 'NAVIGATING_TO_OBJECT': 12.3}
"""
import time


class PhaseTimer(object):
    """按「进入某个标签」累计耗时；标签可重复进入，时长累加。"""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._label = None
        self._entered_at = None
        self._durations = {}

    @property
    def label(self):
        return self._label

    def _close(self, now):
        if self._label is not None and self._entered_at is not None:
            self._durations[self._label] = (
                self._durations.get(self._label, 0.0) + (now - self._entered_at)
            )
        self._entered_at = now

    def reset(self, label=None):
        """清空并（可选）立即进入一个标签。"""
        self._durations = {}
        self._label = label
        self._entered_at = self._clock() if label is not None else None

    def enter(self, label):
        now = self._clock()
        self._close(now)
        self._label = label

    def snapshot(self, digits=3):
        """返回累计时长（含当前阶段的已进行部分），不改变内部状态。"""
        durations = dict(self._durations)
        if self._label is not None and self._entered_at is not None:
            durations[self._label] = durations.get(self._label, 0.0) + (
                self._clock() - self._entered_at
            )
        return {name: round(value, digits) for name, value in sorted(durations.items())}

    def total(self):
        return sum(self.snapshot(digits=6).values())

