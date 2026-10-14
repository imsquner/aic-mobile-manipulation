#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PhaseTimer 的离线单测（不依赖 rclpy，本地与 VM 都能跑）。

先失败证明：本文件与 phase_timer.py 出现之前，编排器里没有阶段计时，
只能靠事后硬凑事件时间戳，拿不到可信的「时间花在哪个状态」。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from aic_task.runtime.phase_timer import PhaseTimer  # noqa: E402

FAIL = []
PASS = [0]


def check(name, expected, actual):
    if expected == actual:
        PASS[0] += 1
    else:
        FAIL.append("%s  期望=%r 实际=%r" % (name, expected, actual))


class FakeClock(object):
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


clock = FakeClock()
timer = PhaseTimer(clock=clock)

# 初始状态
check("初始 label 为 None", None, timer.label)
check("初始 snapshot 为空", {}, timer.snapshot())

# reset 带标签：立刻开始计时
timer.reset("BOOTING")
check("reset 后 label", "BOOTING", timer.label)
clock.advance(2.0)
check("单一阶段时长", {"BOOTING": 2.0}, timer.snapshot())

# 切换阶段会关闭上一个
timer.enter("NAVIGATING")
clock.advance(5.0)
check("两阶段累计", {"BOOTING": 2.0, "NAVIGATING": 5.0}, timer.snapshot())

# snapshot 不改变状态：连续两次调用结果一致（时钟不动）
check("snapshot 幂等", timer.snapshot(), timer.snapshot())

# 重复进入同一标签要累加
timer.enter("NAVIGATING")
clock.advance(3.0)
check("重复标签累加", {"BOOTING": 2.0, "NAVIGATING": 8.0}, timer.snapshot())

# reset 清空
timer.reset()
check("reset 清空", {}, timer.snapshot())
check("reset 后 label 为 None", None, timer.label)

# reset(label) 后 enter 关闭的是新标签
timer.reset("A")
clock.advance(1.0)
timer.enter("B")
clock.advance(1.0)
check("reset 后可继续", {"A": 1.0, "B": 1.0}, timer.snapshot())

# total 等于各段之和
check("total 等于各段之和", 2.0, timer.total())

# 时钟不前进时不会产生负值或 NaN
clock.advance(0.0)
check("零时长安全", {"A": 1.0, "B": 1.0}, timer.snapshot())

# 编排器侧必须真的用上这个组件
ORCH = os.path.join(HERE, "task_orchestrator_node.py")
src = open(ORCH, encoding="utf-8").read()
check("编排器导入 PhaseTimer", True, "from aic_task.phase_timer import PhaseTimer" in src
      or "phase_timer import PhaseTimer" in src)
check("编排器发出 item_timing", True, '"item_timing"' in src)
check("编排器在尝试开始时 reset 计时", True, "_phase_timer.reset(" in src)

print()
print("通过 %d 项，失败 %d 项" % (PASS[0], len(FAIL)))
for line in FAIL:
    print("FAIL: %s" % line)
def test_phase_timer_checks():
    assert not FAIL, "\n".join(FAIL)


if __name__ == "__main__":
    sys.exit(1 if FAIL else 0)
