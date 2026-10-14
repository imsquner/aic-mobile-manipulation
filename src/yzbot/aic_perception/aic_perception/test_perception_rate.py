#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""感知分频策略的离线单测（不依赖 rclpy，本地与 VM 都能跑）。

先失败证明：分频实现之前，检测器只有一个恒定 `processing_period_sec`，
既没有 idle/active 概念，也没有订阅编排器状态，本测试无从通过。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from perception_rate import (  # noqa: E402
    DEFAULT_ACTIVE_STATES,
    effective_period,
    is_active_state,
    normalize_states,
)

FAIL = []
PASS = [0]


def check(name, expected, actual):
    if expected == actual:
        PASS[0] += 1
    else:
        FAIL.append("%s  期望=%r 实际=%r" % (name, expected, actual))


# --- 状态归一化 ---
check("默认活动状态", ("PRE_GRASP", "GRASPING"), normalize_states(None))
check("字符串逗号分隔", ("PRE_GRASP", "GRASPING"), normalize_states("pre_grasp,grasping"))
check("字符串空格分隔", ("PRE_GRASP", "GRASPING"), normalize_states("PRE_GRASP GRASPING"))
check("序列输入", ("PRE_GRASP",), normalize_states(["pre_grasp"]))
check("空列表回退默认", DEFAULT_ACTIVE_STATES, normalize_states([]))
check("空串回退默认", DEFAULT_ACTIVE_STATES, normalize_states(""))

# --- 是否活动状态（大小写/空白不敏感）---
check("PRE_GRASP 是活动", True, is_active_state("PRE_GRASP"))
check("grasping 小写是活动", True, is_active_state("grasping"))
check("带空白也算", True, is_active_state(" GRASPING "))
check("NAVIGATING_TO_OBJECT 不是", False, is_active_state("NAVIGATING_TO_OBJECT"))
check("None 不是", False, is_active_state(None))

# --- 周期选择 ---
check("未知状态用 active（不冒险降频）", 0.05,
      effective_period(state=None, idle_period=0.50, active_period=0.05))
check("抓取阶段用 active", 0.05,
      effective_period(state="GRASPING", idle_period=0.50, active_period=0.05))
check("PRE_GRASP 用 active", 0.05,
      effective_period(state="PRE_GRASP", idle_period=0.50, active_period=0.05))
check("长途导航用 idle", 0.50,
      effective_period(state="NAVIGATING_TO_OBJECT", idle_period=0.50,
                       active_period=0.05))
check("携物导航用 idle", 0.50,
      effective_period(state="NAVIGATING_TO_ZONE", idle_period=0.50,
                       active_period=0.05))
check("固定值优先（向后兼容）", 0.05,
      effective_period(fixed_period=0.05, state="NAVIGATING_TO_ZONE",
                       idle_period=0.50, active_period=0.05))
check("固定值 0 表示不覆盖", 0.50,
      effective_period(fixed_period=0.0, state="NAVIGATING_TO_ZONE",
                       idle_period=0.50, active_period=0.05))
check("自定义活动状态生效", 0.05,
      effective_period(state="PLACING", idle_period=0.50, active_period=0.05,
                       active_states=["PLACING"]))
check("未列入的状态仍走 idle", 0.50,
      effective_period(state="GRASPING", idle_period=0.50, active_period=0.05,
                       active_states=["PLACING"]))
check("负值被夹到 0", 0.0,
      effective_period(state="NAVIGATING", idle_period=-1.0, active_period=0.05))

# --- 检测器必须真的用上这套策略 ---
DETECTOR = os.path.join(HERE, "color_depth_detector.py")
src = open(DETECTOR, encoding="utf-8").read()
for needle, label in (
    ("perception_rate", "导入 perception_rate"),
    ("idle_processing_period_sec", "声明 idle 周期参数"),
    ("active_processing_period_sec", "声明 active 周期参数"),
    ("/competition/current_state", "订阅编排器状态"),
    ("effective_period(", "调用 effective_period"),
):
    check("检测器含 " + label, True, needle in src)

print()
print("通过 %d 项，失败 %d 项" % (PASS[0], len(FAIL)))
for line in FAIL:
    print("FAIL: %s" % line)
def test_perception_rate_checks():
    assert not FAIL, "\n".join(FAIL)


if __name__ == "__main__":
    sys.exit(1 if FAIL else 0)
