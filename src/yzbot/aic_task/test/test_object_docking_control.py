"""World2 物块 docking 纯控制器单元测试（Task 1，先失败后通过）。

不依赖 ROS：直接用 importlib 加载 navigation/object_docking/controller.py。

参数来源（必须与 test13 现役生产入口一致，不能用函数签名默认值）：
- `2.0rl/test/test13/worktree/.../transport_runtime/single_transport_runner.py`
    --final-vx-max          默认 0.18
    --parking-radius        默认 0.35
    --cube-handoff-min/max  默认 0.50 / 0.85
    --final-object-yaw-tolerance 默认 0.20
- `.../transport_runtime/path_edge_runner.py`
    --precision-radius-tolerance 默认 0.04
    --precision-max-wz           默认 0.60
    --precision-linear-gain      默认 1.80
    --precision-angular-gain     默认 1.80
    --parking-min-distance       默认 0.22
    --precision-stable-hold      默认 0.60
    --cube-docking-frequency     默认 50.0

World2 终停放行门由老板 2026-09-28 确认为 [0.35, 0.48] m（上限 0.48，不是 test13 的 0.43，也不是
方案暂定的 0.50）——依据本会话几何计算：抓取可达上限 0.48 m。
"""
import importlib.util
import math
import sys
import unittest
from pathlib import Path

SRC = (
    Path(__file__).resolve().parents[1]
    / "aic_task"
    / "navigation"
    / "object_docking"
    / "controller.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("object_docking_controller_under_test", SRC)
    mod = importlib.util.module_from_spec(spec)
    # dataclasses + `from __future__ import annotations` 需要模块已在 sys.modules 里
    # 才能解析字符串注解，否则 dataclasses 内部拿到 None。
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class DockingControllerTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load()
        cls.config = cls.mod.DockingConfig()


class ProductionParameterTest(DockingControllerTestBase):
    """首轮参数必须与 test13 生产实参一致（防止误用函数签名默认值）。"""

    def test_first_round_parameters_match_test13_production_entry(self):
        c = self.config
        self.assertAlmostEqual(c.nominal_radius_m, 0.35)
        self.assertAlmostEqual(c.hold_tolerance_m, 0.04)
        self.assertAlmostEqual(c.hard_min_distance_m, 0.22)
        self.assertAlmostEqual(c.yaw_tolerance_rad, 0.20)
        self.assertAlmostEqual(c.max_linear_mps, 0.18)
        self.assertAlmostEqual(c.max_angular_rps, 0.60)
        self.assertAlmostEqual(c.linear_gain, 1.80)
        self.assertAlmostEqual(c.angular_gain, 1.80)
        self.assertAlmostEqual(c.stable_hold_sec, 0.60)

    def test_terminal_gate_is_boss_confirmed_world2_window(self):
        c = self.config
        self.assertAlmostEqual(c.gate_min_distance_m, 0.35)
        self.assertAlmostEqual(c.gate_max_distance_m, 0.48)
        self.assertAlmostEqual(c.gate_yaw_tolerance_rad, 0.20)

    def test_gate_window_lies_inside_grasp_reach(self):
        """抓取可达上限 0.48 m（几何计算结论②），门上限不得超过它。"""
        self.assertLessEqual(self.config.gate_max_distance_m, 0.48 + 1e-9)
        """门下限不得高于 hard min，否则 HOLD 带被门切碎。"""
        self.assertGreaterEqual(
            self.config.gate_min_distance_m, self.config.hard_min_distance_m
        )

    def test_parameter_ordering_invariant(self):
        """方案 Task 4 要求的关系：
        hard_min < gate_min <= HOLD_target+tol <= gate_max <= handoff_min。

        这条关系保证：控制器能停在门内、门内的车都够得到、交接带在门之外
        （否则 Nav2 会在已经能抓的距离上继续往前顶）。
        """
        c = self.config
        self.assertLess(c.hard_min_distance_m, c.gate_min_distance_m)
        self.assertLessEqual(c.gate_min_distance_m, c.nominal_radius_m + c.hold_tolerance_m)
        self.assertLessEqual(c.nominal_radius_m + c.hold_tolerance_m, c.gate_max_distance_m)
        # handoff_min 由 flow 提供，这里只锁控制器侧；0.50 是首轮值
        self.assertLessEqual(c.gate_max_distance_m, 0.50)

    def test_hold_band_overlaps_gate_window(self):
        """控制器能停的区间（<=nominal+tol 且 >=hard_min）必须与门有交集。"""
        c = self.config
        hold_lower = c.hard_min_distance_m
        hold_upper = c.nominal_radius_m + c.hold_tolerance_m
        self.assertLess(hold_lower, c.gate_max_distance_m)
        self.assertGreater(hold_upper, c.gate_min_distance_m)


class ControlStepTest(DockingControllerTestBase):
    def step(self, distance, yaw_error):
        return self.mod.control_step(distance, yaw_error, self.config)

    # --- 先转后走 ---

    def test_large_yaw_rotates_in_place(self):
        cmd = self.step(0.60, math.radians(60.0))
        self.assertEqual(cmd.state, "ROTATE")
        self.assertEqual(cmd.linear_x, 0.0)
        self.assertGreater(cmd.angular_z, 0.0)
        self.assertLessEqual(abs(cmd.angular_z), 0.60)

    def test_large_negative_yaw_rotates_other_way(self):
        cmd = self.step(0.60, math.radians(-60.0))
        self.assertEqual(cmd.state, "ROTATE")
        self.assertEqual(cmd.linear_x, 0.0)
        self.assertLess(cmd.angular_z, 0.0)

    def test_rotate_ignores_distance(self):
        """只要 yaw 超差，无论多远都不发线速度（先转后走）。"""
        for distance in (0.30, 0.55, 0.90, 1.50):
            cmd = self.step(distance, math.radians(30.0))
            self.assertEqual(cmd.linear_x, 0.0, f"distance={distance}")

    def test_aligned_and_far_approaches_straight(self):
        """yaw 达标且远：wz=0（纯直走），0 < vx <= 0.18。"""
        cmd = self.step(0.60, 0.0)
        self.assertEqual(cmd.state, "APPROACH")
        self.assertEqual(cmd.angular_z, 0.0)
        self.assertGreater(cmd.linear_x, 0.0)
        self.assertLessEqual(cmd.linear_x, 0.18)

    def test_approach_speed_uses_production_gain(self):
        """vx = min(0.18, 1.80 * (d - 0.35))。"""
        cmd = self.step(0.45, 0.0)
        self.assertAlmostEqual(cmd.linear_x, min(0.18, 1.80 * 0.10), places=9)

    def test_approach_speed_saturates_at_max(self):
        cmd = self.step(1.20, 0.0)
        self.assertAlmostEqual(cmd.linear_x, 0.18, places=9)

    def test_yaw_just_inside_tolerance_may_approach(self):
        cmd = self.step(0.60, self.config.yaw_tolerance_rad - 1e-6)
        self.assertEqual(cmd.state, "APPROACH")
        self.assertEqual(cmd.angular_z, 0.0)

    def test_yaw_just_outside_tolerance_rotates(self):
        cmd = self.step(0.60, self.config.yaw_tolerance_rad + 1e-6)
        self.assertEqual(cmd.state, "ROTATE")

    # --- HOLD ---

    def test_hold_at_nominal_radius(self):
        cmd = self.step(0.35, 0.0)
        self.assertEqual(cmd.state, "HOLD")
        self.assertEqual(cmd.linear_x, 0.0)
        self.assertEqual(cmd.angular_z, 0.0)

    def test_hold_upper_edge_is_nominal_plus_tolerance(self):
        cmd = self.step(0.35 + 0.04, 0.0)
        self.assertEqual(cmd.state, "HOLD")

    def test_just_above_hold_edge_approaches(self):
        cmd = self.step(0.35 + 0.04 + 1e-6, 0.0)
        self.assertEqual(cmd.state, "APPROACH")

    def test_below_nominal_still_holds_but_not_rejected(self):
        """HOLD 带没有下沿（除硬下限），0.30 仍在 HOLD。"""
        cmd = self.step(0.30, 0.0)
        self.assertEqual(cmd.state, "HOLD")

    # --- 硬过近保护 ---

    def test_too_close_is_rejected_with_zero_speed(self):
        cmd = self.step(0.21, 0.0)
        self.assertEqual(cmd.state, "REJECT_TOO_CLOSE")
        self.assertEqual(cmd.linear_x, 0.0)
        self.assertEqual(cmd.angular_z, 0.0)

    def test_too_close_wins_over_yaw(self):
        """过近优先于转向：即使 yaw 超差也两轴零速。"""
        cmd = self.step(0.10, math.radians(90.0))
        self.assertEqual(cmd.state, "REJECT_TOO_CLOSE")
        self.assertEqual(cmd.linear_x, 0.0)
        self.assertEqual(cmd.angular_z, 0.0)

    def test_exactly_hard_min_is_not_rejected(self):
        cmd = self.step(0.22, 0.0)
        self.assertNotEqual(cmd.state, "REJECT_TOO_CLOSE")

    # --- 输入校验 ---

    def test_non_finite_distance_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError, msg=repr(bad)):
                self.step(bad, 0.0)

    def test_non_finite_yaw_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError, msg=repr(bad)):
                self.step(0.50, bad)

    def test_negative_distance_rejected(self):
        with self.assertRaises(ValueError):
            self.step(-0.01, 0.0)

    def test_command_is_immutable(self):
        cmd = self.step(0.50, 0.0)
        with self.assertRaises(Exception):
            cmd.linear_x = 99.0


class TerminalGateTest(DockingControllerTestBase):
    def gate(self, distance, yaw_error, stable_sec):
        return self.mod.terminal_gate(distance, yaw_error, stable_sec, self.config)

    def test_d_039_holds_but_gate_needs_full_stable_hold(self):
        """HOLD 带内（浮点安全值 0.38）能 HOLD，但稳定性不足 0.60 s 时不得过门。

        注：方案原文写 `d=0.39`，但 0.39 与 HOLD 上沿 `0.35+0.04` 是同一个数学点，
        浮点下 `0.39 - 0.35 = 0.040000000000000036 > 0.04` → 严格 `<=`（与 test13 一致）
        会判 APPROACH。故用 0.38 验 HOLD，另用 0.391 验上沿外。
        """
        self.assertEqual(self.mod.control_step(0.38, 0.0, self.config).state, "HOLD")
        self.assertFalse(self.gate(0.38, 0.0, 0.59))
        self.assertTrue(self.gate(0.38, 0.0, 0.60))

    def test_exact_hold_edge_is_floating_point_sensitive(self):
        """记录上沿的浮点行为，防止有人误以为 0.39 一定 HOLD。"""
        self.assertEqual(self.mod.control_step(0.35 + 0.04, 0.0, self.config).state, "HOLD")
        self.assertEqual(self.mod.control_step(0.391, 0.0, self.config).state, "APPROACH")

    def test_d_034_never_passes_world2_gate_even_when_holding(self):
        """d=0.34 在控制器里是 HOLD，但低于老板确认的门下限 0.35 → 不过门。"""
        self.assertEqual(self.mod.control_step(0.34, 0.0, self.config).state, "HOLD")
        self.assertFalse(self.gate(0.34, 0.0, 5.0))

    def test_gate_upper_bound_is_048(self):
        self.assertTrue(self.gate(0.48, 0.0, 1.0))
        self.assertFalse(self.gate(0.48 + 1e-6, 0.0, 1.0))

    def test_gate_lower_bound_is_035(self):
        self.assertTrue(self.gate(0.35, 0.0, 1.0))
        self.assertFalse(self.gate(0.35 - 1e-6, 0.0, 1.0))

    def test_gate_rejects_bad_yaw_even_at_good_distance(self):
        self.assertFalse(self.gate(0.40, math.radians(20.0), 5.0))
        self.assertTrue(self.gate(0.40, math.radians(11.4), 1.0))

    def test_gate_rejects_too_close_even_with_perfect_yaw(self):
        """0.13~0.29 m 过近区间一律不过门（方案 Review Focus #4）。"""
        for distance in (0.13, 0.20, 0.25, 0.29, 0.34):
            self.assertFalse(self.gate(distance, 0.0, 5.0), f"distance={distance}")

    def test_gate_rejects_negative_stable_hold_as_invalid(self):
        """负的稳定时长是非法输入，不是"未达标"，必须报错而不是静默返回 False。"""
        with self.assertRaises(ValueError):
            self.gate(0.40, 0.0, -1.0)

    def test_gate_rejects_non_finite(self):
        with self.assertRaises(ValueError):
            self.gate(float("nan"), 0.0, 1.0)
        with self.assertRaises(ValueError):
            self.gate(0.40, float("inf"), 1.0)
        with self.assertRaises(ValueError):
            self.gate(0.40, 0.0, float("nan"))


if __name__ == "__main__":
    unittest.main()
