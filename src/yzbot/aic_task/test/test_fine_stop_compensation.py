"""细级补偿纯函数的单元测试（PD 旋转 + 直线精调）。

不依赖 ROS：直接用 importlib 加载 common/motion_geometry.py。
"""
import importlib.util
import math
import unittest
from pathlib import Path

SRC = (
    Path(__file__).resolve().parents[1]
    / "aic_task"
    / "common"
    / "motion_geometry.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("motion_geometry_under_test", SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


MG = _load()


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class TestPdRotationVelocity(unittest.TestCase):
    def test_positive_error_gives_positive_velocity(self):
        self.assertGreater(MG.pd_rotation_velocity(0.5, 0.0), 0.0)

    def test_negative_error_gives_negative_velocity(self):
        self.assertLess(MG.pd_rotation_velocity(-0.5, 0.0), 0.0)

    def test_zero_error_zero_rate_is_stopped(self):
        self.assertEqual(MG.pd_rotation_velocity(0.0, 0.0), 0.0)

    def test_saturation_upper(self):
        # 大误差 + 大误差变化率，应被限幅到 +max_angular
        out = MG.pd_rotation_velocity(10.0, 10.0, max_angular=1.0)
        self.assertLessEqual(out, 1.0 + 1e-9)
        self.assertEqual(out, 1.0)

    def test_saturation_lower(self):
        out = MG.pd_rotation_velocity(-10.0, -10.0, max_angular=1.0)
        self.assertEqual(out, -1.0)

    def test_d_term_damps(self):
        # 误差正在减小（负变化率）应比纯 P 更小
        pure_p = MG.pd_rotation_velocity(0.3, 0.0, kd=0.0)
        with_d = MG.pd_rotation_velocity(0.3, -1.0, kd=0.3)
        self.assertLess(with_d, pure_p)


class TestFineForwardVelocity(unittest.TestCase):
    def test_positive_error_forward(self):
        self.assertGreater(MG.fine_forward_velocity(0.3), 0.0)

    def test_nonpositive_error_stops(self):
        self.assertEqual(MG.fine_forward_velocity(0.0), 0.0)
        self.assertEqual(MG.fine_forward_velocity(-0.1), 0.0)

    def test_upper_saturation(self):
        self.assertEqual(MG.fine_forward_velocity(10.0), 0.5)

    def test_min_floor(self):
        # 0.02 m 误差时 kp*err=0.024 < v_min=0.05，应给下限 0.05
        self.assertEqual(MG.fine_forward_velocity(0.02), 0.05)

    def test_midband(self):
        # 0.2 m 误差：kp*err = 0.24，在 [0.05, 0.5] 内，应输出 0.24
        self.assertAlmostEqual(MG.fine_forward_velocity(0.2), 0.24, places=6)


class TestClosedLoopConvergence(unittest.TestCase):
    def test_rotation_converges_small(self):
        ok, final_err, steps = _sim_rotation(0.1)  # ~5.7°
        self.assertTrue(ok, f"未收敛, 最终误差={final_err:.4f}")
        self.assertLess(steps, 200)

    def test_rotation_converges_large(self):
        ok, final_err, steps = _sim_rotation(1.57)  # 90°
        self.assertTrue(ok, f"未收敛, 最终误差={final_err:.4f}")
        self.assertLess(steps, 400)

    def test_rotation_no_wild_overshoot(self):
        # 从 0.5 rad 收敛过程中，误差符号应单调（至多过一次零）
        yaw = 0.0
        target = 0.5
        prev_e = None
        sign_flips = 0
        last_sign = None
        for _ in range(400):
            e = _norm(target - yaw)
            de = 0.0 if prev_e is None else _norm(e - prev_e) / 0.05
            w = MG.pd_rotation_velocity(e, de, kp=1.5, kd=0.3, max_angular=1.0)
            yaw += w * 0.05
            prev_e = e
            if abs(e) < 0.05236:
                break
            s = 1 if e > 0 else -1
            if last_sign is not None and s != last_sign:
                sign_flips += 1
            last_sign = s
        self.assertLessEqual(sign_flips, 2, f"过冲振荡 {sign_flips} 次")

    def test_forward_converges(self):
        ok, final_err, steps = _sim_forward(0.30)
        self.assertTrue(ok, f"未收敛, 最终误差={final_err:.4f}")
        self.assertLess(steps, 200)

    def test_forward_does_not_engage_beyond_limit(self):
        # 0.35 > 0.30 上限，应判定"不触发"（由调用方限制，这里只验证速度函数不前进太远）
        # 纯函数层面：0.35 误差仍会输出速度（v_max 限幅），触发上限是调用方逻辑，
        # 故这里只断言输出在限幅内。
        self.assertLessEqual(MG.fine_forward_velocity(0.35), 0.5)


def _sim_rotation(yaw0, target=0.0, kp=1.5, kd=0.3, max_angular=1.0, dt=0.05,
                 tol=0.05236, max_steps=800):
    yaw = yaw0
    prev_e = None
    for step in range(max_steps):
        e = _norm(target - yaw)
        if abs(e) <= tol:
            return True, abs(e), step
        de = 0.0 if prev_e is None else _norm(e - prev_e) / dt
        w = MG.pd_rotation_velocity(e, de, kp=kp, kd=kd, max_angular=max_angular)
        yaw += w * dt
        prev_e = e
    return False, abs(_norm(target - yaw)), max_steps


def _sim_forward(err0, kp=1.2, v_min=0.05, v_max=0.5, dt=0.05, tol=0.05,
                 max_steps=800):
    err = err0
    for step in range(max_steps):
        v = MG.fine_forward_velocity(err, kp=kp, v_min=v_min, v_max=v_max)
        err -= v * dt  # 前进减少距离误差
        if abs(err) <= tol:
            return True, abs(err), step
    return False, abs(err), max_steps


if __name__ == "__main__":
    unittest.main()
