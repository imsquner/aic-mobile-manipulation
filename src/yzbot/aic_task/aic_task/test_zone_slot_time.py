"""Offline regression checks for World2 slot-flow timestamp gating."""

import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from aic_task.placement.zone_slots.timing import (  # noqa: E402
    AMCL_MAP_ODOM_TRANSFORM_TOLERANCE_SEC,
    evaluate_scan_tf_fallback_age,
    evaluate_transform_age,
    should_log_scan_tf_fallback,
)


class TransformAgeTest(unittest.TestCase):
    def test_accepted_fallback_events_are_rate_limited_and_state_changes_log(self):
        self.assertTrue(
            should_log_scan_tf_fallback(True, 10.0, None, None)
        )
        self.assertFalse(
            should_log_scan_tf_fallback(True, 10.5, True, 10.0)
        )
        self.assertTrue(
            should_log_scan_tf_fallback(True, 10.0, False, 9.9)
        )
        self.assertTrue(
            should_log_scan_tf_fallback(True, 11.0, True, 10.0)
        )
        self.assertTrue(
            should_log_scan_tf_fallback(False, 10.1, True, 10.0)
        )

    def test_scan_and_latest_tf_from_recorded_failure_fit_bounded_window(self):
        accepted, scan_age, tf_age, skew, reason = evaluate_scan_tf_fallback_age(
            1_070_549_000_000,
            1_070_327_000_000,
            1_070_324_000_000,
            0.35,
        )

        self.assertTrue(accepted, reason)
        self.assertAlmostEqual(scan_age, 0.222, places=9)
        self.assertAlmostEqual(tf_age, 0.225, places=9)
        self.assertAlmostEqual(skew, 0.003, places=9)

    def test_latest_tf_stale_beyond_window_is_rejected(self):
        accepted, _, tf_age, _, reason = evaluate_scan_tf_fallback_age(
            10_000_000_000,
            9_900_000_000,
            9_649_000_000,
            0.35,
        )

        self.assertFalse(accepted)
        self.assertAlmostEqual(tf_age, 0.351, places=9)
        self.assertEqual(reason, "transform_stale")

    def test_scan_tf_skew_is_bounded_even_when_both_stamps_are_near_now(self):
        accepted, _, _, skew, reason = evaluate_scan_tf_fallback_age(
            10_000_000_000,
            10_250_000_000,
            9_750_000_000,
            0.35,
        )

        self.assertFalse(accepted)
        self.assertAlmostEqual(skew, 0.5, places=9)
        self.assertEqual(reason, "scan_transform_skew")

    def test_zero_and_nonfinite_fallback_stamps_fail_closed(self):
        for scan_stamp_ns, tf_stamp_ns in (
            (0, 9_900_000_000),
            (9_900_000_000, 0),
            (float("nan"), 9_900_000_000),
            (9_900_000_000, float("inf")),
        ):
            with self.subTest(scan_stamp_ns=scan_stamp_ns, tf_stamp_ns=tf_stamp_ns):
                accepted, _, _, _, _ = evaluate_scan_tf_fallback_age(
                    10_000_000_000,
                    scan_stamp_ns,
                    tf_stamp_ns,
                    0.35,
                )
                self.assertFalse(accepted)

    def test_recorded_future_skew_samples_are_accepted_with_amcl_correction(self):
        now_ns = 381_049_000_000
        max_age_sec = 0.35

        cases = (
            # map->base exact-now lookup returned a latest header 71 ms ahead.
            (now_ns + 71_000_000, 0.0, -0.071),
            # map->odom is future-dated by AMCL's configured 1.0 s tolerance.
            (now_ns + 1_008_000_000, 1.0, -0.008),
            # Second recorded event: AMCL-corrected map->odom age is -117 ms.
            (now_ns + 1_117_000_000, 1.0, -0.117),
            # Second recorded event: odom->base age is -133 ms.
            (now_ns + 133_000_000, 0.0, -0.133),
        )
        for stamp_ns, offset_sec, expected_age_sec in cases:
            with self.subTest(expected_age_sec=expected_age_sec):
                accepted, age_sec, reason = evaluate_transform_age(
                    now_ns,
                    stamp_ns,
                    max_age_sec,
                    age_offset_sec=offset_sec,
                )
                self.assertTrue(accepted, reason)
                self.assertAlmostEqual(age_sec, expected_age_sec, places=9)

    def test_expired_transform_is_rejected(self):
        accepted, age_sec, reason = evaluate_transform_age(
            1_000_000_000,
            649_000_000,
            0.35,
        )
        self.assertFalse(accepted)
        self.assertAlmostEqual(age_sec, 0.351, places=9)
        self.assertEqual(reason, "stale")

    def test_missing_and_nonfinite_timestamps_fail_closed(self):
        for now_ns, stamp_ns in (
            (None, 1_000_000_000),
            (1_000_000_000, None),
            (float("nan"), 1_000_000_000),
            (1_000_000_000, float("inf")),
        ):
            with self.subTest(now_ns=now_ns, stamp_ns=stamp_ns):
                accepted, _, _ = evaluate_transform_age(now_ns, stamp_ns, 0.35)
                self.assertFalse(accepted)

        for max_age, age_offset in (
            (float("nan"), 0.0),
            (0.35, float("inf")),
        ):
            with self.subTest(max_age=max_age, age_offset=age_offset):
                accepted, _, reason = evaluate_transform_age(
                    1_000_000_000,
                    1_000_000_000,
                    max_age,
                    age_offset_sec=age_offset,
                )
                self.assertFalse(accepted)
                self.assertEqual(reason, "invalid_age_configuration")

    def test_future_stamp_beyond_finite_window_is_rejected(self):
        accepted, age_sec, reason = evaluate_transform_age(
            1_000_000_000,
            1_351_000_000,
            0.35,
        )
        self.assertFalse(accepted)
        self.assertAlmostEqual(age_sec, -0.351, places=9)
        self.assertEqual(reason, "future")

    def test_wall_and_sim_clock_mixes_fail_closed_in_both_directions(self):
        sim_now_ns = 381_049_000_000
        wall_now_ns = 1_790_000_000_000_000_000
        cases = (
            (sim_now_ns, wall_now_ns),
            (wall_now_ns, sim_now_ns),
        )
        for now_ns, stamp_ns in cases:
            with self.subTest(now_ns=now_ns, stamp_ns=stamp_ns):
                accepted, _, reason = evaluate_transform_age(
                    now_ns,
                    stamp_ns,
                    0.35,
                )
                self.assertFalse(accepted)
                self.assertEqual(reason, "clock_domain_or_large_jump")

    def test_amcl_tolerance_matches_default_nav2_yaml_contract(self):
        yzbot = HERE.parents[1]
        nav_yaml = yzbot / "bot_navigation" / "param" / "originbot_nav2_2.yaml"
        launch_py = yzbot / "bot_navigation" / "launch" / "nav_bringup_gazebo2.launch.py"
        yaml_text = nav_yaml.read_text(encoding="utf-8")
        launch_text = launch_py.read_text(encoding="utf-8")

        self.assertRegex(
            launch_text,
            r'(?s)DeclareLaunchArgument\(\s*"params_file",\s*'
            r'default_value=os\.path\.join\(\s*navigation_dir,\s*'
            r'"param",\s*"originbot_nav2_2\.yaml"\s*\)\s*,',
        )
        amcl_section = re.search(
            r"(?ms)^amcl:\s*\n(?P<section>(?:[ \t]+.*\n)*)",
            yaml_text,
        )
        self.assertIsNotNone(amcl_section)
        tolerance = re.search(
            r"(?m)^    transform_tolerance:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$",
            amcl_section.group("section"),
        )
        self.assertIsNotNone(tolerance)
        self.assertEqual(
            float(tolerance.group(1)),
            AMCL_MAP_ODOM_TRANSFORM_TOLERANCE_SEC,
        )


if __name__ == "__main__":
    unittest.main()
