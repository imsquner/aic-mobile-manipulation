#!/usr/bin/env python3
"""Regression tests for the launcher's log/disc safety limits.

A 20-round run on 2026-10-14 filled the root filesystem (syslog ENOSPC at
13:54:52) and the machine went down with it. These tests pin the two guards that
were added: child logs are capped by the parent, and the disk watchdog reports
the fastest writers before it stops the stack.
"""

import importlib.util
import io
import os
import tempfile
import unittest
from pathlib import Path

LAUNCHER = Path(__file__).resolve().parent / "start_competition.py"


def load_launcher():
    spec = importlib.util.spec_from_file_location("start_competition_under_test", LAUNCHER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ChildLogLimitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launcher = load_launcher()

    def test_drain_caps_the_file_and_counts_the_rest(self):
        payload = b"L" * 500_000
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "node.log"
            drain = self.launcher._ChildLogDrain("node", path, 4096, io.BytesIO(payload))
            drain.run()
            drain.close()
            self.assertEqual(path.stat().st_size, 4096)
            self.assertEqual(drain.written, 4096)
            self.assertEqual(drain.dropped, len(payload) - 4096)
            self.assertIn("dropped", drain.summary())

    def test_zero_cap_keeps_everything_but_still_drains(self):
        payload = b"x" * 1000
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "node.log"
            drain = self.launcher._ChildLogDrain("node", path, 0, io.BytesIO(payload))
            drain.run()
            drain.close()
            self.assertEqual(path.stat().st_size, 1000)
            self.assertEqual(drain.dropped, 0)
            self.assertNotIn("dropped", drain.summary())

    def test_child_log_default_is_bounded(self):
        self.assertGreater(self.launcher.CHILD_LOG_MAX_BYTES, 0)
        self.assertLessEqual(
            self.launcher.CHILD_LOG_MAX_BYTES, 1024 * 1024 * 1024,
            "the per-node cap must stay well below a gigabyte",
        )


class DiskGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launcher = load_launcher()

    def test_top_writers_are_sorted_and_formatted(self):
        rates = {
            11: ("gzserver", 3.0e6),
            22: ("firefox", 40.0e6),
            33: ("ros2", 0.5e6),
            44: ("rviz2", 1.0e6),
        }
        text = self.launcher._DiskGuard._top(rates, count=2)
        self.assertIn("firefox(pid 22) 40.0MB/s", text)
        self.assertLess(text.index("firefox"), text.index("gzserver"),
                        "the fastest writer must be listed first")
        self.assertNotIn("rviz2", text)

    def test_free_space_floor_is_positive(self):
        self.assertGreater(self.launcher.MIN_FREE_BYTES, 0)

    def test_env_int_falls_back_on_garbage(self):
        os.environ["AIC_TEST_INT_GUARD"] = "not-a-number"
        try:
            self.assertEqual(self.launcher._env_int("AIC_TEST_INT_GUARD", 7), 7)
        finally:
            del os.environ["AIC_TEST_INT_GUARD"]


if __name__ == "__main__":
    unittest.main(verbosity=2)
