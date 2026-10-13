import argparse
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))
SPEC = importlib.util.spec_from_file_location(
    "start_competition", WORKSPACE / "src" / "yzbot" / "start_competition.py"
)
launcher_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher_module)


def args(world):
    return argparse.Namespace(
        mode="competition", world=world, display=":0", ros_domain_id=42,
        initial_pose_x=0.0, initial_pose_y=0.0, initial_pose_yaw=0.0,
        dynamic_obstacle_names="all",
        no_task_parser=False, time_limit_sec=None, development_cube_start_index=0,
    )


class LauncherSceneTest(unittest.TestCase):
    def setUp(self):
        self.old_workspace = launcher_module.WORKSPACE
        self.old_scenes = launcher_module.SCENES_ROOT
        launcher_module.WORKSPACE = WORKSPACE
        launcher_module.SCENES_ROOT = PACKAGE_ROOT / "config" / "scenes"

    def tearDown(self):
        launcher_module.WORKSPACE = self.old_workspace
        launcher_module.SCENES_ROOT = self.old_scenes

    def test_world2_passes_matching_entities_to_gazebo(self):
        launcher = launcher_module.CompetitionLauncher(args("world2"))
        calls = []
        launcher._start_process = lambda name, command: calls.append((name, command))
        launcher.args.headless = True
        launcher._launch_gazebo()
        self.assertEqual(launcher.scene.scene_id, "world2")
        self.assertEqual(launcher.dynamic_obstacle_names, ())
        self.assertEqual(launcher.initial_pose[:2], (-0.884483, -11.938343))
        command = calls[0][1]
        self.assertIn("gazebo_world2.launch.py", command)
        self.assertIn(f"entities_file:={launcher.scene.entities_file}", command)

    def test_auto_and_world2_orchestrator_use_sim_time(self):
        for world in ("auto", "world2"):
            with self.subTest(world=world):
                launcher = launcher_module.CompetitionLauncher(args(world))
                calls = []
                launcher._start_process = lambda name, command: calls.append((name, command))
                launcher._launch_task_system()
                orchestrator = next(
                    command for name, command in calls if name == "task_orchestrator"
                )
                self.assertIn("use_sim_time:=true", orchestrator)


    def test_reset_remains_available_when_scene_selector_is_broken(self):
        with tempfile.TemporaryDirectory() as folder:
            launcher_module.SCENES_ROOT = Path(folder)
            reset_args = args("auto")
            reset_args.reset = True
            launcher = launcher_module.CompetitionLauncher(reset_args)
            self.assertIsNone(launcher.scene)


if __name__ == "__main__":
    unittest.main()
