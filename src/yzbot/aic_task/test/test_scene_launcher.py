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
        no_task_parser=False, no_dynamic_obstacles=False, time_limit_sec=None, development_cube_start_index=0,
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

    def test_ros_file_parameters_remain_ascii_under_unicode_workspace(self):
        with tempfile.TemporaryDirectory(prefix="workspace_桌面_") as folder:
            root = Path(folder)
            file = root / "install/maps/map.yaml"
            file.parent.mkdir(parents=True)
            file.write_text("image: map.pgm\n")
            self.assertEqual(launcher_module.ros_file_parameter(file, root), "install/maps/map.yaml")
            non_ascii = root / "地图.yaml"
            non_ascii.touch()
            with self.assertRaisesRegex(ValueError, "ASCII"):
                launcher_module.ros_file_parameter(non_ascii, root)
            with self.assertRaisesRegex(ValueError, "workspace"):
                launcher_module.ros_file_parameter(root.parent / "outside.yaml", root)

    def test_all_runtime_file_arguments_are_ascii_relative_values(self):
        launcher = launcher_module.CompetitionLauncher(args("world2"))
        calls = []
        launcher._start_process = lambda name, command: calls.append(command)
        launcher.args.headless = True
        launcher.args.no_rviz = False
        launcher._launch_gazebo()
        launcher._launch_nav2()
        launcher._launch_perception()
        launcher._launch_task_system()
        file_keys = {"entities_file", "map", "candidates_config", "points_file", "path_matrix_file"}
        seen = set()
        for command in calls:
            for argument in command:
                if ":=" not in argument:
                    continue
                key, value = argument.split(":=", 1)
                if key in file_keys:
                    seen.add(key)
                    self.assertTrue(value.isascii(), argument)
                    self.assertFalse(Path(value).is_absolute(), argument)
                    self.assertNotIn("..", Path(value).parts, argument)
        self.assertEqual(seen, file_keys)

    def test_world2_passes_matching_entities_to_gazebo(self):
        launcher = launcher_module.CompetitionLauncher(args("world2"))
        calls = []
        launcher._start_process = lambda name, command: calls.append((name, command))
        launcher.args.headless = True
        launcher._launch_gazebo()
        self.assertEqual(launcher.scene.scene_id, "world2")
        self.assertEqual(launcher.dynamic_obstacle_names, ('moving_box_1', 'moving_box_2'))
        self.assertEqual(launcher.initial_pose[:2], (-0.884483, -11.938343))
        command = calls[0][1]
        self.assertIn("gazebo_world2.launch.py", command)
        self.assertIn(f"entities_file:={launcher_module.ros_file_parameter(launcher.scene.entities_file)}", command)

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
