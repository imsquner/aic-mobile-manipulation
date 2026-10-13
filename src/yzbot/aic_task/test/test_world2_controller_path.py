"""Exercise real xacro and launch actions with package shares under a Unicode path."""
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import xacro
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

YZBOT = Path(__file__).resolve().parents[2]


class World2ControllerPathTest(unittest.TestCase):
    def test_gui_and_server_resolve_relative_controller_file_from_config_cwd(self):
        with tempfile.TemporaryDirectory(prefix="controller_path_桌面_") as folder:
            shares = Path(folder)
            for package in ("mybot", "mybot_description"):
                (shares / package).symlink_to(YZBOT / package, target_is_directory=True)
            spec = importlib.util.spec_from_file_location(
                "world2_controller_launch", YZBOT / "mybot/launch/gazebo_world2.launch.py"
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            descriptions = []
            real_process = xacro.process_doc

            def expand(doc, **kwargs):
                real_process(doc, **kwargs)
                descriptions.append(doc.toxml())

            with patch.object(module.FindPackageShare, "find", lambda self, package: str(shares / package)), patch(
                "ament_index_python.packages.get_package_share_directory",
                side_effect=lambda package: str(YZBOT / package),
            ), patch.object(module.xacro, "process_doc", side_effect=expand):
                description = module.generate_launch_description()
            plugin = ET.fromstring(descriptions[0]).find(".//plugin[@name='gazebo_ros2_control']")
            reference = plugin.findtext("parameters")
            self.assertEqual(reference, "ros2_controllers.yaml")
            context = LaunchContext()
            processes = [action for action in description.entities if type(action) is ExecuteProcess
                         and perform_substitutions(context, action.cmd[0]) in ("gazebo", "gzserver")]
            self.assertEqual(len(processes), 2)
            for process in processes:
                cwd = Path(perform_substitutions(context, process.cwd or []))
                self.assertEqual(cwd, shares / "mybot/config")
                self.assertEqual((cwd / reference).read_bytes(), (YZBOT / "mybot/config/ros2_controllers.yaml").read_bytes())


if __name__ == "__main__":
    unittest.main()
