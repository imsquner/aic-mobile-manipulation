import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


YZBOT_SRC = Path(__file__).resolve().parents[2]
LAUNCH_FILE = (
    YZBOT_SRC / "aic_bringup" / "launch" / "competition_world2.launch.py"
)


class FakeLaunchDescription:
    def __init__(self, actions=None):
        self.actions = list(actions or [])

    def add_action(self, action):
        self.actions.append(action)


class FakeLogError:
    def __init__(self, *, msg):
        self.msg = msg


class FakeShutdown:
    def __init__(self, *, reason):
        self.reason = reason


class FakeEmitEvent:
    def __init__(self, *, event):
        self.event = event


class FakeAction:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class FakePackageShare:
    def __init__(self, package):
        self.package = package

    def find(self, package):
        return f"/fake/share/{package}"


def _load_launch_module_with_fakes():
    launch = types.ModuleType("launch")
    launch.LaunchDescription = FakeLaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.LogError = FakeLogError
    actions.EmitEvent = FakeEmitEvent
    actions.IncludeLaunchDescription = FakeAction
    actions.TimerAction = FakeAction
    events = types.ModuleType("launch.events")
    events.Shutdown = FakeShutdown
    sources = types.ModuleType("launch.launch_description_sources")
    sources.PythonLaunchDescriptionSource = FakeAction
    launch_ros = types.ModuleType("launch_ros")
    ros_actions = types.ModuleType("launch_ros.actions")
    ros_actions.Node = FakeAction
    ros_substitutions = types.ModuleType("launch_ros.substitutions")
    ros_substitutions.FindPackageShare = FakePackageShare
    spec = importlib.util.spec_from_file_location("competition_world2_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        "launch": launch,
        "launch.actions": actions,
        "launch.events": events,
        "launch.launch_description_sources": sources,
        "launch_ros": launch_ros,
        "launch_ros.actions": ros_actions,
        "launch_ros.substitutions": ros_substitutions,
    }):
        spec.loader.exec_module(module)
        return module


class World2LaunchContractTest(unittest.TestCase):
    def test_legacy_world2_launch_fails_before_any_gazebo_or_nav2_action(self):
        module = _load_launch_module_with_fakes()
        description = module.generate_launch_description()

        self.assertEqual(len(description.actions), 2)
        log_error, stop_launch = description.actions
        self.assertIsInstance(log_error, FakeLogError)
        self.assertIsInstance(stop_launch, FakeEmitEvent)
        self.assertIsInstance(stop_launch.event, FakeShutdown)
        self.assertIn("python3 start_competition.py --world world2", log_error.msg)
        self.assertEqual(stop_launch.event.reason, log_error.msg)


if __name__ == "__main__":
    unittest.main()
