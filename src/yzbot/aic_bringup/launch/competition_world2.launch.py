#!/usr/bin/env python3
"""Reject the incomplete legacy World2 entry point before starting any process."""

from launch import LaunchDescription
from launch.actions import EmitEvent, LogError
from launch.events import Shutdown


UNSAFE_ENTRY_MESSAGE = (
    "Direct competition_world2.launch.py is disabled because it skips validated "
    "scene, entity, points, and path-matrix configuration. Use "
    "`python3 start_competition.py --world world2` from dev_ws/src/yzbot."
)


def generate_launch_description():
    """Fail before Gazebo/Nav2 start; use the scene-aware competition launcher."""
    return LaunchDescription([
        LogError(msg=UNSAFE_ENTRY_MESSAGE),
        EmitEvent(event=Shutdown(reason=UNSAFE_ENTRY_MESSAGE)),
    ])
