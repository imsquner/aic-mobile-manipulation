#!/usr/bin/env python3
"""Launch the perception-to-grasp pipeline with motion disabled by default."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("aic_perception"))
    allow_motion = LaunchConfiguration("allow_motion")
    allow_attach = LaunchConfiguration("allow_attach")
    allow_detach = LaunchConfiguration("allow_detach")
    cube_model = LaunchConfiguration("cube_model")
    candidates_config = LaunchConfiguration("candidates_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_motion", default_value="false"),
            DeclareLaunchArgument("allow_attach", default_value="false"),
            DeclareLaunchArgument("allow_detach", default_value="false"),
            DeclareLaunchArgument("cube_model", default_value=""),
            DeclareLaunchArgument(
                "candidates_config",
                default_value=str(share / "config" / "world2_points.yaml"),
            ),
            Node(
                package="aic_perception",
                executable="color_detector",
                name="color_depth_detector",
                parameters=[{"debug_mode": False, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="aic_perception",
                executable="target_selector",
                name="target_selector",
                parameters=[
                    {
                        "use_sim_time": True,
                        "candidates_config": candidates_config,
                    }
                ],
                output="screen",
            ),
            Node(
                package="aic_perception",
                executable="pregrasp_validator",
                name="pregrasp_validator",
                parameters=[str(share / "config" / "pregrasp_validator.yaml")],
                output="screen",
            ),
            Node(
                package="aic_perception",
                executable="grasp_executor",
                name="grasp_executor",
                parameters=[
                    str(share / "config" / "grasp_executor.yaml"),
                    {
                        "allow_motion": allow_motion,
                        "allow_attach": allow_attach,
                        "allow_detach": allow_detach,
                        "cube_model": cube_model,
                    },
                ],
                output="screen",
            ),
            Node(
                package="aic_perception",
                executable="pick_sequence_orchestrator",
                name="pick_sequence_orchestrator",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
