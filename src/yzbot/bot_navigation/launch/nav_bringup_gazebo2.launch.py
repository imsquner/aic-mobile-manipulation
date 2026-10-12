#!/usr/bin/env python3
"""Static-map Nav2 bringup for the competition Gazebo worlds."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile


def generate_launch_description():
    navigation_dir = get_package_share_directory("bot_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_path = LaunchConfiguration("map")
    params_path = LaunchConfiguration("params_file")
    use_slam = LaunchConfiguration("slam")
    use_rviz = LaunchConfiguration("rviz")

    # Override applied to every node of this launch, including the lifecycle
    # managers that nav2_bringup creates internally.
    #
    # `bond_timeout` must be set at construction time: nav2_lifecycle_manager reads
    # it once in its constructor and caches it in a member, so a runtime
    # `ros2 param set` has no effect (verified on the VM: the parameter reads back
    # as 30.0 while the CRITICAL FAILURE message still reports 4000 ms).
    # At Nav2's 4 s default this loaded VM drops the bond heartbeat and shuts down
    # every managed node, which silently kills the velocity chain
    # (velocity_smoother is the only subscriber of /cmd_vel_nav).
    # Rationale and trade-offs: param/competition_global_overrides.yaml
    global_param_overrides = SetParametersFromFile(
        os.path.join(navigation_dir, "param", "competition_global_overrides.yaml")
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml_path,
            "use_sim_time": use_sim_time,
            "params_file": params_path,
            "slam": use_slam,
            "autostart": "true",
            # Component loading is unreliable with this VM's Fast DDS setup.
            # Standalone Nav2 processes preserve lifecycle visibility and logs.
            "use_composition": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[params_path, {"use_sim_time": use_sim_time}],
    )

    collision_monitor_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["collision_monitor"]},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    navigation_dir, "param", "originbot_nav2_2.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    navigation_dir, "maps", "mapn3.yaml"
                ),
            ),
            # Nav2 Humble evaluates `not <slam>` as a PythonExpression, so use
            # Python boolean spelling rather than lowercase YAML spelling.
            DeclareLaunchArgument("slam", default_value="False"),
            DeclareLaunchArgument("rviz", default_value="True"),
            # Must run before any node is created so the override reaches all of them.
            global_param_overrides,
            nav2_launch,
            collision_monitor,
            collision_monitor_lifecycle,
            rviz,
        ]
    )
