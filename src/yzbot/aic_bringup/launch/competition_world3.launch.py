"""Competition World3: Gazebo, static-map Nav2 and dynamic obstacles."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    mybot_share = FindPackageShare("mybot").find("mybot")
    bot_nav_share = FindPackageShare("bot_navigation").find("bot_navigation")

    # World3 Gazebo
    world3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mybot_share, "launch", "gazebo_world3.launch.py")
        )
    )

    # Nav2 for World3 (uses same map as World2 for now; new map needed if geometry changes)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_nav_share, "launch", "nav_bringup_gazebo2.launch.py")
        )
    )

    # Dynamic obstacle controller
    dyn_obs_node = Node(
        package="aic_task",
        executable="dynamic_obstacle",
        name="dynamic_obstacle_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    tf_bridge_node = Node(
        package="aic_task",
        executable="tf_bridge",
        name="tf_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "initial_pose_x": 0.0,
                "initial_pose_y": 0.0,
                "initial_pose_yaw": 0.0,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(world3_launch)
    ld.add_action(TimerAction(period=25.0, actions=[nav2_launch]))
    ld.add_action(TimerAction(period=35.0, actions=[dyn_obs_node]))
    ld.add_action(TimerAction(period=35.0, actions=[tf_bridge_node]))
    return ld
