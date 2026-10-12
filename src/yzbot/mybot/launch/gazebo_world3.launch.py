import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'six_arm'
    model_pkg_name = 'mybot_description'
    urdf_name = "originbot_with_rgbd_gazebo_arm.xacro"
    world_name = 'offic_room3.world'

    model_pkg_share = FindPackageShare(package=model_pkg_name).find(model_pkg_name)
    urdf_model_path = os.path.join(model_pkg_share, f'urdf/{urdf_name}')
    world_file_path = os.path.join(model_pkg_share, f'worlds/{world_name}')

    gui = LaunchConfiguration('gui')
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file_path],
        output='screen'
    )
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        condition=IfCondition(gui),
        output='screen'
    )

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen'
    )
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen'
    )

    evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_broadcaster]
        )
    )
    evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )
    evt3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller]
        )
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('gui', default_value='True'))
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_client)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(evt1)
    ld.add_action(evt2)
    ld.add_action(evt3)
    return ld
