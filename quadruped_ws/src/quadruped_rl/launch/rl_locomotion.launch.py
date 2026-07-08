"""Launch RL locomotion: Gazebo + Go2 dieu khien bang policy ONNX (effort/PD).

Startup hand-off (tranh robot nga luc khoi dong):
  1. forward_position_controller ACTIVE giu robot dung (gz giu initial_value).
  2. joint_effort_controller nap san nhung INACTIVE.
  3. rl_policy_node: giu tu the default -> khi on dinh, goi switch_controller
     (tat position, bat effort) roi chay policy.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quadruped_description'),
                'launch', 'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            # forward_position_controller giu robot dung trong luc khoi dong
            'locomotion_controller': 'forward_position_controller',
        }.items(),
    )

    # Nap joint_effort_controller nhung INACTIVE (node se kich hoat sau khi on dinh).
    # Delay de chac controller_manager da san sang (sau khi gz + robot spawn).
    effort_controller_loader = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_effort_controller', '--inactive'],
            output='screen',
        )],
    )

    rl_policy_node = Node(
        package='quadruped_rl',
        executable='rl_policy_node',
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        gazebo_launch,
        effort_controller_loader,
        rl_policy_node,
    ])
