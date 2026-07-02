"""Launch tong: Gazebo + Go2 + ros2_control + gait_node.

Teleop KHONG chay tu day (can ban phim tuong tac) - mo terminal rieng:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quadruped_description'),
                'launch', 'gazebo.launch.py',
            )
        )
    )

    gait_node = Node(
        package='quadruped_gait',
        executable='gait_node',
        output='screen',
    )

    goto_point_server = Node(
        package='quadruped_navigation',
        executable='goto_point_server',
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        gait_node,
        goto_point_server,
    ])
