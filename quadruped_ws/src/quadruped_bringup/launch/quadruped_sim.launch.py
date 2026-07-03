"""Launch tong: Gazebo + Go2 + ros2_control + gait_node.

Teleop KHONG chay tu day (can ban phim tuong tac) - mo terminal rieng:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # headless:=true can khi dung camera/track_object (xem CLAUDE.md - GUI va
    # sensor render tranh context tren GPU hybrid, lam anh camera ra mau xam).
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quadruped_description'),
                'launch', 'gazebo.launch.py',
            )
        ),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
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

    object_detector = Node(
        package='quadruped_perception',
        executable='object_detector',
        output='screen',
    )

    tracker = Node(
        package='quadruped_perception',
        executable='tracker',
        output='screen',
    )

    target_pose_node = Node(
        package='quadruped_perception',
        executable='target_pose_node',
        output='screen',
    )

    track_object_server = Node(
        package='quadruped_follow',
        executable='track_object_server',
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        gazebo_launch,
        gait_node,
        goto_point_server,
        object_detector,
        tracker,
        target_pose_node,
        track_object_server,
    ])
