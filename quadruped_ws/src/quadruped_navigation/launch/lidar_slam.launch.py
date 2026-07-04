"""Launch LiDAR ne vat can + SLAM dung ban do.

Gazebo (co LiDAR + vat can) + gait_node + obstacle_avoider (/scan -> /cmd_vel)
+ slam_toolbox (dung /map) + RViz. Locomotion la gait rule-based (on dinh trong
Gazebo). Robot tu di quanh ne vat can, dong thoi slam_toolbox dung ban do.

Can cai truoc: sudo apt install ros-jazzy-slam-toolbox
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('quadruped_navigation')
    slam_config = os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml')

    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    # Gazebo + robot + ros2_control + gait controller (forward_position)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('quadruped_description'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )

    gait_node = Node(package='quadruped_gait', executable='gait_node', output='screen')

    obstacle_avoider = Node(
        package='quadruped_navigation', executable='obstacle_avoider', output='screen')

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[slam_config, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_nav, 'config', 'lidar_slam.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        headless_arg, rviz_arg,
        gazebo, gait_node, obstacle_avoider, slam, rviz,
    ])
