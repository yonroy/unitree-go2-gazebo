"""Spawn Go2 vao Gazebo (world phang trong) + robot_state_publisher + ros2_control.

gz_ros2_control chay ngay ben trong tien trinh gz sim (nap qua <ros2_control> +
<plugin gz_ros2_control> trong xacro), nen khong can node ros2_control_node rieng -
chi can spawner goi service cua controller_manager do plugin tao ra.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = get_package_share_directory('quadruped_description')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('quadruped_description'), 'xacro', 'robot.xacro']
    )
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    world_file = os.path.join(pkg_description, 'worlds', 'flat_ground.sdf')

    # gz sim can GZ_SIM_RESOURCE_PATH tro toi thu muc CHUA "quadruped_description/"
    # de resolve duoc "model://quadruped_description/meshes/*.dae" trong xacro.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_description)
    )

    # Tren may GPU hybrid (Intel iGPU + NVIDIA dGPU), cua so GUI tuong tac va render
    # offscreen cua sensor (camera/rgbd) TRANH nhau context render -> anh camera ra
    # mau xam dong nhat (da kiem chung thuc te). Dung headless:=true (khong GUI,
    # -s --headless-rendering) de camera/track_object render dung. Xem CLAUDE.md.
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = chay khong GUI (-s --headless-rendering), can cho camera render dung tren may GPU hybrid',
    )
    headless = LaunchConfiguration('headless')

    locomotion_controller_arg = DeclareLaunchArgument(
        'locomotion_controller', default_value='forward_position_controller',
        description='Controller khop: forward_position_controller (gait) hoac joint_effort_controller (RL)',
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-s -r --headless-rendering {world_file}'}.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'go2',
            # trunk = hip_z (hip_offset_z=0) => day chan cham dat khi
            # trunk_z = stance_height (0.30) + foot_radius (0.02) = 0.32
            '-z', '0.33',
        ],
        output='screen',
    )

    # /imu/data: can cho heading-hold trong gait_node (bu drift yaw khi di thang).
    # /odom: ground-truth odometry, can cho quadruped_navigation/goto_point_server.
    # /camera/*: rgbd_camera sensor (xem gazebo.xacro), can cho quadruped_perception.
    # /model/target_ball/cmd_vel: dieu khien van toc qua cau test (VelocityControl
    # plugin trong worlds/flat_ground.sdf), 1 chieu ROS->GZ, dung khi test bam duoi
    # muc tieu di chuyen.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/target_ball/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # /scan: LiDAR 2D (gpu_lidar) cho ne vat can + SLAM (slam_toolbox).
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Controller dieu khien khop: mac dinh forward_position_controller (gait
    # rule-based). RL locomotion truyen locomotion_controller:=joint_effort_controller.
    locomotion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[LaunchConfiguration('locomotion_controller')],
        output='screen',
    )

    # gz_ros2_control's controller_manager chi ton tai SAU khi entity da spawn
    # trong gz sim -> xep hang tuan tu bang event handler, tranh race condition.
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delayed_locomotion_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[locomotion_controller_spawner],
        )
    )

    return LaunchDescription([
        headless_arg,
        locomotion_controller_arg,
        resource_path,
        gz_sim_gui,
        gz_sim_headless,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster,
        delayed_locomotion_controller,
    ])
