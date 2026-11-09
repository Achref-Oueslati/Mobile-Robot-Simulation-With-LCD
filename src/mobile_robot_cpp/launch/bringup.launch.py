import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mobile_robot_cpp'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'model', 'robot.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'differential_drive_robot', '-topic', 'robot_description'],
        output='screen',
    )

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Gazebo bridge
    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'parameters', 'bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # RViz
    rviz_cfg = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'rviz_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    # Static TF map->odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom'],
        output='screen'
    )

    # LCD node (C++ executable)
    lcd = Node(
        package=pkg_name,
        executable='lcd_display_node',
        name='lcd_display_node',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyACM0', 'baudrate': 115200, 'update_period': 0.5}]
    )

    # Static TF lidar_link -> gpu_lidar (Gazebo sensor frame)
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','lidar_link','differential_drive_robot/base_footprint/gpu_lidar'],
        output='screen'
    )

    return LaunchDescription([gz_launch, spawn, rsp, bridge, rviz, static_tf, lcd, static_lidar_tf])

