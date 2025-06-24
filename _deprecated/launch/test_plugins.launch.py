#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='processor_params.yaml',
        description='Name of the config file to use'
    )
    
    # Get the path to the config file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('robot_body_filter'),
        'examples',
        LaunchConfiguration('config_file')
    ])
    
    # LiDAR processor example node
    lidar_processor_node = Node(
        package='robot_body_filter',
        executable='lidar_processor_example',
        name='lidar_processor_example',
        parameters=[config_file_path],
        output='screen',
        remappings=[
            ('input_cloud', '/points_raw'),
            ('input_scan', '/scan'),
            ('output_cloud', '/processed_cloud'),
            ('output_scan', '/processed_scan')
        ]
    )
    
    # Point cloud publisher for testing (simple test data)
    test_publisher_node = Node(
        package='robot_body_filter',
        executable='test_pointcloud_publisher',
        name='test_pointcloud_publisher',
        output='screen',
    )
    
    return LaunchDescription([
        config_file_arg,
        lidar_processor_node,
        # test_publisher_node,  # コメントアウト - 実際のデータがある場合
    ])