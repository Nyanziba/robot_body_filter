#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_body_filter')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'examples', 'configs', 'plugin_config.yaml'),
        description='Path to the config file to load'
    )
    
    # Node configuration
    lidar_processor_example_node = Node(
        package='robot_body_filter',
        executable='lidar_processor_example',
        name='lidar_processor_example',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('input_cloud', '/points_raw'),
            ('input_scan', '/scan'),
            ('output_cloud', '/processed_cloud'),
            ('output_scan', '/processed_scan'),
        ],
        output='screen'
    )
    
    # Test data publisher
    test_pointcloud_publisher_node = Node(
        package='robot_body_filter',
        executable='test_pointcloud_publisher',
        name='test_pointcloud_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        lidar_processor_example_node,
        test_pointcloud_publisher_node,
    ])