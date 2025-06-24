#!/usr/bin/env python3
"""
Simple Point Cloud Publisher Launch File

This launch file starts a simple point cloud publisher that generates
synthetic point cloud data for testing collision filters.

Usage:
    ros2 launch robot_body_filter simple_pointcloud_publisher.launch.py
"""

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
        default_value=os.path.join(pkg_dir, 'examples', 'configs', 'simple_pointcloud_publisher.yaml'),
        description='Path to the configuration file'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/raw_pointcloud',
        description='Output point cloud topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for the point cloud'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    num_points_arg = DeclareLaunchArgument(
        'num_points',
        default_value='50000',
        description='Number of points per cloud'
    )
    
    # Simple point cloud publisher node
    pointcloud_publisher_node = Node(
        package='robot_body_filter',
        executable='simple_pointcloud_publisher',
        name='simple_pointcloud_publisher',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'num_points': LaunchConfiguration('num_points')
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        output_topic_arg,
        frame_id_arg,
        publish_rate_arg,
        num_points_arg,
        
        # Nodes
        pointcloud_publisher_node,
    ])