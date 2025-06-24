#!/usr/bin/env python3
"""
Accurate Collision Filter Launch File

This launch file demonstrates the AccurateCollisionFilter that only removes points
actually inside the robot's collision geometry, without overly conservative filtering.

Usage:
    ros2 launch robot_body_filter accurate_collision_filter.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_body_filter')
    sk75sr_pkg_dir = get_package_share_directory('sk75sr_bringup')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'examples', 'configs', 'accurate_collision_filter.yaml'),
        description='Path to the configuration file'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(sk75sr_pkg_dir, 'urdf', 'sk75sr_with_livox.urdf.xacro'),
        description='Path to the robot URDF file (supports both .urdf and .xacro)'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar1',
        description='Input point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/filtered_pointcloud',
        description='Output filtered point cloud topic'
    )
    
    sensor_frame_arg = DeclareLaunchArgument(
        'sensor_frame',
        default_value='livox_frame',
        description='Sensor frame ID (e.g., livox_frame, laser, velodyne)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    # Robot description - use xacro for .xacro files
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )
    
    # Robot state publisher
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{
    #         'robot_description': robot_description_content,
    #         'use_sim_time': False
    #     }],
    #     output='screen'
    # )
    
    # Static transform from world to base_link
    # static_transform_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    #     output='screen'
    # )
    
    # Accurate collision filter node
    accurate_filter_node = Node(
        package='robot_body_filter',
        executable='accurate_collision_filter',
        name='accurate_collision_filter',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_description': robot_description_content,
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'use_sim_time': True
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Test data publisher (pure pointcloud output only)
    # test_data_node = Node(
    #     package='robot_body_filter',
    #     executable='simple_pointcloud_publisher',
    #     name='test_data_publisher',
    #     parameters=[{
    #         'output_topic': LaunchConfiguration('input_topic'),
    #         'frame_id': LaunchConfiguration('sensor_frame'),
    #         'publish_rate': 10.0,
    #         'num_points': 50000,
    #         'point_range': 3.0,
    #         'use_sim_time': True,
    #         'add_noise': False,
    #         'noise_std': 0.00
    #     }],
    #     output='screen',
    #     emulate_tty=True
    # )
    
    # RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'tutorial.rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        config_file_arg,
        urdf_file_arg,
        input_topic_arg,
        output_topic_arg,
        sensor_frame_arg,
        base_frame_arg,
        
        # Core nodes
        # static_transform_node,
        # robot_state_publisher_node,
        accurate_filter_node,
        # test_data_node,
        
        # Visualization
        rviz_node,
    ])