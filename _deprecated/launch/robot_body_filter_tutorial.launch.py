#!/usr/bin/env python3
"""
Robot Body Filter Tutorial Launch File

This launch file demonstrates the RobotBodyFilterProcessor with the full_example.urdf robot model.
It sets up:
1. Robot description parameter server
2. Robot state publisher  
3. Robot body filter tutorial node
4. RViz for visualization

Usage:
    ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_body_filter')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'examples', 'configs', 'robot_body_filter_tutorial.yaml'),
        description='Path to the configuration file'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_dir, 'examples', 'meshes', 'full_example.urdf'),
        description='Path to the robot URDF file'
    )
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Static transform from world to base_link
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )
    
    # Robot body filter tutorial node
    tutorial_node = Node(
        package='robot_body_filter',
        executable='robot_body_filter_tutorial',
        name='robot_body_filter_tutorial',
        arguments=['--ros-args', '--params-file', LaunchConfiguration('config_file')],
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
        emulate_tty=True
    )
    
    # RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'tutorial.rviz')
    
    # Create RViz config if it doesn't exist
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
        
        # Core nodes
        static_transform_node,
        robot_state_publisher_node,
        tutorial_node,
        
        # Visualization
        rviz_node,
    ])