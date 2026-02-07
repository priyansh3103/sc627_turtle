#!/usr/bin/env python3
"""
Launch file for Map Publisher with C-Space Inflation

Publishes:
- /map: Original occupancy grid (0=free, 100=occupied, -1=unknown)
- /map_inflated: Inflated occupancy grid (100=original obstacle, 99=inflated boundary)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('gauntlet_gazebo')
    
    # Default map file path
    default_map_file = os.path.join(pkg_dir, 'maps', 'gauntlet_map.yaml')
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cspace_view.rviz')
    
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    robot_radius = LaunchConfiguration('robot_radius')
    safety_margin = LaunchConfiguration('safety_margin')
    publish_rate = LaunchConfiguration('publish_rate')
    frame_id = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare launch arguments
    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_file,
        description='Full path to the map YAML file'
    )
    
    declare_robot_radius = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.105',
        description='Robot radius in meters (TurtleBot3 Burger ~0.105m)'
    )
    
    declare_safety_margin = DeclareLaunchArgument(
        'safety_margin',
        default_value='0.05',
        description='Additional safety margin in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Map publish rate in Hz'
    )
    
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the map'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Map publisher node
    map_publisher_node = Node(
        package='gauntlet_gazebo',
        executable='map_publisher.py',
        name='map_publisher',
        output='screen',
        parameters=[{
            'map_yaml_file': map_yaml_file,
            'robot_radius': robot_radius,
            'safety_margin': safety_margin,
            'publish_rate': publish_rate,
            'frame_id': frame_id,
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        declare_map_yaml_file,
        declare_robot_radius,
        declare_safety_margin,
        declare_publish_rate,
        declare_frame_id,
        declare_use_rviz,
        map_publisher_node,
        rviz_node,
    ])
