#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for path visualization'
    )

    planner_node = Node(
        package='gauntlet_gazebo',
        executable='planner_server.py',
        name='planner_server',
        output='screen'
    )


    return LaunchDescription([
        declare_use_rviz,
        planner_node
    ])
