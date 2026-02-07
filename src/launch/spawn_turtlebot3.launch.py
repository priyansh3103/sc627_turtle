#!/usr/bin/env python3
#
# Spawn TurtleBot3 in Gauntlet World (Gazebo Classic)
# Compatible with Gazebo 11 + ros2_control
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # TurtleBot3 model
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Use URDF (NOT SDF) for ros2_control
    urdf_path = os.path.join(
        turtlebot3_gazebo_dir,
        'urdf',
        f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    )

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # Declare launch arguments
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position of the robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position of the robot'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial Z position of the robot'
    )

    # Spawn TurtleBot3 into Gazebo Classic
    spawn_turtlebot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', f'turtlebot3_{TURTLEBOT3_MODEL}',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        spawn_turtlebot3,
    ])
