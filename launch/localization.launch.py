#!/usr/bin/env python3
"""Launch robot_localization EKF for the simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    config_file = os.path.join(pkg_share, 'config', 'localization.yaml')
    mode_config_file = os.path.join(pkg_share, 'config', 'localization_modes.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_description',
            executable='localization_mode_manager.py',
            name='localization_mode_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_file': mode_config_file,
            }],
        ),
        Node(
            package='robot_description',
            executable='lio_wheel_fusion.py',
            name='lio_wheel_fusion',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_description',
            executable='global_localization_backend.py',
            name='global_localization_backend',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
