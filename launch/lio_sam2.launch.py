#!/usr/bin/env python3
"""Launch file for LIO-SAM2 with robot simulation"""

import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'lio_sam2'
    launch_actions = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    try:
        get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        launch_actions.append(LogInfo(
            msg=(
                'LIO-SAM2 precheck failed: missing package lio_sam2. '
                'Install or build lio_sam2 in this workspace, then rerun '
                'this launch file.'
            )
        ))
        return LaunchDescription(launch_actions)

    # Config file
    config_file = os.path.join(
        get_package_share_directory('robot_description'),
        'config', 'lio_sam.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nodes = [
        # LIO-SAM2 Main Node
        Node(
            package='lio_sam2',
            executable='lio_sam2',
            name='lio_sam2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_file': config_file,
            }],
            remappings=[
                ('/points_raw', '/sensing/lidar/points'),
                ('/imu_raw', '/sensing/imu/data'),
                ('/odom_encoded', '/odometry/filtered'),
            ],
        ),

        # Transform publisher (map -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
    ]

    launch_actions.extend(nodes)
    return LaunchDescription(launch_actions)
