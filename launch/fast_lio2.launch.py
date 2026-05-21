#!/usr/bin/env python3
"""Launch FAST-LIO2 or compatible FAST-LIO ROS 2 front end."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FAST_LIO_PACKAGE_CANDIDATES = [
    ('spark_fast_lio', 'spark_lio_mapping'),
    ('fast_lio', 'fast_lio'),
    ('fast_lio2', 'fast_lio2'),
]


def find_fast_lio_package():
    for package_name, executable_name in FAST_LIO_PACKAGE_CANDIDATES:
        try:
            get_package_share_directory(package_name)
            return package_name, executable_name
        except PackageNotFoundError:
            continue
    return None, None


def generate_launch_description():
    launch_actions = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    package_name, executable_name = find_fast_lio_package()
    if package_name is None:
        launch_actions.append(LogInfo(
            msg=(
                'FAST-LIO2 precheck failed: missing package spark_fast_lio, fast_lio, or fast_lio2. '
                'Build a ROS 2 compatible FAST-LIO front end in this workspace, '
                'then rerun this launch file.'
            )
        ))
        return LaunchDescription(launch_actions)

    config_file = os.path.join(
        get_package_share_directory('robot_description'),
        'config',
        'fast_lio.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_actions.append(Node(
        package=package_name,
        executable=executable_name,
        name='fast_lio2',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('lidar', '/sensing/lidar/points'),
            ('imu', '/sensing/imu/data'),
            ('odometry', '/mapping/lio/odom'),
            ('cloud_registered', '/mapping/lio/map_points'),
        ],
    ))
    return LaunchDescription(launch_actions)
