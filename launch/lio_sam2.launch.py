#!/usr/bin/env python3
"""Launch file for LIO-SAM2 with robot simulation"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'lio_sam2'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Config file
    config_file = os.path.join(
        get_package_share_directory('robot_description'),
        '..', '..', 'config', 'lio_sam.yaml'
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
                ('/points_raw', '/robot/velodyne_points'),
                ('/imu_raw', '/robot/imu/data'),
                ('/odom_encoded', '/robot/odom'),
            ],
        ),

        # Transform publisher (map -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Transform publisher (odom -> base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
