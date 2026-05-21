#!/usr/bin/env python3
"""Launch file for Navigation2 with robot simulation"""

import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REQUIRED_NAV2_PACKAGES = [
    'nav2_lifecycle_manager',
    'nav2_controller',
    'nav2_planner',
    'nav2_smoother',
    'nav2_behaviors',
    'nav2_bt_navigator',
    'nav2_waypoint_follower',
]

OPTIONAL_NAV2_PACKAGES = [
    'nav2_velocity_smoother',
]


def find_missing_packages(package_names):
    missing_packages = []
    for package_name in package_names:
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            missing_packages.append(package_name)
    return missing_packages


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    missing_required = find_missing_packages(REQUIRED_NAV2_PACKAGES)

    config_file = os.path.join(pkg_share, 'config', 'navigation.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    launch_actions = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    if missing_required:
        missing_text = ', '.join(missing_required)
        launch_actions.append(LogInfo(
            msg=(
                'Navigation2 precheck failed: missing required packages: '
                f'{missing_text}. Install ros-humble-navigation2 and '
                'ros-humble-nav2-bringup, then rerun this launch file.'
            )
        ))
        return LaunchDescription(launch_actions)

    missing_optional = find_missing_packages(OPTIONAL_NAV2_PACKAGES)
    node_names = [
        'map_server',
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    nodes = [
        # Lifecycle manager for navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': node_names,
            }],
        ),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/control/cmd_vel'),
                ('/control/cmd_vel', '/robot/cmd_vel'),
            ],
        ),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),

        # Smoother server
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),

        # Behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/goal_pose', '/goal_pose'),
            ],
        ),

        # Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),

    ]

    if not missing_optional:
        nodes.append(
            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[config_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/cmd_vel_raw', '/control/cmd_vel'),
                    ('/cmd_vel_smooth', '/robot/cmd_vel'),
                ],
            )
        )
    else:
        nodes.append(LogInfo(
            msg=(
                'Navigation2 optional package missing, skipping velocity '
                f'smoother: {", ".join(missing_optional)}'
            )
        ))

    # Map server — serves the saved occupancy grid map
    map_yaml = os.path.join(pkg_share, '..', '..', '..', '..', 'maps', 'barn_corridor_sim_001.yaml')
    if not os.path.exists(map_yaml):
        map_yaml = os.path.join(pkg_share, 'maps', 'barn_corridor_sim_001.yaml')
    nodes.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    ))

    # TF adapter: publishes map->odom from FAST-LIO + wheel odom
    nodes.append(Node(
        package='robot_description',
        executable='lio_tf_adapter.py',
        name='lio_tf_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    launch_actions.extend(nodes)
    return LaunchDescription(launch_actions)
