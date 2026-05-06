#!/usr/bin/env python3
"""Launch file for Navigation2 with robot simulation"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    config_file = os.path.join(pkg_share, 'config', 'navigation.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # RViz config path
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

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
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
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
                ('/cmd_vel', '/robot/cmd_vel'),
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

        # Velocity smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel_raw', '/robot/cmd_vel'),
                ('/cmd_vel_smooth', '/robot/cmd_vel'),
            ],
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
