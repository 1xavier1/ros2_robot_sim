#!/usr/bin/env python3
"""Launch simulation, FAST-LIO, and saved-map Nav2 in a stable order."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_share, filename, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename)),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz = LaunchConfiguration("rviz", default="true")
    gui = LaunchConfiguration("gui", default="true")
    loop_closure = LaunchConfiguration('loop_closure', default='false')
    default_map_yaml = os.path.join(pkg_share, '..', '..', '..', '..', 'maps', 'barn_corridor_sim_001.yaml')
    if not os.path.exists(default_map_yaml):
        default_map_yaml = os.path.join(pkg_share, 'maps', 'barn_corridor_sim_001.yaml')
    map_yaml = LaunchConfiguration('map', default=default_map_yaml)

    simulation = include_launch(
        pkg_share,
        "robot_simulation.launch.py",
        {
            "use_sim_time": use_sim_time,
            "rviz": rviz,
            'gui': gui,
            'sensing_bridge': 'true',
        },
    )
    fast_lio = include_launch(
        pkg_share,
        "fast_lio2.launch.py",
        {"use_sim_time": use_sim_time},
    )
    navigation = include_launch(
        pkg_share,
        "navigation.launch.py",
        {
            "use_sim_time": use_sim_time,
            'map': map_yaml,
            'loop_closure': loop_closure,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value="true"),
        DeclareLaunchArgument('rviz', default_value="true"),
        DeclareLaunchArgument('gui', default_value="true"),
        DeclareLaunchArgument('map',
                              default_value=default_map_yaml,
                              description='Occupancy grid YAML loaded by Nav2 map_server.'),
        DeclareLaunchArgument('loop_closure',
                              default_value='false',
                              description='Enable conservative odom-proximity loop correction.'),
        simulation,
        TimerAction(period=8.0, actions=[fast_lio]),
        TimerAction(period=14.0, actions=[navigation]),
    ])
