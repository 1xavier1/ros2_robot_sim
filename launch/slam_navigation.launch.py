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
    map_align_x = LaunchConfiguration('map_align_x', default='0.0')
    map_align_y = LaunchConfiguration('map_align_y', default='0.0')
    map_align_yaw = LaunchConfiguration('map_align_yaw', default='0.0')
    gps_anchor_blend_weight = LaunchConfiguration('gps_anchor_blend_weight', default='0.0')
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
            'map_align_x': map_align_x,
            'map_align_y': map_align_y,
            'map_align_yaw': map_align_yaw,
            'gps_anchor_blend_weight': gps_anchor_blend_weight,
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
        DeclareLaunchArgument('map_align_x',
                              default_value='0.0',
                              description='Static x offset from localization map frame to saved map frame.'),
        DeclareLaunchArgument('map_align_y',
                              default_value='0.0',
                              description='Static y offset from localization map frame to saved map frame.'),
        DeclareLaunchArgument('map_align_yaw',
                              default_value='0.0',
                              description='Static yaw offset from localization map frame to saved map frame, radians.'),
        DeclareLaunchArgument('gps_anchor_blend_weight',
                              default_value='0.0',
                              description='GPS anchor blend weight for global localization backend.'),
        simulation,
        TimerAction(period=8.0, actions=[fast_lio]),
        TimerAction(period=14.0, actions=[navigation]),
    ])
