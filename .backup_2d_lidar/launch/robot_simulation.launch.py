#!/usr/bin/env python3
"""Launch file for robot simulation with Gazebo"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Absolute paths
    project_root = '/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim'
    urdf_file = os.path.join(project_root, 'src', 'robot_description', 'urdf', 'robot_base.urdf.xacro')
    world_file = os.path.join(project_root, 'worlds', 'office.world')
    rviz_config = os.path.join(project_root, 'rviz', 'robot_config.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process xacro
    doc = xacro.process_file(urdf_file)
    robot_desc_xml = doc.toxml()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc_xml,
                'publish_frequency': 50.0,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 50,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen',
        shell=True,
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.07',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(period=3.0, actions=nodes),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo,
                on_exit=[
                    ExecuteProcess(
                        cmd=['killall', 'gzserver', 'gzclient'],
                        output='screen',
                        shell=True,
                    )
                ]
            )
        ),
    ])
