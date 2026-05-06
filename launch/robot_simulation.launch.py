#!/usr/bin/env python3
"""Launch file for robot simulation with Gazebo
   4WD Skid Steer Vehicle
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    robot_description_share = get_package_share_directory('robot_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(robot_description_share, 'urdf', 'robot_base.urdf.xacro')
    rviz_config = os.path.join(robot_description_share, 'rviz', 'robot_config.rviz')
    world_file = os.path.join(robot_description_share, 'worlds', 'corridor_tunnel.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('rviz', default='true')

    # Robot spawn position
    spawn_x = LaunchConfiguration('x', default='-5.0')
    spawn_y = LaunchConfiguration('y', default='0.0')
    spawn_z = LaunchConfiguration('z', default='0.07')

    # Set LD_LIBRARY_PATH for velodyne plugin dependencies
    ld_lib_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value='/opt/ros/humble/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
    )

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
            condition=IfCondition(use_rviz),
        ),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'factory': 'true',
            'init': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ackermann_robot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='-5.0',
            description='Robot spawn X position'),
        DeclareLaunchArgument('y', default_value='0.0',
            description='Robot spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.07',
            description='Robot spawn Z position'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='Start RViz with the simulation'),
        ld_lib_path,
        gazebo,
        TimerAction(period=2.0, actions=nodes),
        TimerAction(period=4.0, actions=[spawn_robot]),
    ])
