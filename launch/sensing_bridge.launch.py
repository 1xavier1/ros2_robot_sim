#!/usr/bin/env python3
"""Relay simulation sensor topics to stable /sensing interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SENSING_REMAPS = [
    ('/robot/velodyne_points', '/sensing/lidar/points'),
    ('/robot/imu/data', '/sensing/imu/data'),
    ('/robot/wheel_encoder/rear_average', '/sensing/wheel/speed'),
    ('/robot/rtk_gps/fix', '/sensing/gps/fix'),
]

SENSING_RELAYS = [
    ('relay_lidar_points', 'sensor_msgs/msg/PointCloud2'),
    ('relay_imu_data', 'sensor_msgs/msg/Imu'),
    ('relay_wheel_speed', 'std_msgs/msg/Float64'),
    ('relay_gps_fix', 'sensor_msgs/msg/NavSatFix'),
]


def relay_node(name, input_topic, output_topic, message_type):
    return Node(
        package='robot_description',
        executable='sensing_relay.py',
        name=name,
        output='screen',
        arguments=[
            '--input-topic', input_topic,
            '--output-topic', output_topic,
            '--message-type', message_type,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *[
            relay_node(name, input_topic, output_topic, message_type)
            for (name, message_type), (input_topic, output_topic)
            in zip(SENSING_RELAYS, SENSING_REMAPS)
        ],
    ])
