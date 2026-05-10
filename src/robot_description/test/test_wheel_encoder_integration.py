#!/usr/bin/env python3
"""Static integration checks for wheel encoder simulation wiring."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]


def read(path):
    return path.read_text(encoding="utf-8")


def plugin_child_text(plugin, child_name):
    child = plugin.find(child_name)
    assert child is not None
    return child.text


def test_wheel_encoder_plugin_is_built_and_installed():
    cmake = read(PACKAGE_DIR / "CMakeLists.txt")

    assert re.search(
        r"add_library\s*\(\s*wheel_encoder\s+SHARED\s+src/wheel_encoder.cpp",
        cmake,
        re.S,
    )
    assert "ament_target_dependencies(wheel_encoder" in cmake
    assert "gazebo_ros" in cmake
    assert "rclcpp" in cmake
    assert re.search(r"install\s*\(\s*TARGETS[^)]*wheel_encoder", cmake, re.S)


def test_urdf_loads_four_raw_wheel_encoders_and_rear_average():
    root = ET.fromstring(read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro"))
    plugins = {
        plugin.attrib.get("name"): plugin
        for plugin in root.findall("./gazebo/plugin")
        if plugin.attrib.get("filename") == "libwheel_encoder.so"
    }

    expected_plugins = {
        "wheel_encoder_front_left": {
            "joint": "front_left_joint",
            "topic": "/robot/wheel_encoder/front_left",
            "output_type": "angular_velocity",
        },
        "wheel_encoder_front_right": {
            "joint": "front_right_joint",
            "topic": "/robot/wheel_encoder/front_right",
            "output_type": "angular_velocity",
        },
        "wheel_encoder_rear_left": {
            "joint": "rear_left_joint",
            "topic": "/robot/wheel_encoder/rear_left",
            "output_type": "angular_velocity",
        },
        "wheel_encoder_rear_right": {
            "joint": "rear_right_joint",
            "topic": "/robot/wheel_encoder/rear_right",
            "output_type": "angular_velocity",
        },
        "wheel_encoder_rear_average": {
            "joint": "rear_left_joint",
            "right_joint": "rear_right_joint",
            "topic": "/robot/wheel_encoder/rear_average",
            "wheel_radius": "${wheel_radius}",
            "output_type": "linear_velocity",
        },
    }

    for plugin_name, expected_children in expected_plugins.items():
        plugin = plugins.get(plugin_name)
        assert plugin is not None
        for child_name, expected_text in expected_children.items():
            assert plugin_child_text(plugin, child_name) == expected_text


def test_lio_sam_config_exposes_recommended_rear_average_topic():
    config = read(WORKSPACE_DIR / "config" / "lio_sam.yaml")

    assert 'rear_average: "/robot/wheel_encoder/rear_average"' in config


def test_urdf_uses_unified_ackermann_drive_controller():
    urdf = read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro")

    assert "libackermann_drive_controller.so" in urdf
    assert "libgazebo_ros_diff_drive.so" not in urdf
    assert "libsteering_controller.so" not in urdf
    assert "<cmd_vel_topic>cmd_vel</cmd_vel_topic>" in urdf
    assert "<odom_topic>odom</odom_topic>" in urdf
    assert "<odom_frame>odom</odom_frame>" in urdf
    assert "<base_frame>base_footprint</base_frame>" in urdf

    assert 'left_steering_joint" type="revolute"' in urdf
    assert 'right_steering_joint" type="revolute"' in urdf


def test_launch_does_not_start_cmd_vel_bridge():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "cmd_vel_bridge.py" not in launch
    assert "cmd_vel_bridge" not in launch
    assert "rear/cmd_vel" not in launch
    assert "steering_cmd" not in launch


def test_joint_state_publisher_publishes_all_joints():
    root = ET.fromstring(read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro"))
    plugins = root.findall("./gazebo/plugin")
    joint_state_plugin = next(
        plugin for plugin in plugins
        if plugin.attrib.get("filename") == "libgazebo_ros_joint_state_publisher.so"
    )
    published_joints = [
        joint_name.text for joint_name in joint_state_plugin.findall("joint_name")
    ]

    assert "left_steering_joint" in published_joints
    assert "right_steering_joint" in published_joints
    assert "front_left_joint" in published_joints
    assert "rear_right_joint" in published_joints


def test_wheel_roll_joints_are_not_locked_by_axis_friction():
    urdf_path = PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro"
    root = ET.fromstring(read(urdf_path))
    wheel_joints = {
        "front_left_joint",
        "front_right_joint",
        "rear_left_joint",
        "rear_right_joint",
    }

    for joint_name in wheel_joints:
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        dynamics = joint.find("dynamics")
        assert dynamics is not None
        assert float(dynamics.attrib["friction"]) <= 0.05
        assert float(dynamics.attrib["damping"]) <= 0.05
