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

    assert 'rear_average: "/sensing/wheel/speed"' in config


def test_urdf_uses_unified_ackermann_drive_controller():
    urdf = read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro")

    assert "libackermann_drive_controller.so" in urdf
    assert "libgazebo_ros_diff_drive.so" not in urdf
    assert "libsteering_controller.so" not in urdf
    assert "<cmd_vel_topic>cmd_vel</cmd_vel_topic>" in urdf
    assert "<odom_topic>odom</odom_topic>" in urdf
    assert "<ground_truth_odom_topic>ground_truth/odom</ground_truth_odom_topic>" in urdf
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


def test_ackermann_kinematics_test_is_registered_with_colcon():
    cmake = read(PACKAGE_DIR / "CMakeLists.txt")

    assert re.search(
        r"ament_add_pytest_test\s*\(\s*ackermann_kinematics\s+"
        r"test/test_ackermann_kinematics.py",
        cmake,
        re.S,
    )


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


def test_runtime_sensor_verification_script_covers_required_topics():
    script = read(WORKSPACE_DIR / "scripts" / "verify_runtime_topics.sh")

    required_topics = [
        "/tf",
        "/robot/odom",
        "/robot/ground_truth/odom",
        "/robot/imu/data",
        "/robot/velodyne_points",
        "/robot/wheel_encoder/rear_average",
    ]
    for topic in required_topics:
        assert topic in script

    assert "ROS_LOG_DIR" in script
    assert "ros2 topic echo" in script
    assert "set +u" in script
    assert "set -u" in script


def test_ackermann_controller_publishes_ground_truth_odom_separately():
    source = read(PACKAGE_DIR / "src" / "ackermann_drive_controller.cpp")

    assert "ground_truth_odom_topic" in source
    assert "ground_truth_odom_pub_" in source


def test_sensing_bridge_launch_remaps_simulation_topics():
    launch = read(WORKSPACE_DIR / "launch" / "sensing_bridge.launch.py")

    expected_remaps = [
        "('/robot/velodyne_points', '/sensing/lidar/points')",
        "('/robot/imu/data', '/sensing/imu/data')",
        "('/robot/wheel_encoder/rear_average', '/sensing/wheel/speed')",
        "('/robot/rtk_gps/fix', '/sensing/gps/fix')",
    ]
    for remap in expected_remaps:
        assert remap in launch

    assert "package='robot_description'" in launch
    assert "executable='sensing_relay.py'" in launch
    assert "topic_tools" not in launch


def test_sensing_relay_script_is_installed_and_supports_message_types():
    cmake = read(PACKAGE_DIR / "CMakeLists.txt")
    script = read(WORKSPACE_DIR / "scripts" / "sensing_relay.py")

    assert "install(DIRECTORY ${PARENT_DIR}/scripts/" in cmake
    assert "importlib.import_module" in script
    assert "--message-type" in script
    assert "parse_known_args" in script
    assert "create_subscription" in script
    assert "create_publisher" in script


def test_robot_simulation_launch_can_enable_sensing_bridge():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "sensing_bridge.launch.py" in launch
    assert "sensing_bridge" in launch
    assert "DeclareLaunchArgument('sensing_bridge'" in launch
    assert "IfCondition(LaunchConfiguration('sensing_bridge'))" in launch


def test_lio_sam_config_uses_unified_sensing_topics():
    config = read(WORKSPACE_DIR / "config" / "lio_sam.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "lio_sam2.launch.py")

    assert 'imu: "/sensing/imu/data"' in config
    assert 'velodyne: "/sensing/lidar/points"' in config
    assert 'rear_average: "/sensing/wheel/speed"' in config
    assert 'gps: "/sensing/gps/fix"' in config
    assert "('/points_raw', '/sensing/lidar/points')" in launch
    assert "('/imu_raw', '/sensing/imu/data')" in launch
    assert "('/odom_encoded', '/odometry/filtered')" in launch


def test_localization_uses_unified_topics_and_avoids_ground_truth():
    config = read(WORKSPACE_DIR / "config" / "localization.yaml")

    assert "odom0: /robot/ground_truth/odom" not in config
    assert "/robot/ground_truth/odom" not in config
    assert "odom0: /robot/odom" in config
    assert "imu0: /sensing/imu/data" in config
    assert "gps_quality_gate:" in config
    assert "max_acceptable_covariance: 25.0" in config
    assert "max_position_jump: 3.0" in config


def test_runtime_sensor_verification_script_covers_unified_sensing_topics():
    script = read(WORKSPACE_DIR / "scripts" / "verify_runtime_topics.sh")

    required_topics = [
        "/sensing/lidar/points",
        "/sensing/imu/data",
        "/sensing/wheel/speed",
        "/sensing/gps/fix",
    ]
    for topic in required_topics:
        assert topic in script


def test_navigation_config_respects_ackermann_constraints():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")

    assert "DWB_MAX_VEL_Y: 0.0" in config
    assert "DWB_MIN_VEL_Y: 0.0" in config
    assert "min_turning_radius: 0.78" in config
    assert "w_reverse_cost: 2.5" in config
    assert "('/cmd_vel', '/control/cmd_vel')" in launch
    assert "('/control/cmd_vel', '/robot/cmd_vel')" in launch


def test_localization_verification_script_checks_filtered_odom():
    script = read(WORKSPACE_DIR / "scripts" / "verify_localization.sh")

    assert "/odometry/filtered" in script
    assert "localization.launch.py" in script
    assert "ros2 topic echo" in script
    assert "ROS_LOG_DIR" in script


def test_navigation_precheck_script_reports_missing_nav2_or_active_nodes():
    script = read(WORKSPACE_DIR / "scripts" / "verify_navigation_precheck.sh")

    assert "navigation.launch.py" in script
    assert "Navigation2 precheck failed" in script
    assert "controller_server" in script
    assert "bt_navigator" in script


def test_gps_degradation_scenarios_are_documented():
    doc = read(
        WORKSPACE_DIR
        / "docs"
        / "superpowers"
        / "specs"
        / "2026-05-20-gps-degradation-test-scenarios.md"
    )

    assert "GPS good" in doc
    assert "GPS outage" in doc
    assert "GPS jump" in doc
    assert "/sensing/gps/fix" in doc
    assert "/robot/ground_truth/odom" in doc
