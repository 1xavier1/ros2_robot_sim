#!/usr/bin/env python3
"""Static integration checks for wheel encoder simulation wiring."""

from pathlib import Path
import importlib.util
import math
import re
import xml.etree.ElementTree as ET

import pytest
import yaml
from rclpy.time import Time


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]


def read(path):
    return path.read_text(encoding="utf-8")


def load_script_module(script_name):
    script_path = WORKSPACE_DIR / "scripts" / script_name
    spec = importlib.util.spec_from_file_location(script_name.replace(".py", ""), script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeClock:
    def __init__(self, seconds):
        self.time = Time(seconds=seconds)

    def now(self):
        return self.time


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_odom(module, x, y, yaw=0.0, stamp_sec=1):
    msg = module.Odometry()
    msg.header.stamp = Time(seconds=stamp_sec).to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    qx, qy, qz, qw = module.quat_from_yaw(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def make_wheel_lio_node(module, now_sec=2.0):
    node = object.__new__(module.WheelLioFusion)
    node.parameters = {
        "lio_timeout_sec": 0.5,
        "wheel_timeout_sec": 0.25,
        "gps_timeout_sec": 1.0,
        "gps_anchor_blend_weight": 0.0,
        "max_lio_translation_error": 1.0,
        "motion_window_min_distance": 0.05,
        "wheel_lio_distance_warn": 0.15,
        "wheel_lio_distance_error": 0.35,
        "wheel_lio_speed_ratio_warn": 1.8,
        "wheel_lio_speed_ratio_error": 3.0,
        "yaw_delta_warn": 0.25,
        "yaw_delta_error": 0.60,
        "turning_yaw_rate_threshold": 0.25,
        "max_consecutive_bad_frames": 5,
        "wheel_weight_normal": 1.0,
        "wheel_weight_turning": 0.8,
        "wheel_weight_wheel_suspect": 0.2,
        "wheel_weight_lio_suspect": 1.0,
        "wheel_weight_degraded": 0.0,
    }
    node.clock = FakeClock(now_sec)
    node.get_clock = lambda: node.clock
    node.get_parameter = lambda name: FakeParameter(node.parameters[name])
    node.odom_pub = FakePublisher()
    node.status_pub = FakePublisher()
    node.lio_anchor = (0.0, 0.0, 0.0)
    node.wheel_anchor = (0.0, 0.0, 0.0)
    node.latest_lio = None
    node.latest_wheel = None
    node.latest_gps = None
    node.latest_lio_stamp = None
    node.latest_wheel_stamp = None
    node.latest_gps_stamp = None
    node.gps_origin = None
    node.fused_origin_xy = None
    node.global_offset = (0.0, 0.0)
    node.previous_lio_pose = None
    node.previous_wheel_pose = None
    node.previous_motion_stamp = None
    node.previous_lio_msg_stamp = None
    node.previous_wheel_msg_stamp = None
    node.consecutive_bad_frames = 0
    node.last_trusted_odom = None
    return node


def set_latest_odom(node, module, lio, wheel, now_sec=2.0, lio_age=0.0):
    node.latest_lio = lio
    node.latest_wheel = wheel
    node.latest_lio_stamp = Time(seconds=now_sec - lio_age)
    node.latest_wheel_stamp = Time(seconds=now_sec)


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
        "('/robot/velodyne_points', '/sensing/lidar/points_raw')",
        "('/sensing/lidar/points_filtered', '/sensing/lidar/points')",
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
    assert "gui" in launch
    assert "'gui': use_gazebo_gui" in launch
    assert "DeclareLaunchArgument('sensing_bridge'" in launch
    assert "IfCondition(LaunchConfiguration('sensing_bridge'))" in launch


def test_robot_simulation_launch_allows_slow_gazebo_spawn_service():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "'-timeout', '120'" in launch
    assert "spawn_x = LaunchConfiguration('x', default='0.8')" in launch
    assert "DeclareLaunchArgument('x', default_value='0.8'" in launch


def test_slam_navigation_launch_sequences_sim_fast_lio_and_nav2():
    launch = read(WORKSPACE_DIR / "launch" / "slam_navigation.launch.py")

    assert "robot_simulation.launch.py" in launch
    assert "fast_lio2.launch.py" in launch
    assert "navigation.launch.py" in launch
    assert "TimerAction(period=8.0" in launch
    assert "TimerAction(period=14.0" in launch
    assert "'sensing_bridge': 'true'" in launch
    assert "'gui': gui" in launch
    assert "'map': map_yaml" in launch
    assert "DeclareLaunchArgument('rviz'" in launch
    assert "DeclareLaunchArgument('gui'" in launch
    assert "DeclareLaunchArgument('map'" in launch
    assert "DeclareLaunchArgument('loop_closure'" in launch
    assert "'loop_closure': loop_closure" in launch


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


def test_lidar_self_filter_script_filters_vehicle_box_and_ranges():
    script = read(WORKSPACE_DIR / "scripts" / "lidar_self_filter.py")
    package = read(PACKAGE_DIR / "package.xml")

    assert "PointCloud2" in script
    assert "vehicle_geometry.yaml" in script
    assert "sensor_mount.yaml" in script
    assert "get_package_share_directory" in script
    assert "CONFIG_DIR" in script
    assert 'parents[1] / "config"' not in script
    assert "/sensing/lidar/points_raw" in script
    assert "/sensing/lidar/points_filtered" in script
    assert "box_min" in script
    assert "box_max" in script
    assert "min_range" in script
    assert "max_range" in script
    assert "read_points" in script
    assert '"ring"' in script
    assert '"time"' in script
    assert "estimate_ring" in script
    assert "scan_rate" in script
    assert "time_offset_us" in script
    assert "create_cloud" in script
    assert "<depend>sensor_msgs_py</depend>" in package
    assert "<exec_depend>python3-yaml</exec_depend>" in package
    assert "<exec_depend>ament_index_python</exec_depend>" in package


def test_navigation_config_respects_ackermann_constraints():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")

    assert "allow_reversing: false" in config
    assert "use_rotate_to_heading: false" in config
    assert "min_turning_radius: 0.78" in config
    assert "w_reverse_cost: 2.5" in config
    assert launch.count("('/cmd_vel', '/control/cmd_vel')") >= 2
    assert "('cmd_vel', '/control/cmd_vel')" in launch
    assert "('cmd_vel_smoothed', '/robot/cmd_vel')" in launch
    assert "node_names.append('velocity_smoother')" in launch
    assert "max_velocity: [0.5, 0.0, 1.0]" in config
    assert "scale_velocities: true" in config
    assert "speed_lim_vx" not in config


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


def test_lio_sam2_plan_is_superseded_by_fast_lio2_plan():
    old_plan = read(
        WORKSPACE_DIR
        / "docs"
        / "superpowers"
        / "plans"
        / "2026-05-20-lio-sam2-localization-nav2.md"
    )
    progress = read(WORKSPACE_DIR / "README_项目进度.md")

    assert "Superseded by" in old_plan
    assert "2026-05-20-fast-lio2-fusion-nav2.md" in old_plan
    assert "FAST-LIO2" in progress
    assert "LIO-SAM2建图未验证" not in progress


def test_vehicle_geometry_config_documents_units_and_self_filter():
    config_text = read(WORKSPACE_DIR / "config" / "vehicle_geometry.yaml")
    config = yaml.safe_load(config_text)
    vehicle = config["vehicle_geometry"]

    assert vehicle["wheelbase"] == 0.45
    assert vehicle["track_width"] == 0.35
    assert vehicle["wheel_radius"] == 0.07
    assert vehicle["max_steering_angle"] == 0.5236
    assert vehicle["min_turning_radius"] == 0.78

    assert vehicle["body"]["length"] == 0.55
    assert vehicle["body"]["width"] == 0.38
    assert vehicle["body"]["height"] == 0.12

    polygon = vehicle["footprint"]["polygon"]
    assert len(polygon) == 4
    assert all(len(point) == 2 for point in polygon)
    assert {tuple(point) for point in polygon} == {
        (0.275, 0.19),
        (0.275, -0.19),
        (-0.275, -0.19),
        (-0.275, 0.19),
    }

    assert vehicle["self_filter"]["frame"] == "base_link"
    assert vehicle["self_filter"]["box_min"] == [-0.35, -0.25, -0.05]
    assert vehicle["self_filter"]["box_max"] == [0.35, 0.25, 0.45]

    assert "unit m" in config_text
    assert "base_link" in config_text
    assert "X forward positive" in config_text


def test_sensor_mount_config_documents_lidar_extrinsics():
    config_text = read(WORKSPACE_DIR / "config" / "sensor_mount.yaml")
    config = yaml.safe_load(config_text)
    urdf_root = ET.fromstring(read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro"))

    assert config["lidar"]["parent_frame"] == "base_link"
    assert config["lidar"]["frame"] == "laser_link"
    assert config["lidar"]["xyz"] == [0.0, 0.0, 0.25]
    assert config["lidar"]["rpy"] == [0.0, 0.0, 0.0]
    assert config["lidar"]["min_range"] == 0.1
    assert config["lidar"]["max_range"] == 100.0
    assert config["lidar"]["horizontal_fov"] == 6.28318
    assert config["lidar"]["vertical_fov"] == 0.5236
    assert config["lidar"]["scan_lines"] == 32

    lidar_height = next(
        element for element in urdf_root.iter()
        if element.attrib.get("name") == "lidar_height"
    )
    laser_joint = urdf_root.find("./joint[@name='laser_joint']")
    assert lidar_height.attrib["value"] == "0.25"
    assert laser_joint is not None
    assert laser_joint.find("parent").attrib["link"] == "base_link"
    assert laser_joint.find("child").attrib["link"] == "laser_link"
    assert laser_joint.find("origin").attrib["xyz"] == "0 0 ${lidar_height}"
    assert laser_joint.find("origin").attrib["rpy"] == "0 0 0"

    assert config["imu"]["parent_frame"] == "base_link"
    assert config["imu"]["frame"] == "imu_link"
    assert config["imu"]["xyz"] == [0.0, 0.0, 0.08]
    assert config["imu"]["rpy"] == [0.0, 0.0, 0.0]

    assert config["gps"]["parent_frame"] == "base_link"
    assert config["gps"]["frame"] == "gps_link"
    assert config["gps"]["xyz"] == [0.0, 0.0, 0.3]

    assert "unit rad" in config_text
    assert "Right-hand rule" in config_text
    assert "X forward positive" in config_text


def test_simulated_lidar_mount_keeps_forward_rays_available_for_lio():
    config = yaml.safe_load(read(WORKSPACE_DIR / "config" / "sensor_mount.yaml"))
    lidar = config["lidar"]

    pitch = float(lidar["rpy"][1])
    vertical_fov = float(lidar["vertical_fov"])

    assert abs(pitch) <= vertical_fov / 2.0


def test_sensing_bridge_routes_lidar_through_self_filter():
    launch = read(WORKSPACE_DIR / "launch" / "sensing_bridge.launch.py")

    assert "('/robot/velodyne_points', '/sensing/lidar/points_raw')" in launch
    assert "lidar_self_filter.py" in launch
    assert "('/sensing/lidar/points_filtered', '/sensing/lidar/points')" in launch
    assert "('/robot/velodyne_points', '/sensing/lidar/points')" not in launch


def test_lidar_runtime_verification_checks_filtered_cloud_and_tf():
    runtime = read(WORKSPACE_DIR / "scripts" / "verify_runtime_topics.sh")
    lidar = read(WORKSPACE_DIR / "scripts" / "verify_lidar_mount.sh")

    assert "/sensing/lidar/points_raw" in runtime
    assert "/sensing/lidar/points_filtered" in runtime
    assert "/sensing/lidar/points" in runtime
    assert "base_link" in lidar
    assert "laser_link" in lidar
    assert "tf2_echo" in lidar
    assert "ros2 topic echo /sensing/lidar/points_filtered" in lidar


def test_fast_lio2_config_and_launch_use_filtered_sensing_topics():
    config = read(WORKSPACE_DIR / "config" / "fast_lio.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "fast_lio2.launch.py")

    assert "/sensing/lidar/points" in launch
    assert "/sensing/imu/data" in launch
    assert "laser_link" in config
    assert "base_link" in config
    assert "FAST-LIO2 precheck failed" in launch
    assert "fast_lio" in launch
    assert "static_transform_publisher" in launch
    assert "'base_link'" in launch
    assert "'laser_link'" in launch
    assert "'0.25'" in launch
    assert "'--pitch', '0'" in launch
    assert "/mapping/lio/odom" in launch
    assert "/mapping/lio/map_points" in launch


def test_fast_lio2_precheck_script_reports_missing_or_running_nodes():
    script = read(WORKSPACE_DIR / "scripts" / "verify_fast_lio2_precheck.sh")

    assert "fast_lio2.launch.py" in script
    assert "FAST-LIO2 precheck failed" in script
    assert "spark_lio_mapping" in script
    assert "/mapping/lio/odom" in script
    assert "/mapping/lio/map_points" in script


def test_localization_mode_manager_defines_outdoor_transition_barn_modes():
    config = read(WORKSPACE_DIR / "config" / "localization_modes.yaml")
    script = read(WORKSPACE_DIR / "scripts" / "localization_mode_manager.py")

    assert "OUTDOOR" in config
    assert "TRANSITION" in config
    assert "BARN" in config
    assert "gps_covariance_threshold: 25.0" in config
    assert "gps_jump_threshold: 3.0" in config
    assert "/localization/mode" in script
    assert "/localization/fusion_weights" in script
    assert "/localization/gps/gated" in script
    assert "NavSatFix" in script


def test_localization_launch_starts_mode_manager_and_verification_script():
    launch = read(WORKSPACE_DIR / "launch" / "localization.launch.py")
    script = read(WORKSPACE_DIR / "scripts" / "verify_localization_modes.sh")

    assert "localization_mode_manager.py" in launch
    assert "lio_wheel_fusion.py" in launch
    assert "localization_modes.yaml" in launch
    assert "/localization/mode" in script
    assert "/localization/fusion_weights" in script
    assert "ros2 topic echo" in script


def test_lio_wheel_fusion_publishes_fused_localization_odom():
    script = read(WORKSPACE_DIR / "scripts" / "lio_wheel_fusion.py")

    assert "/mapping/lio/odom" in script
    assert "/robot/odom" in script
    assert "/localization/gps/gated" in script
    assert "/localization/fused_odom" in script
    assert "/localization/fusion_status" in script
    assert "Odometry" in script
    assert "NavSatFix" in script
    assert "frame_id = \"map\"" in script
    assert "child_frame_id = \"base_link\"" in script
    assert "wheel.twist.twist" in script
    assert "gps_blend_weight" in script
    assert "gps_to_local_xy" in script


def test_global_localization_backend_publishes_optimized_odom_and_status():
    script = read(WORKSPACE_DIR / "scripts" / "global_localization_backend.py")

    assert "/localization/fused_odom" in script
    assert "/robot/odom" in script
    assert "/localization/gps/gated" in script
    assert "/sensing/imu/data" in script
    assert "/localization/global_odom" in script
    assert "/localization/backend_status" in script
    assert "/localization/loop_closure_status" in script
    assert "enable_loop_closure" in script
    assert "loop_closure=disabled" in script
    assert "loop_closure=applied_odom_proximity" in script
    assert "gps_anchor=fresh" in script
    assert "wheel=fresh" in script
    assert "imu=fresh" in script
    assert "loop_correction_gain" in script
    assert "max_loop_correction_step" in script
    assert "apply_loop_correction" in script
    assert "pending_loop_correction" in script
    assert "Odometry" in script
    assert "NavSatFix" in script
    assert "Imu" in script


def test_navigation_uses_filtered_lidar_and_vehicle_footprint_contract():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    script = read(WORKSPACE_DIR / "scripts" / "verify_saved_map_nav2_precheck.sh")
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")

    assert "topic: /sensing/lidar/points" in config
    assert "robot_base_frame: base_footprint" in config
    assert "global_frame: map" in config
    assert "footprint:" in config
    assert "[0.275, 0.19]" in config
    assert "[-0.275, -0.19]" in config
    assert "navigation.launch.py" in script
    assert "/control/cmd_vel" in script
    assert "Navigation2 precheck failed" in script
    assert "Created controller : FollowPath" in script
    assert "map->base_footprint TF is still required" in script
    assert 'grep -q "Created controller : FollowPath" <<< "$OUTPUT"' in script
    assert 'TOPICS="$(timeout 8s ros2 topic list 2>/dev/null || true)"' in script
    assert "DeclareLaunchArgument('map'" in launch
    assert "yaml_filename': map_yaml" in launch


def test_global_localization_runtime_verification_script_checks_nav2_chain():
    script = read(WORKSPACE_DIR / "scripts" / "verify_global_localization_runtime.sh")

    assert "/localization/fused_odom" in script
    assert "/localization/global_odom" in script
    assert "/localization/fusion_status" in script
    assert "/localization/backend_status" in script
    assert "/localization/loop_closure_status" in script
    assert "/robot/odom" in script
    assert "/mapping/lio/odom" in script
    assert "tf2_echo map base_footprint" in script
    assert "Robot is out of bounds of the costmap" in script
    assert "ros2 topic echo" in script
    assert "ROS_LOG_DIR" in script
    assert "set +u" in script
    assert "set -u" in script
    assert "global localization runtime verification passed" in script


def test_navigation_launch_starts_wheel_lio_fusion():
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")
    backend = read(WORKSPACE_DIR / "scripts" / "global_localization_backend.py")
    runtime_check = read(WORKSPACE_DIR / "scripts" / "verify_global_localization_runtime.sh")

    assert "wheel_lio_fusion.py" in launch
    assert "/localization/wheel_lio_odom" in launch
    assert "input_odom_topic" in backend
    assert "/localization/wheel_lio_odom" in backend
    assert "/localization/wheel_lio_status" in runtime_check
    assert "FAST-LIO + wheel/GPS wheel-LIO odom" in runtime_check


def test_navigation_controller_uses_humble_regulated_pure_pursuit_contract():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")

    assert 'controller_plugins: ["FollowPath"]' in config
    assert "FollowPath:" in config
    assert (
        'plugin: "nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController"'
    ) in config
    assert "desired_linear_vel: 0.35" in config
    assert "lookahead_dist: 0.6" in config
    assert "use_rotate_to_heading: false" in config
    assert "allow_reversing: false" in config
    assert 'plugin: "dwb_core::DWBLocalPlanner"' not in config


def test_navigation_planner_uses_humble_smac_plugin_name():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")

    assert 'plugin: "nav2_smac_planner/SmacPlannerHybrid"' in config
    assert "nav2_smac_planner::SmacPlannerHybrid" not in config


def test_navigation_bt_plugins_match_humble_installed_libraries():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")

    assert "nav2_rate_controller_bt_node" in config
    assert "nav2_goal_updated_condition_bt_node" in config
    assert "nav2_goal_updater_node_bt_node" in config
    assert "nav2_time_controller_bt_node" not in config
    assert "nav2_recovery_selector_bt_node" not in config
    assert "nav2_goal_behavior_bt_node" not in config


def test_navigation_bt_xml_paths_use_humble_existing_files():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")
    through_poses_bt = read(
        WORKSPACE_DIR
        / "config"
        / "navigate_through_poses_w_replanning_and_recovery_no_remove.xml"
    )

    assert "$(find-pkg-share nav2_bt_navigator)" not in config
    assert "navigate_through_poses_w_replanning_and_recovery_no_remove.xml" in launch
    assert "navigate_to_pose_w_replanning_and_recovery.xml" in config
    assert "RemovePassedGoals" not in through_poses_bt
    assert "ComputePathThroughPoses" in through_poses_bt
    assert "FollowPath" in through_poses_bt


def test_remote_extension_config_reserves_future_namespaces_without_runtime_dependency():
    config = read(WORKSPACE_DIR / "config" / "remote_extension.yaml")

    assert "/mission" in config
    assert "/maps" in config
    assert "/fleet" in config
    assert "/config" in config
    assert "cloud is not part of the real-time control loop" in config
    assert "vehicle must keep local navigation running" in config
    assert "versioned" in config


def test_dependency_installer_keeps_fast_lio_source_out_of_tmp():
    installer = read(WORKSPACE_DIR / "scripts" / "install_dependencies.sh")
    fast_lio_installer = read(
        WORKSPACE_DIR / "scripts" / "install_fast_lio2_source.sh"
    )

    assert "ros-humble-navigation2" in installer
    assert "ros-humble-nav2-bringup" in installer
    assert "git clone https://github.com/AIC-Robotics/fast_lio.git" not in installer
    assert "cd /tmp" not in installer
    assert "install_fast_lio2_source.sh" in installer

    assert "FAST_LIO2_REPO_URL" in fast_lio_installer
    assert "LIO_FRONTEND_REPO_URL" in fast_lio_installer
    assert "src/third_party/spark-fast-lio" in fast_lio_installer
    assert "cd /tmp" not in fast_lio_installer
    assert "colcon build --packages-up-to" in fast_lio_installer


def test_fast_lio_source_installer_defaults_to_spark_fast_lio_candidate():
    fast_lio_installer = read(
        WORKSPACE_DIR / "scripts" / "install_fast_lio2_source.sh"
    )

    assert "https://github.com/MIT-SPARK/spark-fast-lio.git" in fast_lio_installer
    assert "src/third_party/spark-fast-lio" in fast_lio_installer
    assert "LIO_FRONTEND_PACKAGE_NAME" in fast_lio_installer
    assert "spark_fast_lio" in fast_lio_installer
    assert "colcon build --packages-up-to \"$LIO_FRONTEND_PACKAGE_NAME\"" in (
        fast_lio_installer
    )


def test_fast_lio_launch_accepts_spark_fast_lio_ros2_frontend():
    launch = read(WORKSPACE_DIR / "launch" / "fast_lio2.launch.py")

    assert "'spark_fast_lio'" in launch
    assert "'spark_lio_mapping'" in launch
    assert "executable_name" in launch
    assert "missing package spark_fast_lio, fast_lio, or fast_lio2" in launch


def test_fast_lio_config_uses_spark_fast_lio_parameter_schema():
    config = read(WORKSPACE_DIR / "config" / "fast_lio.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "fast_lio2.launch.py")

    assert "/**:" in config
    assert "common:" in config
    assert "map_frame: map" in config
    assert "lidar_frame: laser_link" in config
    assert "base_frame: base_link" in config
    assert "imu_frame: imu_link" in config
    assert "visualization_frame: base" in config
    assert "preprocess:" in config
    assert "lidar_type: 2" in config
    assert "scan_line: 32" in config
    assert "mapping:" in config
    assert "extrinsic_T:" in config
    assert "gravity_alignment:" in config
    assert "enable_gravity_alignment: false" in config
    assert "publish:" in config

    assert "('lidar', '/sensing/lidar/points')" in launch
    assert "('imu', '/sensing/imu/data')" in launch
    assert "('odometry', '/mapping/lio/odom')" in launch
    assert "('cloud_registered', '/mapping/lio/map_points')" in launch
    assert "('/tf', '/mapping/lio/tf')" in launch


def test_fast_lio_lidar_imu_extrinsics_match_sensor_mount():
    fast_lio = yaml.safe_load(read(WORKSPACE_DIR / "config" / "fast_lio.yaml"))
    sensor_mount = yaml.safe_load(read(WORKSPACE_DIR / "config" / "sensor_mount.yaml"))

    mapping = fast_lio["/**"]["ros__parameters"]["mapping"]
    lidar = sensor_mount["lidar"]
    imu = sensor_mount["imu"]

    expected_translation = [
        lidar["xyz"][axis] - imu["xyz"][axis]
        for axis in range(3)
    ]
    for actual, expected in zip(mapping["extrinsic_T"], expected_translation):
        assert math.isclose(actual, expected, abs_tol=1e-6)

    roll, pitch, yaw = lidar["rpy"]
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    expected_rotation = [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]

    for actual, expected in zip(mapping["extrinsic_R"], expected_rotation):
        assert math.isclose(actual, expected, abs_tol=1e-4)


def test_fast_lio_dynamic_tf_is_isolated_from_nav2_tf_tree():
    launch = read(WORKSPACE_DIR / "launch" / "fast_lio2.launch.py")
    runtime_check = read(WORKSPACE_DIR / "scripts" / "verify_global_localization_runtime.sh")

    assert "('/tf', '/mapping/lio/tf')" in launch
    assert "tf2_echo map base_footprint" in runtime_check
    assert "tf2_echo map base_link" not in runtime_check


def test_simulated_lidar_density_supports_fast_lio_mapping():
    urdf = read(PACKAGE_DIR / "urdf" / "robot_base.urdf.xacro")

    assert "<samples>720</samples>" in urdf
    assert "<samples>32</samples>" in urdf


def test_lio_map_to_occupancy_exporter_contract():
    script = read(WORKSPACE_DIR / "scripts" / "export_lio_map_to_occupancy.py")

    assert "/mapping/lio/map_points" in script
    assert "sensor_msgs.msg" in script
    assert "PointCloud2" in script
    assert "sensor_msgs_py" in script
    assert "nav2" in script.lower()
    assert "occupied_thresh: 0.65" in script
    assert "free_thresh: 0.25" in script
    assert "resolution" in script
    assert "min_z" in script
    assert "max_z" in script
    assert ".pgm" in script
    assert ".yaml" in script
    assert "maps" in script


def test_lio_accumulator_exports_nav2_map_with_lower_left_origin():
    script = read(WORKSPACE_DIR / "scripts" / "accumulate_lio_map.py")

    assert "np.flipud(grid)" in script
    assert "origin_y = min_y + height * resolution" not in script
    assert "origin: [{min_x:.6f}, {min_y:.6f}, 0.000000]" in script
    assert "np.where(occ_mask, np.uint8(0), np.uint8(254))" in script


def test_lio_map_exporter_uses_nav2_pgm_occupancy_values():
    script = read(WORKSPACE_DIR / "scripts" / "export_lio_map_to_occupancy.py")

    assert "np.where(grid > 0, np.uint8(0), np.uint8(254))" in script


def test_odom_projected_map_exporter_contract():
    script = read(WORKSPACE_DIR / "scripts" / "export_odom_projected_map.py")

    assert "/sensing/lidar/points" in script
    assert "/robot/odom" in script
    assert "/mapping/lio/odom" in script
    assert "diagnostics" in script
    assert "PointCloud2" in script
    assert "Odometry" in script
    assert ".pgm" in script
    assert ".yaml" in script
    assert ".json" in script
    assert "sensor_mount.yaml" in script
    assert "vehicle_geometry.yaml" in script
    assert "raytrace_cells" in script
    assert "max_range" in script
    assert "self_filter" in script
    assert "filter_stats" in script
    assert "height_high_points" in script
    assert "occupied_free_ratio" in script


def test_odom_projected_map_transforms_body_points_to_odom_frame():
    module = load_script_module("export_odom_projected_map.py")

    transformed = module.transform_body_points(
        [(1.0, 0.0, 0.2), (0.0, 2.0, 0.3)],
        pose=(10.0, -2.0, math.pi / 2.0),
    )

    assert abs(transformed[0][0] - 10.0) < 1e-6
    assert abs(transformed[0][1] - -1.0) < 1e-6
    assert abs(transformed[1][0] - 8.0) < 1e-6
    assert abs(transformed[1][1] - -2.0) < 1e-6


def test_odom_projected_map_applies_lidar_planar_extrinsic():
    module = load_script_module("export_odom_projected_map.py")

    transformed = module.transform_lidar_points_to_odom(
        [(1.0, 0.0, 0.2)],
        odom_pose=(10.0, -2.0, math.pi / 2.0),
        lidar_xyz=(0.5, 0.0, 0.25),
        lidar_yaw=0.0,
    )

    assert abs(transformed[0][0] - 10.0) < 1e-6
    assert abs(transformed[0][1] - -0.5) < 1e-6


def test_odom_projected_map_raytraces_free_cells_before_occupied_endpoint():
    module = load_script_module("export_odom_projected_map.py")

    cells = module.raytrace_cells((0, 0), (3, 0))

    assert cells == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_odom_projected_map_filters_lidar_points_by_planar_range():
    module = load_script_module("export_odom_projected_map.py")

    points = module.filter_lidar_points_by_range(
        [(1.0, 0.0, 0.0), (20.0, 0.0, 0.0), (0.01, 0.0, 0.0)],
        min_range=0.1,
        max_range=15.0,
    )

    assert points == [(1.0, 0.0, 0.0)]


def test_odom_projected_map_filters_vehicle_self_hits_in_base_frame():
    module = load_script_module("export_odom_projected_map.py")

    points = module.filter_points_in_box(
        [(-0.1, 0.0, 0.1), (0.6, 0.0, 0.1), (0.0, 0.4, 0.1)],
        box_min=(-0.35, -0.25, -0.05),
        box_max=(0.35, 0.25, 0.45),
    )

    assert points == [(0.6, 0.0, 0.1), (0.0, 0.4, 0.1)]


def test_odom_projected_map_grid_uses_unknown_free_and_occupied_values():
    module = load_script_module("export_odom_projected_map.py")

    grid = module.make_occupancy_grid(
        occupied_cells={(3, 0)},
        free_cells={(0, 0), (1, 0), (2, 0)},
        width=4,
        height=2,
    )

    assert grid[1, 3] == 0
    assert grid[1, 0] == 254
    assert grid[0, 0] == 205


def test_odom_projected_map_splits_height_filter_stats():
    module = load_script_module("export_odom_projected_map.py")

    in_range, stats = module.filter_points_by_height(
        [(1.0, 0.0, -0.1), (2.0, 0.0, 0.5), (3.0, 0.0, 2.0)],
        min_z=0.0,
        max_z=1.6,
    )

    assert in_range == [(2.0, 0.0, 0.5)]
    assert stats == {
        "height_low_points": 1,
        "height_points": 1,
        "height_high_points": 1,
    }


def test_odom_projected_map_free_evidence_can_clear_weak_occupied_cells():
    module = load_script_module("export_odom_projected_map.py")

    occupied = {(1, 0): 2, (2, 0): 10}
    free = {(1, 0): 20, (2, 0): 3, (3, 0): 5}
    occupied_cells, free_cells = module.classify_cells(
        occupied,
        free,
        min_hits=2,
        min_free_hits=2,
        occupied_free_ratio=0.5,
    )

    assert (1, 0) not in occupied_cells
    assert (1, 0) in free_cells
    assert (2, 0) in occupied_cells
    assert (3, 0) in free_cells


def test_lio_tf_adapter_publishes_map_to_odom_at_current_ros_time():
    script = read(WORKSPACE_DIR / "scripts" / "lio_tf_adapter.py")

    assert "/localization/global_odom" in script
    assert "/localization/fused_odom" not in script
    assert "/mapping/lio/odom" not in script
    assert "t.header.stamp = self.get_clock().now().to_msg()" in script
    assert "t.header.stamp = lio.header.stamp" not in script


def test_lio_tf_adapter_projects_global_pose_to_planar_nav2_tf():
    module = load_script_module("lio_tf_adapter.py")

    qx, qy, qz, qw = module.quat_from_yaw(math.pi / 2.0)
    result = module.compose_planar_map_odom(
        map_base=(2.0, 3.0, 1.2, qx, qy, qz, qw),
        odom_base=(1.0, 0.0, -0.4, 0.0, 0.0, 0.0, 1.0),
    )

    x, y, z, out_qx, out_qy, out_qz, out_qw = result
    assert abs(x - 2.0) < 1e-6
    assert abs(y - 2.0) < 1e-6
    assert z == 0.0
    assert out_qx == 0.0
    assert out_qy == 0.0
    assert abs(module.yaw_from_quat(out_qx, out_qy, out_qz, out_qw) - math.pi / 2.0) < 1e-6


def test_fused_localization_tf_chain_is_owned_by_navigation_launch():
    navigation_launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")
    simulation_launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "localization_mode_manager.py" in navigation_launch
    assert "localization_modes.yaml" in navigation_launch
    assert "lio_wheel_fusion.py" in navigation_launch
    assert "global_localization_backend.py" in navigation_launch
    assert "lio_tf_adapter.py" in navigation_launch
    assert "DeclareLaunchArgument('loop_closure'" in navigation_launch
    assert "'enable_loop_closure': loop_closure" in navigation_launch
    assert "lio_tf_adapter.py" not in simulation_launch
    assert "lio_wheel_fusion.py" not in simulation_launch
    assert "global_localization_backend.py" not in simulation_launch


def test_navigation_launch_exposes_map_alignment_and_gps_anchor_controls():
    navigation_launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")
    slam_navigation_launch = read(WORKSPACE_DIR / "launch" / "slam_navigation.launch.py")

    for parameter in (
        "map_align_x",
        "map_align_y",
        "map_align_yaw",
        "gps_anchor_blend_weight",
    ):
        assert f"DeclareLaunchArgument('{parameter}'" in navigation_launch
        assert f"LaunchConfiguration('{parameter}'" in navigation_launch
        assert f"'{parameter}': {parameter}" in navigation_launch
        assert f"DeclareLaunchArgument('{parameter}'" in slam_navigation_launch
        assert f"'{parameter}': {parameter}" in slam_navigation_launch


def test_rviz_config_opens_with_odom_baseline_and_lio_overlays():
    config = yaml.safe_load(read(WORKSPACE_DIR / "rviz" / "robot_config.rviz"))
    manager = config["Visualization Manager"]
    displays = manager["Displays"]
    display_names = {display["Name"] for display in displays}

    assert manager["Global Options"]["Fixed Frame"] == "odom"
    assert {"Grid", "RobotModel", "TF", "Raw LiDAR", "LIO Map", "LIO Odometry"} <= display_names
    assert not config.get("Window Geometry", {}).get("Hide Left Dock", False)
    assert "QMainWindow State" not in config.get("Window Geometry", {})
    assert all(
        display.get("Class") != "rviz_default_plugins/Path"
        for display in displays
    )


def test_fast_lio_drift_diagnostic_contract():
    script = read(WORKSPACE_DIR / "scripts" / "fast_lio_drift_diagnostic.py")

    assert "/mapping/lio/odom" in script
    assert "/robot/odom" in script
    assert "/localization/wheel_lio_odom" in script
    assert "/robot/ground_truth/odom" in script
    assert "reference_topic" in script
    assert "compute_delta_metrics" in script
    assert "scale_ratio" in script
    assert "drift_per_meter" in script
    assert ".json" in script
    assert "duration_sec" in script
    assert "rclpy.spin_once" in script or "Duration" in script


def test_fast_lio_drift_diagnostic_computes_scale_and_drift():
    module = load_script_module("fast_lio_drift_diagnostic.py")

    result = module.compute_delta_metrics(
        reference_start=(0.0, 0.0, 0.0),
        reference_end=(4.0, 0.0, 0.0),
        estimate_start=(0.0, 0.0, 0.0),
        estimate_end=(2.0, 0.0, 0.1),
    )

    assert abs(result["reference_distance"] - 4.0) < 1e-6
    assert abs(result["estimate_distance"] - 2.0) < 1e-6
    assert abs(result["scale_ratio"] - 0.5) < 1e-6
    assert abs(result["translation_error"] - 2.0) < 1e-6
    assert abs(result["drift_per_meter"] - 0.5) < 1e-6
    assert abs(result["yaw_error"] - 0.1) < 1e-6


def test_wheel_lio_fusion_contract():
    script = read(WORKSPACE_DIR / "scripts" / "wheel_lio_fusion.py")

    assert "/mapping/lio/odom" in script
    assert "/robot/odom" in script
    assert "/localization/gps/gated" in script
    assert "/localization/wheel_lio_odom" in script
    assert "/localization/wheel_lio_status" in script
    assert "gps_anchor_blend_weight" in script
    assert "max_lio_translation_error" in script
    assert "compose_wheel_lio_pose" in script
    assert "lio=stale_waiting_anchor" in script
    for expected in [
        "motion_window_min_distance",
        "wheel_lio_distance_warn",
        "wheel_lio_distance_error",
        "wheel_lio_speed_ratio_warn",
        "wheel_lio_speed_ratio_error",
        "yaw_delta_warn",
        "yaw_delta_error",
        "turning_yaw_rate_threshold",
        "wheel_weight_normal",
        "wheel_weight_turning",
        "wheel_weight_wheel_suspect",
        "wheel_weight_lio_suspect",
        "wheel_weight_degraded",
        "max_consecutive_bad_frames",
        "state=",
        "wheel_weight=",
        "last_trusted_odom",
    ]:
        assert expected in script


def test_wheel_lio_fusion_uses_wheel_translation_and_lio_yaw():
    module = load_script_module("wheel_lio_fusion.py")

    fused = module.compose_wheel_lio_pose(
        lio_anchor=(10.0, 5.0, 0.2),
        wheel_anchor=(1.0, 1.0, 0.0),
        wheel_current=(3.0, 1.0, 0.0),
        lio_current=(10.4, 5.1, 0.25),
        use_lio_yaw=True,
    )

    assert abs(fused[0] - 11.9601331557) < 1e-6
    assert abs(fused[1] - 5.3973386616) < 1e-6
    assert abs(fused[2] - 0.25) < 1e-6


def test_wheel_lio_fusion_refreshes_anchor_when_translation_error_is_large():
    module = load_script_module("wheel_lio_fusion.py")

    assert module.should_refresh_lio_anchor(
        fused_pose=(10.0, 0.0, 0.0),
        lio_pose=(0.0, 0.0, 0.0),
        max_error=1.0,
    )
    assert not module.should_refresh_lio_anchor(
        fused_pose=(0.5, 0.0, 0.0),
        lio_pose=(0.0, 0.0, 0.0),
        max_error=1.0,
    )
    assert not module.should_refresh_lio_anchor(
        fused_pose=(10.0, 0.0, 0.0),
        lio_pose=(0.0, 0.0, 0.0),
        max_error=0.0,
    )


def test_wheel_lio_fusion_uses_current_stamp_when_lio_yaw_is_stale():
    module = load_script_module("wheel_lio_fusion.py")

    old_lio_stamp = object()
    current_stamp = object()

    assert (
        module.select_output_stamp(old_lio_stamp, current_stamp, True)
        is old_lio_stamp
    )
    assert (
        module.select_output_stamp(old_lio_stamp, current_stamp, False)
        is current_stamp
    )


def test_wheel_lio_fusion_recomputes_current_pose_after_anchor_refresh():
    module = load_script_module("wheel_lio_fusion.py")

    lio_anchor, wheel_anchor, fused_pose = module.maybe_refresh_anchor_and_pose(
        lio_anchor=(0.0, 0.0, 0.0),
        wheel_anchor=(0.0, 0.0, 0.0),
        wheel_pose=(20.0, 0.0, 0.4),
        lio_pose=(1.0, 2.0, 0.3),
        use_lio_yaw=True,
        max_error=1.0,
    )

    assert lio_anchor == (1.0, 2.0, 0.3)
    assert wheel_anchor == (20.0, 0.0, 0.4)
    assert abs(fused_pose[0] - 1.0) < 1e-6
    assert abs(fused_pose[1] - 2.0) < 1e-6
    assert abs(fused_pose[2] - 0.3) < 1e-6


def test_wheel_lio_fusion_does_not_refresh_anchor_when_lio_yaw_is_stale():
    module = load_script_module("wheel_lio_fusion.py")

    lio_anchor, wheel_anchor, fused_pose = module.maybe_refresh_anchor_and_pose(
        lio_anchor=(0.0, 0.0, 0.0),
        wheel_anchor=(0.0, 0.0, 0.0),
        wheel_pose=(20.0, 0.0, 0.4),
        lio_pose=(1.0, 2.0, 0.3),
        use_lio_yaw=False,
        max_error=1.0,
    )

    assert lio_anchor == (0.0, 0.0, 0.0)
    assert wheel_anchor == (0.0, 0.0, 0.0)
    assert abs(fused_pose[0] - 20.0) < 1e-6
    assert abs(fused_pose[1] - 0.0) < 1e-6
    assert abs(fused_pose[2] - 0.4) < 1e-6


def test_wheel_lio_fusion_only_initializes_anchor_from_fresh_lio():
    module = load_script_module("wheel_lio_fusion.py")

    assert module.can_initialize_anchor(use_lio_yaw=True)
    assert not module.can_initialize_anchor(use_lio_yaw=False)


def test_wheel_lio_fusion_computes_motion_delta():
    module = load_script_module("wheel_lio_fusion.py")

    delta = module.compute_motion_delta(
        previous_pose=(1.0, 2.0, 0.1),
        current_pose=(4.0, 6.0, 0.4),
        dt=2.0,
    )

    assert abs(delta.dx - 3.0) < 1e-6
    assert abs(delta.dy - 4.0) < 1e-6
    assert abs(delta.distance - 5.0) < 1e-6
    assert abs(delta.heading - 0.9272952180) < 1e-6
    assert abs(delta.yaw_delta - 0.3) < 1e-6
    assert abs(delta.speed - 2.5) < 1e-6


def test_wheel_lio_fusion_compares_motion_metrics():
    module = load_script_module("wheel_lio_fusion.py")

    comparison = module.compare_wheel_lio_motion(
        wheel_delta=module.MotionDelta(1.0, 0.0, 1.0, 0.0, 0.3, 1.0),
        lio_delta=module.MotionDelta(0.0, 1.0, 1.0, 1.5707963268, 0.1, 0.5),
    )

    assert abs(comparison.distance_diff - 0.0) < 1e-6
    assert abs(comparison.direction_diff - 1.5707963268) < 1e-6
    assert abs(comparison.yaw_diff - 0.2) < 1e-6
    assert abs(comparison.wheel_lio_speed_ratio - 2.0) < 1e-6
    assert abs(comparison.lio_wheel_speed_ratio - 0.5) < 1e-6


def test_wheel_lio_fusion_classifies_motion_consistency_states():
    module = load_script_module("wheel_lio_fusion.py")
    thresholds = module.FusionThresholds()

    normal = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(1.0, 0.0, 1.0, 0.0, 0.02, 1.0),
        lio_delta=module.MotionDelta(0.98, 0.0, 0.98, 0.0, 0.02, 0.98),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert normal.state == "normal"
    assert normal.reason == "motion_consistent"

    wheel_suspect = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(2.0, 0.0, 2.0, 0.0, 0.0, 2.0),
        lio_delta=module.MotionDelta(0.4, 0.0, 0.4, 0.0, 0.0, 0.4),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert wheel_suspect.state == "wheel_suspect"
    assert wheel_suspect.reason == "wheel_distance_high"

    lio_suspect = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(0.4, 0.0, 0.4, 0.0, 0.0, 0.4),
        lio_delta=module.MotionDelta(2.0, 0.0, 2.0, 0.0, 0.0, 2.0),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert lio_suspect.state == "lio_suspect"
    assert lio_suspect.reason == "lio_distance_high"

    turning = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(0.9, 0.1, 0.91, 0.11, 0.35, 0.91),
        lio_delta=module.MotionDelta(0.88, 0.12, 0.89, 0.14, 0.34, 0.89),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert turning.state == "turning_caution"
    assert turning.reason == "yaw_rate_high"

    speed_warn = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(1.0, 0.0, 1.0, 0.0, 0.02, 2.0),
        lio_delta=module.MotionDelta(1.0, 0.0, 1.0, 0.0, 0.02, 1.0),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert speed_warn.state == "turning_caution"
    assert speed_warn.reason == "speed_ratio_warn"

    degraded = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(2.0, 0.0, 2.0, 0.0, 0.0, 2.0),
        lio_delta=module.MotionDelta(0.1, 0.0, 0.1, 0.0, 0.0, 0.1),
        thresholds=thresholds,
        consecutive_bad_frames=thresholds.max_consecutive_bad_frames,
    )
    assert degraded.state == "degraded"

    near_stationary = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        lio_delta=module.MotionDelta(0.0000001, 0.0, 0.0000001, 0.0, 0.0, 0.000002),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert near_stationary.state == "normal"
    assert near_stationary.reason == "insufficient_motion"

    yaw_error = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(2.0, 0.0, 2.0, 0.0, 1.0, 2.0),
        lio_delta=module.MotionDelta(0.1, 0.0, 0.1, 0.0, 0.0, 0.1),
        thresholds=thresholds,
        consecutive_bad_frames=0,
    )
    assert yaw_error.state == "degraded"
    assert yaw_error.reason == "yaw_delta_error"


def test_wheel_lio_fusion_weights_and_blends_pose_by_state():
    module = load_script_module("wheel_lio_fusion.py")
    weights = module.FusionWeights()

    assert module.wheel_weight_for_state("normal", weights) == 1.0
    assert module.wheel_weight_for_state("turning_caution", weights) == 0.8
    assert module.wheel_weight_for_state("wheel_suspect", weights) == 0.2
    assert module.wheel_weight_for_state("lio_suspect", weights) == 1.0
    assert module.wheel_weight_for_state("degraded", weights) == 0.0
    assert module.wheel_weight_for_state("unknown", weights) == 0.0
    assert module.wheel_weight_for_state("__class__", weights) == 0.0

    fused_pose = module.blend_fused_pose(
        wheel_projected_pose=(10.0, 0.0, 0.3),
        lio_pose=(0.0, 10.0, 0.8),
        wheel_weight=0.2,
        use_lio_yaw=True,
    )

    assert abs(fused_pose[0] - 2.0) < 1e-6
    assert abs(fused_pose[1] - 8.0) < 1e-6
    assert abs(fused_pose[2] - 0.8) < 1e-6

    lio_only = module.blend_fused_pose(
        wheel_projected_pose=(10.0, 0.0, 0.3),
        lio_pose=(0.0, 10.0, 0.8),
        wheel_weight=-0.5,
        use_lio_yaw=True,
    )
    assert abs(lio_only[0] - 0.0) < 1e-6
    assert abs(lio_only[1] - 10.0) < 1e-6
    assert abs(lio_only[2] - 0.8) < 1e-6

    wheel_only = module.blend_fused_pose(
        wheel_projected_pose=(10.0, 0.0, 4.0),
        lio_pose=(0.0, 10.0, 0.8),
        wheel_weight=1.5,
        use_lio_yaw=False,
    )
    assert abs(wheel_only[0] - 10.0) < 1e-6
    assert abs(wheel_only[1] - 0.0) < 1e-6
    assert abs(wheel_only[2] - module.wrap_angle(4.0)) < 1e-6


def test_wheel_lio_fusion_old_anchor_projection_stays_available():
    module = load_script_module("wheel_lio_fusion.py")

    fused_pose = module.compose_wheel_lio_pose(
        lio_anchor=(10.0, 5.0, 0.2),
        wheel_anchor=(1.0, 1.0, 0.0),
        wheel_current=(3.0, 1.0, 0.0),
        lio_current=(10.4, 5.1, 0.25),
        use_lio_yaw=True,
    )

    assert abs(fused_pose[0] - 11.9601331557) < 1e-6
    assert abs(fused_pose[1] - 5.3973386616) < 1e-6
    assert abs(fused_pose[2] - 0.25) < 1e-6


def test_wheel_lio_fusion_lio_suspect_keeps_old_anchor_projection():
    module = load_script_module("wheel_lio_fusion.py")
    node = make_wheel_lio_node(module)
    set_latest_odom(
        node,
        module,
        lio=make_odom(module, 10.0, 0.0, stamp_sec=2),
        wheel=make_odom(module, 1.0, 0.0, stamp_sec=2),
    )
    node.classify_current_motion = lambda wheel_pose, lio_pose, use_lio_yaw: (
        module.FusionDecision("lio_suspect", "lio_distance_high")
    )

    node.publish_if_ready()

    fused = node.odom_pub.messages[-1]
    assert abs(fused.pose.pose.position.x - 1.0) < 1e-6
    assert node.lio_anchor == (0.0, 0.0, 0.0)
    assert node.wheel_anchor == (0.0, 0.0, 0.0)
    assert "state=lio_suspect" in node.status_pub.messages[-1].data


def test_wheel_lio_fusion_stale_lio_does_not_accumulate_bad_frames():
    module = load_script_module("wheel_lio_fusion.py")
    node = make_wheel_lio_node(module, now_sec=2.0)
    node.previous_lio_pose = (0.0, 0.0, 0.0)
    node.previous_wheel_pose = (0.0, 0.0, 0.0)
    node.previous_motion_stamp = Time(seconds=1)
    node.previous_lio_msg_stamp = ("header", 1, 0)
    node.previous_wheel_msg_stamp = ("header", 1, 0)
    set_latest_odom(
        node,
        module,
        lio=make_odom(module, 0.0, 0.0, stamp_sec=1),
        wheel=make_odom(module, 2.0, 0.0, stamp_sec=2),
        now_sec=2.0,
        lio_age=1.0,
    )

    node.publish_if_ready()

    assert node.consecutive_bad_frames == 0
    assert "reason=lio_stale_fallback" in node.status_pub.messages[-1].data


def test_wheel_lio_fusion_waits_for_paired_motion_samples():
    module = load_script_module("wheel_lio_fusion.py")
    node = make_wheel_lio_node(module, now_sec=2.0)
    node.previous_lio_pose = (0.0, 0.0, 0.0)
    node.previous_wheel_pose = (0.0, 0.0, 0.0)
    node.previous_motion_stamp = Time(seconds=1)
    node.previous_lio_msg_stamp = ("header", 1, 0)
    node.previous_wheel_msg_stamp = ("header", 1, 0)
    set_latest_odom(
        node,
        module,
        lio=make_odom(module, 0.0, 0.0, stamp_sec=1),
        wheel=make_odom(module, 2.0, 0.0, stamp_sec=2),
        now_sec=2.0,
    )

    node.publish_if_ready()

    assert node.consecutive_bad_frames == 0
    assert node.lio_anchor == (0.0, 0.0, 0.0)
    assert node.wheel_anchor == (0.0, 0.0, 0.0)
    assert "reason=waiting_paired_motion" in node.status_pub.messages[-1].data


def test_wheel_lio_fusion_degraded_hold_refreshes_stamp():
    module = load_script_module("wheel_lio_fusion.py")
    node = make_wheel_lio_node(module, now_sec=2.0)
    node.last_trusted_odom = make_odom(module, 3.0, 0.0, stamp_sec=1)
    set_latest_odom(
        node,
        module,
        lio=make_odom(module, 10.0, 0.0, stamp_sec=2),
        wheel=make_odom(module, 1.0, 0.0, stamp_sec=2),
    )
    node.classify_current_motion = lambda wheel_pose, lio_pose, use_lio_yaw: (
        module.FusionDecision("degraded", "yaw_delta_error")
    )

    node.publish_if_ready()

    held = node.odom_pub.messages[-1]
    assert held is not node.last_trusted_odom
    assert held.header.stamp.sec == 2
    assert held.pose.pose.position.x == 3.0
    assert "reason=holding_last_trusted" in node.status_pub.messages[-1].data


def test_wheel_lio_robust_fusion_does_not_use_ground_truth_for_production():
    fusion = read(WORKSPACE_DIR / "scripts" / "wheel_lio_fusion.py")
    exporter = read(WORKSPACE_DIR / "scripts" / "export_odom_projected_map.py")

    assert "/robot/ground_truth/odom" not in fusion
    assert 'default="/localization/wheel_lio_odom"' in exporter
    assert "--reference-topic" in exporter
    assert "/robot/ground_truth/odom" in exporter


def test_task_map_core_loads_example_task_map():
    module = load_script_module("task_map_core.py")

    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    assert task_map["site"]["map_frame"] == "map"
    assert task_map["maps"]["nav2_map"].endswith(".yaml")
    assert task_map["motion_profiles"][0]["allow_reverse"] is False
    assert task_map["tasks"][0]["type"] == "taught_route"


def test_task_map_core_finds_region_for_pose():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    region = module.region_for_pose(task_map, x=1.0, y=1.0)

    assert region["id"] == "sim_yard"
    assert region["localization_mode"] == "OUTDOOR"


def test_localization_mode_supervisor_declares_mode_topics():
    script = read(WORKSPACE_DIR / "scripts" / "localization_mode_supervisor.py")

    assert "class LocalizationModeSupervisor" in script
    assert "/localization/global_odom" in script
    assert "/localization/wheel_lio_status" in script
    assert "/localization/supervised_mode" in script
    assert "OUTDOOR" in script
    assert "TRANSITION" in script
    assert "INDOOR" in script
    assert "DEGRADED" in script


def test_task_map_core_rejects_reverse_route_when_profile_disallows_reverse():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")
    route = {
        "id": "bad_reverse",
        "motion_profile": "forward_only_safe",
        "path": [
            {"pose": [0.0, 0.0, 0.0], "direction": "forward"},
            {"pose": [0.5, 0.0, 0.0], "direction": "reverse"},
        ],
    }

    assert module.route_allows_execution(task_map, route) is False


def test_task_map_core_rejects_duplicate_motion_profile_ids():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")
    task_map["motion_profiles"].append(dict(task_map["motion_profiles"][0]))

    with pytest.raises(ValueError, match="duplicate motion_profile id"):
        module.motion_profiles_by_id(task_map)


def test_task_map_core_rejects_invalid_core_field_types():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")
    task_map["site"] = None

    with pytest.raises(ValueError, match="task_map.site must be a mapping"):
        module.validate_task_map(task_map)


def test_task_map_core_converts_taught_route_to_goal_poses():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    poses = module.goal_poses_for_task(task_map, "daily_patrol")

    assert len(poses) == 3
    assert poses[0] == (0.0, 0.0, 0.0)
    assert poses[-1] == (2.0, 0.2, 0.1)


def test_goal_poses_rejects_task_without_route_context():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")
    del task_map["tasks"][0]["route"]

    with pytest.raises(ValueError, match="task daily_patrol missing route"):
        module.goal_poses_for_task(task_map, "daily_patrol")


def test_goal_poses_rejects_invalid_route_pose_context():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")
    task_map["recorded_routes"][0]["path"][1]["pose"] = [1.0, "bad", 0.0]

    with pytest.raises(ValueError, match="route .* point 1 pose"):
        module.goal_poses_for_task(task_map, "daily_patrol")


def test_route_recorder_samples_by_distance_or_yaw_change():
    module = load_script_module("task_map_core.py")
    samples = []

    assert module.should_append_route_sample(samples, (0.0, 0.0, 0.0), 0.3, 0.25)
    samples.append({"pose": [0.0, 0.0, 0.0], "direction": "forward"})
    assert not module.should_append_route_sample(samples, (0.1, 0.0, 0.01), 0.3, 0.25)
    assert module.should_append_route_sample(samples, (0.31, 0.0, 0.01), 0.3, 0.25)
    assert module.should_append_route_sample(samples, (0.1, 0.0, 0.30), 0.3, 0.25)


def test_route_recorder_direction_from_any_negative_linear_velocity():
    module = load_script_module("task_map_core.py")

    assert module.direction_from_linear_velocity(-0.00001) == "reverse"
    assert module.direction_from_linear_velocity(0.0) == "forward"
    assert module.direction_from_linear_velocity(0.00001) == "forward"


def test_route_recorder_declares_teach_interfaces_and_outputs_task_map():
    script = read(WORKSPACE_DIR / "scripts" / "route_recorder.py")

    assert "class RouteRecorder" in script
    assert 'declare_parameter("pose_topic", "/localization/global_odom")' in script
    assert "/teach/command" in script
    assert "/teach/status" in script
    assert "/localization/global_odom" in script
    assert "/localization/wheel_lio_odom" in script
    assert "/robot/odom" in script
    assert "mark_waypoint" in script
    assert "start_recording" in script
    assert "stop_recording" in script
    assert "save_task_map" in script
    assert "task_map.yaml" in script


def test_route_recorder_uses_single_configurable_pose_topic():
    script = read(WORKSPACE_DIR / "scripts" / "route_recorder.py")

    assert 'pose_topic = self.get_parameter("pose_topic").value' in script
    assert "Odometry, pose_topic, self.on_pose, 10" in script
    assert script.count("self.on_pose") == 1


def test_task_executor_exposes_task_commands_and_status_topics():
    script = read(WORKSPACE_DIR / "scripts" / "task_executor.py")

    assert "class TaskExecutor" in script
    assert "/task/command" in script
    assert "/task/status" in script
    assert "/task/current_goal" in script
    assert "NavigateThroughPoses" in script or "NavigateToPose" in script
    assert "BLOCKED" in script
    assert "allow_reverse" in script
    assert "unsupported_command" in script
    assert "task_already_running" in script
    assert "active_goal_handle" in script
    assert "nav_goal_pending" in script
    assert "def has_active_or_pending_goal" in script
    assert "STATUS_SUCCEEDED" in script
    assert "result.status == 4" not in script
    assert "if self.has_active_or_pending_goal():" in script
    assert 'if self.state == "RUNNING":' not in script
    assert "RUNNING; warning=unsupported_command" in script
    assert "active_goal=true" not in script
    for unsupported_state in ("PAUSED", "CANCELLED", "RETURNING_HOME"):
        assert unsupported_state not in script
