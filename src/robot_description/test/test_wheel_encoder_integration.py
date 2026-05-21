#!/usr/bin/env python3
"""Static integration checks for wheel encoder simulation wiring."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


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
    assert "DeclareLaunchArgument('sensing_bridge'" in launch
    assert "IfCondition(LaunchConfiguration('sensing_bridge'))" in launch


def test_robot_simulation_launch_allows_slow_gazebo_spawn_service():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "'-timeout', '120'" in launch


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
    assert "create_cloud" in script
    assert "<depend>sensor_msgs_py</depend>" in package
    assert "<exec_depend>python3-yaml</exec_depend>" in package
    assert "<exec_depend>ament_index_python</exec_depend>" in package


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
    assert config["lidar"]["rpy"] == [0.0, 0.524, 0.0]
    assert config["lidar"]["min_range"] == 0.1
    assert config["lidar"]["max_range"] == 100.0
    assert config["lidar"]["horizontal_fov"] == 6.28318
    assert config["lidar"]["vertical_fov"] == 0.5236

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
    assert laser_joint.find("origin").attrib["rpy"] == "0 0.524 0"

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

    assert "/sensing/lidar/points" in config
    assert "/sensing/imu/data" in config
    assert "laser_link" in config
    assert "base_link" in config
    assert "FAST-LIO2 precheck failed" in launch
    assert "fast_lio" in launch
    assert "/mapping/lio/odom" in launch
    assert "/mapping/lio/map_points" in launch


def test_fast_lio2_precheck_script_reports_missing_or_running_nodes():
    script = read(WORKSPACE_DIR / "scripts" / "verify_fast_lio2_precheck.sh")

    assert "fast_lio2.launch.py" in script
    assert "FAST-LIO2 precheck failed" in script
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
    assert "localization_modes.yaml" in launch
    assert "/localization/mode" in script
    assert "/localization/fusion_weights" in script
    assert "ros2 topic echo" in script


def test_navigation_uses_filtered_lidar_and_vehicle_footprint_contract():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    script = read(WORKSPACE_DIR / "scripts" / "verify_saved_map_nav2_precheck.sh")

    assert "topic: /sensing/lidar/points" in config
    assert "footprint:" in config
    assert "[0.275, 0.19]" in config
    assert "[-0.275, -0.19]" in config
    assert "navigation.launch.py" in script
    assert "/control/cmd_vel" in script
    assert "Navigation2 precheck failed" in script


def test_remote_extension_config_reserves_future_namespaces_without_runtime_dependency():
    config = read(WORKSPACE_DIR / "config" / "remote_extension.yaml")

    assert "/mission" in config
    assert "/maps" in config
    assert "/fleet" in config
    assert "/config" in config
    assert "cloud is not part of the real-time control loop" in config
    assert "vehicle must keep local navigation running" in config
    assert "versioned" in config
