# LIO-SAM2 Localization Nav2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a simulation-first LIO-SAM2 mapping, localization, and saved-map Nav2 navigation path with unified sensor/control interfaces and clear future real-vehicle adapter boundaries.

**Architecture:** Keep `robot_description` as the simulation owner, add lightweight remap/bridge launch boundaries for unified `/sensing/...` and `/control/cmd_vel` topics, update LIO-SAM2/localization/Nav2 config to consume those stable interfaces, and preserve `/robot/ground_truth/odom` strictly for evaluation. The first working target is saved-map navigation; online mapping while navigating remains a second-phase extension.

**Tech Stack:** ROS 2 Humble, Gazebo Classic 11, LIO-SAM2 launch/config integration, `robot_localization`, Nav2, `ament_cmake_pytest`, Python static tests, shell runtime verification scripts.

---

## Current Constraints

- The workspace root `/home/xavier/Workspace/ClaudeSpace` is not a git repository. Replace commit steps with file-change checkpoints unless the project is later placed under git.
- Current environment does not have required Nav2 packages installed.
- Current environment may not have LIO-SAM2 installed.
- Do not use `/robot/ground_truth/odom` as an input to localization or navigation.
- Keep `/robot/cmd_vel` as the simulation control input, but introduce `/control/cmd_vel` as the internal navigation command interface.

## File Structure

- Modify `ros2_robot_sim/README_项目进度.md`: record the verified baseline and next-stage scope.
- Create `ros2_robot_sim/launch/sensing_bridge.launch.py`: publish remap bridge nodes for current simulation topics into unified sensing topics.
- Modify `ros2_robot_sim/launch/robot_simulation.launch.py`: optionally include the sensing bridge by launch argument.
- Modify `ros2_robot_sim/config/lio_sam.yaml`: consume `/sensing/...` topics.
- Modify `ros2_robot_sim/launch/lio_sam2.launch.py`: remap LIO-SAM2 inputs from unified sensing topics.
- Modify `ros2_robot_sim/config/localization.yaml`: use unified sensing topics and keep ground truth out of fusion.
- Modify `ros2_robot_sim/launch/localization.launch.py`: document and expose GPS mode through parameters or launch arguments when supported.
- Modify `ros2_robot_sim/config/navigation.yaml`: correct Ackermann constraints, use non-holonomic velocity limits, and set minimum turning radius.
- Modify `ros2_robot_sim/launch/navigation.launch.py`: route `/control/cmd_vel` to `/robot/cmd_vel`.
- Modify `ros2_robot_sim/scripts/verify_runtime_topics.sh`: verify unified `/sensing/...` topics.
- Create `ros2_robot_sim/scripts/verify_localization.sh`: verify `/odometry/filtered`.
- Create `ros2_robot_sim/scripts/verify_navigation_precheck.sh`: verify Nav2 missing dependency handling or Nav2 node activation.
- Modify `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`: add static tests for the above contracts.

## Task 1: Record Current Baseline

**Files:**
- Modify: `ros2_robot_sim/README_项目进度.md`
- Test: manual document grep

- [ ] **Step 1: Add a baseline section to `README_项目进度.md`**

Add this section near the top, after the title:

```markdown
## 2026-05-20 当前基线

- 已切换到统一 `libackermann_drive_controller.so`，外部控制接口保持 `/robot/cmd_vel`。
- 已验证 `/robot/odom`、`/robot/ground_truth/odom`、`/tf`、`/robot/imu/data`、`/robot/velodyne_points`、`/robot/wheel_encoder/rear_average` 可发布。
- `/robot/ground_truth/odom` 仅用于仿真评估，不参与定位和导航闭环。
- `scripts/verify_runtime_topics.sh` 可验收关键运行话题。
- `localization.launch.py` 可启动 `robot_localization`，并输出 `/odometry/filtered`。
- `navigation.launch.py` 已有 Nav2 缺包预检查；当前环境缺 Nav2 必需包时应清晰提示。
- `lio_sam2.launch.py` 已有 LIO-SAM2 缺包预检查；建图链路尚未跑通。

下一阶段目标：

1. 建立 `/sensing/...` 与 `/control/cmd_vel` 统一接口。
2. 使用 LIO-SAM2 做仿真建图。
3. 使用 LiDAR + IMU + 轮速 + GPS 弱约束做可降级定位。
4. 先完成已建图 Nav2 导航，再扩展边建图边导航。
```

- [ ] **Step 2: Verify the baseline section exists**

Run:

```bash
rg -n "2026-05-20 当前基线|/robot/ground_truth/odom|/sensing|LIO-SAM2" ros2_robot_sim/README_项目进度.md
```

Expected: matching lines for the new baseline, ground truth odom rule, unified sensing target, and LIO-SAM2 target.

- [ ] **Step 3: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/README_项目进度.md
```

## Task 2: Add Unified Sensing Bridge Launch

**Files:**
- Create: `ros2_robot_sim/launch/sensing_bridge.launch.py`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_sensing_bridge_launch_remaps_simulation_topics -v`

- [ ] **Step 1: Write the failing static test**

Add this test to `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`:

```python
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

    assert "topic_tools" in launch
    assert "relay" in launch
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_sensing_bridge_launch_remaps_simulation_topics -v
```

Expected: FAIL with `FileNotFoundError` for `sensing_bridge.launch.py`.

- [ ] **Step 3: Create `sensing_bridge.launch.py`**

Create `ros2_robot_sim/launch/sensing_bridge.launch.py`:

```python
#!/usr/bin/env python3
"""Relay simulation sensor topics to stable /sensing interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def relay_node(name, input_topic, output_topic):
    return Node(
        package='topic_tools',
        executable='relay',
        name=name,
        output='screen',
        arguments=[input_topic, output_topic],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            (input_topic, output_topic),
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        relay_node(
            'relay_lidar_points',
            '/robot/velodyne_points',
            '/sensing/lidar/points',
        ),
        relay_node(
            'relay_imu_data',
            '/robot/imu/data',
            '/sensing/imu/data',
        ),
        relay_node(
            'relay_wheel_speed',
            '/robot/wheel_encoder/rear_average',
            '/sensing/wheel/speed',
        ),
        relay_node(
            'relay_gps_fix',
            '/robot/rtk_gps/fix',
            '/sensing/gps/fix',
        ),
    ])
```

- [ ] **Step 4: Run the static test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_sensing_bridge_launch_remaps_simulation_topics -v
```

Expected: PASS.

- [ ] **Step 5: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/launch/sensing_bridge.launch.py
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 3: Include Sensing Bridge In Simulation Launch

**Files:**
- Modify: `ros2_robot_sim/launch/robot_simulation.launch.py`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_robot_simulation_launch_can_enable_sensing_bridge -v`

- [ ] **Step 1: Write the failing static test**

Add this test:

```python
def test_robot_simulation_launch_can_enable_sensing_bridge():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "sensing_bridge.launch.py" in launch
    assert "sensing_bridge" in launch
    assert "DeclareLaunchArgument('sensing_bridge'" in launch
    assert "IfCondition(LaunchConfiguration('sensing_bridge'))" in launch
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_robot_simulation_launch_can_enable_sensing_bridge -v
```

Expected: FAIL because `robot_simulation.launch.py` does not include `sensing_bridge.launch.py`.

- [ ] **Step 3: Update imports in `robot_simulation.launch.py`**

Ensure the imports include:

```python
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
```

- [ ] **Step 4: Add launch configuration and include action**

Inside `generate_launch_description()`, add:

```python
    sensing_bridge = LaunchConfiguration('sensing_bridge', default='true')
```

After `spawn_robot`, add:

```python
    sensing_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'sensing_bridge.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('sensing_bridge')),
    )
```

In the returned `LaunchDescription`, add:

```python
        DeclareLaunchArgument('sensing_bridge', default_value='true',
            description='Relay simulation topics to /sensing interfaces'),
        TimerAction(period=5.0, actions=[sensing_bridge_launch]),
```

- [ ] **Step 5: Run the test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_robot_simulation_launch_can_enable_sensing_bridge -v
```

Expected: PASS.

- [ ] **Step 6: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/launch/robot_simulation.launch.py
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 4: Update LIO-SAM2 Config To Unified Topics

**Files:**
- Modify: `ros2_robot_sim/config/lio_sam.yaml`
- Modify: `ros2_robot_sim/launch/lio_sam2.launch.py`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_lio_sam_config_uses_unified_sensing_topics -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_lio_sam_config_uses_unified_sensing_topics -v
```

Expected: FAIL because config still points at `/robot/...` topics.

- [ ] **Step 3: Update `config/lio_sam.yaml` sensor topics**

Change the sensor section to:

```yaml
sensor:
  imu: "/sensing/imu/data"
  velodyne: "/sensing/lidar/points"
  gps: "/sensing/gps/fix"
  wheel_encoder:
    rear_average: "/sensing/wheel/speed"
    front_left: "/robot/wheel_encoder/front_left"
    front_right: "/robot/wheel_encoder/front_right"
    rear_left: "/robot/wheel_encoder/rear_left"
    rear_right: "/robot/wheel_encoder/rear_right"
```

Change:

```yaml
imu:
  imu_topic: "/sensing/imu/data"
```

- [ ] **Step 4: Update `launch/lio_sam2.launch.py` remaps**

Set remappings to:

```python
            remappings=[
                ('/points_raw', '/sensing/lidar/points'),
                ('/imu_raw', '/sensing/imu/data'),
                ('/odom_encoded', '/odometry/filtered'),
            ],
```

- [ ] **Step 5: Run the test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_lio_sam_config_uses_unified_sensing_topics -v
```

Expected: PASS.

- [ ] **Step 6: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/config/lio_sam.yaml
Changed: ros2_robot_sim/launch/lio_sam2.launch.py
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 5: Update Localization To Unified Topics And Keep Ground Truth Out

**Files:**
- Modify: `ros2_robot_sim/config/localization.yaml`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_uses_unified_topics_and_avoids_ground_truth -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
def test_localization_uses_unified_topics_and_avoids_ground_truth():
    config = read(WORKSPACE_DIR / "config" / "localization.yaml")

    assert "odom0: /robot/ground_truth/odom" not in config
    assert "/robot/ground_truth/odom" not in config
    assert "odom0: /robot/odom" in config
    assert "imu0: /sensing/imu/data" in config
    assert "gps_quality_gate:" in config
    assert "max_acceptable_covariance: 25.0" in config
    assert "max_position_jump: 3.0" in config
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_uses_unified_topics_and_avoids_ground_truth -v
```

Expected: FAIL because `imu0` still uses `/robot/imu/data` and GPS quality gate config does not exist.

- [ ] **Step 3: Update `config/localization.yaml` IMU input**

Change:

```yaml
    imu0: /robot/imu/data
```

to:

```yaml
    imu0: /sensing/imu/data
```

- [ ] **Step 4: Add GPS quality gate config block**

Add this block under `ekf_filter_node.ros__parameters`:

```yaml
    gps_quality_gate:
      topic: /sensing/gps/fix
      mode: weak_constraint
      max_acceptable_covariance: 25.0
      max_position_jump: 3.0
      dropout_timeout: 2.0
      recovery_blend_time: 5.0
```

This block documents the required GPS gate behavior. A later task can implement an adapter node that consumes it.

- [ ] **Step 5: Run the test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_uses_unified_topics_and_avoids_ground_truth -v
```

Expected: PASS.

- [ ] **Step 6: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/config/localization.yaml
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 6: Update Runtime Verification For Unified Topics

**Files:**
- Modify: `ros2_robot_sim/scripts/verify_runtime_topics.sh`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_runtime_sensor_verification_script_covers_unified_sensing_topics -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_runtime_sensor_verification_script_covers_unified_sensing_topics -v
```

Expected: FAIL because `verify_runtime_topics.sh` does not check unified sensing topics.

- [ ] **Step 3: Update `scripts/verify_runtime_topics.sh` topic checks**

Add:

```bash
require_topic "/sensing/lidar/points"
require_topic "/sensing/imu/data"
require_topic "/sensing/wheel/speed"
require_topic "/sensing/gps/fix"
```

Add:

```bash
require_echo_once "/sensing/lidar/points"
require_echo_once "/sensing/imu/data"
require_echo_once "/sensing/wheel/speed"
require_echo_once "/sensing/gps/fix"
```

- [ ] **Step 4: Run the static test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_runtime_sensor_verification_script_covers_unified_sensing_topics -v
```

Expected: PASS.

- [ ] **Step 5: Runtime verification**

Start simulation:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./start.sh --no-rviz
```

In another terminal:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./scripts/verify_runtime_topics.sh
```

Expected:

```text
runtime topic verification passed
```

Stop simulation:

```bash
./stop.sh -f
```

- [ ] **Step 6: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/scripts/verify_runtime_topics.sh
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 7: Add Localization Runtime Verification Script

**Files:**
- Create: `ros2_robot_sim/scripts/verify_localization.sh`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_verification_script_checks_filtered_odom -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
def test_localization_verification_script_checks_filtered_odom():
    script = read(WORKSPACE_DIR / "scripts" / "verify_localization.sh")

    assert "/odometry/filtered" in script
    assert "localization.launch.py" in script
    assert "ros2 topic echo" in script
    assert "ROS_LOG_DIR" in script
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_verification_script_checks_filtered_odom -v
```

Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Create `scripts/verify_localization.sh`**

Create:

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_SOURCE="/opt/ros/humble/setup.bash"
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source "$ROS2_SOURCE"
source "$SETUP_FILE"
set -u

echo "starting localization.launch.py"
timeout 20s ros2 launch robot_description localization.launch.py &
LAUNCH_PID=$!

cleanup() {
    kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 4

if ! timeout 8s ros2 topic echo /odometry/filtered --once >/dev/null; then
    echo "no message received on /odometry/filtered" >&2
    exit 1
fi

echo "localization verification passed"
```

- [ ] **Step 4: Make the script executable**

Run:

```bash
chmod +x ros2_robot_sim/scripts/verify_localization.sh
```

- [ ] **Step 5: Run the static test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_localization_verification_script_checks_filtered_odom -v
```

Expected: PASS.

- [ ] **Step 6: Runtime verification**

With simulation running:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./scripts/verify_localization.sh
```

Expected:

```text
localization verification passed
```

- [ ] **Step 7: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/scripts/verify_localization.sh
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 8: Correct Nav2 Ackermann Constraints And Command Interface

**Files:**
- Modify: `ros2_robot_sim/config/navigation.yaml`
- Modify: `ros2_robot_sim/launch/navigation.launch.py`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_config_respects_ackermann_constraints -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
def test_navigation_config_respects_ackermann_constraints():
    config = read(WORKSPACE_DIR / "config" / "navigation.yaml")
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")

    assert "DWB_MAX_VEL_Y: 0.0" in config
    assert "DWB_MIN_VEL_Y: 0.0" in config
    assert "min_turning_radius: 0.78" in config
    assert "w_reverse_cost: 2.5" in config
    assert "('/cmd_vel', '/control/cmd_vel')" in launch
    assert "('/control/cmd_vel', '/robot/cmd_vel')" in launch
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_config_respects_ackermann_constraints -v
```

Expected: FAIL because `min_turning_radius` is `0.20` and launch remaps directly to `/robot/cmd_vel`.

- [ ] **Step 3: Update `config/navigation.yaml`**

Change:

```yaml
      min_turning_radius: 0.20
```

to:

```yaml
      min_turning_radius: 0.78
```

Keep:

```yaml
    DWB_MIN_VEL_Y: 0.0
    DWB_MAX_VEL_Y: 0.0
      w_reverse_cost: 2.5
```

- [ ] **Step 4: Update `launch/navigation.launch.py` remaps**

For `controller_server`, use:

```python
            remappings=[
                ('/cmd_vel', '/control/cmd_vel'),
                ('/control/cmd_vel', '/robot/cmd_vel'),
            ],
```

For `velocity_smoother`, use:

```python
                remappings=[
                    ('/cmd_vel_raw', '/control/cmd_vel'),
                    ('/cmd_vel_smooth', '/robot/cmd_vel'),
                ],
```

- [ ] **Step 5: Run the test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_config_respects_ackermann_constraints -v
```

Expected: PASS.

- [ ] **Step 6: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/config/navigation.yaml
Changed: ros2_robot_sim/launch/navigation.launch.py
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 9: Add Nav2 Precheck Verification Script

**Files:**
- Create: `ros2_robot_sim/scripts/verify_navigation_precheck.sh`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_precheck_script_reports_missing_nav2_or_active_nodes -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
def test_navigation_precheck_script_reports_missing_nav2_or_active_nodes():
    script = read(WORKSPACE_DIR / "scripts" / "verify_navigation_precheck.sh")

    assert "navigation.launch.py" in script
    assert "Navigation2 precheck failed" in script
    assert "controller_server" in script
    assert "bt_navigator" in script
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_precheck_script_reports_missing_nav2_or_active_nodes -v
```

Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Create `scripts/verify_navigation_precheck.sh`**

Create:

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

OUTPUT="$(timeout 15s ros2 launch robot_description navigation.launch.py 2>&1 || true)"
echo "$OUTPUT"

if echo "$OUTPUT" | grep -q "Navigation2 precheck failed"; then
    echo "navigation precheck reported missing Nav2 dependencies"
    exit 0
fi

if timeout 8s ros2 node list | grep -Eq "controller_server|bt_navigator"; then
    echo "navigation nodes are present"
    exit 0
fi

echo "navigation precheck did not report missing dependencies and Nav2 nodes were not found" >&2
exit 1
```

- [ ] **Step 4: Make the script executable**

Run:

```bash
chmod +x ros2_robot_sim/scripts/verify_navigation_precheck.sh
```

- [ ] **Step 5: Run the static test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_precheck_script_reports_missing_nav2_or_active_nodes -v
```

Expected: PASS.

- [ ] **Step 6: Runtime verification**

Run:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./scripts/verify_navigation_precheck.sh
```

Expected, when Nav2 is not installed:

```text
navigation precheck reported missing Nav2 dependencies
```

Expected, when Nav2 is installed:

```text
navigation nodes are present
```

- [ ] **Step 7: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/scripts/verify_navigation_precheck.sh
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 10: Add GPS Degradation Test Scenario Documentation

**Files:**
- Create: `ros2_robot_sim/docs/superpowers/specs/2026-05-20-gps-degradation-test-scenarios.md`
- Modify: `ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py`
- Test: `python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_gps_degradation_scenarios_are_documented -v`

- [ ] **Step 1: Write the failing static test**

Add:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_gps_degradation_scenarios_are_documented -v
```

Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Create the GPS scenario document**

Create `ros2_robot_sim/docs/superpowers/specs/2026-05-20-gps-degradation-test-scenarios.md`:

```markdown
# GPS Degradation Test Scenarios

## Purpose

Verify that localization and Nav2 remain usable when GPS quality changes.

## Scenario 1: GPS good

- `/sensing/gps/fix` publishes a valid fix with low covariance.
- Localization should remain stable.
- Ground-truth comparison uses `/robot/ground_truth/odom`.
- Expected result: map-frame drift is lower than GPS-off mode.

## Scenario 2: GPS outage

- `/sensing/gps/fix` is absent or ignored.
- Localization continues with LiDAR, IMU, and wheel speed.
- Expected result: `/odometry/filtered` continues publishing and Nav2 can complete a short saved-map goal.

## Scenario 3: GPS jump

- `/sensing/gps/fix` changes abruptly by more than `max_position_jump`.
- GPS quality gate should reject or down-weight the jump.
- Expected result: `map -> odom -> base_footprint` does not jump abruptly.

## Metrics

- Drift against `/robot/ground_truth/odom`.
- Goal success rate.
- Maximum pose jump during GPS recovery.
- Time until localization recovers after GPS outage.
```

- [ ] **Step 4: Run the test**

Run:

```bash
python3 -m pytest ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py::test_gps_degradation_scenarios_are_documented -v
```

Expected: PASS.

- [ ] **Step 5: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/docs/superpowers/specs/2026-05-20-gps-degradation-test-scenarios.md
Changed: ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py
```

## Task 11: Run Full Verification

**Files:**
- No source changes expected.
- Test: all static tests, build, package tests, runtime checks.

- [ ] **Step 1: Run Python static tests**

Run:

```bash
python3 -m pytest \
  ros2_robot_sim/src/robot_description/test/test_ackermann_kinematics.py \
  ros2_robot_sim/src/robot_description/test/test_wheel_encoder_integration.py \
  -v
```

Expected: all tests pass.

- [ ] **Step 2: Build package**

Run:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
```

Expected:

```text
Summary: 1 package finished
```

- [ ] **Step 3: Run package tests**

Run:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_description --event-handlers console_direct+
```

Expected:

```text
100% tests passed, 0 tests failed
```

- [ ] **Step 4: Run runtime topic verification**

Run:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./start.sh --no-rviz --no-build
./scripts/verify_runtime_topics.sh
./stop.sh -f
```

Expected:

```text
runtime topic verification passed
```

- [ ] **Step 5: Run localization verification**

Run simulation in one terminal:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./start.sh --no-rviz --no-build
```

Run localization check in another terminal:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./scripts/verify_localization.sh
```

Expected:

```text
localization verification passed
```

Stop simulation:

```bash
./stop.sh -f
```

- [ ] **Step 6: Run Nav2 precheck verification**

Run:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./scripts/verify_navigation_precheck.sh
```

Expected when Nav2 is absent:

```text
navigation precheck reported missing Nav2 dependencies
```

Expected when Nav2 is installed:

```text
navigation nodes are present
```

- [ ] **Step 7: Final checkpoint**

Record all changed files and verification outputs in the final implementation report.

## Task 12: Phase 2 Planning Boundary

**Files:**
- Modify: `ros2_robot_sim/docs/superpowers/specs/2026-05-20-lio-sam2-localization-nav2-design.md`
- Test: manual review

- [ ] **Step 1: Add Phase 2 boundary note**

Add this section to the design spec:

```markdown
## Phase 2 Boundary

After saved-map navigation passes, the next design cycle should cover online mapping while navigating. That phase should decide how LIO-SAM2 map updates are converted into Nav2 map or costmap updates, and whether autonomous exploration is included.
```

- [ ] **Step 2: Verify the note exists**

Run:

```bash
rg -n "Phase 2 Boundary|online mapping while navigating|costmap updates" ros2_robot_sim/docs/superpowers/specs/2026-05-20-lio-sam2-localization-nav2-design.md
```

Expected: all three phrases are present.

- [ ] **Step 3: Checkpoint changed files**

Record:

```text
Changed: ros2_robot_sim/docs/superpowers/specs/2026-05-20-lio-sam2-localization-nav2-design.md
```
