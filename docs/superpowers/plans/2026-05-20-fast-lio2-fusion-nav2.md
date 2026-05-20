# FAST-LIO2 融合定位与 Nav2 保存地图导航实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前 LIO-SAM2 主线迁移为 FAST-LIO2 / FAST-LIO ROS 2 前端 + GPS/轮速/IMU/LiDAR 融合定位 + Nav2 保存地图导航，并为真实车、雷达外参、自车点云过滤、云端客户端扩展预留清晰接口。

**架构：** 保留 `/sensing/...` 与 `/control/cmd_vel` 统一接口。新增车辆几何与传感器安装配置，先过滤有效 3D 点云，再接入 FAST-LIO2 前端输出 `/mapping/lio/odom`；定位模式管理器根据 GPS 质量和作业区域发布 OUTDOOR/TRANSITION/BARN 模式与融合权重；Nav2 先只做保存地图导航闭环。

**技术栈：** ROS 2 Humble、Gazebo Classic 11、FAST-LIO2/FAST-LIO ROS 2 源码包、`robot_localization`、Nav2、Python/rclpy 工具节点、`ament_cmake_pytest`、shell 运行时验证脚本。

---

## 当前约束

- `ros2_robot_sim` 是 Git 仓库，当前 `main` 已领先 `origin/main` 2 个文档提交。
- 当前环境未安装 Nav2 二进制包。
- 当前仓库没有 FAST-LIO2 ROS 2 源码包。
- 不把 `/robot/ground_truth/odom` 用作生产定位或导航输入。
- 本阶段不实现云端服务端、客户端、4G 通信、远程任务下发、靠边控制、动态牛只识别。
- 本阶段只预留 `/mission/*`、`/maps/*`、`/fleet/*`、`/config/*` 命名空间和配置版本化约束。

## 文件结构

- 创建 `config/vehicle_geometry.yaml`：车辆几何、运动学、footprint、自车点云过滤包围盒，所有关键参数带单位和坐标系注释。
- 创建 `config/sensor_mount.yaml`：LiDAR/IMU/GPS 安装外参、LiDAR 有效范围和视场配置，所有关键参数带单位和坐标系注释。
- 创建 `scripts/lidar_self_filter.py`：订阅原始点云，按车辆包围盒与距离范围过滤，发布 `/sensing/lidar/points_filtered`。
- 修改 `src/robot_description/package.xml`：声明 `sensor_msgs_py` 和 Python YAML 运行依赖。
- 修改 `launch/sensing_bridge.launch.py`：LiDAR 原始桥接到 `/sensing/lidar/points_raw`，启动自车过滤节点，并把默认算法输入 `/sensing/lidar/points` 指向过滤后点云。
- 创建 `config/fast_lio.yaml`：FAST-LIO2 前端配置，使用 `/sensing/lidar/points` 与 `/sensing/imu/data`。
- 创建 `launch/fast_lio2.launch.py`：FAST-LIO2 缺包预检查与 launch 边界，输出 `/mapping/lio/odom` 和 `/mapping/lio/map_points` 约定。
- 保留但降级 `launch/lio_sam2.launch.py`：只作为兼容提醒或废弃入口，不作为主线验证目标。
- 创建 `scripts/verify_lidar_mount.sh`：检查 TF、点云 frame、点云消息和过滤后点云。
- 创建 `scripts/verify_fast_lio2_precheck.sh`：检查 FAST-LIO2 缺包提示或节点启动。
- 创建 `scripts/verify_saved_map_nav2_precheck.sh`：检查 Nav2 依赖、地图文件、`/control/cmd_vel` 约定。
- 创建 `config/localization_modes.yaml`：OUTDOOR/TRANSITION/BARN 模式阈值与融合权重。
- 创建 `scripts/localization_mode_manager.py`：第一阶段模式管理器，发布 `/localization/mode`、`/localization/fusion_weights`、`/localization/gps/gated`。
- 修改 `config/localization.yaml`：记录 FAST-LIO2 odom、wheel odom、gated GPS 的融合入口；第一阶段允许 wheel/GPS 以接口配置形式预留。
- 修改 `config/navigation.yaml`：Nav2 点云/costmap 默认使用 `/sensing/lidar/points` 或过滤后点云，footprint 与 `vehicle_geometry.yaml` 对齐。
- 修改 `README_项目进度.md`：将下一阶段主线从 LIO-SAM2 改为 FAST-LIO2。
- 修改 `src/robot_description/test/test_wheel_encoder_integration.py`：增加静态契约测试。
- 修改或新增 `docs/superpowers/plans/2026-05-20-lio-sam2-localization-nav2.md` 顶部说明：该计划已被 FAST-LIO2 计划取代。

## 任务 1：标记 LIO-SAM2 计划被 FAST-LIO2 计划取代

**文件：**
- 修改：`docs/superpowers/plans/2026-05-20-lio-sam2-localization-nav2.md`
- 修改：`README_项目进度.md`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

在 `src/robot_description/test/test_wheel_encoder_integration.py` 末尾添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lio_sam2_plan_is_superseded_by_fast_lio2_plan -v
```

预期：FAIL，因为旧计划没有 superseded 标记，进度文档仍保留 LIO-SAM2 作为下一步主线。

- [ ] **步骤 3：更新旧计划头部**

在 `docs/superpowers/plans/2026-05-20-lio-sam2-localization-nav2.md` 标题后插入：

```markdown
> **Superseded by:** `docs/superpowers/plans/2026-05-20-fast-lio2-fusion-nav2.md`
>
> 该计划保留为历史记录。当前主线已切换为 FAST-LIO2 / FAST-LIO ROS 2 前端 + 融合定位 + Nav2 保存地图导航。
```

- [ ] **步骤 4：更新进度文档**

在 `README_项目进度.md` 中把 “LIO-SAM2建图未验证” 改为：

```markdown
2. **FAST-LIO2建图未接入** - 已确定以 FAST-LIO2 / FAST-LIO ROS 2 前端替代 LIO-SAM2 主线
```

把下一步命令示例改为：

```bash
# 2. 验证FAST-LIO2建图前端
ros2 launch robot_description fast_lio2.launch.py
```

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lio_sam2_plan_is_superseded_by_fast_lio2_plan -v
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add README_项目进度.md docs/superpowers/plans/2026-05-20-lio-sam2-localization-nav2.md src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "docs: 标记 FAST-LIO2 取代 LIO-SAM2 主线"
```

## 任务 2：新增统一车辆几何配置

**文件：**
- 创建：`config/vehicle_geometry.yaml`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_vehicle_geometry_config_documents_units_and_self_filter():
    config = read(WORKSPACE_DIR / "config" / "vehicle_geometry.yaml")

    assert "wheelbase: 0.45" in config
    assert "track_width: 0.35" in config
    assert "wheel_radius: 0.07" in config
    assert "max_steering_angle: 0.5236" in config
    assert "min_turning_radius: 0.78" in config
    assert "length: 0.55" in config
    assert "width: 0.38" in config
    assert "height: 0.12" in config
    assert "box_min: [-0.35, -0.25, -0.05]" in config
    assert "box_max: [0.35, 0.25, 0.45]" in config
    assert "unit m" in config
    assert "base_link" in config
    assert "X forward positive" in config
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_vehicle_geometry_config_documents_units_and_self_filter -v
```

预期：FAIL，`vehicle_geometry.yaml` 不存在。

- [ ] **步骤 3：创建 `config/vehicle_geometry.yaml`**

写入：

```yaml
vehicle:
  # wheelbase: distance between front axle center and rear axle center, unit m.
  # Affects Ackermann kinematics, wheel odometry, and Nav2 minimum turning radius.
  wheelbase: 0.45

  # track_width: lateral distance between left and right wheel centers, unit m.
  # Affects URDF wheel placement and wheel-speed geometry.
  track_width: 0.35

  # wheel_radius: wheel radius, unit m.
  # Affects wheel encoder angular-to-linear speed conversion.
  wheel_radius: 0.07

  # max_steering_angle: maximum front-wheel steering angle, unit rad.
  # Affects min_turning_radius and Ackermann command limits.
  max_steering_angle: 0.5236

  # min_turning_radius: minimum feasible turning radius, unit m.
  # Computed as wheelbase / tan(max_steering_angle), about 0.78 m.
  min_turning_radius: 0.78

  body:
    # length: vehicle body envelope along base_link +X direction, unit m.
    # X forward positive. Used by URDF checks, Nav2 footprint, and LiDAR self filtering.
    length: 0.55

    # width: vehicle body envelope along base_link Y axis, unit m.
    # Y left positive and right negative.
    width: 0.38

    # height: vehicle body envelope along base_link Z axis, unit m.
    # Z upward positive.
    height: 0.12

  footprint:
    # polygon: 2D vehicle footprint in base_link frame, unit m.
    # Each point is [x, y]. X forward positive, Y left positive.
    polygon:
      - [0.275, 0.19]
      - [0.275, -0.19]
      - [-0.275, -0.19]
      - [-0.275, 0.19]

  self_filter:
    # frame: coordinate frame for self-filter box.
    frame: base_link

    # box_min / box_max: 3D vehicle self-filter box in base_link frame, unit m.
    # Points inside this box are treated as vehicle body returns and removed.
    box_min: [-0.35, -0.25, -0.05]
    box_max: [0.35, 0.25, 0.45]
```

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_vehicle_geometry_config_documents_units_and_self_filter -v
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add config/vehicle_geometry.yaml src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加车辆几何统一配置"
```

## 任务 3：新增传感器安装与点云有效性配置

**文件：**
- 创建：`config/sensor_mount.yaml`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_sensor_mount_config_documents_lidar_extrinsics():
    config = read(WORKSPACE_DIR / "config" / "sensor_mount.yaml")

    assert "parent_frame: base_link" in config
    assert "frame: laser_link" in config
    assert "xyz: [0.0, 0.0, 0.18]" in config
    assert "rpy: [0.0, 0.524, 0.0]" in config
    assert "min_range: 0.1" in config
    assert "max_range: 100.0" in config
    assert "horizontal_fov: 6.28318" in config
    assert "vertical_fov: 0.5236" in config
    assert "unit rad" in config
    assert "Right-hand rule" in config
    assert "X forward positive" in config
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_sensor_mount_config_documents_lidar_extrinsics -v
```

预期：FAIL，`sensor_mount.yaml` 不存在。

- [ ] **步骤 3：创建 `config/sensor_mount.yaml`**

写入：

```yaml
lidar:
  # parent_frame: frame that LiDAR extrinsics are relative to.
  parent_frame: base_link

  # frame: LiDAR point cloud frame_id.
  frame: laser_link

  # xyz: LiDAR origin relative to parent_frame, unit m.
  # X forward positive, Y left positive, Z upward positive.
  xyz: [0.0, 0.0, 0.18]

  # rpy: LiDAR roll, pitch, yaw relative to parent_frame, unit rad.
  # Right-hand rule: roll about X, pitch about Y, yaw about Z.
  rpy: [0.0, 0.524, 0.0]

  # min_range / max_range: accepted LiDAR point distance, unit m.
  min_range: 0.1
  max_range: 100.0

  # horizontal_fov / vertical_fov: expected useful field of view, unit rad.
  # Used by simulation sanity checks and later real LiDAR validation.
  horizontal_fov: 6.28318
  vertical_fov: 0.5236

imu:
  # parent_frame: frame that IMU extrinsics are relative to.
  parent_frame: base_link

  # frame: IMU message frame_id.
  frame: imu_link

  # xyz: IMU origin relative to parent_frame, unit m.
  xyz: [0.0, 0.0, 0.08]

  # rpy: IMU roll, pitch, yaw relative to parent_frame, unit rad.
  rpy: [0.0, 0.0, 0.0]

gps:
  # parent_frame: frame that GPS antenna position is relative to.
  parent_frame: base_link

  # frame: GPS fix frame_id.
  frame: gps_link

  # xyz: GPS antenna origin relative to parent_frame, unit m.
  xyz: [0.0, 0.0, 0.3]
```

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_sensor_mount_config_documents_lidar_extrinsics -v
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add config/sensor_mount.yaml src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加传感器安装外参配置"
```

## 任务 4：新增 LiDAR 自车点云过滤节点

**文件：**
- 创建：`scripts/lidar_self_filter.py`
- 修改：`src/robot_description/package.xml`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_lidar_self_filter_script_filters_vehicle_box_and_ranges():
    script = read(WORKSPACE_DIR / "scripts" / "lidar_self_filter.py")
    package = read(PACKAGE_DIR / "package.xml")

    assert "PointCloud2" in script
    assert "vehicle_geometry.yaml" in script
    assert "sensor_mount.yaml" in script
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lidar_self_filter_script_filters_vehicle_box_and_ranges -v
```

预期：FAIL，脚本不存在。

- [ ] **步骤 3：创建最小可运行脚本**

创建 `scripts/lidar_self_filter.py`：

```python
#!/usr/bin/env python3
"""Filter vehicle-body returns and invalid ranges from LiDAR PointCloud2."""

from pathlib import Path
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import yaml


WORKSPACE_DIR = Path(__file__).resolve().parents[1]


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def inside_box(point, box_min, box_max):
    x, y, z = point[:3]
    return (
        box_min[0] <= x <= box_max[0]
        and box_min[1] <= y <= box_max[1]
        and box_min[2] <= z <= box_max[2]
    )


def in_range(point, min_range, max_range):
    x, y, z = point[:3]
    distance = math.sqrt(x * x + y * y + z * z)
    return min_range <= distance <= max_range


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__("lidar_self_filter")
        vehicle = load_yaml(WORKSPACE_DIR / "config" / "vehicle_geometry.yaml")
        mount = load_yaml(WORKSPACE_DIR / "config" / "sensor_mount.yaml")
        self.box_min = vehicle["vehicle"]["self_filter"]["box_min"]
        self.box_max = vehicle["vehicle"]["self_filter"]["box_max"]
        self.min_range = mount["lidar"]["min_range"]
        self.max_range = mount["lidar"]["max_range"]
        self.publisher = self.create_publisher(
            PointCloud2,
            "/sensing/lidar/points_filtered",
            10,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            "/sensing/lidar/points_raw",
            self.on_cloud,
            10,
        )

    def on_cloud(self, msg):
        points = []
        for point in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            if inside_box(point, self.box_min, self.box_max):
                continue
            if not in_range(point, self.min_range, self.max_range):
                continue
            points.append(point)
        filtered = point_cloud2.create_cloud(
            msg.header,
            msg.fields[:3],
            points,
        )
        self.publisher.publish(filtered)


def main():
    rclpy.init()
    node = LidarSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 4：更新 `src/robot_description/package.xml` 运行依赖**

在已有消息依赖附近增加：

```xml
  <depend>sensor_msgs_py</depend>
```

在 `ros2launch` 执行依赖附近增加：

```xml
  <exec_depend>python3-yaml</exec_depend>
```

- [ ] **步骤 5：设置可执行权限**

```bash
chmod +x scripts/lidar_self_filter.py
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lidar_self_filter_script_filters_vehicle_box_and_ranges -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add scripts/lidar_self_filter.py src/robot_description/package.xml src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 LiDAR 自车点云过滤节点"
```

## 任务 5：把 sensing bridge 改为 raw -> filtered -> algorithm input

**文件：**
- 修改：`launch/sensing_bridge.launch.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_sensing_bridge_routes_lidar_through_self_filter():
    launch = read(WORKSPACE_DIR / "launch" / "sensing_bridge.launch.py")

    assert "('/robot/velodyne_points', '/sensing/lidar/points_raw')" in launch
    assert "lidar_self_filter.py" in launch
    assert "('/sensing/lidar/points_filtered', '/sensing/lidar/points')" in launch
    assert "('/robot/velodyne_points', '/sensing/lidar/points')" not in launch
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_sensing_bridge_routes_lidar_through_self_filter -v
```

预期：FAIL，因为当前 LiDAR 直接桥接到 `/sensing/lidar/points`。

- [ ] **步骤 3：更新 `launch/sensing_bridge.launch.py`**

将：

```python
('/robot/velodyne_points', '/sensing/lidar/points')
```

改为：

```python
('/robot/velodyne_points', '/sensing/lidar/points_raw')
```

增加过滤节点：

```python
    lidar_self_filter = Node(
        package='robot_description',
        executable='lidar_self_filter.py',
        name='lidar_self_filter',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
```

增加 filtered 到 algorithm topic 的 relay：

```python
SENSING_REMAPS.append(
    ('/sensing/lidar/points_filtered', '/sensing/lidar/points')
)
SENSING_RELAYS.append(
    ('relay_lidar_points_filtered', 'sensor_msgs/msg/PointCloud2')
)
```

在 `LaunchDescription` 中加入 `lidar_self_filter`。

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_sensing_bridge_routes_lidar_through_self_filter -v
```

预期：PASS。

- [ ] **步骤 5：运行 runtime topic 静态覆盖测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_runtime_sensor_verification_script_covers_unified_sensing_topics -v
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add launch/sensing_bridge.launch.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 接入 LiDAR 过滤后的算法点云"
```

## 任务 6：更新运行时验证脚本覆盖过滤点云和雷达外参

**文件：**
- 修改：`scripts/verify_runtime_topics.sh`
- 创建：`scripts/verify_lidar_mount.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lidar_runtime_verification_checks_filtered_cloud_and_tf -v
```

预期：FAIL，`verify_lidar_mount.sh` 不存在，runtime 脚本未检查 raw/filtered。

- [ ] **步骤 3：更新 `verify_runtime_topics.sh`**

增加：

```bash
require_topic "/sensing/lidar/points_raw"
require_topic "/sensing/lidar/points_filtered"
require_echo_once "/sensing/lidar/points_raw"
require_echo_once "/sensing/lidar/points_filtered"
```

- [ ] **步骤 4：创建 `scripts/verify_lidar_mount.sh`**

写入：

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

if ! timeout 8s ros2 run tf2_ros tf2_echo base_link laser_link >/dev/null; then
    echo "missing TF: base_link -> laser_link" >&2
    exit 1
fi

if ! timeout 8s ros2 topic echo /sensing/lidar/points_filtered --once >/dev/null; then
    echo "no filtered LiDAR point cloud received" >&2
    exit 1
fi

FRAME_ID="$(timeout 8s ros2 topic echo /sensing/lidar/points_filtered --once --field header.frame_id || true)"
if [ -z "$FRAME_ID" ]; then
    echo "filtered LiDAR frame_id is empty" >&2
    exit 1
fi

echo "lidar mount verification passed"
```

- [ ] **步骤 5：设置可执行权限**

```bash
chmod +x scripts/verify_lidar_mount.sh
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_lidar_runtime_verification_checks_filtered_cloud_and_tf -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add scripts/verify_runtime_topics.sh scripts/verify_lidar_mount.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: 验证 LiDAR 外参与过滤点云"
```

## 任务 7：新增 FAST-LIO2 配置与 launch 预检查

**文件：**
- 创建：`config/fast_lio.yaml`
- 创建：`launch/fast_lio2.launch.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio2_config_and_launch_use_filtered_sensing_topics -v
```

预期：FAIL，配置和 launch 不存在。

- [ ] **步骤 3：创建 `config/fast_lio.yaml`**

写入：

```yaml
fast_lio:
  ros__parameters:
    # point_cloud_topic: FAST-LIO2 LiDAR input topic.
    # Uses self-filtered 3D point cloud from the unified sensing interface.
    point_cloud_topic: /sensing/lidar/points

    # imu_topic: FAST-LIO2 IMU input topic.
    imu_topic: /sensing/imu/data

    # lidar_frame: LiDAR frame_id, must match sensor_mount.yaml and TF.
    lidar_frame: laser_link

    # body_frame: vehicle base frame used by localization and Nav2.
    body_frame: base_link

    # odom_topic: normalized LIO odometry output consumed by fusion.
    odom_topic: /mapping/lio/odom

    # map_points_topic: normalized map cloud output for saving and inspection.
    map_points_topic: /mapping/lio/map_points
```

- [ ] **步骤 4：创建 `launch/fast_lio2.launch.py`**

写入：

```python
#!/usr/bin/env python3
"""Launch FAST-LIO2 or compatible FAST-LIO ROS 2 front end."""

import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FAST_LIO_PACKAGE_CANDIDATES = [
    'fast_lio',
    'fast_lio2',
]


def find_fast_lio_package():
    for package_name in FAST_LIO_PACKAGE_CANDIDATES:
        try:
            get_package_share_directory(package_name)
            return package_name
        except PackageNotFoundError:
            continue
    return None


def generate_launch_description():
    launch_actions = [
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    package_name = find_fast_lio_package()
    if package_name is None:
        launch_actions.append(LogInfo(
            msg=(
                'FAST-LIO2 precheck failed: missing package fast_lio or fast_lio2. '
                'Build a ROS 2 compatible FAST-LIO front end in this workspace, '
                'then rerun this launch file.'
            )
        ))
        return LaunchDescription(launch_actions)

    config_file = os.path.join(
        get_package_share_directory('robot_description'),
        'config',
        'fast_lio.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_actions.append(Node(
        package=package_name,
        executable=package_name,
        name='fast_lio2',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/points_raw', '/sensing/lidar/points'),
            ('/imu_raw', '/sensing/imu/data'),
            ('/Odometry', '/mapping/lio/odom'),
            ('/cloud_registered', '/mapping/lio/map_points'),
        ],
    ))
    return LaunchDescription(launch_actions)
```

- [ ] **步骤 5：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio2_config_and_launch_use_filtered_sensing_topics -v
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add config/fast_lio.yaml launch/fast_lio2.launch.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 FAST-LIO2 前端配置与启动入口"
```

## 任务 8：新增 FAST-LIO2 预检查脚本

**文件：**
- 创建：`scripts/verify_fast_lio2_precheck.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_fast_lio2_precheck_script_reports_missing_or_running_nodes():
    script = read(WORKSPACE_DIR / "scripts" / "verify_fast_lio2_precheck.sh")

    assert "fast_lio2.launch.py" in script
    assert "FAST-LIO2 precheck failed" in script
    assert "/mapping/lio/odom" in script
    assert "/mapping/lio/map_points" in script
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio2_precheck_script_reports_missing_or_running_nodes -v
```

预期：FAIL，脚本不存在。

- [ ] **步骤 3：创建脚本**

写入 `scripts/verify_fast_lio2_precheck.sh`：

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

OUTPUT="$(timeout 15s ros2 launch robot_description fast_lio2.launch.py 2>&1 || true)"
echo "$OUTPUT"

if echo "$OUTPUT" | grep -q "FAST-LIO2 precheck failed"; then
    echo "fast-lio2 precheck reported missing FAST-LIO dependency"
    exit 0
fi

if timeout 8s ros2 topic list | grep -Eq "/mapping/lio/odom|/mapping/lio/map_points"; then
    echo "fast-lio2 mapping topics are present"
    exit 0
fi

echo "fast-lio2 precheck did not report missing dependencies and mapping topics were not found" >&2
exit 1
```

- [ ] **步骤 4：设置可执行权限**

```bash
chmod +x scripts/verify_fast_lio2_precheck.sh
```

- [ ] **步骤 5：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio2_precheck_script_reports_missing_or_running_nodes -v
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add scripts/verify_fast_lio2_precheck.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: 添加 FAST-LIO2 预检查脚本"
```

## 任务 9：新增定位模式配置与模式管理器

**文件：**
- 创建：`config/localization_modes.yaml`
- 创建：`scripts/localization_mode_manager.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_localization_mode_manager_defines_outdoor_transition_barn_modes -v
```

预期：FAIL，配置和脚本不存在。

- [ ] **步骤 3：创建 `config/localization_modes.yaml`**

写入：

```yaml
localization_modes:
  # gps_covariance_threshold: maximum accepted position covariance, unit m^2.
  gps_covariance_threshold: 25.0

  # gps_jump_threshold: maximum accepted GPS position jump between updates, unit m.
  gps_jump_threshold: 3.0

  OUTDOOR:
    # gps_weight: relative GPS influence in outdoor mode.
    gps_weight: 1.0
    lio_weight: 0.6
    wheel_weight: 0.4

  TRANSITION:
    # GPS is gradually reduced near barn entrances or when quality degrades.
    gps_weight: 0.25
    lio_weight: 0.9
    wheel_weight: 0.7

  BARN:
    # GPS is not trusted inside barns.
    gps_weight: 0.0
    lio_weight: 1.0
    wheel_weight: 0.9
```

- [ ] **步骤 4：创建 `scripts/localization_mode_manager.py`**

写入：

```python
#!/usr/bin/env python3
"""Publish localization mode, gated GPS, and fusion weight hints."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import yaml


WORKSPACE_DIR = Path(__file__).resolve().parents[1]


def load_modes():
    with (WORKSPACE_DIR / "config" / "localization_modes.yaml").open(
        "r",
        encoding="utf-8",
    ) as stream:
        return yaml.safe_load(stream)["localization_modes"]


def max_position_covariance(msg):
    covariance = msg.position_covariance
    return max(covariance[0], covariance[4], covariance[8])


class LocalizationModeManager(Node):
    def __init__(self):
        super().__init__("localization_mode_manager")
        self.modes = load_modes()
        self.mode_pub = self.create_publisher(String, "/localization/mode", 10)
        self.weights_pub = self.create_publisher(
            String,
            "/localization/fusion_weights",
            10,
        )
        self.gps_pub = self.create_publisher(
            NavSatFix,
            "/localization/gps/gated",
            10,
        )
        self.subscription = self.create_subscription(
            NavSatFix,
            "/sensing/gps/fix",
            self.on_gps,
            10,
        )

    def on_gps(self, msg):
        good_status = msg.status.status >= NavSatStatus.STATUS_FIX
        good_covariance = (
            max_position_covariance(msg)
            <= self.modes["gps_covariance_threshold"]
        )
        mode = "OUTDOOR" if good_status and good_covariance else "BARN"
        self.mode_pub.publish(String(data=mode))
        weights = self.modes[mode]
        self.weights_pub.publish(String(data=str(weights)))
        if mode == "OUTDOOR":
            self.gps_pub.publish(msg)


def main():
    rclpy.init()
    node = LocalizationModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 5：设置可执行权限**

```bash
chmod +x scripts/localization_mode_manager.py
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_localization_mode_manager_defines_outdoor_transition_barn_modes -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add config/localization_modes.yaml scripts/localization_mode_manager.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加定位模式管理器"
```

## 任务 10：接入定位模式管理器 launch 与验证脚本

**文件：**
- 修改：`launch/localization.launch.py`
- 创建：`scripts/verify_localization_modes.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_localization_launch_starts_mode_manager_and_verification_script():
    launch = read(WORKSPACE_DIR / "launch" / "localization.launch.py")
    script = read(WORKSPACE_DIR / "scripts" / "verify_localization_modes.sh")

    assert "localization_mode_manager.py" in launch
    assert "localization_modes.yaml" in launch
    assert "/localization/mode" in script
    assert "/localization/fusion_weights" in script
    assert "ros2 topic echo" in script
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_localization_launch_starts_mode_manager_and_verification_script -v
```

预期：FAIL，因为 launch 未启动模式管理器，验证脚本不存在。

- [ ] **步骤 3：修改 `launch/localization.launch.py`**

在返回的节点列表中增加：

```python
Node(
    package='robot_description',
    executable='localization_mode_manager.py',
    name='localization_mode_manager',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'config_file': os.path.join(pkg_share, 'config', 'localization_modes.yaml'),
    }],
)
```

- [ ] **步骤 4：创建 `scripts/verify_localization_modes.sh`**

写入：

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

if ! timeout 8s ros2 topic echo /localization/mode --once >/dev/null; then
    echo "no localization mode received" >&2
    exit 1
fi

if ! timeout 8s ros2 topic echo /localization/fusion_weights --once >/dev/null; then
    echo "no localization fusion weights received" >&2
    exit 1
fi

echo "localization mode verification passed"
```

- [ ] **步骤 5：设置可执行权限**

```bash
chmod +x scripts/verify_localization_modes.sh
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_localization_launch_starts_mode_manager_and_verification_script -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add launch/localization.launch.py scripts/verify_localization_modes.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 接入定位模式管理器启动与验证"
```

## 任务 11：Nav2 配置对齐过滤点云与车辆 footprint

**文件：**
- 修改：`config/navigation.yaml`
- 创建：`scripts/verify_saved_map_nav2_precheck.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_uses_filtered_lidar_and_vehicle_footprint_contract -v
```

预期：FAIL，Nav2 仍可能使用 `/robot/velodyne_points`，验证脚本不存在。

- [ ] **步骤 3：修改 `config/navigation.yaml` 点云 topic**

把所有 costmap pointcloud topic 改为：

```yaml
topic: /sensing/lidar/points
```

增加 footprint：

```yaml
    footprint: "[[0.275, 0.19], [0.275, -0.19], [-0.275, -0.19], [-0.275, 0.19]]"
```

至少在 `global_costmap.ros__parameters` 和 `local_costmap.ros__parameters` 下添加同样 footprint。

- [ ] **步骤 4：创建 `scripts/verify_saved_map_nav2_precheck.sh`**

写入：

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
    echo "saved-map Nav2 precheck reported missing Nav2 dependencies"
    exit 0
fi

if timeout 8s ros2 topic list | grep -Fxq "/control/cmd_vel"; then
    echo "saved-map Nav2 command interface is present"
    exit 0
fi

echo "Nav2 did not report missing dependencies and /control/cmd_vel was not found" >&2
exit 1
```

- [ ] **步骤 5：设置可执行权限**

```bash
chmod +x scripts/verify_saved_map_nav2_precheck.sh
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_uses_filtered_lidar_and_vehicle_footprint_contract -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add config/navigation.yaml scripts/verify_saved_map_nav2_precheck.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 对齐 Nav2 点云与车辆 footprint"
```

## 任务 12：预留云端客户端命名空间与版本化文档

**文件：**
- 创建：`config/remote_extension.yaml`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_remote_extension_config_reserves_future_namespaces_without_runtime_dependency():
    config = read(WORKSPACE_DIR / "config" / "remote_extension.yaml")

    assert "/mission" in config
    assert "/maps" in config
    assert "/fleet" in config
    assert "/config" in config
    assert "cloud is not part of the real-time control loop" in config
    assert "vehicle must keep local navigation running" in config
    assert "versioned" in config
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_remote_extension_config_reserves_future_namespaces_without_runtime_dependency -v
```

预期：FAIL，配置不存在。

- [ ] **步骤 3：创建 `config/remote_extension.yaml`**

写入：

```yaml
remote_extension:
  # This file reserves names and constraints for future cloud/client integration.
  # The cloud is not part of the real-time control loop.
  # The vehicle must keep local navigation running when 4G or cloud connectivity is unavailable.
  # Maps, routes, jobs, and remote configuration must be versioned before cloud management is enabled.

  namespaces:
    # /mission: future local mission state, commands, and progress.
    mission: /mission

    # /maps: future map upload, download, selection, and versioning.
    maps: /maps

    # /fleet: future vehicle state, heartbeat, and remote configuration.
    fleet: /fleet

    # /config: future validated vehicle, sensor, localization, and navigation configuration.
    config: /config

  constraints:
    cloud_realtime_control: false
    local_navigation_required_without_cloud: true
    configuration_must_be_validated_on_vehicle: true
```

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_remote_extension_config_reserves_future_namespaces_without_runtime_dependency -v
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add config/remote_extension.yaml src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "docs: 预留远程任务与客户端扩展配置"
```

## 任务 13：完整静态、构建与运行时验证

**文件：**
- 不应新增源码变更。

- [ ] **步骤 1：运行完整 Python 静态测试**

```bash
python3 -m pytest \
  src/robot_description/test/test_ackermann_kinematics.py \
  src/robot_description/test/test_wheel_encoder_integration.py \
  -v
```

预期：所有测试 PASS。

- [ ] **步骤 2：构建包**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
```

预期：

```text
Summary: 1 package finished
```

- [ ] **步骤 3：运行包测试**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select robot_description --event-handlers console_direct+
```

预期：

```text
100% tests passed, 0 tests failed
```

- [ ] **步骤 4：启动仿真并验证运行时话题**

```bash
ROS_LOG_DIR=$PWD/log/ros ./start.sh --no-rviz --no-build
```

另一个终端运行：

```bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_runtime_topics.sh
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_lidar_mount.sh
```

预期：

```text
runtime topic verification passed
lidar mount verification passed
```

- [ ] **步骤 5：验证 FAST-LIO2 预检查**

```bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_fast_lio2_precheck.sh
```

预期，当 FAST-LIO2 未安装：

```text
fast-lio2 precheck reported missing FAST-LIO dependency
```

预期，当 FAST-LIO2 已安装且运行：

```text
fast-lio2 mapping topics are present
```

- [ ] **步骤 6：验证定位与模式管理**

仿真运行时执行：

```bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_localization.sh
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_localization_modes.sh
```

预期：

```text
localization verification passed
localization mode verification passed
```

- [ ] **步骤 7：验证 Nav2 保存地图预检查**

```bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_saved_map_nav2_precheck.sh
```

预期，当 Nav2 未安装：

```text
saved-map Nav2 precheck reported missing Nav2 dependencies
```

预期，当 Nav2 已安装：

```text
saved-map Nav2 command interface is present
```

- [ ] **步骤 8：停止仿真**

```bash
./stop.sh -f
```

预期：

```text
仿真系统已完全停止
```

- [ ] **步骤 9：Commit 验证记录**

如果验证过程中只产生日志，不提交日志目录。若需要记录验收摘要，更新 `README_项目进度.md` 的当前基线，并提交：

```bash
git add README_项目进度.md
git commit -m "docs: 记录 FAST-LIO2 主线验证结果"
```
