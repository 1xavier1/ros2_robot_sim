# FAST-LIO2 轮速辅助优化实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 增加 FAST-LIO 漂移诊断、wheel/LIO/GPS 外部辅助融合位姿，并让建图使用该融合位姿，同时保证 ground truth 只用于仿真评分。

**架构：** FAST-LIO2 继续作为 LiDAR+IMU 前端输出 `/mapping/lio/odom`。新增 `/localization/wheel_lio_odom` 外部融合层：轮速修平移尺度，FAST-LIO 提供 yaw/局部约束，GPS 质量好时只做慢速全局锚定。地图 exporter 默认使用融合位姿，仿真 ground truth 只写评估指标。

**技术栈：** ROS 2 Humble、rclpy、nav_msgs/Odometry、sensor_msgs/NavSatFix、std_msgs/String、pytest 静态/单元测试、现有 Gazebo 仿真和 FAST-LIO2 launch。

---

## 文件结构

- 创建 `scripts/fast_lio_drift_diagnostic.py`：订阅 LIO、wheel、可选 fused、可选 ground truth，输出漂移评分 JSON 和状态。
- 创建 `scripts/wheel_lio_fusion.py`：生产融合节点，输出 `/localization/wheel_lio_odom` 和 `/localization/wheel_lio_status`。
- 修改 `scripts/export_odom_projected_map.py`：支持 `--pose-topic` 和 `--reference-topic`，默认 pose 改为 `/localization/wheel_lio_odom`。
- 修改 `scripts/global_localization_backend.py`：增加 `input_odom_topic` 参数，默认订阅 `/localization/wheel_lio_odom`。
- 修改 `launch/navigation.launch.py`：启动 `wheel_lio_fusion.py`，并把全局定位后端输入设为 `/localization/wheel_lio_odom`。
- 修改 `launch/fast_lio2.launch.py` 或新增验证脚本：保留 FAST-LIO 独立输出，同时明确不接 GPS/wheel 到 FAST-LIO 内部。
- 修改 `config/fast_lio.yaml`：收敛 `det_range` 到仿真隧道更合理范围，并保留注释说明。
- 修改 `src/robot_description/test/test_wheel_encoder_integration.py`：增加静态契约和纯函数单元测试。

## 任务 1：FAST-LIO 漂移诊断工具

**文件：**
- 创建：`scripts/fast_lio_drift_diagnostic.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

在 `src/robot_description/test/test_wheel_encoder_integration.py` 添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio_drift_diagnostic_contract -v
```

预期：FAIL，脚本不存在。

- [ ] **步骤 3：编写纯函数测试**

添加：

```python
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
```

- [ ] **步骤 4：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'fast_lio_drift_diagnostic'
```

预期：FAIL，`compute_delta_metrics` 不存在。

- [ ] **步骤 5：创建诊断脚本最小实现**

创建 `scripts/fast_lio_drift_diagnostic.py`，包含：

```python
#!/usr/bin/env python3
"""Measure FAST-LIO, wheel/LIO, and optional ground-truth drift."""

import argparse
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_tuple(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def delta_pose(start, end):
    return (
        end[0] - start[0],
        end[1] - start[1],
        wrap_angle(end[2] - start[2]),
    )


def compute_delta_metrics(reference_start, reference_end, estimate_start, estimate_end):
    ref = delta_pose(reference_start, reference_end)
    est = delta_pose(estimate_start, estimate_end)
    ref_distance = math.hypot(ref[0], ref[1])
    est_distance = math.hypot(est[0], est[1])
    translation_error = math.hypot(est[0] - ref[0], est[1] - ref[1])
    return {
        "reference_distance": ref_distance,
        "estimate_distance": est_distance,
        "scale_ratio": est_distance / ref_distance if ref_distance > 1e-6 else 0.0,
        "translation_error": translation_error,
        "drift_per_meter": translation_error / ref_distance if ref_distance > 1e-6 else 0.0,
        "yaw_error": wrap_angle(est[2] - ref[2]),
    }


class DriftDiagnostic(Node):
    def __init__(self, args):
        super().__init__("fast_lio_drift_diagnostic")
        self.args = args
        self.first = {}
        self.latest = {}
        self.create_subscription(Odometry, "/mapping/lio/odom", self.capture("lio"), 10)
        self.create_subscription(Odometry, "/robot/odom", self.capture("wheel"), 10)
        self.create_subscription(Odometry, "/localization/wheel_lio_odom", self.capture("wheel_lio"), 10)
        if args.reference_topic:
            self.create_subscription(Odometry, args.reference_topic, self.capture("reference"), 10)

    def capture(self, name):
        def _capture(msg):
            pose = pose_tuple(msg)
            self.latest[name] = pose
            self.first.setdefault(name, pose)
        return _capture

    def export(self):
        reference_name = "reference" if "reference" in self.latest else "wheel"
        data = {"reference": reference_name, "metrics": {}}
        for name in ("lio", "wheel_lio"):
            if name in self.first and name in self.latest and reference_name in self.first:
                data["metrics"][name] = compute_delta_metrics(
                    self.first[reference_name],
                    self.latest[reference_name],
                    self.first[name],
                    self.latest[name],
                )
        Path(self.args.output).write_text(json.dumps(data, indent=2), encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--output", default="maps/fast_lio_drift_diagnostic.json")
    parser.add_argument("--reference-topic", default="/robot/ground_truth/odom")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DriftDiagnostic(args)
    try:
        deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=args.duration_sec)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.export()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 6：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'fast_lio_drift_diagnostic'
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add scripts/fast_lio_drift_diagnostic.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 FAST-LIO 漂移诊断工具"
```

## 任务 2：wheel/LIO/GPS 外部融合节点

**文件：**
- 创建：`scripts/wheel_lio_fusion.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的契约测试**

添加：

```python
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
```

- [ ] **步骤 2：编写融合数学测试**

添加：

```python
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
```

- [ ] **步骤 3：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio_fusion'
```

预期：FAIL，脚本和函数不存在。

- [ ] **步骤 4：创建融合节点最小实现**

创建 `scripts/wheel_lio_fusion.py`，包含：

```python
#!/usr/bin/env python3
"""Fuse FAST-LIO yaw, wheel translation scale, and optional GPS anchor."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

EARTH_RADIUS_M = 6378137.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def pose_tuple(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def gps_to_local_xy(gps, origin):
    origin_lat, origin_lon = origin
    lat_rad = math.radians(origin_lat)
    dx = math.radians(gps.longitude - origin_lon) * EARTH_RADIUS_M * math.cos(lat_rad)
    dy = math.radians(gps.latitude - origin_lat) * EARTH_RADIUS_M
    return (dx, dy)


def compose_wheel_lio_pose(lio_anchor, wheel_anchor, wheel_current, lio_current, use_lio_yaw=True):
    wheel_dx = wheel_current[0] - wheel_anchor[0]
    wheel_dy = wheel_current[1] - wheel_anchor[1]
    anchor_yaw = lio_anchor[2] - wheel_anchor[2]
    cos_a = math.cos(anchor_yaw)
    sin_a = math.sin(anchor_yaw)
    x = lio_anchor[0] + cos_a * wheel_dx - sin_a * wheel_dy
    y = lio_anchor[1] + sin_a * wheel_dx + cos_a * wheel_dy
    yaw = lio_current[2] if use_lio_yaw else lio_anchor[2] + wrap_angle(wheel_current[2] - wheel_anchor[2])
    return (x, y, yaw)


class WheelLioFusion(Node):
    def __init__(self):
        super().__init__("wheel_lio_fusion")
        self.declare_parameter("lio_timeout_sec", 0.5)
        self.declare_parameter("wheel_timeout_sec", 0.25)
        self.declare_parameter("gps_timeout_sec", 1.0)
        self.declare_parameter("gps_anchor_blend_weight", 0.0)
        self.declare_parameter("max_lio_translation_error", 1.0)
        self.lio_anchor = None
        self.wheel_anchor = None
        self.latest_lio = None
        self.latest_wheel = None
        self.latest_gps = None
        self.latest_lio_stamp = None
        self.latest_wheel_stamp = None
        self.latest_gps_stamp = None
        self.gps_origin = None
        self.global_offset = (0.0, 0.0)
        self.pub = self.create_publisher(Odometry, "/localization/wheel_lio_odom", 10)
        self.status_pub = self.create_publisher(String, "/localization/wheel_lio_status", 10)
        self.create_subscription(Odometry, "/mapping/lio/odom", self.on_lio, 10)
        self.create_subscription(Odometry, "/robot/odom", self.on_wheel, 50)
        self.create_subscription(NavSatFix, "/localization/gps/gated", self.on_gps, 10)

    def on_lio(self, msg):
        self.latest_lio = msg
        self.latest_lio_stamp = self.get_clock().now()
        self.publish_if_ready()

    def on_wheel(self, msg):
        self.latest_wheel = msg
        self.latest_wheel_stamp = self.get_clock().now()
        self.publish_if_ready()

    def on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_stamp = self.get_clock().now()
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude)

    def fresh(self, stamp, param):
        return stamp is not None and self.get_clock().now() - stamp <= Duration(seconds=float(self.get_parameter(param).value))

    def publish_if_ready(self):
        if self.latest_lio is None or self.latest_wheel is None:
            return
        lio_pose = pose_tuple(self.latest_lio)
        wheel_pose = pose_tuple(self.latest_wheel)
        if self.lio_anchor is None:
            self.lio_anchor = lio_pose
            self.wheel_anchor = wheel_pose
        use_lio_yaw = self.fresh(self.latest_lio_stamp, "lio_timeout_sec")
        fused_pose = compose_wheel_lio_pose(self.lio_anchor, self.wheel_anchor, wheel_pose, lio_pose, use_lio_yaw)
        fused = Odometry()
        fused.header.stamp = self.latest_lio.header.stamp
        fused.header.frame_id = "map"
        fused.child_frame_id = "base_link"
        fused.pose.pose.position.x = fused_pose[0] + self.global_offset[0]
        fused.pose.pose.position.y = fused_pose[1] + self.global_offset[1]
        qx, qy, qz, qw = quat_from_yaw(fused_pose[2])
        fused.pose.pose.orientation.x = qx
        fused.pose.pose.orientation.y = qy
        fused.pose.pose.orientation.z = qz
        fused.pose.pose.orientation.w = qw
        fused.twist = self.latest_wheel.twist
        self.apply_gps_anchor(fused)
        self.pub.publish(fused)
        self.status_pub.publish(String(data=f"lio_yaw={'fresh' if use_lio_yaw else 'fallback'}; gps={self.gps_state()}"))

    def gps_state(self):
        if self.latest_gps is None:
            return "none"
        return "fresh" if self.fresh(self.latest_gps_stamp, "gps_timeout_sec") else "stale"

    def apply_gps_anchor(self, fused):
        if self.latest_gps is None or self.gps_origin is None:
            return
        if not self.fresh(self.latest_gps_stamp, "gps_timeout_sec"):
            return
        weight = max(0.0, min(1.0, float(self.get_parameter("gps_anchor_blend_weight").value)))
        if weight <= 0.0:
            return
        gps_x, gps_y = gps_to_local_xy(self.latest_gps, self.gps_origin)
        residual_x = gps_x - fused.pose.pose.position.x
        residual_y = gps_y - fused.pose.pose.position.y
        self.global_offset = (
            self.global_offset[0] + weight * residual_x,
            self.global_offset[1] + weight * residual_y,
        )
        fused.pose.pose.position.x += weight * residual_x
        fused.pose.pose.position.y += weight * residual_y


def main():
    rclpy.init()
    node = WheelLioFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio_fusion'
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add scripts/wheel_lio_fusion.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 wheel-LIO 外部融合节点"
```

## 任务 3：把 wheel/LIO 融合接入 launch 和验证脚本

**文件：**
- 修改：`launch/navigation.launch.py`
- 修改：`scripts/global_localization_backend.py`
- 修改：`scripts/verify_global_localization_runtime.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_launch_starts_wheel_lio_fusion -v
```

预期：FAIL。

- [ ] **步骤 3：修改 `launch/navigation.launch.py`**

在本地化节点列表里加入：

```python
Node(
    package='robot_description',
    executable='wheel_lio_fusion.py',
    name='wheel_lio_fusion',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'gps_anchor_blend_weight': gps_anchor_blend_weight,
    }],
)
```

- [ ] **步骤 4：修改 `scripts/global_localization_backend.py` 输入 topic**

把固定订阅：

```python
self.create_subscription(
    Odometry,
    "/localization/fused_odom",
    self.on_fused_odom,
    10,
)
```

改为参数化订阅：

```python
self.declare_parameter("input_odom_topic", "/localization/wheel_lio_odom")
input_odom_topic = str(self.get_parameter("input_odom_topic").value)
self.create_subscription(
    Odometry,
    input_odom_topic,
    self.on_fused_odom,
    10,
)
```

- [ ] **步骤 5：修改运行时验证脚本**

在 `scripts/verify_global_localization_runtime.sh` 增加：

```bash
check_topic_once "/localization/wheel_lio_odom" "FAST-LIO + wheel/GPS wheel-LIO odom"
check_topic_once "/localization/wheel_lio_status" "wheel-LIO fusion status"
```

- [ ] **步骤 6：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_navigation_launch_starts_wheel_lio_fusion -v
```

预期：PASS。

- [ ] **步骤 7：Commit**

```bash
git add launch/navigation.launch.py scripts/global_localization_backend.py scripts/verify_global_localization_runtime.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 接入 wheel-LIO 融合定位输出"
```

## 任务 4：地图 exporter 支持 pose-topic 和 reference-topic

**文件：**
- 修改：`scripts/export_odom_projected_map.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的测试**

添加：

```python
def test_odom_projected_exporter_uses_configurable_pose_and_reference_topics():
    script = read(WORKSPACE_DIR / "scripts" / "export_odom_projected_map.py")

    assert "--pose-topic" in script
    assert "/localization/wheel_lio_odom" in script
    assert "--reference-topic" in script
    assert "/robot/ground_truth/odom" in script
    assert "reference_metrics" in script
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_odom_projected_exporter_uses_configurable_pose_and_reference_topics -v
```

预期：FAIL。

- [ ] **步骤 3：修改 exporter 订阅 pose topic**

在 `parse_args()` 增加：

```python
parser.add_argument("--pose-topic", default="/localization/wheel_lio_odom")
parser.add_argument("--reference-topic", default="")
```

把当前固定的：

```python
self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
```

改为：

```python
self.create_subscription(Odometry, args.pose_topic, self.on_odom, 10)
if args.reference_topic:
    self.create_subscription(Odometry, args.reference_topic, self.on_reference_odom, 10)
```

保留旧 `--odom-topic` 作为兼容别名，并统一解析为一个生产位姿 topic：

```python
parser.add_argument("--pose-topic", default="/localization/wheel_lio_odom")
parser.add_argument("--odom-topic", default=None)
```

在 `main()` 初始化节点前解析：

```python
if args.odom_topic is not None:
    args.pose_topic = args.odom_topic
```

节点内部只订阅 `args.pose_topic`，不同时订阅两个生产位姿。

- [ ] **步骤 4：增加 reference 评估输出**

新增：

```python
def on_reference_odom(self, msg):
    self.latest_reference_odom = msg
```

在 JSON 中增加：

```python
"reference_metrics": {
    "reference_topic": self.args.reference_topic,
    "available": self.latest_reference_odom is not None,
}
```

本任务只记录 reference 是否可用；完整 drift 评分由 `fast_lio_drift_diagnostic.py` 负责。

- [ ] **步骤 5：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'odom_projected_exporter_uses_configurable_pose'
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add scripts/export_odom_projected_map.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 允许地图导出选择融合位姿源"
```

## 任务 5：FAST-LIO 输入与参数治理

**文件：**
- 修改：`config/fast_lio.yaml`
- 修改：`scripts/verify_fast_lio2_precheck.sh`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

添加：

```python
def test_fast_lio_parameters_are_corridor_scoped_and_diagnostic_friendly():
    config = yaml.safe_load(read(WORKSPACE_DIR / "config" / "fast_lio.yaml"))
    text = read(WORKSPACE_DIR / "config" / "fast_lio.yaml")
    verify = read(WORKSPACE_DIR / "scripts" / "verify_fast_lio2_precheck.sh")

    params = config["/**"]["ros__parameters"]
    assert params["mapping"]["det_range"] <= 30.0
    assert "corridor" in text.lower() or "tunnel" in text.lower()
    assert "/sensing/lidar/points" in verify
    assert "/sensing/imu/data" in verify
    assert "/mapping/lio/odom" in verify
    assert "/mapping/lio/map_points" in verify
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio_parameters_are_corridor_scoped_and_diagnostic_friendly -v
```

预期：FAIL，`det_range` 当前为 100 或验证脚本没有完整 topic 检查。

- [ ] **步骤 3：修改 FAST-LIO 参数**

把 `config/fast_lio.yaml` 中：

```yaml
det_range: 100.0
```

改为：

```yaml
# det_range: maximum mapping detection range.
# Unit: m. Corridor/tunnel simulation uses a bounded range to avoid distant
# ceiling and repeated structure dominating local registration.
det_range: 30.0
```

- [ ] **步骤 4：强化 precheck**

在 `scripts/verify_fast_lio2_precheck.sh` 检查：

```bash
timeout 8s ros2 topic echo /sensing/lidar/points --once >/dev/null
timeout 8s ros2 topic echo /sensing/imu/data --once >/dev/null
timeout 8s ros2 topic echo /mapping/lio/odom --once >/dev/null
timeout 8s ros2 topic echo /mapping/lio/map_points --once >/dev/null
```

保留现有缺包提示逻辑。

- [ ] **步骤 5：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio_parameters_are_corridor_scoped_and_diagnostic_friendly -v
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add config/fast_lio.yaml scripts/verify_fast_lio2_precheck.sh src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "chore: 收敛 FAST-LIO 隧道场景参数和预检"
```

## 任务 6：端到端仿真验证流程文档

**文件：**
- 创建：`docs/FAST_LIO2_WHEEL_AIDED_VALIDATION.md`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的文档契约测试**

添加：

```python
def test_fast_lio_wheel_aided_validation_doc_exists():
    doc = read(WORKSPACE_DIR / "docs" / "FAST_LIO2_WHEEL_AIDED_VALIDATION.md")

    assert "fast_lio_drift_diagnostic.py" in doc
    assert "wheel_lio_fusion.py" in doc
    assert "export_odom_projected_map.py" in doc
    assert "/robot/ground_truth/odom" in doc
    assert "evaluation only" in doc
    assert "/localization/wheel_lio_odom" in doc
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio_wheel_aided_validation_doc_exists -v
```

预期：FAIL，文档不存在。

- [ ] **步骤 3：创建验证文档**

创建 `docs/FAST_LIO2_WHEEL_AIDED_VALIDATION.md`：

```markdown
# FAST-LIO2 Wheel-Aided Validation

## Rule

`/robot/ground_truth/odom` is evaluation only. It is never a production localization input.

## Terminals

Terminal 1:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description fast_lio2.launch.py
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/wheel_lio_fusion.py
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/fast_lio_drift_diagnostic.py \
  --duration-sec 60 \
  --output maps/fast_lio_drift_check.json \
  --reference-topic /robot/ground_truth/odom
```

Terminal 5:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/export_odom_projected_map.py \
  --output maps/wheel_lio_map_check \
  --duration-sec 60 \
  --pose-topic /localization/wheel_lio_odom \
  --reference-topic /robot/ground_truth/odom
```

## Correctness

- `/localization/wheel_lio_odom` is published continuously.
- Drift diagnostic shows `wheel_lio` translation scale closer to reference than raw `lio`.
- The generated occupancy map is less warped than raw FAST-LIO map accumulation.
- Ground truth appears only in diagnostic/reference arguments.
```

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_fast_lio_wheel_aided_validation_doc_exists -v
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add docs/FAST_LIO2_WHEEL_AIDED_VALIDATION.md src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "docs: 添加 wheel-LIO 仿真验证流程"
```

## 任务 7：完整验证

**文件：**
- 不新增文件。

- [ ] **步骤 1：运行静态测试**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
```

预期：全部 PASS。

- [ ] **步骤 2：构建包**

```bash
colcon build --packages-select robot_description
```

预期：`Summary: 1 package finished`。

- [ ] **步骤 3：运行短时 ROS topic 验证**

在仿真、FAST-LIO、`wheel_lio_fusion.py` 都启动后运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /localization/wheel_lio_odom --once
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /localization/wheel_lio_status --once
```

预期：两个 topic 都能收到消息。

- [ ] **步骤 4：运行漂移诊断**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/fast_lio_drift_diagnostic.py \
  --duration-sec 30 \
  --output maps/fast_lio_drift_validation.json \
  --reference-topic /robot/ground_truth/odom
```

预期：输出 JSON 包含 `metrics.lio`，如果 wheel/LIO 融合节点已运行，也包含 `metrics.wheel_lio`。

- [ ] **步骤 5：运行融合位姿建图**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/export_odom_projected_map.py \
  --output maps/wheel_lio_odom_projected_validation \
  --duration-sec 30 \
  --pose-topic /localization/wheel_lio_odom \
  --reference-topic /robot/ground_truth/odom
```

预期：输出 `.pgm`、`.yaml`、`.json`。

- [ ] **步骤 6：Commit 验证文档更新或结果记录**

如果只产生临时地图，不提交地图。若需要记录验证结论，更新 `docs/CLAUDE_CODE_HANDOFF.md` 后提交：

```bash
git add docs/CLAUDE_CODE_HANDOFF.md
git commit -m "docs: 记录 wheel-LIO 验证结果"
```
