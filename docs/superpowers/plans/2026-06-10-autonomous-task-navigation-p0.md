# Autonomous Task Navigation P0 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在仿真中完成“遥控示教 -> 保存任务地图 -> 地图平面查看 -> 自动执行示教路线”的 P0 自主任务导航闭环。

**架构：** 新增小型任务导航核心模块，负责 `task_map.yaml` 读写、示教采样和 Nav2 目标生成；新增 ROS 节点 `route_recorder.py`、`task_executor.py`、`localization_mode_supervisor.py`。现有 `remote/remote_control.py` 只做增量扩展，通过 ROS topic/service/action 调用车辆端接口，不重做遥控器。

**技术栈：** ROS 2 Humble、rclpy、nav_msgs/Odometry、geometry_msgs/PoseStamped、std_msgs/String、Nav2 action、PyYAML、pytest/unittest、现有 WebSocket remote。

---

## 文件结构

- 创建 `scripts/task_map_core.py`
  - 纯函数和数据结构：任务地图校验、pose 解析、region 判断、路线采样、方向判断、Nav2 pose 转换。
  - 不依赖 ROS runtime，方便单元测试。
- 创建 `scripts/route_recorder.py`
  - ROS 节点：订阅定位和轮速/odom，接收示教命令，保存 `task_map.yaml`。
- 创建 `scripts/task_executor.py`
  - ROS 节点：读取 `task_map.yaml`，将示教路线转换为 Nav2 goal，发布任务状态。
- 创建 `scripts/localization_mode_supervisor.py`
  - ROS 节点：根据 region、GPS freshness、wheel-LIO 状态发布定位模式。
- 修改 `remote/remote_control.py`
  - 在现有 WebSocket/HTML 遥控器基础上增加 Teach、Task、Map Monitor、Status 的消息和面板。
- 修改 `remote/test_remote_control.py`
  - 测试 remote 新增消息构造、HTML 按钮和 WebSocket 命令映射。
- 修改 `src/robot_description/test/test_wheel_encoder_integration.py`
  - 继续作为本仓库静态/集成测试入口，增加 P0 脚本和配置契约测试。
- 创建 `config/task_map.example.yaml`
  - 可运行的示例任务地图，供测试和用户参考。
- 修改 `launch/navigation.launch.py`
  - 增加 P0 节点启动参数，默认可以关闭或显式开启。
- 修改 `src/robot_description/CMakeLists.txt`
  - 现有 `scripts/` 和 `config/` 目录安装已覆盖新增文件；只需测试确认不漏。

## 任务 1：任务地图核心和示例配置

**文件：**
- 创建：`scripts/task_map_core.py`
- 创建：`config/task_map.example.yaml`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的任务地图加载测试**

在 `src/robot_description/test/test_wheel_encoder_integration.py` 末尾追加：

```python
def test_task_map_core_loads_example_task_map():
    module = load_script_module("task_map_core.py")

    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    assert task_map["site"]["map_frame"] == "map"
    assert task_map["maps"]["nav2_map"].endswith(".yaml")
    assert task_map["motion_profiles"][0]["allow_reverse"] is False
    assert task_map["tasks"][0]["type"] == "taught_route"
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'task_map_core_loads_example'
```

预期：FAIL，报错包含 `FileNotFoundError` 或缺少 `task_map_core.py`。

- [ ] **步骤 3：创建示例任务地图**

创建 `config/task_map.example.yaml`：

```yaml
site:
  id: barn_sim_site
  map_frame: map

maps:
  nav2_map: maps/manual_wheel_lio_map_test.yaml
  pointcloud_map: maps/lio_cloud_map_manual.pcd
  source: simulation

motion_profiles:
  - id: forward_only_safe
    vehicle_model: ackermann
    allow_reverse: false
    min_turning_radius: 0.78
    max_forward_speed: 0.35
    max_reverse_speed: 0.0

regions:
  - id: sim_yard
    type: outdoor
    polygon: [[-2.0, -2.0], [6.0, -2.0], [6.0, 4.0], [-2.0, 4.0]]
    localization_mode: OUTDOOR
    gps_policy: prefer_gps
    speed_limit: 0.35
  - id: sim_lane
    type: indoor
    polygon: [[6.0, -2.0], [30.0, -2.0], [30.0, 4.0], [6.0, 4.0]]
    localization_mode: INDOOR
    gps_policy: disabled
    speed_limit: 0.20

waypoints:
  - id: home
    pose: [0.0, 0.0, 0.0]
    role: docking_home
    source: example

recorded_routes:
  - id: example_route
    motion_profile: forward_only_safe
    source: example
    sample_policy:
      min_distance: 0.3
      min_yaw_change: 0.25
    path:
      - pose: [0.0, 0.0, 0.0]
        direction: forward
      - pose: [1.0, 0.0, 0.0]
        direction: forward
      - pose: [2.0, 0.2, 0.1]
        direction: forward

tasks:
  - id: daily_patrol
    type: taught_route
    route: example_route
    start: home
    finish: home
    failure_policy:
      nav_retry_count: 1
      on_failure: stop_and_wait

map_overlays:
  keepout_zones: []
  speed_zones: []
  temporary_obstacles: []
  map_corrections: []
```

- [ ] **步骤 4：实现最小加载函数**

创建 `scripts/task_map_core.py`：

```python
#!/usr/bin/env python3
"""Pure helpers for taught task maps and P0 autonomous task navigation."""

import math
from pathlib import Path

import yaml


REQUIRED_TOP_LEVEL_KEYS = (
    "site",
    "maps",
    "motion_profiles",
    "regions",
    "waypoints",
    "recorded_routes",
    "tasks",
    "map_overlays",
)


def load_task_map(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        task_map = yaml.safe_load(stream)
    validate_task_map(task_map)
    return task_map


def validate_task_map(task_map):
    if not isinstance(task_map, dict):
        raise ValueError("task_map must be a mapping")
    missing = [key for key in REQUIRED_TOP_LEVEL_KEYS if key not in task_map]
    if missing:
        raise ValueError(f"task_map missing keys: {', '.join(missing)}")
    if task_map["site"].get("map_frame") != "map":
        raise ValueError("P0 task_map site.map_frame must be map")
    return task_map


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
```

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'task_map_core_loads_example'
```

预期：PASS。

- [ ] **步骤 6：编写路线和倒车契约失败测试**

继续追加：

```python
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
```

- [ ] **步骤 7：实现 profile 查找和倒车约束**

在 `scripts/task_map_core.py` 添加：

```python
def motion_profiles_by_id(task_map):
    return {profile["id"]: profile for profile in task_map.get("motion_profiles", [])}


def route_allows_execution(task_map, route):
    profiles = motion_profiles_by_id(task_map)
    profile = profiles.get(route.get("motion_profile"))
    if profile is None:
        raise ValueError(f"unknown motion_profile: {route.get('motion_profile')}")
    if profile.get("allow_reverse", False):
        return True
    return all(point.get("direction", "forward") != "reverse" for point in route["path"])
```

- [ ] **步骤 8：运行任务 1 相关测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'task_map_core'
```

预期：2 个相关测试 PASS。

- [ ] **步骤 9：Commit**

```bash
git add scripts/task_map_core.py config/task_map.example.yaml src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加任务地图核心契约"
```

## 任务 2：Route Recorder 示教采集核心和节点

**文件：**
- 修改：`scripts/task_map_core.py`
- 创建：`scripts/route_recorder.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写采样策略失败测试**

追加：

```python
def test_route_recorder_samples_by_distance_or_yaw_change():
    module = load_script_module("task_map_core.py")
    samples = []

    assert module.should_append_route_sample(samples, (0.0, 0.0, 0.0), 0.3, 0.25)
    samples.append({"pose": [0.0, 0.0, 0.0], "direction": "forward"})
    assert not module.should_append_route_sample(samples, (0.1, 0.0, 0.01), 0.3, 0.25)
    assert module.should_append_route_sample(samples, (0.31, 0.0, 0.01), 0.3, 0.25)
    assert module.should_append_route_sample(samples, (0.1, 0.0, 0.30), 0.3, 0.25)
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'samples_by_distance'
```

预期：FAIL，缺少 `should_append_route_sample`。

- [ ] **步骤 3：实现采样和方向判断纯函数**

在 `scripts/task_map_core.py` 添加：

```python
def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def should_append_route_sample(samples, pose, min_distance, min_yaw_change):
    if not samples:
        return True
    previous = samples[-1]["pose"]
    distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
    yaw_change = abs(wrap_angle(pose[2] - previous[2]))
    return distance >= min_distance or yaw_change >= min_yaw_change


def direction_from_linear_velocity(linear_x):
    return "reverse" if linear_x < -1e-4 else "forward"
```

- [ ] **步骤 4：运行测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'samples_by_distance'
```

预期：PASS。

- [ ] **步骤 5：编写 route_recorder 脚本契约失败测试**

追加：

```python
def test_route_recorder_declares_teach_interfaces_and_outputs_task_map():
    script = read(WORKSPACE_DIR / "scripts" / "route_recorder.py")

    assert "class RouteRecorder" in script
    assert "/teach/command" in script
    assert "/teach/status" in script
    assert "/localization/global_odom" in script
    assert "/localization/wheel_lio_odom" in script
    assert "save_task_map" in script
    assert "task_map.yaml" in script
```

- [ ] **步骤 6：创建最小 RouteRecorder 节点**

创建 `scripts/route_recorder.py`：

```python
#!/usr/bin/env python3
"""Record taught waypoints and routes into a task_map.yaml file."""

import copy
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from task_map_core import (
    direction_from_linear_velocity,
    load_task_map,
    should_append_route_sample,
    yaw_from_quaternion,
)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__("route_recorder")
        self.declare_parameter("task_map_template", "config/task_map.example.yaml")
        self.declare_parameter("task_map_output", "maps/task_map.yaml")
        self.declare_parameter("min_sample_distance", 0.3)
        self.declare_parameter("min_yaw_change", 0.25)

        self.latest_pose = None
        self.latest_direction = "forward"
        self.recording_route_id = None
        self.samples = []
        self.task_map = load_task_map(self.get_parameter("task_map_template").value)

        self.status_pub = self.create_publisher(String, "/teach/status", 10)
        self.create_subscription(String, "/teach/command", self.on_command, 10)
        self.create_subscription(
            Odometry, "/localization/global_odom", self.on_pose, 10
        )
        self.create_subscription(
            Odometry, "/localization/wheel_lio_odom", self.on_pose, 10
        )
        self.create_subscription(Odometry, "/robot/odom", self.on_wheel_odom, 10)

    def on_pose(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        ]
        if self.recording_route_id:
            self.append_sample_if_needed()

    def on_wheel_odom(self, msg):
        self.latest_direction = direction_from_linear_velocity(msg.twist.twist.linear.x)

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        command = parts[0]
        args = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        if command == "mark_waypoint":
            self.mark_waypoint(args.get("id", "waypoint"), args.get("role", "taught"))
        elif command == "start_recording":
            self.start_recording(args.get("id", "taught_route"))
        elif command == "stop_recording":
            self.stop_recording()
        elif command == "save_task_map":
            self.save_task_map()

    def mark_waypoint(self, waypoint_id, role):
        if self.latest_pose is None:
            self.publish_status("teach=blocked; reason=no_pose")
            return
        self.task_map["waypoints"].append({
            "id": waypoint_id,
            "pose": list(self.latest_pose),
            "role": role,
            "source": "taught",
        })
        self.publish_status(f"teach=marked; waypoint={waypoint_id}")

    def start_recording(self, route_id):
        self.recording_route_id = route_id
        self.samples = []
        self.publish_status(f"teach=recording; route={route_id}; samples=0")

    def append_sample_if_needed(self):
        min_distance = float(self.get_parameter("min_sample_distance").value)
        min_yaw = float(self.get_parameter("min_yaw_change").value)
        if should_append_route_sample(self.samples, self.latest_pose, min_distance, min_yaw):
            self.samples.append({
                "pose": list(self.latest_pose),
                "direction": self.latest_direction,
            })

    def stop_recording(self):
        if not self.recording_route_id:
            return
        route = {
            "id": self.recording_route_id,
            "motion_profile": "forward_only_safe",
            "source": "taught",
            "sample_policy": {
                "min_distance": float(self.get_parameter("min_sample_distance").value),
                "min_yaw_change": float(self.get_parameter("min_yaw_change").value),
            },
            "path": copy.deepcopy(self.samples),
        }
        self.task_map["recorded_routes"].append(route)
        self.publish_status(
            f"teach=stopped; route={self.recording_route_id}; samples={len(self.samples)}"
        )
        self.recording_route_id = None

    def save_task_map(self):
        output = Path(self.get_parameter("task_map_output").value)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(self.task_map, stream, allow_unicode=False, sort_keys=False)
        self.publish_status(f"teach=saved; path={output}")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 7：运行任务 2 测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'route_recorder or samples_by_distance'
python3 -m py_compile scripts/task_map_core.py scripts/route_recorder.py
```

预期：测试 PASS，编译无输出。

- [ ] **步骤 8：Commit**

```bash
git add scripts/task_map_core.py scripts/route_recorder.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加遥控示教路线记录器"
```

## 任务 3：Task Executor 和 Nav2 目标转换

**文件：**
- 修改：`scripts/task_map_core.py`
- 创建：`scripts/task_executor.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写 route 到 pose list 的失败测试**

追加：

```python
def test_task_map_core_converts_taught_route_to_goal_poses():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    poses = module.goal_poses_for_task(task_map, "daily_patrol")

    assert len(poses) == 3
    assert poses[0] == (0.0, 0.0, 0.0)
    assert poses[-1] == (2.0, 0.2, 0.1)
```

- [ ] **步骤 2：运行测试验证失败**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'converts_taught_route'
```

预期：FAIL，缺少 `goal_poses_for_task`。

- [ ] **步骤 3：实现任务查找和目标转换**

在 `scripts/task_map_core.py` 添加：

```python
def item_by_id(items, item_id, kind):
    for item in items:
        if item.get("id") == item_id:
            return item
    raise ValueError(f"unknown {kind}: {item_id}")


def goal_poses_for_task(task_map, task_id):
    task = item_by_id(task_map["tasks"], task_id, "task")
    if task.get("type") != "taught_route":
        raise ValueError(f"unsupported task type: {task.get('type')}")
    route = item_by_id(task_map["recorded_routes"], task["route"], "recorded_route")
    if not route_allows_execution(task_map, route):
        raise ValueError(f"route {route['id']} violates motion profile")
    return [tuple(point["pose"]) for point in route["path"]]
```

- [ ] **步骤 4：运行转换测试验证通过**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'converts_taught_route'
```

预期：PASS。

- [ ] **步骤 5：编写 task_executor 契约失败测试**

追加：

```python
def test_task_executor_exposes_task_commands_and_status_topics():
    script = read(WORKSPACE_DIR / "scripts" / "task_executor.py")

    assert "class TaskExecutor" in script
    assert "/task/command" in script
    assert "/task/status" in script
    assert "/task/current_goal" in script
    assert "NavigateThroughPoses" in script or "NavigateToPose" in script
    assert "BLOCKED" in script
    assert "allow_reverse" in script
```

- [ ] **步骤 6：创建最小 TaskExecutor 节点**

创建 `scripts/task_executor.py`：

```python
#!/usr/bin/env python3
"""Execute taught task_map routes through Nav2."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from task_map_core import goal_poses_for_task, load_task_map, motion_profiles_by_id


class TaskExecutor(Node):
    def __init__(self):
        super().__init__("task_executor")
        self.declare_parameter("task_map", "config/task_map.example.yaml")
        self.declare_parameter("default_task_id", "daily_patrol")
        self.task_map = load_task_map(self.get_parameter("task_map").value)
        self.state = "IDLE"
        self.active_task_id = None

        self.status_pub = self.create_publisher(String, "/task/status", 10)
        self.goal_pub = self.create_publisher(String, "/task/current_goal", 10)
        self.create_subscription(String, "/task/command", self.on_command, 10)
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
        )
        self.publish_status("IDLE")

    def on_command(self, msg):
        parts = msg.data.split()
        if not parts:
            return
        command = parts[0]
        args = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        if command == "start_task":
            self.start_task(args.get("id", self.get_parameter("default_task_id").value))
        elif command == "pause_task":
            self.state = "PAUSED"
            self.publish_status("PAUSED")
        elif command == "resume_task":
            self.state = "RUNNING"
            self.publish_status("RUNNING")
        elif command == "cancel_task":
            self.state = "CANCELLED"
            self.publish_status("CANCELLED")
        elif command == "return_home":
            self.state = "RETURNING_HOME"
            self.publish_status("RETURNING_HOME")

    def start_task(self, task_id):
        try:
            poses = goal_poses_for_task(self.task_map, task_id)
            self.ensure_reverse_policy(task_id)
        except ValueError as exc:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason={exc}")
            return
        self.active_task_id = task_id
        self.state = "RUNNING"
        self.publish_status(f"RUNNING; task={task_id}; goals={len(poses)}")
        self.send_nav_goal(poses)

    def ensure_reverse_policy(self, task_id):
        task = next(item for item in self.task_map["tasks"] if item["id"] == task_id)
        route = next(item for item in self.task_map["recorded_routes"] if item["id"] == task["route"])
        profile = motion_profiles_by_id(self.task_map)[route["motion_profile"]]
        if not profile.get("allow_reverse", False):
            self.get_logger().info("allow_reverse=false; executing forward-only route")

    def send_nav_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self.pose_stamped(x, y, yaw) for x, y, yaw in poses]
        if goal_msg.poses:
            first = goal_msg.poses[0].pose.position
            self.goal_pub.publish(String(data=f"x={first.x:.3f}; y={first.y:.3f}"))
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_action_unavailable")
            return
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

    def pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.task_map["site"]["map_frame"]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "BLOCKED"
            self.publish_status("BLOCKED; reason=nav2_goal_rejected")
            return
        goal_handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result()
        if result.status == 4:
            self.state = "COMPLETED"
            self.publish_status("COMPLETED")
        else:
            self.state = "BLOCKED"
            self.publish_status(f"BLOCKED; reason=nav2_status_{result.status}")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 7：如果 `nav2_msgs` 缺依赖，补 package.xml**

若 `python3 -m py_compile scripts/task_executor.py` 或 colcon 报 `nav2_msgs` 缺失，在 `src/robot_description/package.xml` 添加：

```xml
  <depend>nav2_msgs</depend>
```

- [ ] **步骤 8：运行任务 3 测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'task_executor or converts_taught_route'
python3 -m py_compile scripts/task_map_core.py scripts/task_executor.py
```

预期：测试 PASS，编译无输出。

- [ ] **步骤 9：Commit**

```bash
git add scripts/task_map_core.py scripts/task_executor.py src/robot_description/test/test_wheel_encoder_integration.py src/robot_description/package.xml
git commit -m "feat: 添加示教任务执行器"
```

## 任务 4：Localization Mode Supervisor

**文件：**
- 修改：`scripts/task_map_core.py`
- 创建：`scripts/localization_mode_supervisor.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写点在区域内的失败测试**

追加：

```python
def test_task_map_core_finds_region_for_pose():
    module = load_script_module("task_map_core.py")
    task_map = module.load_task_map(WORKSPACE_DIR / "config" / "task_map.example.yaml")

    region = module.region_for_pose(task_map, x=1.0, y=1.0)

    assert region["id"] == "sim_yard"
    assert region["localization_mode"] == "OUTDOOR"
```

- [ ] **步骤 2：实现 region 查找**

在 `scripts/task_map_core.py` 添加：

```python
def point_in_polygon(x, y, polygon):
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def region_for_pose(task_map, x, y):
    for region in task_map.get("regions", []):
        if point_in_polygon(x, y, region["polygon"]):
            return region
    return None
```

- [ ] **步骤 3：运行 region 测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'finds_region_for_pose'
```

预期：PASS。

- [ ] **步骤 4：编写 supervisor 契约失败测试**

追加：

```python
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
```

- [ ] **步骤 5：创建最小 supervisor 节点**

创建 `scripts/localization_mode_supervisor.py`：

```python
#!/usr/bin/env python3
"""Publish route-aware localization mode for P0 task navigation."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from task_map_core import load_task_map, region_for_pose


class LocalizationModeSupervisor(Node):
    def __init__(self):
        super().__init__("localization_mode_supervisor")
        self.declare_parameter("task_map", "config/task_map.example.yaml")
        self.task_map = load_task_map(self.get_parameter("task_map").value)
        self.latest_wheel_lio_status = ""

        self.mode_pub = self.create_publisher(
            String,
            "/localization/supervised_mode",
            10,
        )
        self.create_subscription(
            Odometry,
            "/localization/global_odom",
            self.on_pose,
            10,
        )
        self.create_subscription(
            String,
            "/localization/wheel_lio_status",
            self.on_wheel_lio_status,
            10,
        )

    def on_wheel_lio_status(self, msg):
        self.latest_wheel_lio_status = msg.data

    def on_pose(self, msg):
        if "state=degraded" in self.latest_wheel_lio_status:
            self.mode_pub.publish(String(data="DEGRADED; reason=wheel_lio_degraded"))
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        region = region_for_pose(self.task_map, x, y)
        if region is None:
            self.mode_pub.publish(String(data="DEGRADED; reason=outside_regions"))
            return
        mode = region.get("localization_mode", "INDOOR")
        self.mode_pub.publish(String(data=f"{mode}; region={region['id']}"))


def main():
    rclpy.init()
    node = LocalizationModeSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **步骤 6：运行任务 4 测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'localization_mode_supervisor or finds_region'
python3 -m py_compile scripts/task_map_core.py scripts/localization_mode_supervisor.py
```

预期：测试 PASS，编译无输出。

- [ ] **步骤 7：Commit**

```bash
git add scripts/task_map_core.py scripts/localization_mode_supervisor.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加任务区域定位模式监督"
```

## 任务 5：基于现有 remote 增量扩展示教和任务入口

**文件：**
- 修改：`/home/xavier/Workspace/ClaudeSpace/remote/remote_control.py`
- 修改：`/home/xavier/Workspace/ClaudeSpace/remote/test_remote_control.py`

- [ ] **步骤 1：编写 remote 命令 payload 失败测试**

在 `remote/test_remote_control.py` 中添加：

```python
class TaskCommandPayloadTest(unittest.TestCase):
    def test_teach_command_payload_is_json_safe(self):
        payload = remote_control.make_ros_command_payload(
            "teach",
            "mark_waypoint",
            {"id": "home", "role": "docking_home"},
        )

        encoded = json.dumps(payload, allow_nan=False)
        decoded = json.loads(encoded)

        self.assertEqual(decoded["type"], "ros_command")
        self.assertEqual(decoded["channel"], "teach")
        self.assertEqual(decoded["command"], "mark_waypoint")
        self.assertEqual(decoded["args"]["id"], "home")
```

- [ ] **步骤 2：运行 remote 测试验证失败**

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
python3 -m unittest test_remote_control.py -v
```

预期：FAIL，缺少 `make_ros_command_payload`。

- [ ] **步骤 3：实现 payload helper**

在 `remote/remote_control.py` 的 `make_odom_payload` 后添加：

```python
def make_ros_command_payload(channel: str, command: str, args: dict | None = None) -> dict:
    return {
        "type": "ros_command",
        "channel": channel,
        "command": command,
        "args": args or {},
    }
```

- [ ] **步骤 4：增加 ROS 命令 publishers**

在 `RemoteControlNode.__init__` 中 `cmd_vel` publisher 后添加：

```python
        from std_msgs.msg import String
        self.teach_command_pub = self.create_publisher(String, "/teach/command", 10)
        self.task_command_pub = self.create_publisher(String, "/task/command", 10)
```

在 `RemoteControlNode` 类中添加：

```python
    def publish_ros_command(self, channel: str, command: str, args: dict) -> str:
        from std_msgs.msg import String
        suffix = " ".join(f"{key}={value}" for key, value in sorted(args.items()))
        text = command if not suffix else f"{command} {suffix}"
        if channel == "teach":
            self.teach_command_pub.publish(String(data=text))
        elif channel == "task":
            self.task_command_pub.publish(String(data=text))
        else:
            raise ValueError(f"unsupported command channel: {channel}")
        return text
```

- [ ] **步骤 5：WebSocket handler 支持 ros_command**

在 `RemoteWebServer._process_message` 中加入分支：

```python
        elif msg_type == "ros_command":
            sent = self.ros_node.publish_ros_command(
                data.get("channel", ""),
                data.get("command", ""),
                data.get("args", {}),
            )
            await websocket.send(json.dumps({
                "type": "ros_command_ack",
                "sent": sent,
            }))
```

- [ ] **步骤 6：HTML 增加 Teach/Task 按钮**

在 `HTML_PAGE` 中现有控制面板附近加入按钮区域，按钮调用：

```javascript
function sendTeach(command, args) {
  send({type: "ros_command", channel: "teach", command: command, args: args || {}});
}

function sendTask(command, args) {
  send({type: "ros_command", channel: "task", command: command, args: args || {}});
}
```

按钮示例：

```html
<button onclick="sendTeach('mark_waypoint', {id: document.getElementById('waypointId').value || 'waypoint', role: 'taught'})">MARK</button>
<button onclick="sendTeach('start_recording', {id: document.getElementById('routeId').value || 'taught_route'})">REC</button>
<button onclick="sendTeach('stop_recording', {})">STOP REC</button>
<button onclick="sendTeach('save_task_map', {})">SAVE MAP</button>
<button onclick="sendTask('start_task', {id: document.getElementById('taskId').value || 'daily_patrol'})">START TASK</button>
<button onclick="sendTask('pause_task', {})">PAUSE</button>
<button onclick="sendTask('resume_task', {})">RESUME</button>
<button onclick="sendTask('cancel_task', {})">CANCEL</button>
<button onclick="sendTask('return_home', {})">HOME</button>
```

- [ ] **步骤 7：增加 HTML 契约测试**

在 `FrontendScriptTest` 添加：

```python
    def test_teach_and_task_controls_are_present(self):
        html = remote_control.HTML_PAGE

        self.assertIn("sendTeach", html)
        self.assertIn("sendTask", html)
        self.assertIn("mark_waypoint", html)
        self.assertIn("start_recording", html)
        self.assertIn("start_task", html)
        self.assertIn("return_home", html)
```

- [ ] **步骤 8：运行 remote 测试**

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
python3 -m unittest test_remote_control.py -v
```

预期：全部 PASS；如果 JS check 失败，修正嵌入脚本语法。

- [ ] **步骤 9：Commit remote 仓库**

`remote` 是独立 git 仓库，进入该目录提交：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
git status --short
git add remote_control.py test_remote_control.py
git commit -m "feat: 扩展示教和任务控制入口"
```

## 任务 6：P0 launch 集成、地图监控契约和端到端验证

**文件：**
- 修改：`launch/navigation.launch.py`
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 修改：`docs/CLAUDE_CODE_HANDOFF.md`

- [ ] **步骤 1：编写 launch 契约失败测试**

在 `test_wheel_encoder_integration.py` 追加：

```python
def test_navigation_launch_can_start_p0_task_nodes():
    launch = read(WORKSPACE_DIR / "launch" / "navigation.launch.py")

    assert "route_recorder.py" in launch
    assert "task_executor.py" in launch
    assert "localization_mode_supervisor.py" in launch
    assert "task_map" in launch
    assert "enable_task_navigation" in launch
```

- [ ] **步骤 2：修改 navigation launch**

在 `launch/navigation.launch.py` 中添加 launch argument：

```python
    enable_task_navigation = LaunchConfiguration('enable_task_navigation', default='false')
    task_map = LaunchConfiguration('task_map', default=os.path.join(pkg_share, '..', '..', '..', '..', 'config', 'task_map.example.yaml'))
```

在 `LaunchDescription` 参数列表加入：

```python
        DeclareLaunchArgument('enable_task_navigation',
                              default_value='false',
                              description='Start P0 route_recorder, task_executor, and localization mode supervisor.'),
        DeclareLaunchArgument('task_map',
                              default_value=os.path.join(pkg_share, '..', '..', '..', '..', 'config', 'task_map.example.yaml'),
                              description='Task map yaml used by taught task navigation.'),
```

在 nodes 末尾添加受控启动：

```python
    task_nodes = [
        Node(
            package='robot_description',
            executable='route_recorder.py',
            name='route_recorder',
            output='screen',
            condition=IfCondition(enable_task_navigation),
            parameters=[{
                'use_sim_time': use_sim_time,
                'task_map_template': task_map,
                'task_map_output': 'maps/task_map.yaml',
            }],
        ),
        Node(
            package='robot_description',
            executable='task_executor.py',
            name='task_executor',
            output='screen',
            condition=IfCondition(enable_task_navigation),
            parameters=[{
                'use_sim_time': use_sim_time,
                'task_map': task_map,
            }],
        ),
        Node(
            package='robot_description',
            executable='localization_mode_supervisor.py',
            name='localization_mode_supervisor',
            output='screen',
            condition=IfCondition(enable_task_navigation),
            parameters=[{
                'use_sim_time': use_sim_time,
                'task_map': task_map,
            }],
        ),
    ]
    nodes.extend(task_nodes)
```

确保 import 中有：

```python
from launch.conditions import IfCondition
```

- [ ] **步骤 3：运行 launch 契约测试**

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'navigation_launch_can_start_p0'
```

预期：PASS。

- [ ] **步骤 4：编写 P0 手动验收文档到 handoff**

在 `docs/CLAUDE_CODE_HANDOFF.md` 末尾追加：

```markdown
### 2026-06-10：P0 自主任务导航计划验收入口

P0 手动验证顺序：

1. 启动仿真、FAST-LIO2、wheel-LIO、Nav2。
2. 启动 navigation launch 时设置 `enable_task_navigation:=true`。
3. 启动 `remote`，使用现有遥控器手动驾驶。
4. 使用 Teach 面板 mark waypoint、start/stop recording、save task map。
5. 使用 Task 面板 start task。
6. 观察 `/task/status`、`/task/current_goal`、`/localization/supervised_mode`。

生产任务链路不得使用 `/robot/ground_truth/odom`。
```

- [ ] **步骤 5：运行完整静态和编译验证**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
python3 -m py_compile scripts/task_map_core.py scripts/route_recorder.py scripts/task_executor.py scripts/localization_mode_supervisor.py
colcon build --packages-select robot_description
```

预期：

- pytest 全部 PASS。
- py_compile 无输出。
- colcon 输出 `Summary: 1 package finished`。

- [ ] **步骤 6：运行 runtime smoke**

终端 1：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description fast_lio2.launch.py
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/wheel_lio_fusion.py
```

终端 4：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description navigation.launch.py enable_task_navigation:=true
```

检查：

```bash
ros2 topic list | grep -E '/teach/status|/task/status|/localization/supervised_mode'
ros2 topic pub --once /teach/command std_msgs/msg/String "{data: 'mark_waypoint id=home role=docking_home'}"
ros2 topic pub --once /teach/command std_msgs/msg/String "{data: 'start_recording id=smoke_route'}"
ros2 topic pub --once /teach/command std_msgs/msg/String "{data: 'stop_recording'}"
ros2 topic pub --once /teach/command std_msgs/msg/String "{data: 'save_task_map'}"
ros2 topic echo /teach/status --once
```

预期：

- 三个 P0 topic 存在。
- `/teach/status` 输出 `teach=marked`、`teach=recording`、`teach=stopped` 或 `teach=saved`。
- `maps/task_map.yaml` 生成。

- [ ] **步骤 7：Commit ros2_robot_sim 仓库**

```bash
git add launch/navigation.launch.py docs/CLAUDE_CODE_HANDOFF.md src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 集成 P0 任务导航启动入口"
```

## 计划自检清单

- 规格覆盖：
  - P0 仿真闭环：任务 1-6 覆盖。
  - 遥控示教：任务 2 和任务 5 覆盖。
  - `task_map.yaml`：任务 1 覆盖。
  - 倒车配置：任务 1 和任务 3 覆盖。
  - 地图实时查看：任务 5 先提供 remote Map Monitor 入口；完整图形化 overlay 编辑留 P1。
  - `remote` 基于现有项目扩展：任务 5 覆盖。
  - P1-P3：规格文档已有路线图，本计划不实现。
- 范围控制：
  - 不实现真实 4G、真实平板 App、蓝牙/WiFi 配网。
  - 不实现作业区自动识别、视觉/信标硬件接入。
  - 不直接修改 `.pgm` 底图。
- 验证：
  - 每个任务有失败测试、实现、通过测试和 commit。
  - 最终有 pytest、py_compile、colcon build 和 runtime smoke。
