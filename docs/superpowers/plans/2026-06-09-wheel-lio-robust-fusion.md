# Wheel-LIO 鲁棒融合实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 为 `scripts/wheel_lio_fusion.py` 增加状态机式一致性检测和动态轮速权重，提升对轮速打滑、急转和短时 FAST-LIO 跳变的鲁棒性。

**架构：** 保持现有输出 `/localization/wheel_lio_odom` 和 `/localization/wheel_lio_status` 不变。新增纯函数层计算 wheel/LIO 短时运动差异并分类为 `normal`、`turning_caution`、`wheel_suspect`、`lio_suspect`、`degraded`，节点运行时按状态选择 `wheel_weight` 并增强 status 诊断信息。`/robot/ground_truth/odom` 仍只用于诊断，不进入生产融合。

**技术栈：** ROS 2 Humble、rclpy、nav_msgs/Odometry、std_msgs/String、pytest 静态/纯函数测试、现有 Gazebo + FAST-LIO2 验证链路。

---

## 文件结构

- 修改 `scripts/wheel_lio_fusion.py`
  - 保留现有 ROS 节点职责。
  - 新增纯函数和小型数据结构，用于 motion delta、状态分类、权重选择和位姿混合。
  - 增强 status 输出，包含 `state=`、`wheel_weight=`、`reason=`。
- 修改 `src/robot_description/test/test_wheel_encoder_integration.py`
  - 在现有 wheel-LIO 测试附近新增纯函数单元测试和静态契约测试。
  - 保留现有 topic 和建图默认链路测试。
- 修改 `docs/CLAUDE_CODE_HANDOFF.md`
  - 实现完成后记录本轮验证结果、状态机行为和下一步建议。

不新增独立 Python 模块。当前功能范围集中，`wheel_lio_fusion.py` 仍可保持可读；若后续继续增长，再拆出 `scripts/wheel_lio_fusion_core.py`。

## 任务 1：新增 motion delta 和状态分类纯函数

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 修改：`scripts/wheel_lio_fusion.py`

- [ ] **步骤 1：编写失败的 motion delta 测试**

在 `src/robot_description/test/test_wheel_encoder_integration.py` 的 `test_wheel_lio_fusion_only_initializes_anchor_from_fresh_lio` 后追加：

```python
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
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_wheel_lio_fusion_computes_motion_delta -v
```

预期：FAIL，报错包含 `AttributeError: module 'wheel_lio_fusion' has no attribute 'compute_motion_delta'`。

- [ ] **步骤 3：实现 motion delta 最小代码**

在 `scripts/wheel_lio_fusion.py` 的标准库 import 区域添加：

```python
from dataclasses import dataclass
```

在 `EARTH_RADIUS_M` 后添加：

```python


@dataclass(frozen=True)
class MotionDelta:
    dx: float
    dy: float
    distance: float
    heading: float
    yaw_delta: float
    speed: float


def compute_motion_delta(previous_pose, current_pose, dt):
    dx = current_pose[0] - previous_pose[0]
    dy = current_pose[1] - previous_pose[1]
    distance = math.hypot(dx, dy)
    heading = math.atan2(dy, dx) if distance > 1e-9 else previous_pose[2]
    yaw_delta = wrap_angle(current_pose[2] - previous_pose[2])
    speed = distance / dt if dt > 1e-6 else 0.0
    return MotionDelta(
        dx=dx,
        dy=dy,
        distance=distance,
        heading=heading,
        yaw_delta=yaw_delta,
        speed=speed,
    )
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_wheel_lio_fusion_computes_motion_delta -v
```

预期：PASS。

- [ ] **步骤 5：编写运动比较失败测试**

继续添加：

```python
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
```

- [ ] **步骤 6：运行运动比较测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'compares_motion_metrics'
```

预期：FAIL，报错包含缺少 `compare_wheel_lio_motion`。

- [ ] **步骤 7：实现运动比较最小代码**

在 `FusionDecision` 后添加：

```python
@dataclass(frozen=True)
class MotionComparison:
    distance_diff: float
    direction_diff: float
    yaw_diff: float
    wheel_lio_speed_ratio: float
    lio_wheel_speed_ratio: float


def compare_wheel_lio_motion(wheel_delta, lio_delta):
    return MotionComparison(
        distance_diff=abs(wheel_delta.distance - lio_delta.distance),
        direction_diff=abs(wrap_angle(wheel_delta.heading - lio_delta.heading)),
        yaw_diff=abs(wrap_angle(wheel_delta.yaw_delta - lio_delta.yaw_delta)),
        wheel_lio_speed_ratio=speed_ratio(wheel_delta.speed, lio_delta.speed),
        lio_wheel_speed_ratio=speed_ratio(lio_delta.speed, wheel_delta.speed),
    )
```

- [ ] **步骤 8：编写状态分类失败测试**

继续添加：

```python
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

    degraded = module.classify_fusion_state(
        wheel_delta=module.MotionDelta(2.0, 0.0, 2.0, 0.0, 0.0, 2.0),
        lio_delta=module.MotionDelta(0.1, 0.0, 0.1, 0.0, 0.0, 0.1),
        thresholds=thresholds,
        consecutive_bad_frames=thresholds.max_consecutive_bad_frames,
    )
    assert degraded.state == "degraded"
```

- [ ] **步骤 9：运行状态分类测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'classifies_motion_consistency'
```

预期：FAIL，报错包含缺少 `FusionThresholds` 或 `classify_fusion_state`。

- [ ] **步骤 10：实现状态分类最小代码**

在 `MotionDelta` 后添加：

```python
@dataclass(frozen=True)
class FusionThresholds:
    motion_window_min_distance: float = 0.05
    wheel_lio_distance_warn: float = 0.15
    wheel_lio_distance_error: float = 0.35
    wheel_lio_speed_ratio_warn: float = 1.8
    wheel_lio_speed_ratio_error: float = 3.0
    yaw_delta_warn: float = 0.25
    yaw_delta_error: float = 0.60
    turning_yaw_rate_threshold: float = 0.25
    max_consecutive_bad_frames: int = 5


@dataclass(frozen=True)
class FusionDecision:
    state: str
    reason: str


def speed_ratio(high, low):
    if low <= 1e-6:
        return float("inf") if high > 1e-6 else 1.0
    return high / low


def classify_fusion_state(wheel_delta, lio_delta, thresholds, consecutive_bad_frames):
    comparison = compare_wheel_lio_motion(wheel_delta, lio_delta)

    if consecutive_bad_frames >= thresholds.max_consecutive_bad_frames:
        return FusionDecision("degraded", "consecutive_bad_frames")

    if (
        wheel_delta.distance > lio_delta.distance
        and comparison.distance_diff >= thresholds.wheel_lio_distance_error
    ) or comparison.wheel_lio_speed_ratio >= thresholds.wheel_lio_speed_ratio_error:
        return FusionDecision("wheel_suspect", "wheel_distance_high")

    if (
        lio_delta.distance > wheel_delta.distance
        and comparison.distance_diff >= thresholds.wheel_lio_distance_error
    ) or comparison.lio_wheel_speed_ratio >= thresholds.wheel_lio_speed_ratio_error:
        return FusionDecision("lio_suspect", "lio_distance_high")

    if comparison.yaw_diff >= thresholds.yaw_delta_error:
        return FusionDecision("degraded", "yaw_delta_error")

    if abs(wheel_delta.yaw_delta) >= thresholds.turning_yaw_rate_threshold:
        return FusionDecision("turning_caution", "yaw_rate_high")

    if comparison.distance_diff >= thresholds.wheel_lio_distance_warn:
        return FusionDecision("turning_caution", "distance_warn")

    if comparison.direction_diff >= thresholds.yaw_delta_warn:
        return FusionDecision("turning_caution", "direction_warn")

    if comparison.yaw_diff >= thresholds.yaw_delta_warn:
        return FusionDecision("turning_caution", "yaw_delta_warn")

    return FusionDecision("normal", "motion_consistent")
```

- [ ] **步骤 11：运行任务 1 测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio_fusion_computes_motion_delta or compares_motion_metrics or classifies_motion_consistency'
```

预期：`3 passed`。

- [ ] **步骤 12：Commit**

```bash
git add scripts/wheel_lio_fusion.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 wheel-LIO 一致性分类"
```

## 任务 2：新增权重选择和位姿混合

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 修改：`scripts/wheel_lio_fusion.py`

- [ ] **步骤 1：编写权重和混合失败测试**

在任务 1 测试后添加：

```python
def test_wheel_lio_fusion_weights_and_blends_pose_by_state():
    module = load_script_module("wheel_lio_fusion.py")
    weights = module.FusionWeights()

    assert module.wheel_weight_for_state("normal", weights) == 1.0
    assert module.wheel_weight_for_state("turning_caution", weights) == 0.8
    assert module.wheel_weight_for_state("wheel_suspect", weights) == 0.2
    assert module.wheel_weight_for_state("lio_suspect", weights) == 1.0
    assert module.wheel_weight_for_state("degraded", weights) == 0.0

    blended = module.blend_fused_pose(
        wheel_projected_pose=(10.0, 0.0, 0.3),
        lio_pose=(0.0, 10.0, 0.8),
        wheel_weight=0.2,
        use_lio_yaw=True,
    )

    assert abs(blended[0] - 2.0) < 1e-6
    assert abs(blended[1] - 8.0) < 1e-6
    assert abs(blended[2] - 0.8) < 1e-6
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'weights_and_blends'
```

预期：FAIL，缺少 `FusionWeights` 或 `blend_fused_pose`。

- [ ] **步骤 3：实现权重和混合纯函数**

在 `FusionThresholds` 后添加：

```python
@dataclass(frozen=True)
class FusionWeights:
    normal: float = 1.0
    turning_caution: float = 0.8
    wheel_suspect: float = 0.2
    lio_suspect: float = 1.0
    degraded: float = 0.0


def wheel_weight_for_state(state, weights):
    return {
        "normal": weights.normal,
        "turning_caution": weights.turning_caution,
        "wheel_suspect": weights.wheel_suspect,
        "lio_suspect": weights.lio_suspect,
        "degraded": weights.degraded,
    }.get(state, weights.degraded)


def blend_fused_pose(wheel_projected_pose, lio_pose, wheel_weight, use_lio_yaw):
    weight = max(0.0, min(1.0, wheel_weight))
    x = weight * wheel_projected_pose[0] + (1.0 - weight) * lio_pose[0]
    y = weight * wheel_projected_pose[1] + (1.0 - weight) * lio_pose[1]
    if use_lio_yaw:
        yaw = lio_pose[2]
    else:
        yaw = wheel_projected_pose[2]
    return (x, y, wrap_angle(yaw))
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'weights_and_blends'
```

预期：PASS。

- [ ] **步骤 5：新增 anchor 组合函数回归测试**

继续添加测试，确保 `compose_wheel_lio_pose` 旧投影行为仍保持可用：

```python
def test_wheel_lio_fusion_old_anchor_projection_stays_available():
    module = load_script_module("wheel_lio_fusion.py")

    projected = module.compose_wheel_lio_pose(
        lio_anchor=(10.0, 5.0, 0.2),
        wheel_anchor=(1.0, 1.0, 0.0),
        wheel_current=(3.0, 1.0, 0.0),
        lio_current=(10.4, 5.1, 0.25),
        use_lio_yaw=True,
    )

    assert abs(projected[0] - 11.9601331557) < 1e-6
    assert abs(projected[1] - 5.3973386616) < 1e-6
    assert abs(projected[2] - 0.25) < 1e-6
```

- [ ] **步骤 6：运行 wheel-LIO 纯函数测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio_fusion'
```

预期：所有 wheel-LIO 相关测试通过。

- [ ] **步骤 7：Commit**

```bash
git add scripts/wheel_lio_fusion.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 添加 wheel-LIO 动态权重"
```

## 任务 3：把状态机接入 ROS 节点输出

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 修改：`scripts/wheel_lio_fusion.py`

- [ ] **步骤 1：编写静态契约失败测试**

扩展 `test_wheel_lio_fusion_contract`：

```python
    assert "motion_window_min_distance" in script
    assert "wheel_lio_distance_warn" in script
    assert "wheel_lio_distance_error" in script
    assert "wheel_lio_speed_ratio_warn" in script
    assert "wheel_lio_speed_ratio_error" in script
    assert "yaw_delta_warn" in script
    assert "yaw_delta_error" in script
    assert "turning_yaw_rate_threshold" in script
    assert "wheel_weight_normal" in script
    assert "wheel_weight_turning" in script
    assert "wheel_weight_wheel_suspect" in script
    assert "wheel_weight_lio_suspect" in script
    assert "wheel_weight_degraded" in script
    assert "max_consecutive_bad_frames" in script
    assert "state=" in script
    assert "wheel_weight=" in script
    assert "last_trusted_odom" in script
```

- [ ] **步骤 2：运行契约测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_wheel_lio_fusion_contract -v
```

预期：FAIL，缺少新增参数或 status 字段。

- [ ] **步骤 3：声明 ROS 参数和状态字段**

在 `WheelLioFusion.__init__` 中现有参数后添加：

```python
        self.declare_parameter("motion_window_min_distance", 0.05)
        self.declare_parameter("wheel_lio_distance_warn", 0.15)
        self.declare_parameter("wheel_lio_distance_error", 0.35)
        self.declare_parameter("wheel_lio_speed_ratio_warn", 1.8)
        self.declare_parameter("wheel_lio_speed_ratio_error", 3.0)
        self.declare_parameter("yaw_delta_warn", 0.25)
        self.declare_parameter("yaw_delta_error", 0.60)
        self.declare_parameter("turning_yaw_rate_threshold", 0.25)
        self.declare_parameter("wheel_weight_normal", 1.0)
        self.declare_parameter("wheel_weight_turning", 0.8)
        self.declare_parameter("wheel_weight_wheel_suspect", 0.2)
        self.declare_parameter("wheel_weight_lio_suspect", 1.0)
        self.declare_parameter("wheel_weight_degraded", 0.0)
        self.declare_parameter("max_consecutive_bad_frames", 5)
```

继续在运行状态字段处添加：

```python
        self.previous_lio_pose = None
        self.previous_wheel_pose = None
        self.previous_motion_stamp = None
        self.consecutive_bad_frames = 0
        self.last_trusted_odom = None
```

- [ ] **步骤 4：添加参数读取 helper**

在 `is_fresh` 后添加：

```python
    def fusion_thresholds(self):
        return FusionThresholds(
            motion_window_min_distance=float(self.get_parameter("motion_window_min_distance").value),
            wheel_lio_distance_warn=float(self.get_parameter("wheel_lio_distance_warn").value),
            wheel_lio_distance_error=float(self.get_parameter("wheel_lio_distance_error").value),
            wheel_lio_speed_ratio_warn=float(self.get_parameter("wheel_lio_speed_ratio_warn").value),
            wheel_lio_speed_ratio_error=float(self.get_parameter("wheel_lio_speed_ratio_error").value),
            yaw_delta_warn=float(self.get_parameter("yaw_delta_warn").value),
            yaw_delta_error=float(self.get_parameter("yaw_delta_error").value),
            turning_yaw_rate_threshold=float(self.get_parameter("turning_yaw_rate_threshold").value),
            max_consecutive_bad_frames=int(self.get_parameter("max_consecutive_bad_frames").value),
        )

    def fusion_weights(self):
        return FusionWeights(
            normal=float(self.get_parameter("wheel_weight_normal").value),
            turning_caution=float(self.get_parameter("wheel_weight_turning").value),
            wheel_suspect=float(self.get_parameter("wheel_weight_wheel_suspect").value),
            lio_suspect=float(self.get_parameter("wheel_weight_lio_suspect").value),
            degraded=float(self.get_parameter("wheel_weight_degraded").value),
        )
```

- [ ] **步骤 5：添加状态决策 helper**

在 `fusion_weights` 后添加：

```python
    def classify_current_motion(self, wheel_pose, lio_pose):
        thresholds = self.fusion_thresholds()
        now = self.get_clock().now()
        if (
            self.previous_lio_pose is None
            or self.previous_wheel_pose is None
            or self.previous_motion_stamp is None
        ):
            self.previous_lio_pose = lio_pose
            self.previous_wheel_pose = wheel_pose
            self.previous_motion_stamp = now
            return FusionDecision("normal", "initializing")

        dt = (now - self.previous_motion_stamp).nanoseconds / 1e9
        wheel_delta = compute_motion_delta(self.previous_wheel_pose, wheel_pose, dt)
        lio_delta = compute_motion_delta(self.previous_lio_pose, lio_pose, dt)

        if (
            wheel_delta.distance < thresholds.motion_window_min_distance
            and lio_delta.distance < thresholds.motion_window_min_distance
        ):
            return FusionDecision("normal", "motion_window_small")

        decision = classify_fusion_state(
            wheel_delta=wheel_delta,
            lio_delta=lio_delta,
            thresholds=thresholds,
            consecutive_bad_frames=self.consecutive_bad_frames,
        )
        self.previous_lio_pose = lio_pose
        self.previous_wheel_pose = wheel_pose
        self.previous_motion_stamp = now
        return decision
```

- [ ] **步骤 6：修改 publish_if_ready 融合逻辑**

在 `publish_if_ready` 中，计算 `lio_pose`、`wheel_pose` 后，用以下逻辑替换现有 `maybe_refresh_anchor_and_pose` 到 `fused.pose` 赋值前的部分：

```python
        decision = self.classify_current_motion(wheel_pose, lio_pose)
        if decision.state in ("wheel_suspect", "lio_suspect", "degraded"):
            self.consecutive_bad_frames += 1
        else:
            self.consecutive_bad_frames = 0

        max_error = float(self.get_parameter("max_lio_translation_error").value)
        if decision.state in ("normal", "lio_suspect"):
            self.lio_anchor, self.wheel_anchor, wheel_projected_pose = maybe_refresh_anchor_and_pose(
                self.lio_anchor,
                self.wheel_anchor,
                wheel_pose,
                lio_pose,
                use_lio_yaw,
                max_error,
            )
        else:
            wheel_projected_pose = compose_wheel_lio_pose(
                self.lio_anchor,
                self.wheel_anchor,
                wheel_pose,
                lio_pose,
                use_lio_yaw=use_lio_yaw,
            )

        weights = self.fusion_weights()
        wheel_weight = wheel_weight_for_state(decision.state, weights)
        if decision.state == "degraded" and self.last_trusted_odom is not None:
            self.odom_pub.publish(self.last_trusted_odom)
            self.publish_status(
                self.status_text(decision, wheel_weight, use_lio_yaw, "holding_last_trusted")
            )
            return
        if decision.state == "degraded" and self.last_trusted_odom is None:
            self.publish_status(
                self.status_text(decision, wheel_weight, use_lio_yaw, "no_trusted_pose")
            )
            return

        fused_pose = blend_fused_pose(
            wheel_projected_pose=wheel_projected_pose,
            lio_pose=lio_pose,
            wheel_weight=wheel_weight,
            use_lio_yaw=use_lio_yaw,
        )
```

Keep the existing `Odometry()` creation and field assignments. After `self.odom_pub.publish(fused)`, add:

```python
        if decision.state != "degraded":
            self.last_trusted_odom = fused
```

- [ ] **步骤 7：增强 status helper**

Replace the old final status construction:

```python
        lio_state = "lio_yaw=fresh" if use_lio_yaw else "lio_yaw=fallback"
        self.publish_status(f"{lio_state}; wheel=fresh")
```

with:

```python
        self.publish_status(
            self.status_text(decision, wheel_weight, use_lio_yaw, decision.reason)
        )
```

Add helper:

```python
    def status_text(self, decision, wheel_weight, use_lio_yaw, reason):
        lio_state = "lio_yaw=fresh" if use_lio_yaw else "lio_yaw=fallback"
        return (
            f"state={decision.state}; "
            f"wheel_weight={wheel_weight:.2f}; "
            f"reason={reason}; "
            f"{lio_state}; wheel=fresh"
        )
```

Leave `publish_status` appending GPS state:

```python
    def publish_status(self, prefix):
        self.status_pub.publish(String(data=f"{prefix}; {self.gps_state()}"))
```

- [ ] **步骤 8：运行契约和 wheel-LIO 测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio_fusion'
```

预期：所有 wheel-LIO 相关测试通过。

- [ ] **步骤 9：Commit**

```bash
git add scripts/wheel_lio_fusion.py src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "feat: 接入 wheel-LIO 鲁棒状态机"
```

## 任务 4：验证生产链路不使用 ground truth 并保留建图默认

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：新增生产输入契约测试**

添加：

```python
def test_wheel_lio_robust_fusion_does_not_use_ground_truth_for_production():
    fusion = read(WORKSPACE_DIR / "scripts" / "wheel_lio_fusion.py")
    exporter = read(WORKSPACE_DIR / "scripts" / "export_odom_projected_map.py")

    assert "/robot/ground_truth/odom" not in fusion
    assert 'default="/localization/wheel_lio_odom"' in exporter
    assert "--reference-topic" in exporter
    assert "/robot/ground_truth/odom" in exporter
```

- [ ] **步骤 2：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_wheel_lio_robust_fusion_does_not_use_ground_truth_for_production -v
```

预期：PASS。

- [ ] **步骤 3：运行相关契约测试集合**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q -k 'wheel_lio or odom_projected_exporter_uses_configurable_pose'
```

预期：PASS。

- [ ] **步骤 4：Commit**

```bash
git add src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: 固定 wheel-LIO 生产输入契约"
```

## 任务 5：静态验证、构建和运行态 smoke test

**文件：**
- 修改：`docs/CLAUDE_CODE_HANDOFF.md`

- [ ] **步骤 1：运行完整静态测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
```

预期：全部通过，例如 `84 passed` 或更高测试数。

- [ ] **步骤 2：运行 Python 语法检查**

运行：

```bash
python3 -m py_compile scripts/wheel_lio_fusion.py scripts/export_odom_projected_map.py scripts/fast_lio_drift_diagnostic.py launch/fast_lio2.launch.py
```

预期：无输出，退出码 0。

- [ ] **步骤 3：构建包**

运行：

```bash
colcon build --packages-select robot_description
```

预期：`Summary: 1 package finished`。

- [ ] **步骤 4：启动运行态验证**

终端 1：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

预期：出现 `Successfully spawned entity [ackermann_robot]`。

终端 2：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description fast_lio2.launch.py
```

预期：出现 `Extrinsics detected`。

终端 3：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/wheel_lio_fusion.py
```

- [ ] **步骤 5：采样状态 topic**

运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 12s ros2 topic echo /localization/wheel_lio_status --once
```

预期：输出包含：

```text
state=
wheel_weight=
lio_yaw=
gps=
```

正常短程仿真优先预期 `state=normal`，急转时允许 `state=turning_caution`。

- [ ] **步骤 6：运行漂移诊断**

在另一个终端让车运动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 20s ros2 topic pub -r 5 /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.30}, angular: {z: 0.04}}"
```

同时运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/fast_lio_drift_diagnostic.py \
  --duration-sec 22 \
  --output maps/fast_lio_drift_robust_validation.json \
  --reference-topic /robot/ground_truth/odom
```

预期：JSON 包含 `/mapping/lio/odom` 和 `/localization/wheel_lio_odom`。

- [ ] **步骤 7：运行建图 smoke test**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/export_odom_projected_map.py \
  --output maps/wheel_lio_robust_projected_validation \
  --duration-sec 22 \
  --pose-topic /localization/wheel_lio_odom \
  --reference-topic /robot/ground_truth/odom
```

预期：生成：

```text
maps/wheel_lio_robust_projected_validation.yaml
maps/wheel_lio_robust_projected_validation.pgm
maps/wheel_lio_robust_projected_validation.json
```

这些是临时验证产物，不作为正式地图提交。

- [ ] **步骤 8：清理运行态**

```bash
pkill -f "ros2 launch robot_description robot_simulation.launch.py" || true
pkill -f "ros2 launch robot_description fast_lio2.launch.py" || true
pkill -f "wheel_lio_fusion.py" || true
pkill -f "gzserver" || true
```

然后确认：

```bash
ps -ef | rg 'ros2|gazebo|gzserver|spark_lio|wheel_lio'
```

预期：只剩当前 `ps` / `rg` 命令本身或 ROS daemon。

- [ ] **步骤 9：更新交接文档**

在 `docs/CLAUDE_CODE_HANDOFF.md` 追加：

```markdown
### 2026-06-09：Codex 实现 wheel-LIO 鲁棒状态机

**本次处理：**

- `scripts/wheel_lio_fusion.py` 增加 motion consistency state machine。
- `/localization/wheel_lio_status` 增加 `state=`、`wheel_weight=`、`reason=`。
- 保持 `/localization/wheel_lio_odom` 作为建图默认位姿源。
- `/robot/ground_truth/odom` 仍只用于诊断。

**验证：**

- `python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q` 通过。
- `colcon build --packages-select robot_description` 通过。
- 运行态 status 可采样，正常仿真主要为 `state=normal` 或 `state=turning_caution`。
```

- [ ] **步骤 10：Commit**

```bash
git add docs/CLAUDE_CODE_HANDOFF.md
git commit -m "docs: 记录 wheel-LIO 鲁棒融合验证"
```

## 任务 6：最终状态检查

**文件：**
- 不修改文件。

- [ ] **步骤 1：检查工作区**

运行：

```bash
git status --short
```

预期：只剩用户之前已有的未提交临时地图或历史改动；本计划产生的代码、测试、文档改动已经提交。不要删除用户未明确要求删除的验证产物。

- [ ] **步骤 2：总结验证证据**

记录以下事实供最终回复使用：

```text
pytest: use the exact passed count printed by the final pytest command
build: Summary: 1 package finished
status sample: copy the exact /localization/wheel_lio_status line from runtime validation
drift output: maps/fast_lio_drift_robust_validation.json
map output: maps/wheel_lio_robust_projected_validation.{yaml,pgm,json}
```

- [ ] **步骤 3：不要提交临时地图**

确认以下文件如果存在，默认不提交：

```text
maps/*validation*
maps/*quick*
maps/lio_map_live_check.*
```

除非用户明确要求保留为仓库资产。
