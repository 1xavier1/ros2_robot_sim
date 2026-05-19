# Real Ackermann Drive Controller 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 用统一的 Gazebo ModelPlugin 替换当前 `diff_drive + steering_controller + cmd_vel_bridge.py` 三段式链路，让小车真实按 Ackermann 几何转向，并保持 `/robot/cmd_vel` 外部接口不变。

**架构：** 新增 `libackermann_drive_controller.so`，直接订阅 `/robot/cmd_vel`，在同一 Gazebo update loop 中计算左右前轮转角、左右后轮速度、发布 `/robot/odom` 和 `odom -> base_footprint` TF。URDF 移除 `libgazebo_ros_diff_drive.so` 和 `libsteering_controller.so`，launch 移除 `cmd_vel_bridge.py`。

**技术栈：** ROS 2 Humble、Gazebo Classic 11、C++ Gazebo `ModelPlugin`、`gazebo_ros::Node`、`rclcpp`、`geometry_msgs`、`nav_msgs`、`tf2_ros`、`ament_cmake_pytest`、Python 静态和公式测试。

---

## 文件结构

- `src/robot_description/src/ackermann_drive_controller.cpp`：新增统一底盘控制插件。职责：读取 SDF 参数、订阅 `/robot/cmd_vel`、计算 Ackermann 目标、控制前轮转向和后轮速度、发布 `/robot/odom` 和 TF。
- `src/robot_description/src/steering_controller.cpp`：删除旧前轮转向插件源文件。统一插件接管前轮转向后，该文件不能继续构建。
- `scripts/cmd_vel_bridge.py`：删除旧桥接节点。统一插件直接订阅 `/robot/cmd_vel`，不再需要 `/robot/rear/cmd_vel` 或 `/robot/steering_cmd`。
- `src/robot_description/CMakeLists.txt`：新增 `ackermann_drive_controller` shared library，补齐依赖，移除 `steering_controller` target 和安装项。
- `src/robot_description/package.xml`：确认 `geometry_msgs`、`nav_msgs`、`tf2_ros`、`std_msgs`、`gazebo_ros`、`gazebo_dev`、`rclcpp` 依赖存在。
- `src/robot_description/urdf/robot_base.urdf.xacro`：移除旧 drive/steering plugin block，新增 `ackermann_drive_controller` plugin block。保留传感器、joint state publisher、wheel encoder。
- `launch/robot_simulation.launch.py`：移除 `cmd_vel_bridge.py` node 和对应 `TimerAction`。
- `src/robot_description/test/ackermann_math_reference.py`：新增 Python 参考公式，用于测试 Ackermann 几何数值。
- `src/robot_description/test/test_ackermann_kinematics.py`：新增公式单元测试。
- `src/robot_description/test/test_wheel_encoder_integration.py`：更新静态集成测试，验证新插件布线和旧链路移除。

## 任务 1：建立 Ackermann 几何公式测试

**文件：**
- 创建：`src/robot_description/test/ackermann_math_reference.py`
- 创建：`src/robot_description/test/test_ackermann_kinematics.py`

- [ ] **步骤 1：创建参考公式模块**

创建 `src/robot_description/test/ackermann_math_reference.py`：

```python
import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def compute_ackermann_targets(
    linear_x,
    angular_z,
    wheelbase=0.45,
    track_width=0.35,
    wheel_radius=0.07,
    max_steering_angle=0.5236,
):
    if abs(linear_x) < 0.01:
        steer = clamp(angular_z * 0.5, -max_steering_angle, max_steering_angle)
        return steer, steer, 0.0, 0.0

    steer = math.atan(wheelbase * angular_z / linear_x)
    steer = clamp(steer, -max_steering_angle, max_steering_angle)

    if abs(steer) < 1e-6:
        wheel_velocity = linear_x / wheel_radius
        return 0.0, 0.0, wheel_velocity, wheel_velocity

    sign = 1.0 if steer > 0.0 else -1.0
    radius = wheelbase / math.tan(abs(steer))

    inner_angle = math.atan(wheelbase / (radius - track_width / 2.0))
    outer_angle = math.atan(wheelbase / (radius + track_width / 2.0))

    inner_rear_linear = abs(linear_x) * (radius - track_width / 2.0) / radius
    outer_rear_linear = abs(linear_x) * (radius + track_width / 2.0) / radius

    if linear_x < 0.0:
        inner_rear_linear = -inner_rear_linear
        outer_rear_linear = -outer_rear_linear

    if sign > 0.0:
        left_steer = inner_angle
        right_steer = outer_angle
        left_wheel = inner_rear_linear / wheel_radius
        right_wheel = outer_rear_linear / wheel_radius
    else:
        left_steer = -outer_angle
        right_steer = -inner_angle
        left_wheel = outer_rear_linear / wheel_radius
        right_wheel = inner_rear_linear / wheel_radius

    return left_steer, right_steer, left_wheel, right_wheel
```

- [ ] **步骤 2：创建公式单元测试**

创建 `src/robot_description/test/test_ackermann_kinematics.py`：

```python
import pytest

from ackermann_math_reference import compute_ackermann_targets


def test_left_turn_has_larger_left_steering_angle_and_outer_rear_speed():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.5, 0.3)

    assert left > right > 0.0
    assert left == pytest.approx(0.293, abs=0.02)
    assert right == pytest.approx(0.239, abs=0.02)
    assert rear_right > rear_left > 0.0


def test_right_turn_has_larger_right_steering_angle_and_outer_rear_speed():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.5, -0.3)

    assert right < left < 0.0
    assert abs(right) > abs(left)
    assert rear_left > rear_right > 0.0


def test_zero_linear_velocity_does_not_drive_rear_wheels():
    left, right, rear_left, rear_right = compute_ackermann_targets(0.0, 0.3)

    assert left > 0.0
    assert right > 0.0
    assert rear_left == 0.0
    assert rear_right == 0.0
```

- [ ] **步骤 3：运行公式测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py -v
```

预期：PASS，3 个测试通过。

- [ ] **步骤 4：Commit 公式测试**

```bash
git add src/robot_description/test/ackermann_math_reference.py src/robot_description/test/test_ackermann_kinematics.py
git commit -m "test: add ackermann kinematics reference"
```

## 任务 2：建立静态布线失败测试

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 测试：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：添加统一插件布线测试**

在 `src/robot_description/test/test_wheel_encoder_integration.py` 中加入或更新以下测试函数：

```python
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
```

- [ ] **步骤 2：添加 launch 旧桥接移除测试**

在同一文件中加入或更新：

```python
def test_launch_does_not_start_cmd_vel_bridge():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "cmd_vel_bridge.py" not in launch
    assert "cmd_vel_bridge" not in launch
    assert "rear/cmd_vel" not in launch
    assert "steering_cmd" not in launch
```

- [ ] **步骤 3：运行静态测试验证失败**

运行：

```bash
python3 -m pytest \
  src/robot_description/test/test_wheel_encoder_integration.py::test_urdf_uses_unified_ackermann_drive_controller \
  src/robot_description/test/test_wheel_encoder_integration.py::test_launch_does_not_start_cmd_vel_bridge \
  -v
```

预期：FAIL。当前 URDF 仍包含旧 `diff_drive` 和 `steering_controller`，launch 仍启动 `cmd_vel_bridge.py`。

- [ ] **步骤 4：Commit 失败测试**

```bash
git add src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: expect unified ackermann drive wiring"
```

## 任务 3：实现统一 Gazebo 插件

**文件：**
- 创建：`src/robot_description/src/ackermann_drive_controller.cpp`

- [ ] **步骤 1：创建插件源文件和成员结构**

创建 `src/robot_description/src/ackermann_drive_controller.cpp`：

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace gazebo
{

class AckermannDriveControllerPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  struct Targets
  {
    double left_steer = 0.0;
    double right_steer = 0.0;
    double rear_left_velocity = 0.0;
    double rear_right_velocity = 0.0;
  };

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void OnUpdate();
  Targets ComputeTargets(double linear_x, double angular_z) const;
  void ApplyTargets(const Targets & targets);
  void PublishOdometry(const common::Time & sim_time);
  double ReadDouble(sdf::ElementPtr sdf, const std::string & name, double fallback) const;
  std::string ReadString(sdf::ElementPtr sdf, const std::string & name, const std::string & fallback) const;

  physics::ModelPtr model_;
  physics::JointPtr left_steering_joint_;
  physics::JointPtr right_steering_joint_;
  physics::JointPtr rear_left_wheel_joint_;
  physics::JointPtr rear_right_wheel_joint_;
  event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_footprint";
  double wheelbase_ = 0.45;
  double track_width_ = 0.35;
  double wheel_radius_ = 0.07;
  double max_steering_angle_ = 0.5236;
  double cmd_timeout_ = 0.5;
  double update_period_ = 0.02;
  double max_wheel_torque_ = 50.0;

  double target_linear_ = 0.0;
  double target_angular_ = 0.0;
  common::Time last_cmd_time_;
  common::Time last_update_time_;
};

}  // namespace gazebo
```

- [ ] **步骤 2：实现参数读取辅助函数**

在同一文件、类定义后加入：

```cpp
namespace gazebo
{

double AckermannDriveControllerPlugin::ReadDouble(
    sdf::ElementPtr sdf, const std::string & name, double fallback) const
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}

std::string AckermannDriveControllerPlugin::ReadString(
    sdf::ElementPtr sdf, const std::string & name, const std::string & fallback) const
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}
```

- [ ] **步骤 3：实现 Load**

继续加入：

```cpp
void AckermannDriveControllerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  const auto left_steering_name = ReadString(sdf, "left_steering_joint", "left_steering_joint");
  const auto right_steering_name = ReadString(sdf, "right_steering_joint", "right_steering_joint");
  const auto rear_left_name = ReadString(sdf, "rear_left_wheel_joint", "rear_left_joint");
  const auto rear_right_name = ReadString(sdf, "rear_right_wheel_joint", "rear_right_joint");

  const auto cmd_vel_topic = ReadString(sdf, "cmd_vel_topic", "cmd_vel");
  const auto odom_topic = ReadString(sdf, "odom_topic", "odom");
  odom_frame_ = ReadString(sdf, "odom_frame", "odom");
  base_frame_ = ReadString(sdf, "base_frame", "base_footprint");

  wheelbase_ = ReadDouble(sdf, "wheelbase", wheelbase_);
  track_width_ = ReadDouble(sdf, "track_width", track_width_);
  wheel_radius_ = ReadDouble(sdf, "wheel_radius", wheel_radius_);
  max_steering_angle_ = ReadDouble(sdf, "max_steering_angle", max_steering_angle_);
  cmd_timeout_ = ReadDouble(sdf, "cmd_timeout", cmd_timeout_);
  max_wheel_torque_ = ReadDouble(sdf, "max_wheel_torque", max_wheel_torque_);

  const double update_rate = ReadDouble(sdf, "update_rate", 50.0);
  update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.02;

  left_steering_joint_ = model_->GetJoint(left_steering_name);
  right_steering_joint_ = model_->GetJoint(right_steering_name);
  rear_left_wheel_joint_ = model_->GetJoint(rear_left_name);
  rear_right_wheel_joint_ = model_->GetJoint(rear_right_name);

  if (!left_steering_joint_ || !right_steering_joint_ ||
      !rear_left_wheel_joint_ || !rear_right_wheel_joint_) {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "AckermannDriveController missing joints: %s=%d %s=%d %s=%d %s=%d",
      left_steering_name.c_str(), static_cast<int>(left_steering_joint_ != nullptr),
      right_steering_name.c_str(), static_cast<int>(right_steering_joint_ != nullptr),
      rear_left_name.c_str(), static_cast<int>(rear_left_wheel_joint_ != nullptr),
      rear_right_name.c_str(), static_cast<int>(rear_right_wheel_joint_ != nullptr));
    return;
  }

  const double steering_p = ReadDouble(sdf, "steering_p", 50.0);
  const double steering_i = ReadDouble(sdf, "steering_i", 0.0);
  const double steering_d = ReadDouble(sdf, "steering_d", 5.0);

  auto joint_controller = model_->GetJointController();
  joint_controller->AddJoint(left_steering_joint_);
  joint_controller->AddJoint(right_steering_joint_);
  joint_controller->SetPositionPID(left_steering_joint_->GetScopedName(), common::PID(steering_p, steering_i, steering_d));
  joint_controller->SetPositionPID(right_steering_joint_->GetScopedName(), common::PID(steering_p, steering_i, steering_d));

  rear_left_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  rear_right_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);

  cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&AckermannDriveControllerPlugin::OnCmdVel, this, std::placeholders::_1));
  odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);

  last_cmd_time_ = model_->GetWorld()->SimTime();
  last_update_time_ = last_cmd_time_;
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AckermannDriveControllerPlugin::OnUpdate, this));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "AckermannDriveController ready: cmd=%s odom=%s wb=%.3f track=%.3f radius=%.3f",
    cmd_vel_topic.c_str(), odom_topic.c_str(), wheelbase_, track_width_, wheel_radius_);
}
```

- [ ] **步骤 4：实现命令回调和 Ackermann 目标计算**

继续加入：

```cpp
void AckermannDriveControllerPlugin::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_linear_ = msg->linear.x;
  target_angular_ = msg->angular.z;
  last_cmd_time_ = model_->GetWorld()->SimTime();
}

AckermannDriveControllerPlugin::Targets AckermannDriveControllerPlugin::ComputeTargets(
    double linear_x, double angular_z) const
{
  Targets targets;

  if (std::abs(linear_x) < 0.01) {
    const double steer = std::clamp(angular_z * 0.5, -max_steering_angle_, max_steering_angle_);
    targets.left_steer = steer;
    targets.right_steer = steer;
    return targets;
  }

  double steer = std::atan(wheelbase_ * angular_z / linear_x);
  steer = std::clamp(steer, -max_steering_angle_, max_steering_angle_);

  if (std::abs(steer) < 1e-6) {
    targets.rear_left_velocity = linear_x / wheel_radius_;
    targets.rear_right_velocity = linear_x / wheel_radius_;
    return targets;
  }

  const double sign = steer > 0.0 ? 1.0 : -1.0;
  const double radius = wheelbase_ / std::tan(std::abs(steer));
  const double inner_angle = std::atan(wheelbase_ / (radius - track_width_ / 2.0));
  const double outer_angle = std::atan(wheelbase_ / (radius + track_width_ / 2.0));

  double inner_rear_linear = std::abs(linear_x) * (radius - track_width_ / 2.0) / radius;
  double outer_rear_linear = std::abs(linear_x) * (radius + track_width_ / 2.0) / radius;

  if (linear_x < 0.0) {
    inner_rear_linear = -inner_rear_linear;
    outer_rear_linear = -outer_rear_linear;
  }

  if (sign > 0.0) {
    targets.left_steer = inner_angle;
    targets.right_steer = outer_angle;
    targets.rear_left_velocity = inner_rear_linear / wheel_radius_;
    targets.rear_right_velocity = outer_rear_linear / wheel_radius_;
  } else {
    targets.left_steer = -outer_angle;
    targets.right_steer = -inner_angle;
    targets.rear_left_velocity = outer_rear_linear / wheel_radius_;
    targets.rear_right_velocity = inner_rear_linear / wheel_radius_;
  }

  return targets;
}
```

- [ ] **步骤 5：实现目标应用、odom 和 update loop**

继续加入：

```cpp
void AckermannDriveControllerPlugin::ApplyTargets(const Targets & targets)
{
  auto joint_controller = model_->GetJointController();
  joint_controller->SetPositionTarget(left_steering_joint_->GetScopedName(), targets.left_steer);
  joint_controller->SetPositionTarget(right_steering_joint_->GetScopedName(), targets.right_steer);
  joint_controller->SetVelocityTarget(rear_left_wheel_joint_->GetScopedName(), targets.rear_left_velocity);
  joint_controller->SetVelocityTarget(rear_right_wheel_joint_->GetScopedName(), targets.rear_right_velocity);
  joint_controller->Update();
}

void AckermannDriveControllerPlugin::PublishOdometry(const common::Time & sim_time)
{
  const auto pose = model_->WorldPose();
  const auto linear = model_->WorldLinearVel();
  const auto angular = model_->WorldAngularVel();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(sim_time.sec);
  stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x = linear.X();
  odom.twist.twist.linear.y = linear.Y();
  odom.twist.twist.linear.z = linear.Z();
  odom.twist.twist.angular.x = angular.X();
  odom.twist.twist.angular.y = angular.Y();
  odom.twist.twist.angular.z = angular.Z();
  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = pose.Pos().X();
  tf.transform.translation.y = pose.Pos().Y();
  tf.transform.translation.z = pose.Pos().Z();
  tf.transform.rotation.x = pose.Rot().X();
  tf.transform.rotation.y = pose.Rot().Y();
  tf.transform.rotation.z = pose.Rot().Z();
  tf.transform.rotation.w = pose.Rot().W();
  tf_broadcaster_->sendTransform(tf);
}

void AckermannDriveControllerPlugin::OnUpdate()
{
  const common::Time now = model_->GetWorld()->SimTime();
  if ((now - last_update_time_).Double() < update_period_) {
    return;
  }
  last_update_time_ = now;

  double linear = target_linear_;
  double angular = target_angular_;
  if ((now - last_cmd_time_).Double() > cmd_timeout_) {
    linear = 0.0;
    angular = 0.0;
  }

  ApplyTargets(ComputeTargets(linear, angular));
  PublishOdometry(now);
}

GZ_REGISTER_MODEL_PLUGIN(AckermannDriveControllerPlugin)

}  // namespace gazebo
```

- [ ] **步骤 6：构建验证当前预期失败**

运行：

```bash
colcon build --packages-select robot_description
```

预期：FAIL。CMake 还没有构建 `ackermann_drive_controller` target，或者依赖尚未链接。

## 任务 4：接入构建系统和依赖

**文件：**
- 修改：`src/robot_description/CMakeLists.txt`
- 修改：`src/robot_description/package.xml`
- 删除：`src/robot_description/src/steering_controller.cpp`

- [ ] **步骤 1：更新 CMake 依赖和目标**

在 `src/robot_description/CMakeLists.txt` 中：

```cmake
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
```

删除旧 target：

```cmake
add_library(steering_controller SHARED src/steering_controller.cpp)
ament_target_dependencies(steering_controller
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
)
```

新增 target：

```cmake
add_library(ackermann_drive_controller SHARED src/ackermann_drive_controller.cpp)
ament_target_dependencies(ackermann_drive_controller
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)
```

把安装目标改为：

```cmake
install(TARGETS wheel_encoder ackermann_drive_controller
  DESTINATION lib
)
```

把导出库改为：

```cmake
ament_export_libraries(wheel_encoder ackermann_drive_controller)
ament_export_dependencies(
  ament_cmake
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)
```

- [ ] **步骤 2：确认 package.xml 依赖**

确认 `src/robot_description/package.xml` 至少包含：

```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
<depend>std_msgs</depend>
<depend>gazebo_dev</depend>
<depend>gazebo_ros</depend>
```

如果缺失，补入对应 `<depend>`。当前仓库已经包含这些依赖时，不改该文件。

- [ ] **步骤 3：删除旧 steering 插件源文件**

删除：

```bash
rm src/robot_description/src/steering_controller.cpp
```

如果执行环境要求审批，使用非交互删除命令并确认只删除此文件。

- [ ] **步骤 4：构建验证通过**

运行：

```bash
colcon build --packages-select robot_description
```

预期：PASS，输出包含：

```text
Finished <<< robot_description
```

- [ ] **步骤 5：Commit 构建接入**

```bash
git add src/robot_description/CMakeLists.txt src/robot_description/package.xml src/robot_description/src/ackermann_drive_controller.cpp
git add -u src/robot_description/src/steering_controller.cpp
git commit -m "feat: build unified ackermann drive controller"
```

## 任务 5：替换 URDF 和 launch 控制链路

**文件：**
- 修改：`src/robot_description/urdf/robot_base.urdf.xacro`
- 修改：`launch/robot_simulation.launch.py`
- 删除：`scripts/cmd_vel_bridge.py`

- [ ] **步骤 1：替换 URDF drive plugin**

在 `src/robot_description/urdf/robot_base.urdf.xacro` 中删除整个 `gazebo_ros_diff_drive` plugin block：

```xml
<plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
  ...
</plugin>
```

删除整个 `steering_controller` plugin block：

```xml
<plugin name="steering_controller" filename="libsteering_controller.so">
  ...
</plugin>
```

在相同 Gazebo plugin 区域加入：

```xml
<gazebo>
  <plugin name="ackermann_drive_controller" filename="libackermann_drive_controller.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <cmd_vel_topic>cmd_vel</cmd_vel_topic>
    <odom_topic>odom</odom_topic>
    <odom_frame>odom</odom_frame>
    <base_frame>base_footprint</base_frame>

    <left_steering_joint>left_steering_joint</left_steering_joint>
    <right_steering_joint>right_steering_joint</right_steering_joint>
    <rear_left_wheel_joint>rear_left_joint</rear_left_wheel_joint>
    <rear_right_wheel_joint>rear_right_joint</rear_right_wheel_joint>

    <wheelbase>${wheelbase}</wheelbase>
    <track_width>${track_width}</track_width>
    <wheel_radius>${wheel_radius}</wheel_radius>
    <max_steering_angle>0.5236</max_steering_angle>
    <cmd_timeout>0.5</cmd_timeout>
    <update_rate>50.0</update_rate>
    <steering_p>50.0</steering_p>
    <steering_i>0.0</steering_i>
    <steering_d>5.0</steering_d>
    <max_wheel_torque>50.0</max_wheel_torque>
  </plugin>
</gazebo>
```

- [ ] **步骤 2：移除 launch 桥接节点**

在 `launch/robot_simulation.launch.py` 中删除 `bridge = Node(...)` 整个变量定义。

把 return list 从：

```python
TimerAction(period=5.0, actions=[bridge]),
```

改为不包含该行。

保留 Gazebo、robot_state_publisher、RViz、spawn_entity 的启动逻辑。

- [ ] **步骤 3：删除旧桥接脚本**

删除：

```bash
rm scripts/cmd_vel_bridge.py
```

如果执行环境要求审批，使用非交互删除命令并确认只删除此文件。

- [ ] **步骤 4：运行静态测试验证通过**

运行：

```bash
python3 -m pytest \
  src/robot_description/test/test_wheel_encoder_integration.py::test_urdf_uses_unified_ackermann_drive_controller \
  src/robot_description/test/test_wheel_encoder_integration.py::test_launch_does_not_start_cmd_vel_bridge \
  -v
```

预期：PASS，2 个测试通过。

- [ ] **步骤 5：构建验证通过**

运行：

```bash
colcon build --packages-select robot_description
```

预期：PASS。

- [ ] **步骤 6：Commit 控制链路替换**

```bash
git add src/robot_description/urdf/robot_base.urdf.xacro launch/robot_simulation.launch.py
git add -u scripts/cmd_vel_bridge.py
git commit -m "feat: wire simulation to unified ackermann drive"
```

## 任务 6：完整测试和动态验收

**文件：**
- 测试：`src/robot_description/test/test_ackermann_kinematics.py`
- 测试：`src/robot_description/test/test_wheel_encoder_integration.py`
- 运行时验证：ROS 2/Gazebo 仿真

- [ ] **步骤 1：运行 Python 测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -v
```

预期：PASS，所有测试通过。

- [ ] **步骤 2：运行 colcon test**

运行：

```bash
colcon test --packages-select robot_description
colcon test-result --verbose
```

预期：PASS，没有 failed test。

- [ ] **步骤 3：启动无 GUI 仿真**

先清理旧仿真进程：

```bash
pkill -f "ros2 launch robot_description robot_simulation.launch.py" || true
pkill -f "gzserver" || true
pkill -f "gzclient" || true
pkill -f "rviz2" || true
```

启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p log/ros
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

预期日志包含：

```text
AckermannDriveController ready
Spawn status: SpawnEntity: Successfully spawned entity [ackermann_robot]
```

- [ ] **步骤 4：确认 ROS 图不再包含旧控制链路**

在另一个 shell 中运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic list --no-daemon
```

预期：

- 包含 `/robot/cmd_vel`
- 包含 `/robot/odom`
- 不包含 `/robot/rear/cmd_vel`
- 不包含 `/robot/steering_cmd`

- [ ] **步骤 5：发布持续左转指令**

运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

预期：命令持续发布 8 秒。

- [ ] **步骤 6：采样 joint_states 验证真实 Ackermann 转向**

发布持续左转指令期间运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /joint_states --once
```

预期：

- `left_steering_joint` 位置大于 `0.20`
- `right_steering_joint` 位置大于 `0.15`
- `left_steering_joint` 位置大于 `right_steering_joint`
- `rear_left_joint` 和 `rear_right_joint` velocity 非零
- `rear_right_joint` velocity 大于 `rear_left_joint` velocity

- [ ] **步骤 7：采样 odom 验证 yaw 变化**

左转发布前采样一次：

```bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /robot/odom --once
```

持续左转 5 秒后再采样一次同一命令。

预期：

- 第二次 odom pose 的 orientation z/w 与第一次不同。
- 第二次 odom pose 的 position x/y 与第一次不同。
- Gazebo 中车体轨迹呈弧线。

- [ ] **步骤 8：停止仿真并 Commit 验收记录**

停止 launch shell，或运行：

```bash
pkill -f "ros2 launch robot_description robot_simulation.launch.py" || true
pkill -f "gzserver" || true
pkill -f "gzclient" || true
pkill -f "rviz2" || true
```

如果为验收新增了记录文档，提交该文档：

```bash
git add 第一阶段_功能测试与验收.md
git commit -m "test: document ackermann dynamic validation"
```

如果没有新增或修改验收文档，不创建空提交。

## 任务 7：最终回归和工作区检查

**文件：**
- 检查：所有本计划涉及文件

- [ ] **步骤 1：运行完整构建**

运行：

```bash
colcon build --packages-select robot_description
```

预期：PASS。

- [ ] **步骤 2：运行完整测试**

运行：

```bash
colcon test --packages-select robot_description
colcon test-result --verbose
```

预期：PASS。

- [ ] **步骤 3：检查旧链路残留**

运行：

```bash
rg -n "libgazebo_ros_diff_drive|libsteering_controller|cmd_vel_bridge|rear/cmd_vel|steering_cmd" \
  src/robot_description launch scripts
```

预期：不在 active URDF、launch 或 installed script 中出现旧控制链路。允许历史文档或废弃配置文件中出现文字说明。

- [ ] **步骤 4：检查 Git 状态**

运行：

```bash
git status --short
```

预期：只显示用户已有的无关改动，或显示 clean worktree。不要 revert 用户已有改动。

- [ ] **步骤 5：最终提交**

如果前面任务留下已验证但未提交的代码改动，提交：

```bash
git add src/robot_description launch scripts
git commit -m "fix: enable real ackermann steering simulation"
```

如果没有未提交的本计划改动，不创建空提交。

