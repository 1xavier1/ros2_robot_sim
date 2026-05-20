# Ackermann Drive Controller 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 用统一的 Gazebo Model Plugin 替换 `diff_drive + steering_controller + cmd_vel_bridge`，让 `/robot/cmd_vel` 直接驱动车辆，并由同一插件发布基于 Gazebo 真实位姿的 `/robot/odom` 和 `odom -> base_footprint` TF。

**架构：** 新增 `libackermann_drive_controller.so`，在插件内部完成 Ackermann 转向角计算、后轮速度分配、关节控制、odom 和 TF 发布。URDF 只保留统一插件、joint state publisher、传感器和 wheel encoder，launch 不再启动桥接节点。

**技术栈：** ROS 2 Humble、Gazebo Classic 11、C++ Gazebo ModelPlugin、`gazebo_ros::Node`、`rclcpp`、`geometry_msgs`、`nav_msgs`、`tf2_ros`、`ament_cmake_pytest`、Python 静态集成测试。

---

## 文件结构

- `src/robot_description/src/ackermann_drive_controller.cpp`：新增统一 Gazebo Model Plugin。职责包括读取 SDF 参数、订阅 `/robot/cmd_vel`、计算 Ackermann 目标、控制转向/后轮关节、发布 odom 和 TF。
- `src/robot_description/src/steering_controller.cpp`：删除旧转向插件源文件，避免继续构建旧链路。
- `scripts/cmd_vel_bridge.py`：删除旧桥接节点，避免 launch 或用户误用三段式控制链路。
- `src/robot_description/CMakeLists.txt`：新增新插件构建目标和依赖，移除 `steering_controller` 目标，安装 `ackermann_drive_controller`。
- `src/robot_description/package.xml`：确保 `geometry_msgs`、`nav_msgs`、`tf2_ros`、`std_msgs`、`gazebo_ros`、`gazebo_dev`、`rclcpp` 依赖存在。
- `src/robot_description/urdf/robot_base.urdf.xacro`：移除 `libgazebo_ros_diff_drive.so` 和 `libsteering_controller.so` 插件块，加入 `libackermann_drive_controller.so` 插件块。
- `launch/robot_simulation.launch.py`：移除 `cmd_vel_bridge.py` 节点和 TimerAction，只保留 Gazebo、robot_state_publisher、RViz、spawn_entity。
- `src/robot_description/test/test_wheel_encoder_integration.py`：更新静态集成测试，证明新插件布线、旧链路移除、launch 不再启动桥接节点。
- `src/robot_description/test/test_ackermann_kinematics.py`：新增纯 Python 单元测试，用和 C++ 插件同一公式验证 Ackermann 几何的期望数值，作为实现基线。
- `src/robot_description/test/ackermann_math_reference.py`：新增测试用参考公式，避免把较长计算逻辑塞进测试断言里。

## 任务 1：建立静态迁移失败测试

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`
- 测试：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：编写失败的静态测试**

在 `test_wheel_encoder_integration.py` 中替换 `test_urdf_uses_diff_drive_with_revolute_steering`，并新增 launch 检查：

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


def test_launch_does_not_start_cmd_vel_bridge():
    launch = read(WORKSPACE_DIR / "launch" / "robot_simulation.launch.py")

    assert "cmd_vel_bridge.py" not in launch
    assert "cmd_vel_bridge" not in launch
    assert "rear/cmd_vel" not in launch
    assert "steering_cmd" not in launch
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_urdf_uses_unified_ackermann_drive_controller src/robot_description/test/test_wheel_encoder_integration.py::test_launch_does_not_start_cmd_vel_bridge -v
```

预期：FAIL。当前 URDF 仍包含 `libgazebo_ros_diff_drive.so` 和 `libsteering_controller.so`，launch 仍包含 `cmd_vel_bridge.py`。

- [ ] **步骤 3：Commit 测试**

```bash
git add src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: expect unified ackermann drive wiring"
```

## 任务 2：新增 Ackermann 参考公式单元测试

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

- [ ] **步骤 2：编写单元测试**

创建 `src/robot_description/test/test_ackermann_kinematics.py`：

```python
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

在文件顶部补充导入：

```python
import pytest
```

- [ ] **步骤 3：运行测试验证通过**

运行：

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py -v
```

预期：PASS，3 个公式基线测试通过。

- [ ] **步骤 4：Commit 参考测试**

```bash
git add src/robot_description/test/ackermann_math_reference.py src/robot_description/test/test_ackermann_kinematics.py
git commit -m "test: add ackermann kinematics reference"
```

## 任务 3：实现统一 Ackermann Gazebo 插件

**文件：**
- 创建：`src/robot_description/src/ackermann_drive_controller.cpp`
- 删除：`src/robot_description/src/steering_controller.cpp`

- [ ] **步骤 1：创建插件源文件骨架**

创建 `src/robot_description/src/ackermann_drive_controller.cpp`，包含这些 include 和类成员：

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  double update_rate_ = 50.0;
  double max_wheel_torque_ = 50.0;
  double target_linear_ = 0.0;
  double target_angular_ = 0.0;
  common::Time last_cmd_time_;
  common::Time last_update_time_;
};

GZ_REGISTER_MODEL_PLUGIN(AckermannDriveControllerPlugin)

}  // namespace gazebo
```

- [ ] **步骤 2：实现参数读取和 Load**

在同一文件中实现 `ReadDouble`、`ReadString` 和 `Load`。关键逻辑：

```cpp
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

`Load` 必须：

```cpp
this->model_ = model;
this->ros_node_ = gazebo_ros::Node::Get(sdf);
this->wheelbase_ = ReadDouble(sdf, "wheelbase", 0.45);
this->track_width_ = ReadDouble(sdf, "track_width", 0.35);
this->wheel_radius_ = ReadDouble(sdf, "wheel_radius", 0.07);
this->max_steering_angle_ = ReadDouble(sdf, "max_steering_angle", 0.5236);
this->cmd_timeout_ = ReadDouble(sdf, "cmd_timeout", 0.5);
this->update_rate_ = ReadDouble(sdf, "update_rate", 50.0);
this->max_wheel_torque_ = ReadDouble(sdf, "max_wheel_torque", 50.0);
this->odom_frame_ = ReadString(sdf, "odom_frame", "odom");
this->base_frame_ = ReadString(sdf, "base_frame", "base_footprint");

const auto left_steering = ReadString(sdf, "left_steering_joint", "left_steering_joint");
const auto right_steering = ReadString(sdf, "right_steering_joint", "right_steering_joint");
const auto rear_left = ReadString(sdf, "rear_left_wheel_joint", "rear_left_joint");
const auto rear_right = ReadString(sdf, "rear_right_wheel_joint", "rear_right_joint");

this->left_steering_joint_ = model->GetJoint(left_steering);
this->right_steering_joint_ = model->GetJoint(right_steering);
this->rear_left_wheel_joint_ = model->GetJoint(rear_left);
this->rear_right_wheel_joint_ = model->GetJoint(rear_right);
```

如果任一关节为空，使用 `RCLCPP_ERROR` 记录四个关节名并 `return`。

- [ ] **步骤 3：实现关节控制初始化和 ROS 接口**

在 `Load` 中继续加入：

```cpp
this->left_steering_joint_->SetParam("fmax", 0, this->max_wheel_torque_);
this->right_steering_joint_->SetParam("fmax", 0, this->max_wheel_torque_);
this->rear_left_wheel_joint_->SetParam("fmax", 0, this->max_wheel_torque_);
this->rear_right_wheel_joint_->SetParam("fmax", 0, this->max_wheel_torque_);

auto joint_controller = model->GetJointController();
joint_controller->AddJoint(this->left_steering_joint_);
joint_controller->AddJoint(this->right_steering_joint_);
joint_controller->SetPositionPID(
  this->left_steering_joint_->GetScopedName(), common::PID(50.0, 0.0, 5.0));
joint_controller->SetPositionPID(
  this->right_steering_joint_->GetScopedName(), common::PID(50.0, 0.0, 5.0));

const auto cmd_topic = ReadString(sdf, "cmd_vel_topic", "cmd_vel");
const auto odom_topic = ReadString(sdf, "odom_topic", "odom");
this->cmd_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
  cmd_topic, 10,
  std::bind(&AckermannDriveControllerPlugin::OnCmdVel, this, std::placeholders::_1));
this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this->ros_node_);
this->last_cmd_time_ = model->GetWorld()->SimTime();
this->last_update_time_ = this->last_cmd_time_;
this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
  std::bind(&AckermannDriveControllerPlugin::OnUpdate, this));
```

- [ ] **步骤 4：实现目标计算**

实现 `OnCmdVel` 和 `ComputeTargets`：

```cpp
void AckermannDriveControllerPlugin::OnCmdVel(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  this->target_linear_ = msg->linear.x;
  this->target_angular_ = msg->angular.z;
  this->last_cmd_time_ = this->model_->GetWorld()->SimTime();
}
```

`ComputeTargets` 使用任务 2 的同一公式。必须满足：

- 左转 `linear=0.5, angular=0.3` 时 `left_steer > right_steer > 0`
- 右转 `linear=0.5, angular=-0.3` 时 `right_steer` 绝对值更大
- `abs(linear) < 0.01` 时后轮速度为 0

- [ ] **步骤 5：实现应用目标和 odom 发布**

`ApplyTargets` 必须设置转向位置和后轮速度：

```cpp
auto joint_controller = this->model_->GetJointController();
joint_controller->SetPositionTarget(
  this->left_steering_joint_->GetScopedName(), targets.left_steer);
joint_controller->SetPositionTarget(
  this->right_steering_joint_->GetScopedName(), targets.right_steer);
joint_controller->SetVelocityTarget(
  this->rear_left_wheel_joint_->GetScopedName(), targets.rear_left_velocity);
joint_controller->SetVelocityTarget(
  this->rear_right_wheel_joint_->GetScopedName(), targets.rear_right_velocity);
```

`PublishOdometry` 必须从 `model_->WorldPose()` 读取 pose，从 `WorldLinearVel()` 和 `WorldAngularVel()` 读取 twist，发布 `nav_msgs::msg::Odometry`，并发送同一 pose 的 `geometry_msgs::msg::TransformStamped`。

- [ ] **步骤 6：实现 OnUpdate**

`OnUpdate` 必须：

```cpp
const auto now = this->model_->GetWorld()->SimTime();
const double period = 1.0 / this->update_rate_;
if ((now - this->last_update_time_).Double() < period) {
  return;
}
this->last_update_time_ = now;

double linear = this->target_linear_;
double angular = this->target_angular_;
if ((now - this->last_cmd_time_).Double() > this->cmd_timeout_) {
  linear = 0.0;
  angular = 0.0;
}

const auto targets = this->ComputeTargets(linear, angular);
this->ApplyTargets(targets);
this->model_->GetJointController()->Update();
this->PublishOdometry(now);
```

- [ ] **步骤 7：删除旧插件源文件**

删除：

```bash
git rm src/robot_description/src/steering_controller.cpp
```

- [ ] **步骤 8：暂不提交，等待构建文件更新**

本任务新增源文件依赖 CMake，先不要 commit；在任务 4 构建通过后一起提交。

## 任务 4：更新构建依赖

**文件：**
- 修改：`src/robot_description/CMakeLists.txt`
- 修改：`src/robot_description/package.xml`

- [ ] **步骤 1：更新 CMake 依赖和目标**

在 `CMakeLists.txt` 中加入：

```cmake
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
```

替换旧 `steering_controller` 目标：

```cmake
add_library(ackermann_drive_controller SHARED src/ackermann_drive_controller.cpp)
ament_target_dependencies(ackermann_drive_controller
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)
```

安装目标改为：

```cmake
install(TARGETS wheel_encoder ackermann_drive_controller
  DESTINATION lib
)
```

导出库改为：

```cmake
ament_export_libraries(wheel_encoder ackermann_drive_controller)
ament_export_dependencies(
  ament_cmake
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2_ros
)
```

- [ ] **步骤 2：确认 package.xml 依赖**

确认 `package.xml` 包含：

```xml
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
```

如果已存在，不重复添加。

- [ ] **步骤 3：运行构建验证**

运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
```

预期：如果 C++ API 名称有误，构建失败并给出具体编译错误；按错误修正 `ackermann_drive_controller.cpp` 后重跑，直到构建通过。

- [ ] **步骤 4：Commit 插件和构建更新**

```bash
git add src/robot_description/src/ackermann_drive_controller.cpp src/robot_description/src/steering_controller.cpp src/robot_description/CMakeLists.txt src/robot_description/package.xml
git commit -m "feat: add unified ackermann drive plugin"
```

如果 `src/robot_description/src/steering_controller.cpp` 已通过 `git rm` 删除，`git add` 会记录删除。

## 任务 5：替换 URDF 插件布线

**文件：**
- 修改：`src/robot_description/urdf/robot_base.urdf.xacro`

- [ ] **步骤 1：移除旧插件块**

从 `robot_base.urdf.xacro` 删除完整的 `Gazebo Drive Plugin` 块：

```xml
<plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/robot</namespace>
    <remapping>cmd_vel:=rear/cmd_vel</remapping>
  </ros>
  <update_rate>50.0</update_rate>
  <left_joint>rear_left_joint</left_joint>
  <right_joint>rear_right_joint</right_joint>
  <wheel_separation>${track_width}</wheel_separation>
  <wheel_diameter>${wheel_diameter}</wheel_diameter>
  <max_wheel_torque>50</max_wheel_torque>
  <max_wheel_acceleration>2.0</max_wheel_acceleration>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>false</publish_wheel_tf>
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```

删除完整的 `Steering Controller` 块：

```xml
<plugin name="steering_controller" filename="libsteering_controller.so">
  <left_steering_joint>left_steering_joint</left_steering_joint>
  <right_steering_joint>right_steering_joint</right_steering_joint>
  <steering_p>50.0</steering_p>
  <steering_i>0.0</steering_i>
  <steering_d>5.0</steering_d>
  <topic>/robot/steering_cmd</topic>
</plugin>
```

- [ ] **步骤 2：加入新统一插件块**

在 joint state publisher 之前加入：

```xml
  <!-- ==================== Ackermann Drive Controller ==================== -->
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

- [ ] **步骤 3：运行静态迁移测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_urdf_uses_unified_ackermann_drive_controller -v
```

预期：PASS。

- [ ] **步骤 4：Commit URDF 更新**

```bash
git add src/robot_description/urdf/robot_base.urdf.xacro
git commit -m "fix: wire robot to unified ackermann drive plugin"
```

## 任务 6：移除 bridge launch 链路

**文件：**
- 修改：`launch/robot_simulation.launch.py`
- 删除：`scripts/cmd_vel_bridge.py`

- [ ] **步骤 1：删除 bridge 节点定义**

从 `launch/robot_simulation.launch.py` 删除完整的 `bridge` 节点定义，包括参数：

```python
bridge = Node(
    package='robot_description',
    executable='cmd_vel_bridge.py',
    name='cmd_vel_bridge',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'wheelbase': 0.45,
        'track_width': 0.35,
        'max_steering_angle': 0.5236,
        'steering_kp': 5.0,
        'steering_kd': 0.5,
        'publish_rate': 50.0,
    }],
)
```

- [ ] **步骤 2：删除 bridge TimerAction**

将返回列表中的：

```python
TimerAction(period=5.0, actions=[bridge]),
```

删除。返回列表保留：

```python
ld_lib_path,
gazebo,
TimerAction(period=2.0, actions=nodes),
TimerAction(period=4.0, actions=[spawn_robot]),
```

- [ ] **步骤 3：删除 bridge 脚本**

运行：

```bash
git rm scripts/cmd_vel_bridge.py
```

- [ ] **步骤 4：运行 launch 静态测试**

运行：

```bash
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py::test_launch_does_not_start_cmd_vel_bridge -v
```

预期：PASS。

- [ ] **步骤 5：Commit launch 更新**

```bash
git add launch/robot_simulation.launch.py scripts/cmd_vel_bridge.py
git commit -m "fix: remove cmd_vel bridge launch path"
```

## 任务 7：补全静态测试覆盖

**文件：**
- 修改：`src/robot_description/test/test_wheel_encoder_integration.py`

- [ ] **步骤 1：更新 CMake 安装测试**

将 `test_wheel_encoder_plugin_is_built_and_installed` 改为同时检查新插件：

```python
def test_plugins_are_built_and_installed():
    cmake = read(PACKAGE_DIR / "CMakeLists.txt")

    assert "add_library(wheel_encoder SHARED src/wheel_encoder.cpp)" in cmake
    assert "add_library(ackermann_drive_controller SHARED src/ackermann_drive_controller.cpp)" in cmake
    assert "ament_target_dependencies(ackermann_drive_controller" in cmake
    assert "geometry_msgs" in cmake
    assert "nav_msgs" in cmake
    assert "tf2_ros" in cmake
    assert "install(TARGETS wheel_encoder ackermann_drive_controller" in cmake
    assert "steering_controller" not in cmake
```

- [ ] **步骤 2：增加旧控制话题移除检查**

在 `test_urdf_uses_unified_ackermann_drive_controller` 中补充：

```python
assert "rear/cmd_vel" not in urdf
assert "steering_cmd" not in urdf
```

- [ ] **步骤 3：运行全部静态测试**

运行：

```bash
python3 -m pytest src/robot_description/test/ -v
```

预期：全部 PASS。

- [ ] **步骤 4：Commit 测试完善**

```bash
git add src/robot_description/test/test_wheel_encoder_integration.py
git commit -m "test: cover unified ackermann plugin wiring"
```

## 任务 8：完整构建与动态验收

**文件：**
- 不修改文件

- [ ] **步骤 1：完整构建**

运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
```

预期：`Summary: 1 package finished`，无编译错误。

- [ ] **步骤 2：运行全部 Python 测试**

运行：

```bash
python3 -m pytest src/robot_description/test/ -v
```

预期：全部 PASS。

- [ ] **步骤 3：启动无 GUI 仿真**

运行：

```bash
mkdir -p log/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 120s ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

预期 launch 日志包含：

```text
SpawnEntity: Successfully spawned entity [ackermann_robot]
AckermannDriveController
```

且不包含：

```text
gazebo_ros_diff_drive
cmd_vel_bridge
SteeringController
```

- [ ] **步骤 4：在仿真运行期间发布持续转弯指令**

另一个终端运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

预期：publisher 连续发布多条消息。

- [ ] **步骤 5：检查 joint states**

在发布期间运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /joint_states --once
```

预期：

- `left_steering_joint` 位置大于 0。
- `right_steering_joint` 位置大于 0。
- `left_steering_joint` 位置大于 `right_steering_joint`。
- `rear_left_joint` 和 `rear_right_joint` velocity 非零。

- [ ] **步骤 6：检查 odom yaw 变化**

运行两次：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic echo /robot/odom --once
```

预期：第二次 odom orientation 的 yaw 与第一次不同，且 position 有位移。

- [ ] **步骤 7：检查旧控制链路消失**

运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 8s ros2 topic list
```

预期列表中没有：

```text
/robot/rear/cmd_vel
/robot/steering_cmd
```

预期列表中包含：

```text
/robot/cmd_vel
/robot/odom
/joint_states
/tf
```

- [ ] **步骤 8：Commit 验收记录**

如果动态验收暴露小修复，修复后重新运行任务 8 的步骤 1-7，再提交：

```bash
git add src/robot_description launch src/robot_description/test
git commit -m "test: verify ackermann drive simulation behavior"
```

如果没有代码改动，不创建空 commit。

## 任务 9：清理旧 ros2_control 配置引用

**文件：**
- 修改或删除：`config/ackermann_robot_controllers.yaml`

- [ ] **步骤 1：确认是否仍被引用**

运行：

```bash
rg -n "ackermann_robot_controllers.yaml|ackermann_steer_controller|controller_manager" .
```

预期：只有该配置文件自身或文档引用。

- [ ] **步骤 2：如果无运行时引用则删除配置**

运行：

```bash
git rm config/ackermann_robot_controllers.yaml
```

如果仍有运行时引用，不删除文件；改为在文件顶部添加注释：

```yaml
# Deprecated: retained for reference only. The simulation now uses
# libackermann_drive_controller.so instead of gazebo_ros2_control.
```

- [ ] **步骤 3：运行静态测试**

运行：

```bash
python3 -m pytest src/robot_description/test/ -v
```

预期：PASS。

- [ ] **步骤 4：Commit 清理**

删除时：

```bash
git add config/ackermann_robot_controllers.yaml
git commit -m "chore: remove obsolete ackermann ros2 control config"
```

仅标记废弃时：

```bash
git add config/ackermann_robot_controllers.yaml
git commit -m "docs: mark old ackermann controller config deprecated"
```

## 最终验证清单

- [ ] `python3 -m pytest src/robot_description/test/ -v` 通过。
- [ ] `source /opt/ros/humble/setup.bash && colcon build --packages-select robot_description` 通过。
- [ ] 无 GUI launch 能成功 spawn `ackermann_robot`。
- [ ] 发布 `/robot/cmd_vel` 后，`/joint_states` 中左右前轮角非零且有 Ackermann 差值。
- [ ] 发布 `/robot/cmd_vel` 后，后轮关节 velocity 非零。
- [ ] `/robot/odom` yaw 随转弯变化。
- [ ] `/robot/rear/cmd_vel` 和 `/robot/steering_cmd` 不再是控制链路话题。
- [ ] 没有重复 `odom -> base_footprint` TF 发布。
