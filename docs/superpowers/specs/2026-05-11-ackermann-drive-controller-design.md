# Ackermann Drive Controller Design

## 背景

当前阿克曼转向修复方案采用 `diff_drive + steering_controller + cmd_vel_bridge`：

- `diff_drive` 负责后轮驱动和 `/robot/odom`
- `steering_controller` 负责前轮转向角
- `cmd_vel_bridge.py` 将 `/robot/cmd_vel` 拆成后轮速度和转向角

该方案能绕开 ROS 2 Humble 中 `gazebo_ros2_control` 与 `ackermann_steering_controller` 的兼容性风险，但链路被拆成三段，动态验证中暴露出两个关键问题：

- 桥接输出、转向控制和后轮驱动之间缺少单一闭环，容易出现话题存在但目标未生效的情况。
- `/robot/odom` 由 `diff_drive` 发布，但实际转向由另一个插件控制，里程计语义可能与 Gazebo 中的真实车体运动不一致。

本设计用一个统一的 Gazebo Model Plugin 替代三段式方案，让 `/robot/cmd_vel`、关节控制、`/robot/odom` 和 TF 在同一个控制器内闭环。

## 目标

- 保持外部控制接口 `/robot/cmd_vel` 不变，兼容 remote control、键盘控制和 Nav2。
- 用一个自定义 Gazebo Model Plugin 控制前轮转向和后轮驱动。
- 从 Gazebo 模型真实位姿发布 `/robot/odom`，确保仿真里程计与车体实际运动一致。
- 发布唯一的 `odom -> base_footprint` TF，避免重复 TF。
- 保留现有 IMU、GPS、LiDAR、wheel encoder 和 joint state 发布能力。
- 增加静态和动态验收测试，证明车辆实际转向、左右前轮角存在 Ackermann 差值、odom yaw 随转弯变化。

## 非目标

- 不在本次迁移到新 Gazebo / Ignition。
- 不重新启用 `gazebo_ros2_control` 或 `ackermann_steering_controller`。
- 不改变 Nav2、LIO-SAM、remote control 对 `/robot/cmd_vel` 的调用方式。
- 不实现真实车辆硬件驱动接口。

## 架构

新增 `libackermann_drive_controller.so`，由 `src/ackermann_drive_controller.cpp` 构建。

数据流：

```text
/robot/cmd_vel
    |
    v
libackermann_drive_controller.so
    |-- front steering joints: position PID
    |-- rear wheel joints: velocity target
    |-- /robot/odom: Gazebo model pose
    `-- /tf: odom -> base_footprint
```

替换并移除 launch/URDF 中的旧链路：

- 移除 `libgazebo_ros_diff_drive.so`
- 移除 `libsteering_controller.so`
- 不再启动 `cmd_vel_bridge.py`

保留：

- `left_steering_joint` / `right_steering_joint` 为 `revolute`
- `rear_left_joint` / `rear_right_joint` 为后轮驱动关节
- `libgazebo_ros_joint_state_publisher.so`
- `libwheel_encoder.so`
- IMU、GPS、LiDAR 插件

## URDF 插件接口

URDF 中新增统一插件配置：

```xml
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

  <wheelbase>0.45</wheelbase>
  <track_width>0.35</track_width>
  <wheel_radius>0.07</wheel_radius>
  <max_steering_angle>0.5236</max_steering_angle>
  <cmd_timeout>0.5</cmd_timeout>
  <update_rate>50.0</update_rate>
  <steering_p>50.0</steering_p>
  <steering_i>0.0</steering_i>
  <steering_d>5.0</steering_d>
  <max_wheel_torque>50.0</max_wheel_torque>
</plugin>
```

话题在 `/robot` 命名空间下解析，因此插件订阅 `/robot/cmd_vel`，发布 `/robot/odom`。

## 控制逻辑

插件订阅 `geometry_msgs/msg/Twist`：

- `linear.x` 表示车辆前进/后退速度。
- `angular.z` 表示期望 yaw rate。
- 其他 Twist 分量忽略。

当 `abs(linear.x)` 大于阈值时：

1. 计算中心转角：

   ```text
   steer = atan(wheelbase * angular_z / linear_x)
   ```

2. 将中心转角限制到 `[-max_steering_angle, max_steering_angle]`。
3. 根据 Ackermann 几何计算左右前轮角。
4. 根据转弯半径分配左右后轮线速度，外侧后轮速度更大，内侧后轮速度更小。
5. 将后轮线速度转换为关节角速度：

   ```text
   wheel_angular_velocity = wheel_linear_velocity / wheel_radius
   ```

当 `abs(linear.x)` 接近 0 时：

- 不做原地旋转，因为 Ackermann 结构不能真实原地转向。
- 后轮速度设为 0。
- 可以根据 `angular.z` 给前轮一个限幅转向角，便于静止预打角；默认不产生 yaw motion。

命令超时：

- 超过 `cmd_timeout` 未收到新 `/robot/cmd_vel` 时，后轮速度归零，转向角回中。

关节控制：

- 前轮转向关节使用 Gazebo `JointController` 的位置 PID。
- 后轮关节使用速度目标和力矩上限控制。
- 插件加载时如果关键关节缺失，记录错误并不进入控制循环。

## Odom 与 TF

`/robot/odom` 使用 Gazebo 模型真实位姿：

- `header.frame_id = odom`
- `child_frame_id = base_footprint`
- pose 来自模型 world pose
- twist 来自 Gazebo world/body velocity，并按 `nav_msgs/Odometry` 语义发布

插件同时发布 `odom -> base_footprint` TF。

旧 `diff_drive` 插件必须移除，避免重复发布 `/robot/odom` 和 `odom -> base_footprint`。

## 文件变更范围

预计修改：

- `src/robot_description/src/ackermann_drive_controller.cpp`：新增统一控制器插件。
- `src/robot_description/CMakeLists.txt`：构建和安装新插件，移除旧 `steering_controller` 构建目标。
- `src/robot_description/package.xml`：补齐 `nav_msgs`、`tf2_ros` 等插件依赖。
- `src/robot_description/urdf/robot_base.urdf.xacro`：移除旧 drive/steering 插件，加入新插件。
- `launch/robot_simulation.launch.py`：不再启动 `cmd_vel_bridge.py`。
- `src/robot_description/test/test_wheel_encoder_integration.py`：更新静态集成检查。

预计删除或废弃：

- `src/robot_description/src/steering_controller.cpp`
- `scripts/cmd_vel_bridge.py`
- `config/ackermann_robot_controllers.yaml` 可删除或标记为废弃。若删除会影响文档引用，则本次先保留并在后续清理。

## 测试与验收

静态测试：

- URDF 包含 `libackermann_drive_controller.so`。
- URDF 不包含 `libgazebo_ros_diff_drive.so`。
- URDF 不包含 `libsteering_controller.so`。
- `left_steering_joint` 和 `right_steering_joint` 仍为 `revolute`。
- launch 不再启动 `cmd_vel_bridge.py`。
- CMake 构建并安装新插件。

构建测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
```

动态烟测：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros timeout 120s ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

在仿真运行期间持续发布：

```bash
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

验收条件：

- `/joint_states` 中 `left_steering_joint` 和 `right_steering_joint` 非零。
- 左右前轮转向角不同，且方向符合 Ackermann 几何。
- 后轮关节速度非零。
- `/robot/odom` 的 yaw 随时间变化。
- ROS 图中没有 `/robot/rear/cmd_vel` 或 `/robot/steering_cmd` 作为必要控制链路。
- 不存在重复 `odom -> base_footprint` TF 发布。

## 风险与缓解

- Gazebo Classic 已 EOL：本设计不扩展 Gazebo 版本范围，只解决当前 Humble workspace 的仿真控制问题。
- 物理接触和轮胎侧滑会影响真实轨迹：本次 `/robot/odom` 采用 Gazebo 真实位姿，可直接反映仿真实际运动。
- 自定义插件增加维护成本：插件职责集中但边界清晰，输入只有 `/robot/cmd_vel`，输出为关节控制、odom 和 TF，便于动态验收。
- 后续若迁移到 ros2_control：可用本插件的控制公式和测试用例作为行为基线。
