# ROS2 阿克曼四驱机器人仿真系统 - 项目进度

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

## 最新运行教程

### 1. 启动仿真环境

```bash
# 杀死可能残留的进程
killall -9 gzserver gazebo gzclient rviz2 spawn_entity.py 2>/dev/null

# 启动仿真
source /opt/ros/humble/setup.bash
gazebo --verbose /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim/worlds/corridor_tunnel.world -s libgazebo_ros_factory.so -s libgazebo_ros_init.so &
sleep 5

# 处理并发布URDF
xacro /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim/src/robot_description/urdf/robot_base.urdf.xacro > /tmp/robot.urdf

# 启动robot_state_publisher
/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher /tmp/robot.urdf &

# 启动joint_state_publisher
python3 /opt/ros/humble/lib/joint_state_publisher/joint_state_publisher &

# 启动spawn_entity
python3 /opt/ros/humble/lib/gazebo_ros/spawn_entity.py -entity ackermann_robot -topic robot_description -x -5.0 -y 0.0 -z 0.07 &
```

### 2. 键盘控制机器人

```bash
# 发送速度命令 (0.5m/s 前进)
/robot/cmd_vel
source /opt/ros/humble/setup.bash && ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 转向命令 (0.3 rad/s角速度左转)
/robot/cmd_vel
source /opt/ros/humble/setup.bash && ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

### 3. 启动RViz可视化

```bash
source /opt/ros/humble/setup.bash
rviz2 -d /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim/rviz/robot_config.rviz &
```

---

## 机器人关键参数

| 参数 | 数值 |
|------|------|
| **类型** | 阿克曼四驱 (前轮转向) |
| **wheelbase** | 0.45m |
| **track_width** | 0.35m |
| **wheel_radius** | 0.07m |
| **chassis_size** | 0.55m x 0.38m x 0.12m |
| **max_speed** | 5.0 m/s |
| **max_steer** | ±0.5236 rad (±30°) |

---

## 项目完成进度

| 阶段 | 内容 | 状态 | 说明 |
|------|------|------|------|
| **阶段1** | 基础环境搭建 | ✅ 完成 | Gazebo仿真、URDF模型、传感器配置 |
| **阶段2** | 传感器驱动与融合 | ⚠️ 部分 | Gazebo插件正常，EKF融合配置存在 |
| **阶段3** | 3D SLAM建图 | ❌ 未完成 | LIO-SAM2配置存在但未验证 |
| **阶段4** | 导航系统 | ⚠️ 部分 | Nav2配置存在，但控制器需要适配Ackermann |
| **阶段5** | 真实小车移植 | ❌ 未开始 | 需要硬件接口重构 |

### 当前已验证功能 ✅
- Gazebo仿真世界加载 (corridor_tunnel.world)
- 阿克曼四驱机器人模型正确加载
- 轮子直立 (已修复之前的"平躺地面"问题)
- `/robot/odom` 里程计正常发布 (~49Hz)
- `/robot/cmd_vel` 速度控制响应
- `/robot/velodyne_points` 激光雷达点云
- `/gazebo_ros_imu/out` IMU数据

### 待解决问题 ⚠️
1. **Nav2导航未验证** - `navigation.launch.py` 存在但未测试
2. **LIO-SAM2建图未验证** - `lio_sam2.launch.py` 存在但未测试
3. **EKF定位融合** - `localization.yaml` 配置存在但未启动
4. **rviz_config.rviz** - 可能需要更新link列表以匹配当前URDF

### 建议下一步
```bash
# 1. 验证Nav2导航
ros2 launch robot_description navigation.launch.py

# 2. 验证LIO-SAM2建图
ros2 launch robot_description lio_sam2.launch.py
```
