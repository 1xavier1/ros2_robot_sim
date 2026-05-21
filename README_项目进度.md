# ROS2 阿克曼四驱机器人仿真系统 - 项目进度

## 2026-05-21 当前基线

- 已切换到统一 `libackermann_drive_controller.so`，外部控制接口保持 `/robot/cmd_vel`。
- 已验证 `/robot/odom`、`/robot/ground_truth/odom`、`/tf`、`/robot/imu/data`、`/robot/velodyne_points`、`/robot/wheel_encoder/rear_average` 可发布。
- `/robot/ground_truth/odom` 仅用于仿真评估，不参与定位和导航闭环。
- `scripts/verify_runtime_topics.sh` 可验收关键运行话题。
- 已建立 `/sensing/...` 与 `/control/cmd_vel` 统一接口，仿真与未来真实车接口边界保持分离。
- 已新增 `config/vehicle_geometry.yaml`，统一记录车辆尺寸、运动学、Nav2 footprint 与自车点云过滤包围盒参数。
- 已新增 `config/sensor_mount.yaml`，统一记录 LiDAR、IMU、GPS 的安装位置、方向、有效距离和视场参数。
- 已新增 `scripts/lidar_self_filter.py`，用于过滤落在车身包围盒内或无效范围内的点云。
- `fast_lio2.launch.py` 已作为 FAST-LIO2 / FAST-LIO ROS 2 前端入口；`scripts/install_fast_lio2_source.sh` 默认接入 `MIT-SPARK/spark-fast-lio`。
- `spark_fast_lio` 已完成源码 clone 和 colcon 构建，`spark_lio_mapping` 可执行入口可启动；建图输出仍需在仿真数据联跑中验证。
- 已新增 `localization_mode_manager.py` 与 `localization_modes.yaml`，可根据 GPS 质量发布 OUTDOOR、TRANSITION、BARN 模式和融合权重。
- Nav2 Humble 依赖已安装并完成核心插件预检查：DWB `FollowPath`、Smac Hybrid planner、BT navigator 可配置启动。
- 当前 Nav2 运行边界停在缺少保存地图和定位 TF：仍需补齐 `map -> odom -> base_footprint/base_link` 闭环后才能完成激活与导航目标测试。

下一阶段目标：

1. 启动仿真数据链，验证 `spark_fast_lio` 输出 `/mapping/lio/odom` 与 `/mapping/lio/map_points`。
2. 基于 3D 点云生成或导出 Nav2 可用的 2D 保存地图。
3. 打通保存地图定位链，发布 `map -> odom -> base_footprint -> base_link`。
4. 启动 Nav2 保存地图导航，验证路径规划、控制输出和 `/control/cmd_vel` 到 `/robot/cmd_vel` 的闭环。

## 最新运行教程

### 1. 启动仿真环境

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./start.sh
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
| **阶段2** | 传感器驱动与融合 | ⚠️ 部分 | `/sensing/...` 统一接口、车身点云过滤、定位模式管理已加入；完整融合后端待接入 |
| **阶段3** | 3D SLAM建图 | ⚠️ 部分 | `spark_fast_lio` 已构建且可执行入口可启动；仿真数据下 odom/map_points 输出待验证 |
| **阶段4** | 导航系统 | ⚠️ 部分 | Nav2 Humble核心插件预检通过；保存地图与定位TF闭环待完成 |
| **阶段5** | 真实小车移植 | ❌ 未开始 | 已预留车辆几何、传感器外参、云端/客户端扩展配置边界 |

### 当前已验证功能 ✅
- Gazebo仿真世界加载 (corridor_tunnel.world)
- 阿克曼四驱机器人模型正确加载
- 轮子直立 (已修复之前的"平躺地面"问题)
- `/robot/odom` 里程计正常发布 (~49Hz)
- `/robot/cmd_vel` 速度控制响应
- `/robot/velodyne_points` 激光雷达点云
- `/gazebo_ros_imu/out` IMU数据
- `/sensing/lidar/points_raw`、`/sensing/lidar/points_filtered`、`/sensing/lidar/points` 统一点云链路
- `/sensing/imu/data`、`/sensing/gps/fix`、`/control/cmd_vel` 统一接口约定
- `/localization/mode`、`/localization/fusion_weights`、`/localization/gps/gated` 定位模式管理输出
- `spark_fast_lio` 源码构建完成，`spark_lio_mapping` 可执行入口可启动
- Nav2 DWB、Smac Hybrid、BT navigator 插件可加载配置

### 待解决问题 ⚠️
1. **FAST-LIO2建图未闭环** - `spark_fast_lio` 已接入并可启动，需要在仿真 LiDAR/IMU 数据下验证 `/mapping/lio/odom`、`/mapping/lio/map_points`。
2. **保存地图生成未完成** - 需要从 3D 点云生成 Nav2 可用 2D occupancy map。
3. **定位 TF 闭环未完成** - 需要发布 `map -> odom -> base_footprint/base_link`，当前 Nav2 激活会等待该变换。
4. **Nav2目标导航未验证** - 核心插件预检已通过，但保存地图和定位链未打通前不能发送完整导航目标。
5. **rviz_config.rviz** - 可能需要更新link列表以匹配当前URDF和新 `/sensing`、`/mapping` 话题。

### 建议下一步
```bash
# 1. 验证FAST-LIO2前端入口
./scripts/verify_fast_lio2_precheck.sh

# 2. 验证Nav2保存地图预检
./scripts/verify_saved_map_nav2_precheck.sh

# 3. 后续主线：补齐保存地图和定位TF闭环
```
