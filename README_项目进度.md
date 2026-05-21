# ROS2 阿克曼四驱机器人仿真系统 - 项目进度

## 2026-05-22 当前基线

- 已切换到统一 `libackermann_drive_controller.so`，外部控制接口保持 `/robot/cmd_vel`。
- `/robot/ground_truth/odom` 仅用于仿真评估，不参与定位和导航闭环。
- `scripts/verify_runtime_topics.sh` 可验收关键运行话题。
- 已建立 `/sensing/...` 与 `/control/cmd_vel` 统一接口，仿真与未来真实车接口边界保持分离。
- `config/vehicle_geometry.yaml`、`config/sensor_mount.yaml`、`config/fast_lio.yaml` 参数配置已就绪。
- `scripts/lidar_self_filter.py` 过滤车身点云并补齐 FAST-LIO `ring`/`time` 字段，已联跑验证。
- `spark_fast_lio` 源码构建完成，`spark_lio_mapping` 可执行；`/mapping/lio/odom` (map->base_link) 和 `/mapping/lio/map_points` (frame_id: map) 可发布。
- 新增 `scripts/accumulate_lio_map.py` — 多帧点云累积建图脚本，支持体素降采样和密度过滤，替代旧的单帧导出。
- 已生成正式仿真地图 `maps/barn_corridor_sim_001.{yaml,pgm}` — 7.4m x 5.8m @ 0.1m，走廊墙壁结构清晰可见。
- 新增 `scripts/lio_tf_adapter.py` — 将 FAST-LIO `map->base_link` 分解为 `map->odom`，与 `/robot/odom` (odom->base_footprint) 组合补齐 Nav2 TF 树。
- `launch/navigation.launch.py` 已集成 map_server + TF 适配器 + 生命周期管理。
- `localization_mode_manager.py` 可发布 OUTDOOR/TRANSITION/BARN 模式和融合权重，完整融合后端待接入。
- Nav2 Humble 依赖已安装，DWB `FollowPath`、Smac Hybrid planner、BT navigator 插件已通过预检。
- map_server 成功加载 `barn_corridor_sim_001` 地图，Nav2 lifecycle 节点全部可通过配置阶段。

**当前阻塞点：**

- Nav2 local_costmap TF lookup 时间戳不匹配：map_server 加载地图的时间戳早于 LIO odom 数据开始时间，导致 costmap 请求的历史 TF 不可用（差距 ~87s，`transform_tolerance: 60.0` 不足以覆盖）。
- 推荐解法：clean restart 所有进程（仿真 + FAST-LIO + TF adapter + Nav2 在同一次启动序列内）使时间戳对齐。

下一阶段目标：

1. Clean restart 消除 TF 时间戳偏移，验证 Nav2 lifecycle 节点全部 activate。
2. 发送短距离目标点做端到端保存地图导航验证。
3. 可选：用更完整行驶轨迹生成更大覆盖范围的正式地图。

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
| **阶段3** | 3D SLAM建图 | ✅ 基本完成 | `spark_fast_lio` 已构建并联跑；`accumulate_lio_map.py` 累积建图；`barn_corridor_sim_001` 正式地图已生成 |
| **阶段4** | 导航系统 | ⚠️ 部分 | Nav2 核心插件预检通过、map_server 可加载地图、TF 适配器已接入；TF 时间戳对齐后即可端到端导航验证 |
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
- `/mapping/lio/odom` FAST-LIO2 仿真里程计输出
- `/mapping/lio/map_points` FAST-LIO2 仿真地图点云输出
- `accumulate_lio_map.py` 多帧累积建图（体素降采样+密度过滤），可导出 Nav2 地图
- `maps/barn_corridor_sim_001.{yaml,pgm}` 正式仿真地图 (7.4m x 5.8m, 0.1m)
- `lio_tf_adapter.py` map->odom TF 适配器可正常发布变换
- Nav2 DWB、Smac Hybrid、BT navigator 插件可加载配置
- `map_server` 加载正式地图成功，lifecycle 节点全部通过配置阶段

### 待解决问题 ⚠️
1. **TF 时间戳对齐** - Nav2 costmap 请求 TF 的时间戳早于 LIO odom 数据开始时间（差距 ~87s），`transform_tolerance: 60.0` 不足以覆盖。需 clean restart 所有进程对齐时间戳。
2. **Nav2 lifecycle 激活未完成** - 核心插件已通过配置阶段，但 TF 时间戳问题解决后才能完成 activate 并发送导航目标。
3. **正式地图覆盖范围有限** - 当前 `barn_corridor_sim_001` 覆盖 7.4m x 5.8m，走廊全长 30m。FAST-LIO 仅维护局部窗口地图，更大覆盖需多段采集拼接或开启 PCD 保存功能。
4. **rviz_config.rviz** - 可能需要更新 link 列表以匹配当前 URDF 和新 `/sensing`、`/mapping` 话题。

### 建议下一步
```bash
# 1. 验证基线
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
colcon build --packages-select robot_description

# 2. Clean restart 全栈对齐时间戳
# 按顺序在同一 shell 序列中启动：仿真 -> FAST-LIO -> lio_tf_adapter -> Nav2

# 3. 端到端导航验证
# 启动全栈后发送目标点：ros2 action send_goal /navigate_to_pose ...

# 4. 可选：生成更大覆盖地图
python3 scripts/accumulate_lio_map.py --output maps/barn_corridor_sim_002 --duration-sec 120 --voxel-size 0.15 --min-hits 3
```
