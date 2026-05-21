# Claude Code 交接说明

## 交接时间

- 日期：2026-05-21
- 仓库：`/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim`
- 分支：`main`
- 远端：`origin/main`
- 当前最新提交：`48b0a9b feat: 添加 LIO 地图导出脚本`
- 当前工作区状态：交接时应保持干净，先运行 `git status --short --branch` 确认。

## 当前主线目标

项目主线已经从 LIO-SAM2 切换为：

> FAST-LIO2 兼容 ROS 2 前端 + GPS / 轮速 / IMU / LiDAR 融合定位 + Nav2 保存地图导航。

仿真优先，真实车接口预留。真实车未来主控可能是 RK3588 或地平线 X5，LiDAR 可能是 Unitree L1、Livox Mid-360 或同类 3D LiDAR。

当前不优先处理 `remote` 遥控器，不优先处理棚内靠边作业和动态牛只处理。

## 已完成内容

### 仿真与统一接口

- 外部控制接口保持 `/robot/cmd_vel`。
- 已建立 `/sensing/...` 与 `/control/cmd_vel` 统一接口。
- `/robot/ground_truth/odom` 仅用于仿真评估，不进入定位和导航闭环。
- `launch/robot_simulation.launch.py` 默认启动 `sensing_bridge.launch.py`。
- `scripts/lidar_self_filter.py` 会过滤车身点云，并发布 FAST-LIO 可用点云。

### 参数配置

- `config/vehicle_geometry.yaml`
  - 车辆尺寸、运动学参数、Nav2 footprint、自车点云过滤包围盒。
  - 参数均带单位和坐标系备注。
- `config/sensor_mount.yaml`
  - LiDAR / IMU / GPS 外参、有效距离、FOV。
  - 已加入 `scan_lines` 和 `scan_rate`，用于给仿真点云补齐 FAST-LIO 字段。
- `config/fast_lio.yaml`
  - 已适配 `MIT-SPARK/spark-fast-lio` 的参数结构。
  - `visualization_frame: base`
  - 仿真预检阶段关闭 `gravity_alignment.enable_gravity_alignment`。

### FAST-LIO2 前端

- 默认源码入口：`scripts/install_fast_lio2_source.sh`
- 默认仓库：`https://github.com/MIT-SPARK/spark-fast-lio.git`
- 默认包名：`spark_fast_lio`
- 默认可执行文件：`spark_lio_mapping`
- 第三方源码路径：`src/third_party/spark-fast-lio`
- `src/third_party/` 已加入 `.gitignore`，不要把第三方源码直接提交进主仓库。
- `launch/fast_lio2.launch.py` 已适配：
  - `lidar -> /sensing/lidar/points`
  - `imu -> /sensing/imu/data`
  - `odometry -> /mapping/lio/odom`
  - `cloud_registered -> /mapping/lio/map_points`

### FAST-LIO 仿真点云适配

Gazebo 原始点云只有：

- `x`
- `y`
- `z`
- `intensity`

FAST-LIO Velodyne 预处理需要：

- `ring`
- `time`

因此 `scripts/lidar_self_filter.py` 在输出 `/sensing/lidar/points_filtered` 时补齐：

- `ring(uint16)`：按垂直角估算到 `scan_lines`。
- `time(float32)`：按 `scan_rate` 生成单帧内微秒级偏移。

该适配已经联跑验证。

### FAST-LIO 联跑验证结果

已验证：

- `/sensing/lidar/points` 字段包含 `x/y/z/intensity/ring/time`
- `/sensing/lidar/points` 有有效点云，宽度约 `2582`
- `spark_fast_lio` 能识别 `base_link -> laser_link` 外参
- `/mapping/lio/odom` 可发布
  - `frame_id: map`
  - `child_frame_id: base_link`
- `/mapping/lio/map_points` 可发布，宽度约 `2588`

### 地图导出

已新增：

- `scripts/export_lio_map_to_occupancy.py`

功能：

- 从 `/mapping/lio/map_points` 读取一帧点云。
- 按 `min_z/max_z` 高度窗口投影到 2D。
- 输出 Nav2 可用：
  - `map.yaml`
  - `map.pgm`

已实际验证过一次：

```bash
python3 scripts/export_lio_map_to_occupancy.py \
  --output maps/test_lio_map \
  --timeout-sec 20 \
  --resolution 0.10 \
  --min-z 0.05 \
  --max-z 2.0
```

验证输出过：

- `maps/test_lio_map.yaml`
- `maps/test_lio_map.pgm`
- 样本点数：`239`

这两个测试文件已删除，没有提交。后续正式地图应通过更完整行驶轨迹重新生成。

### Nav2

Nav2 Humble 依赖已安装并通过核心插件预检。

已修正：

- DWB 使用 Humble 的 `FollowPath` 结构。
- Smac Hybrid planner 插件名使用 `nav2_smac_planner/SmacPlannerHybrid`。
- 移除当前 Humble 环境中不存在的 BT 插件。

当前边界：

- Nav2 插件可配置启动。
- 还缺正式保存地图和定位 TF 闭环。
- 当前 Nav2 激活会卡在缺少 `map -> base_link` 或完整 `map -> odom -> base_footprint/base_link`。

## 必跑验证命令

常规静态验证：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
colcon build --packages-select robot_description
```

当前基线结果：

- `46 passed`
- `colcon build --packages-select robot_description` 通过

FAST-LIO 前端预检：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_fast_lio2_precheck.sh
```

预期至少看到：

```text
fast-lio2 front end executable is available
```

Nav2 保存地图预检：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_saved_map_nav2_precheck.sh
```

预期当前边界：

```text
saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

## 推荐下一步

下一步主线不是再改前端接入，而是生成更完整的正式 Nav2 保存地图。

建议顺序：

1. 启动仿真：

   ```bash
   cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
   ```

2. 启动 FAST-LIO：

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description fast_lio2.launch.py
   ```

3. 让车沿牛棚长廊走一段更完整轨迹。可先手动发 `/robot/cmd_vel`，后续再做自动采集脚本。

4. 确认输出：

   ```bash
   ros2 topic echo /mapping/lio/odom --once
   ros2 topic echo /mapping/lio/map_points --once --field width
   ```

5. 导出正式地图：

   ```bash
   python3 scripts/export_lio_map_to_occupancy.py --output maps/lio_map
   ```

6. 检查 `maps/lio_map.yaml` 和 `maps/lio_map.pgm`。

7. 下一阶段再接 `map_server`、定位 TF 链和 Nav2 目标导航。

## 需要特别注意

- 不要把 `src/third_party/spark-fast-lio` 直接提交进主仓库。
- 不要把 `build/`、`install/`、`log/` 提交。
- 生成的测试地图如果只是临时验证，提交前删除。
- 如果生成了正式地图，应明确命名并记录来源轨迹，比如：
  - `maps/barn_corridor_sim_001.yaml`
  - `maps/barn_corridor_sim_001.pgm`
- ROS 运行命令通常需要宿主机权限和完整 ROS 环境。
- 如果启动 Gazebo 报 `11345` 端口占用，先清理残留：

  ```bash
  pkill -f gzserver
  pkill -f gzclient
  pkill -f "ros2 launch robot_description"
  ```

- Claude Code 修改后请保持小步提交，并在交回前更新本文件的「回交记录」。

## 回交给 Codex 时需要提供的信息

请 Claude Code 在交回前补充：

- 最新提交 hash。
- 新增/修改文件列表。
- 已运行的验证命令和结果。
- 是否生成正式地图，地图文件路径是什么。
- Nav2 是否已经能加载地图。
- 当前阻塞点。
- 是否有残留进程需要清理。

## 回交记录

### 2026-05-21：Codex -> Claude Code

- 已 push 到 `origin/main`，最新提交 `48b0a9b`。
- 工作区干净。
- FAST-LIO 仿真输出和地图导出入口已验证。
- 下一步建议：用更完整轨迹导出正式 Nav2 保存地图，然后打通定位 TF 和 Nav2 保存地图导航。
