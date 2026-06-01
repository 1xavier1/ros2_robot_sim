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

### 2026-05-21：Claude Code 阅读理解验证

以下为 Claude Code 阅读完所有交接文件、plan、spec 后的理解，供 Codex 验证是否理解正确。

**主线目标确认：**
- 项目主线已从 LIO-SAM2 切换为 FAST-LIO2（`MIT-SPARK/spark-fast-lio`，包名 `spark_fast_lio`，可执行文件 `spark_lio_mapping`）
- 当前目标：FAST-LIO2 + GPS/轮速/IMU/LiDAR 融合定位 + Nav2 保存地图导航
- 仿真优先，真实车接口预留。真实车主控可能是 RK3588 或地平线 X5，LiDAR 可能是 Unitree L1 或 Livox Mid-360
- 不优先处理 remote 遥控器、棚内靠边作业、动态牛只处理

**已完成的计划：**
- `2026-05-11-ackermann-drive-controller.md` — 统一 Ackermann 驱动插件（已完成）
- `2026-05-19-real-ackermann-drive.md` — 英文版 Ackermann 驱动细化（已完成）

**已废弃的计划：**
- `2026-05-20-lio-sam2-localization-nav2.md` — 顶部已标记 `Superseded by: 2026-05-20-fast-lio2-fusion-nav2.md`

**当前活跃计划：**
- `2026-05-20-fast-lio2-fusion-nav2.md`（13 个任务），大部分任务已完成，包括：
  - 任务 1-12：标记 LIO-SAM2 被取代、车辆几何配置、传感器外参、LiDAR 自车过滤、sensing bridge 改造、运行时验证、FAST-LIO2 配置与 launch、预检查脚本、定位模式管理器、launch 接入、Nav2 对齐、云端命名空间预留
  - 任务 13（完整验证）：部分完成

**已验证通过的内容（来自交接文件）：**
- 46 个 pytest 通过
- `colcon build --packages-select robot_description` 通过
- FAST-LIO 可执行文件可用（`spark_lio_mapping`）
- `/sensing/lidar/points` 包含 `x/y/z/intensity/ring/time`，宽度约 2582
- `/mapping/lio/odom` 可发布（frame_id: map, child_frame_id: base_link）
- `/mapping/lio/map_points` 可发布，宽度约 2588
- 地图导出脚本功能已验证（临时文件已删除）
- Nav2 保存地图预检查通过，但提示 `map->base_link TF is still required for activation`

**当前阻塞点：**
- Nav2 激活需要完整的 `map -> odom -> base_footprint -> base_link` TF 链，目前该链尚未闭环
- 还未生成正式地图（临时测试地图已删除）

**推荐下一步（与交接文件一致）：**
1. 启动仿真 + FAST-LIO
2. 让车沿牛棚长廊走更完整轨迹（先手动发 `/robot/cmd_vel`）
3. 导出正式地图 `maps/barn_corridor_sim_001.{yaml,pgm}`
4. 接入 `map_server` + 定位 TF 链 + Nav2 目标导航

**需注意的约束：**
- `src/third_party/spark-fast-lio` 在 `.gitignore` 中，不提交主仓库
- 不提交 `build/`、`install/`、`log/`
- `/robot/ground_truth/odom` 仅用于仿真评估，不参与定位和导航闭环
- 外部控制接口保持 `/robot/cmd_vel`

**未来计划（未开始）：**
- GPS 融合定位模式管理器（OUTDOOR/TRANSITION/BARN）— 配置和脚本已创建但未联跑验证
- RL 控制器（3 阶段：端到端导航 → 局部规划 → 轨迹跟踪补偿）
- 云端/客户端扩展（仅预留命名空间 `/mission/*`、`/maps/*`、`/fleet/*`、`/config/*`）
- 真实车硬件驱动、棚内靠边作业、动态牛只处理

**当前工作区状态：**
- 分支 `main`，领先 `origin/main` 1 个提交（交接文档本身）
- 工作区干净，无未提交改动
- 最新提交：`3d42ddf docs: 添加 Claude Code 交接说明`

### 2026-05-21：Codex 校验 Claude Code 理解

Claude Code 对主线方向、已完成内容、当前阻塞点和下一步计划的理解基本正确，可以按该理解继续工作。

需要补充和修正以下几点：

1. `localization_mode_manager.py` 与 `localization_modes.yaml` 不是完全未验证；此前已经做过基础运行验证，能发布 `/localization/mode`、`/localization/fusion_weights`、`/localization/gps/gated`。但它还没有接入完整融合后端，也没有形成 Nav2 可用的定位 TF 闭环。

2. 当前 FAST-LIO 输出 `/mapping/lio/odom` 的 `frame_id` 是 `map`，`child_frame_id` 是 `base_link`。这证明 LIO 前端可运行，但 Nav2 主线仍要求项目约定的 `map -> odom -> base_footprint -> base_link`。下一步不要简单把 `map -> base_link` 当成最终定位链，应补一个定位/TF 适配层，至少明确：
   - `map -> odom` 由保存地图定位或融合定位发布；
   - `odom -> base_footprint/base_link` 由里程计或融合结果发布；
   - `base_footprint -> base_link` 保持 URDF 固定变换。

3. `scripts/export_lio_map_to_occupancy.py` 是当前最小可用的 bootstrap 工具：它从一帧 `/mapping/lio/map_points` 投影出 Nav2 `map.yaml + map.pgm`。它还不是最终高质量建图工具，当前没有做 free-space ray tracing、轨迹累计策略、动态物体过滤或地图后处理。正式地图需要车辆跑完整轨迹后再导出，并人工检查地图质量。

4. 如果 Claude Code 在同一工作区继续，不需要重新 clone `spark-fast-lio`；当前 `src/third_party/spark-fast-lio` 和 `install/spark_fast_lio` 已存在。如果换到干净环境，需要先运行：

   ```bash
   ./scripts/install_fast_lio2_source.sh
   ```

5. 交接文件本身是本地交接用，不需要 push 到远端。当前 `main` 领先 `origin/main` 的 1 个提交就是本地交接文档提交 `3d42ddf`。如果 Claude Code 继续在本文件夹工作，可以保留该状态；如果后续要推业务代码，先明确是否要把交接文档提交一并保留在本地还是 squash/拆分。

6. 下一步推荐保持小步推进：
   - 先生成更完整的正式仿真地图；
   - 再写一个地图加载/定位 TF 预检；
   - 最后再让 Nav2 lifecycle 节点真正激活并发送短距离目标。

### 2026-05-22：Claude Code -> Codex（未提交，工作区有修改）

**工作区未提交修改：**

| 文件 | 变更 |
|------|------|
| `rviz/robot_config.rviz` | Fixed Frame 设为 `map`，新增 LIO map 点云、LIO odom 路径和 Odometry 显示 |
| `launch/robot_simulation.launch.py` | 新增 TimerAction 自动启动 `lio_tf_adapter.py`（delay 8s） |
| `maps/barn_corridor_sim_001.yaml` | 最后一次建图产生的 yaml（地图覆盖 90m x 227m，过于稀疏，不推荐使用） |
| `maps/barn_corridor_sim_001.pgm` | 同上，1.9MB |

**已提交（main 分支，领先 origin/main 4 commits）：**

- `6d6837c` — `scripts/accumulate_lio_map.py` 累积建图脚本（体素降采样+密度过滤）
- `fad2d8f` — `scripts/lio_tf_adapter.py` TF 适配节点
- `387a3e3` — `launch/navigation.launch.py` 集成 map_server + TF 适配器
- `bd630c0` — `README_项目进度.md` 更新到 2026-05-22 基线

**本会话遇到并解决的问题：**

1. **TF_OLD_DATA 警告风暴** — 根因是多次 Nav2 启动失败残留了 9 个 orphaned `lifecycle_manager_navigation` 进程 + 2 个重复 `spark_lio_mapping` 实例 + 残存 `lio_tf_adapter`，共 5 个 `/tf` publisher 互冲。`pkill -9` 清理全部孤儿进程 + `ros2 daemon stop/start` 后恢复正常。

2. **FAST-LIO 启动时找不到 `base_link` 帧** — spark_lio_mapping 查 `laser_link -> base_link` 时报 `Invalid frame ID "base_link"`。原因是 ros2 daemon 重启导致 TF 缓存清空，robot_state_publisher 未及时发布 `/tf_static`。等仿真稳定运行后重启 FAST-LIO 即可获取到外参。

3. **`ros2 topic echo --once` 偶尔挂起** — DDS 通信因 daemon 频繁重启出现丢包（`A message was lost!!!`），不影响实际建图，但排查时干扰判断。

**当前仍未解决的问题：**

1. **RViz 显示 `map` 帧时 RobotModel 断开** — RViz 配置文件已将 Fixed Frame 设为 `map`，并配置了 `lio_tf_adapter.py` 自动启动（在 `robot_simulation.launch.py` TimerAction 8s 处）。**理论流程**：仿真启动 → sensing bridge → lio_tf_adapter 订阅 FAST-LIO odom 和 wheel odom 发布 `map -> odom` TF → RViz 显示完整 TF 树 `map -> odom -> base_footprint -> base_link -> laser_link`。但**没有在真实运行中验证过**，因为 lio_tf_adapter 依赖 FAST-LIO 先运行，而 FAST-LIO 是独立 launch 的（不在 start.sh 中）。建议：要么单独启 fast_lio2 后等 odom 有数据再启动 lio_tf_adapter；要么将 fast_lio2 也纳入 robot_simulation.launch.py。

2. **累积建图覆盖过大** — 最后一次建图是 90m x 227m @ 0.6% 占据率，因为 FAST-LIO 无回环检测，长时间行驶 map frame 漂移严重。**短距离采集（车不动太远）可得到合理地图**（之前的 7.4m x 5.8m @ 6.4% 占据率走廊结构清晰）。

3. **Nav2 lifecycle 未完整 activate** — 受 TF 时间戳对齐 + map frame 漂移双重影响，未完成端到端导航验证。

**推荐 Codex 接手后的步骤：**

1. 先验证当前未提交的 rviz 配置是否能在实际运行中正确显示（需先启 fast_lio2，再等 odom 有数据后验证 `map -> odom` TF 链）
2. 用遥控器短距离驱动机器人，`accumulate_lio_map.py` 采集小范围地图
3. 依次启动 TF adapter + Nav2 map_server，解决 costmap TF 时间戳问题
4. 发送导航目标做端到端验证

### 2026-05-25：Codex 接手处理记录

**当前提交与工作区：**

- 最新提交：`bd630c0 docs: 更新项目进度到 2026-05-22 基线`
- 分支：`main`，领先 `origin/main` 5 个提交
- 工作区仍有未提交修改，包含 Claude Code 交接时留下的地图/RViz 改动，以及本次 Codex 修复

**本次新增/修改：**

| 文件 | 变更 |
|------|------|
| `scripts/lio_tf_adapter.py` | `map -> odom` TF 的 stamp 改为当前 ROS 时间，避免复用 FAST-LIO odom 时间导致 Nav2 costmap 查旧 TF |
| `launch/robot_simulation.launch.py` | 移除自动启动 `lio_tf_adapter.py` 的 TimerAction，避免与 `navigation.launch.py` 中的 TF adapter 双发布 |
| `src/robot_description/test/test_wheel_encoder_integration.py` | 新增回归测试，约束 TF stamp 策略和 TF adapter launch 归属 |
| `docs/CLAUDE_CODE_HANDOFF.md` | 更新本次处理记录 |

**重要结论：**

- `lio_tf_adapter.py` 应由 `navigation.launch.py` 拥有；仿真 launch 只负责 Gazebo、robot_state_publisher、RViz 和 sensing bridge。否则全栈启动时会出现重复 `map -> odom` publisher，复现此前的 TF 冲突风险。
- `map -> odom` 发布时使用当前 ROS 时间更符合 Nav2 对实时定位 TF 的使用方式；位姿值仍由最新 FAST-LIO odom 和 wheel odom 组合得到。
- 当前 `maps/barn_corridor_sim_001.{yaml,pgm}` 仍是 90.4m x 215.4m 的稀疏地图版本，不推荐作为最终导航地图。需要重新短距离采集高质量地图，或恢复/另存较小范围清晰版本。

**已运行验证：**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 48 passed

colcon build --packages-select robot_description
# 1 package finished

source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_fast_lio2_precheck.sh
# fast-lio2 front end executable is available

source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_saved_map_nav2_precheck.sh
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**Nav2 状态：**

- `map_server` 能加载当前 `barn_corridor_sim_001` 地图。
- Nav2 核心插件、planner、controller、BT navigator 可配置。
- 因预检没有同时运行仿真、FAST-LIO 和 wheel odom，Nav2 激活仍停在缺少 `map -> base_link` / `map -> odom -> base_footprint -> base_link` TF，尚未完成端到端目标导航。

**残留进程：**

- 本次预检结束后清理了 ROS/Nav2 孤儿进程。
- 最后检查未发现 `lifecycle_manager_navigation`、Nav2 节点、`lio_tf_adapter.py`、`spark_lio_mapping`、Gazebo、RViz 或 `robot_state_publisher` 残留。

**推荐下一步：**

1. Clean restart：仿真 → FAST-LIO → Nav2（由 Nav2 launch 启动 `lio_tf_adapter.py`）。
2. 等 `/mapping/lio/odom` 和 `/robot/odom` 都有数据后，验证 `ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true`。
3. 若 TF 链稳定，再发送短距离 Nav2 目标。
4. 重新短距离采集 `maps/barn_corridor_sim_002.{yaml,pgm}`，不要继续沿用当前 90m x 215m 稀疏地图作为正式成果。

### 2026-05-26：Codex 建图稳定性调整

**本次处理：**

- LiDAR 外参恢复为原始安装角 `rpy: [0.0, 0.524, 0.0]`，避免为了临时建图效果篡改真实传感器安装关系。
- 保留仿真 LiDAR 高密度设置：水平 `720` 采样、垂直 `32` 线，FAST-LIO `scan_line: 32`，用于减少 `No Effective Points!`。
- 仿真默认 spawn 位置改为隧道入口附近 `x=0.8, y=0.0, z=0.07`。
- 新增 `scripts/lio_wheel_fusion.py`，订阅 `/mapping/lio/odom` 和 `/robot/odom`，发布 `/localization/fused_odom`。当前融合策略为 FAST-LIO pose + wheel odom twist，不包含 GPS 或回环优化。
- `launch/localization.launch.py` 已启动 `lio_wheel_fusion.py`。

**已验证：**

```bash
python3 -m py_compile scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py scripts/localization_mode_manager.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 51 passed

colcon build --packages-select robot_description
# 1 package finished

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables robot_description | grep lio_wheel_fusion.py
# robot_description lio_wheel_fusion.py
```

**仍未完成：**

- `/localization/fused_odom` 还没有反向约束 FAST-LIO 建图前端，只是下游定位输出。
- GPS 只通过 `localization_mode_manager.py` 做质量门控和模式权重提示，还没有进入 fused odom。
- 没有回环检测或 pose graph，全长隧道快速建图仍可能累计漂移。

### 2026-05-26：Codex GPS 门控融合小步接入

**本次处理：**

- `scripts/lio_wheel_fusion.py` 现在订阅 `/localization/gps/gated`。
- 新增 GPS 到局部 ENU 的简化转换：第一帧 gated GPS 为 GPS 原点，第一帧 FAST-LIO pose 为局部 map 原点。
- 新增 `gps_blend_weight` 参数，默认 `0.05`，只对 fused odom 的 `x/y` 做低频小权重修正。
- 新增 `/localization/fusion_status`，输出 `wheel=fresh/stale` 和 `gps=gated/none`，便于运行时确认融合输入是否生效。

**已验证：**

```bash
python3 -m py_compile scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py scripts/localization_mode_manager.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 51 passed

colcon build --packages-select robot_description
# 1 package finished

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables robot_description | grep lio_wheel_fusion.py
# robot_description lio_wheel_fusion.py
```

**当前边界：**

- GPS 已进入 `/localization/fused_odom` 的下游融合输出。
- GPS 仍未反向约束 FAST-LIO 建图前端。
- 仍没有回环检测、pose graph 或全局地图优化。

### 2026-05-26：Codex 将 fused odom 接入 Nav2 TF 链

**本次处理：**

- `scripts/lio_tf_adapter.py` 默认改为订阅 `/localization/fused_odom`，再与 `/robot/odom` 组合发布 `map -> odom`。
- `launch/navigation.launch.py` 现在会启动 `lio_wheel_fusion.py`，然后启动 `lio_tf_adapter.py`，让 Nav2 TF 链消费融合定位输出。

**已验证：**

```bash
python3 -m py_compile scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py scripts/localization_mode_manager.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 51 passed

colcon build --packages-select robot_description
# 1 package finished

source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_saved_map_nav2_precheck.sh
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**说明：**

- 预检没有同时运行仿真、FAST-LIO 和 `/robot/odom`，所以仍然停在缺少 `map -> base_link` TF，这是预期边界。
- 全栈运行时顺序应为：仿真 → FAST-LIO → Nav2。Nav2 launch 会启动 fused localization 和 TF adapter。

### 2026-05-26：Codex 将 GPS gate 并入 Nav2 启动链

**本次处理：**

- `launch/navigation.launch.py` 现在同时启动：
  - `localization_mode_manager.py`
  - `lio_wheel_fusion.py`
  - `lio_tf_adapter.py`
- 这样只启动 Nav2 时，`/localization/gps/gated`、`/localization/fused_odom` 和 `map -> odom` TF 链都由同一个 launch 管理。

**已验证：**

```bash
python3 -m py_compile scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py scripts/localization_mode_manager.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 51 passed

colcon build --packages-select robot_description
# 1 package finished

source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_saved_map_nav2_precheck.sh
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**残留进程：**

- 本次 Nav2 预检后已清理相关 ROS 进程。
- 最后检查未发现 Nav2、map_server、localization_mode_manager、lio_wheel_fusion 或 lio_tf_adapter 残留。

### 2026-05-26：Codex 新增一键全栈 SLAM + Nav2 launch

**本次处理：**

- 新增 `launch/slam_navigation.launch.py`，按顺序启动：
  1. `robot_simulation.launch.py`
  2. `fast_lio2.launch.py`（延迟 8s）
  3. `navigation.launch.py`（延迟 14s）
- `robot_simulation.launch.py` 新增并透传 `gui` 参数到 Gazebo，`gui:=false` 现在可以真正只启动 `gzserver`。
- `slam_navigation.launch.py` 支持 `rviz:=true/false` 和 `gui:=true/false`。

**推荐运行：**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py gui:=true rviz:=true
```

无 GUI 验证：

```bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py gui:=false rviz:=false
```

**已验证：**

```bash
python3 -m py_compile launch/slam_navigation.launch.py launch/robot_simulation.launch.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 52 passed

colcon build --packages-select robot_description
# 1 package finished

timeout 16s ros2 launch robot_description slam_navigation.launch.py rviz:=false gui:=false
# 启动到 gzserver、robot_state_publisher、spawn_entity、sensing bridge、spark_lio_mapping、Nav2、localization_mode_manager、lio_wheel_fusion、lio_tf_adapter。
# gui:=false 未启动 gzclient。
```

**残留进程：**

- 短时全栈验证后已清理 Gazebo/ROS 进程。
- 最后检查未发现相关残留。

### 2026-05-26：Codex 修复 Nav2 地图 origin 越界问题

**问题现象：**

- Nav2 报 `Robot is out of bounds of the costmap!`。
- 默认地图 `maps/barn_corridor_sim_001.yaml` 的 `origin[1]` 为 `104.280446`，但地图尺寸为 `904 x 2154 @ 0.1m/cell`，导致机器人初始位置附近的 `y=0` 不在 Nav2 认为的地图范围内。

**本次处理：**

- `scripts/accumulate_lio_map.py` 导出 PGM 时改为 `np.flipud(grid)`，YAML `origin` 固定写入左下角 `min_x, min_y`，符合 Nav2 map_server 约定。
- `maps/barn_corridor_sim_001.yaml` 的 origin 临时修正为 `[-46.881957, -111.119554, 0.000000]`，避免默认地图启动时直接越界。
- `launch/navigation.launch.py` 新增 `map:=...` 参数，`map_server` 通过该参数加载地图。
- `launch/slam_navigation.launch.py` 同样新增并向 Nav2 透传 `map:=...`，后续新建地图可显式指定，避免继续加载旧图。

**推荐重新生成地图后运行：**

```bash
python3 scripts/accumulate_lio_map.py \
  --output maps/barn_corridor_sim_002 \
  --duration-sec 60 \
  --resolution 0.10 \
  --voxel-size 0.15 \
  --min-hits 3

ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py \
  gui:=true rviz:=true map:=$PWD/maps/barn_corridor_sim_002.yaml
```

**已验证：**

```bash
python3 -m py_compile launch/navigation.launch.py launch/slam_navigation.launch.py scripts/accumulate_lio_map.py scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 53 passed

colcon build --packages-select robot_description
# 1 package finished

./scripts/verify_saved_map_nav2_precheck.sh
# map_server 加载 origin[1]: -111.12
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**残留进程：**

- Nav2 预检后已清理相关 ROS 进程。
- 最后检查未发现 Nav2、map_server、localization_mode_manager、lio_wheel_fusion 或 lio_tf_adapter 残留。

### 2026-05-26：Codex 下一阶段全局定位后端骨架

**本次处理：**

- 新增 `scripts/global_localization_backend.py`：
  - 输入 `/localization/fused_odom`、`/robot/odom`、`/localization/gps/gated`、`/sensing/imu/data`。
  - 输出 `/localization/global_odom`、`/localization/backend_status`、`/localization/loop_closure_status`。
  - 默认用 gated GPS 做小权重全局锚定偏移，降低下游导航定位漂移。
  - `enable_loop_closure` 默认 `false`，回环只先输出明确状态，不假装已经有 scan/context matcher 或 pose graph 优化。
- `scripts/lio_tf_adapter.py` 改为消费 `/localization/global_odom`，再组合 `/robot/odom` 发布 `map -> odom`。
- `launch/navigation.launch.py` 和 `launch/localization.launch.py` 都启动 `global_localization_backend.py`。

**当前传感器进入链路的位置：**

- 建图前端：FAST-LIO2 使用 LiDAR + IMU。
- 下游融合：`lio_wheel_fusion.py` 使用 FAST-LIO pose + wheel twist + gated GPS 小权重修正。
- 全局后端：`global_localization_backend.py` 使用 fused odom + gated GPS anchor，并监控 wheel/IMU freshness。
- 回环：尚未接入真实回环优化，当前为显式 disabled/status-only。

**运行时可检查：**

```bash
ros2 topic echo /localization/fusion_status --once
ros2 topic echo /localization/backend_status --once
ros2 topic echo /localization/loop_closure_status --once
ros2 topic echo /localization/global_odom --once
```

**已验证：**

```bash
python3 -m py_compile scripts/global_localization_backend.py scripts/lio_wheel_fusion.py scripts/lio_tf_adapter.py launch/navigation.launch.py launch/localization.launch.py launch/slam_navigation.launch.py
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 54 passed

colcon build --packages-select robot_description
# 1 package finished

./scripts/verify_saved_map_nav2_precheck.sh
# global_localization_backend.py 启动
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**残留进程：**

- Nav2 预检后已清理相关 ROS 进程。
- 最后检查未发现 Nav2、map_server、localization_mode_manager、lio_wheel_fusion、global_localization_backend 或 lio_tf_adapter 残留。

### 2026-06-01：Codex 继续打通 FAST-LIO + Nav2 运行链路

**当前提交与工作区：**

- 分支：`main`，跟踪 `origin/main`。
- 工作区有未提交修改，本次继续沿用现有脏工作区，不回退已有改动。
- `scripts/install_dependencies.sh` 的可执行位变化是接手前已有状态，本次没有刻意修改。

**本次已处理：**

| 文件 | 变更 |
|------|------|
| `launch/fast_lio2.launch.py` | 新增 `base_link -> laser_link` 静态 TF，保证 `spark_lio_mapping` 启动时可同步检测 LiDAR 外参 |
| `config/navigation.yaml` | 修正 Nav2 Humble BT 插件列表、BT XML 路径、costmap frame；控制器从 DWB 切换为 `RegulatedPurePursuitController`；按 Humble `velocity_smoother` 参数结构改为 `max_velocity/min_velocity/scale_velocities` |
| `launch/navigation.launch.py` | `map:=...` 继续透传到 `map_server`；`navigate_through_poses` 使用本地无 `RemovePassedGoals` 的 BT XML；controller 和 behavior recovery 输出统一进 `/control/cmd_vel`；`velocity_smoother` 使用 `cmd_vel -> /control/cmd_vel` 和 `cmd_vel_smoothed -> /robot/cmd_vel`；并纳入 lifecycle manager |
| `config/navigate_through_poses_w_replanning_and_recovery_no_remove.xml` | 新增 Humble 兼容的 through-poses BT，移除当前环境不支持的 `RemovePassedGoals` |
| `scripts/lio_tf_adapter.py` | `map -> odom` 组合改为 SE2 平面投影，避免把 FAST-LIO 的 z/roll/pitch 注入 Nav2 TF 链 |
| `scripts/accumulate_lio_map.py` | 导出的 PGM 改为 Nav2 期望语义：障碍为 `0`，自由为 `254` |
| `scripts/export_lio_map_to_occupancy.py` | 同步改为 Nav2 PGM 语义，并使用 NumPy 生成栅格 |
| `maps/barn_corridor_sim_001.pgm` | 已把旧反向像素语义转换为 Nav2 语义 |
| `maps/barn_corridor_sim_002.{yaml,pgm}` | 新增一张短距离小地图，但仍只是调试产物，不建议视为最终正式地图 |
| `src/robot_description/test/test_wheel_encoder_integration.py` | 新增/更新 Nav2、BT、PGM、TF adapter、velocity smoother 契约测试 |

**已确认解决的问题：**

1. `spark_fast_lio` 启动时可检测到 `base_link -> laser_link` 外参。
2. Nav2 lifecycle 可启动到 `Managed nodes are active`。
3. `/bt_navigator` lifecycle 可查询到 `active [3]`。
4. `tf2_echo map base_footprint` 能拿到 TF。
5. 旧地图 PGM 黑白语义反了，导致 planner 报 `Starting point in lethal space!`。该问题已通过导出脚本修正和现有地图像素转换解决。
6. DWB 在当前配置下 `controller_server` 曾在 `FollowPath` 后段错误退出；切到 Regulated Pure Pursuit 后不再复现该段错误。
7. `/control/cmd_vel` 到 `/robot/cmd_vel` 链路此前断开，原因是 `velocity_smoother` 话题名和参数名按了错误接口写法，且 lifecycle 未纳入管理。代码已修正。

**最新验证结果：**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 58 passed

colcon build --packages-select robot_description
# 1 package finished
```

运行时曾验证到：

```text
spark_lio_mapping: Extrinsics detected
lifecycle_manager_navigation: Managed nodes are active
ros2 lifecycle get /bt_navigator -> active [3]
tf2_echo map base_footprint -> 有实时变换输出
```

**本轮验证检查点：**

1. lifecycle manager 的 `node_names` 已提前加入 `velocity_smoother`，重启全栈后需要确认日志中出现：

   ```text
   Configuring velocity_smoother
   Activating velocity_smoother
   ```

2. 全栈启动后检查话题链路：

   ```bash
   ros2 topic info /control/cmd_vel --verbose
   ros2 topic info /robot/cmd_vel --verbose
   ```

   预期：

   - `/control/cmd_vel` 有 Nav2 controller 发布者、`velocity_smoother` 订阅者；
   - `/robot/cmd_vel` 有 `velocity_smoother` 发布者、`/robot/ackermann_drive_controller` 订阅者。

3. 发送短距离目标：

   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.6, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
   ```

   用于确认车辆是否实际移动、目标是否完成，或是否仍出现 `Failed to make progress`。

4. 仍不建议把当前 90.4m x 215.4m 的 `barn_corridor_sim_001` 视为最终地图。后续应在命令链路稳定后重新采集更小范围、高质量地图。

**推荐下一步：**

1. 重新构建后 clean restart：

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py gui:=false rviz:=false
   ```

2. 等待 `Managed nodes are active`，确认 `velocity_smoother` 也被配置和激活。
3. 检查 `/control/cmd_vel`、`/robot/cmd_vel` 话题发布订阅关系。
4. 发送短距离 Nav2 目标，观察 `/robot/odom` 是否明显变化。
5. 若目标仍失败，优先看：
   - `velocity_smoother` 是否 active；
   - `/robot/cmd_vel` 是否持续发布非零速度；
   - costmap 是否再次出现 `Robot is out of bounds of the costmap`；
   - 当前地图与机器人真实初始位姿是否匹配。

**继续验证补充：**

已完成上述第 1～3 步，并确认：

```text
lifecycle_manager_navigation: Configuring velocity_smoother
lifecycle_manager_navigation: Activating velocity_smoother
ros2 lifecycle get /velocity_smoother -> active [3]
```

话题链路也已确认：

```text
/control/cmd_vel:
  publisher: /controller_server
  subscription: /velocity_smoother

/robot/cmd_vel:
  publisher: /velocity_smoother
  subscription: /robot/ackermann_drive_controller
```

发送 `map` 坐标下短目标时，`/control/cmd_vel` 和 `/robot/cmd_vel` 都出现非零速度，底盘命令链路已经打通。

新的阻塞点：

- 默认大地图 `barn_corridor_sim_001` 上，向 `x=0.0` 的目标会让 RPP 正常发出速度，但 `map -> base_footprint` 中机器人 x 从约 `-0.4` 漂到 `-1.75`，目标反而远离，最终 `Failed to make progress`。
- 读取 `/mapping/lio/odom` 和 `/localization/global_odom` 后确认，FAST-LIO/global localization 当前在 map 坐标中的前进方向与预期目标方向不一致；`backend_status` 中还有 GPS anchor offset，例如 `offset=(-0.694,-0.015)`，会进一步改变 `global_odom`。
- 换用小地图 `barn_corridor_sim_002` 后，map_server 可加载，Nav2 lifecycle 仍可激活，但运行中定位漂到地图边界附近，planner/controller 报：

  ```text
  Robot is out of bounds of the costmap!
  GridBased plugin failed to plan calculation to (-0.40, 0.00): "Start pose is out of costmap!"
  ```

结论：当前端到端阻塞已经从「Nav2 启动 / 命令链路」转为「保存地图与在线定位坐标系对齐」。下一步不要继续盲发目标，应先固定一个坐标一致性策略：

1. 暂时关闭 GPS offset，验证纯 FAST-LIO + wheel odom 的 `map -> odom` 是否稳定。
2. 用同一次 FAST-LIO map 坐标重新导出地图，并记录导出时机器人初始 pose。
3. 或新增一个 `map_alignment` 参数/节点，将保存地图 origin 与当前 localization map frame 对齐后，再进入 Nav2 目标验证。

补充修正：`behavior_server` 的 recovery 速度输出也已 remap 到 `/control/cmd_vel`，避免 backup/spin 等恢复动作绕过 `velocity_smoother` 或发布到默认 `/cmd_vel`。

**继续推进：坐标对齐入口已贯通**

- `navigation.launch.py` 和 `slam_navigation.launch.py` 新增并透传：
  - `map_align_x:=0.0`
  - `map_align_y:=0.0`
  - `map_align_yaw:=0.0`
  - `gps_anchor_blend_weight:=0.0`
- 这些参数会传给 `global_localization_backend.py`。其中 `map_align_*` 用于把在线 localization map frame 静态对齐到保存地图 frame；`gps_anchor_blend_weight` 用于显式控制 GPS anchor 对全局定位 offset 的影响。
- 推荐下一轮验证命令先保持 GPS anchor 关闭：

  ```bash
  ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py \
    gui:=false rviz:=false gps_anchor_blend_weight:=0.0 \
    map_align_x:=0.0 map_align_y:=0.0 map_align_yaw:=0.0
  ```

- 如果启动后 `map -> base_footprint` 的初始位姿仍不落在保存地图可通行区域内，就不要继续发导航目标；先根据 `/localization/global_odom` 与地图 YAML origin 计算 `map_align_*`，或用同一在线 map frame 重新导出地图。

### 2026-05-26：Codex 新增全局定位运行时验证脚本

**本次处理：**

- 新增 `scripts/verify_global_localization_runtime.sh`，用于在全栈仿真/建图/导航已经运行时做一键健康检查。
- 脚本检查：
  - `/mapping/lio/odom`
  - `/robot/odom`
  - `/localization/fused_odom`
  - `/localization/global_odom`
  - `/localization/fusion_status`
  - `/localization/backend_status`
  - `/localization/loop_closure_status`
  - `map -> base_link` TF
  - `ROS_LOG_DIR` 下是否出现 `Robot is out of bounds of the costmap`

**使用方式：**

先启动全栈：

```bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py \
  gui:=true rviz:=true loop_closure:=false
```

另开终端运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ./scripts/verify_global_localization_runtime.sh
```

**已验证：**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 55 passed

bash -n scripts/verify_global_localization_runtime.sh scripts/verify_saved_map_nav2_precheck.sh

colcon build --packages-select robot_description
# 1 package finished
```

**说明：**

- 本脚本是运行时检查脚本，不会自动启动 Gazebo/Nav2，避免重复启动节点造成 TF 冲突。
- 当前会话没有保持全栈仿真运行，因此未强行执行 topic/TF 检查；需要在 `slam_navigation.launch.py` 已运行时执行。

### 2026-05-26：Codex 增加可控轻量闭环修正

**本次处理：**

- `scripts/global_localization_backend.py` 新增显式启用的轻量闭环修正：
  - `enable_loop_closure` 默认 `false`。
  - `loop_correction_gain` 默认 `0.15`。
  - `max_loop_correction_step` 默认 `0.20m`，避免一次闭环造成 TF 大跳。
  - 当轨迹回到历史 keyframe 附近时，状态从 `loop_closure=candidate_odom_proximity` 进入 `loop_closure=applied_odom_proximity`，通过小步平移修正 `global_offset`。
- `launch/navigation.launch.py` 新增 `loop_closure:=false/true` 参数，并传给 `global_localization_backend.py`。
- `launch/slam_navigation.launch.py` 新增 `loop_closure:=false/true` 参数，并透传给 Nav2。
- `scripts/verify_saved_map_nav2_precheck.sh` 修复 `pipefail + echo | grep -q` 导致的大输出断管问题，fallback `ros2 topic list` 也改为容忍沙箱 DDS 权限失败。

**重要边界：**

- 这仍不是完整 scan-matching pose graph。
- 当前闭环依据是 odom/global 轨迹回到历史 keyframe 附近，适合小范围仿真闭合路线验证。
- 在长直隧道、重复结构、没有明显闭合路线时，不建议打开 `loop_closure:=true`，否则可能产生错误近邻修正。

**运行方式：**

默认保守关闭：

```bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py \
  gui:=true rviz:=true loop_closure:=false
```

小范围闭合路线测试时打开：

```bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description slam_navigation.launch.py \
  gui:=true rviz:=true loop_closure:=true
```

观察状态：

```bash
ros2 topic echo /localization/backend_status --once
ros2 topic echo /localization/loop_closure_status --once
```

**已验证：**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 54 passed

python3 -m py_compile scripts/global_localization_backend.py scripts/lio_tf_adapter.py launch/navigation.launch.py launch/slam_navigation.launch.py launch/localization.launch.py
bash -n scripts/verify_saved_map_nav2_precheck.sh

colcon build --packages-select robot_description
# 1 package finished

./scripts/verify_saved_map_nav2_precheck.sh
# global_localization_backend.py 启动
# saved-map Nav2 plugins are available; map->base_link TF is still required for activation
```

**残留进程：**

- Nav2 预检后已清理相关 ROS 进程。
- 最后检查未发现 Nav2、map_server、localization_mode_manager、lio_wheel_fusion、global_localization_backend 或 lio_tf_adapter 残留。

### 2026-06-01：Claude Code 坐标系对齐修复与端到端验证

**当前提交与工作区：**

- 分支：`main`，领先 `origin/main` 5 个提交（含此前 Codex 未推的 3 个提交）
- 最新提交：`b55c3cc feat: navigation launch 透传 map_align/gps_anchor 参数`
- 工作区：仅本交接文件待提交

**本次新增提交：**

| 提交 | 说明 |
|------|------|
| `a323a95` | Nav2 Humble 适配与命令链路修复（此前 Codex 未提交） |
| `b607286` | 地图 PGM 语义修正与 TF adapter SE2 平面投影（此前 Codex 未提交） |
| `b1f5f23` | Nav2/PGM/TF adapter 契约测试更新（此前 Codex 未提交） |
| `14af44d` | 坐标对齐机制 — GPS 默认偏移关闭 + map_align 参数 + pose sidecar |
| `b55c3cc` | navigation launch 透传 map_align/gps_anchor 参数 |

**核心修改：**

| 文件 | 变更 |
|------|------|
| `scripts/lio_wheel_fusion.py` | `gps_blend_weight` 默认从 0.05 改为 0.0 |
| `scripts/global_localization_backend.py` | `gps_anchor_blend_weight` 默认从 0.02 改为 0.0；新增 `map_align_x/y/yaw` 参数 |
| `scripts/accumulate_lio_map.py` | 导出地图时记录机器人 pose 到 `_pose.json` sidecar |
| `launch/navigation.launch.py` | 透传 `map_align_x/y/yaw`、`gps_anchor_blend_weight` 参数 |
| `launch/slam_navigation.launch.py` | 透传 `map_align_x/y/yaw`、`gps_anchor_blend_weight` 参数 |
| `src/robot_description/test/test_wheel_encoder_integration.py` | 新增坐标对齐参数契约测试 |

**已确认解决的问题：**

1. **GPS 偏移导致坐标系不对齐** — `gps_blend_weight` 和 `gps_anchor_blend_weight` 默认改为 0.0。运行时验证 `offset=(0.000,0.000)`，global_odom 与 FAST-LIO map frame 一致。

2. **同一次会话中坐标系一致性** — LiDAR 链正常（~14000+ filtered points），FAST-LIO 发布 odom（x=-1.64），后端 offset=(0,0)，global_odom 与 map 坐标帧一致。

3. **Nav2 小地图端到端** — `barn_corridor_sim_002`（7m x 5.7m），costmap 70x57，lifecycle 全部 active，目标接受并规划路径，无 out-of-bounds 错误。

**已运行验证：**

```bash
python3 -m pytest src/robot_description/test/test_ackermann_kinematics.py src/robot_description/test/test_wheel_encoder_integration.py -q
# 59 passed

colcon build --packages-select robot_description
# 1 package finished
```

全栈运行时验证：

```text
lifecycle: bt_navigator=active[3], velocity_smoother=active[3]
backend_status: gps_anchor=fresh; offset=(0.000,0.000)
tf map->base_footprint: translation=[0.27, 0.02, 0.07]
costmap: 70x57 @ 0.1m/pix
controller: Received a goal, begin computing control effort
controller: Passing new path to controller
```

**仍需注意：**

- `barn_corridor_sim_001`（90m x 215m）仍是大地图，不建议端到端导航。`barn_corridor_sim_002`（~7m x 6m）是当前调试用小地图。
- GPS 偏移默认关闭利于同会话建图导航；如需 GPS 融合，启动时显式设 `gps_anchor_blend_weight:=0.02`。
- `map_align_x/y/yaw` 参数已预留，用于跨会话手动对齐坐标帧。
- 生成正式导航地图：同一次会话驱动机器人 → `accumulate_lio_map.py` 采集 → 立即用同一声明的地图启动 Nav2。
- 首次运行全栈前务必 `pkill -9 -f gzserver` 清理残留 Gazebo 进程。

**推荐下一步：**

1. 用 `accumulate_lio_map.py` 在同一次会话中采集更完整轨迹，生成正式导航地图。
2. 新地图生成后立即启动 Nav2 做端到端目标导航验证。
3. 若需跨会话使用地图，用 `map_align` 参数手动对齐坐标帧。
4. 长距离导航前在小范围闭合路线上测试 `loop_closure:=true`。
