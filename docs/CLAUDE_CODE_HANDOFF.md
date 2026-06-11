# Codex 交接说明

## 交接时间

- 日期：2026-06-11
- 仓库：`/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim`
- 当前主线：仿真闭环验证 FAST-LIO2 / Wheel-LIO / Nav2 / 任务执行，并为后续真车迁移保留接口。
- 配套遥控器仓库：`/home/xavier/Workspace/ClaudeSpace/remote`

## 当前目标

项目当前目标不是单纯建图，而是逐步形成完整的固定任务机器人流程：

1. 在仿真中启动车辆、传感器、FAST-LIO2、Wheel-LIO、Nav2 和任务执行器。
2. 使用遥控器手动行驶，完成建图、点位标注、路线示教、任务编辑。
3. 加载已保存地图后执行固定任务，并能在 Web 和 RViz 中观察车辆、任务路径、Nav2 规划路径。
4. 后续迁移到真车时，GPS 信号好的区域用于校准和全局约束；无 GPS 区域依靠轮速、IMU、LiDAR 维持定位。

## 当前节点状态

当前仿真栈以 `scripts/start_full_stack.sh` 为准。最近一次运行中确认存在以下核心节点：

- 仿真与车体：
  - `/gazebo`
  - `/robot_state_publisher`
  - `/robot/ackermann_drive_controller`
  - `/wheel_encoder_front_left`
  - `/wheel_encoder_front_right`
  - `/wheel_encoder_rear_left`
  - `/wheel_encoder_rear_right`
  - `/wheel_encoder_rear_average`
- 传感器与桥接：
  - `/lidar_self_filter`
  - `/relay_lidar_points_raw`
  - `/relay_lidar_points_filtered`
  - `/relay_imu_data`
  - `/relay_gps_fix`
  - `/relay_wheel_speed`
- FAST-LIO / Wheel-LIO / 全局定位：
  - `/fast_lio2`
  - `/lio_tf_adapter`
  - `/lio_wheel_fusion`
  - `/wheel_lio_fusion`
  - `/global_localization_backend`
  - `/localization_mode_manager`
  - `/localization_mode_supervisor`
- Nav2：
  - `/map_server`
  - `/planner_server`
  - `/controller_server`
  - `/smoother_server`
  - `/behavior_server`
  - `/bt_navigator`
  - `/waypoint_follower`
  - `/velocity_smoother`
- 任务与 Web：
  - `/route_recorder`
  - `/task_executor`
  - `/remote_control_node`

当前速度链路已经确认：

```text
controller_server / behavior_server
  -> /control/cmd_vel
  -> velocity_smoother
  -> /robot/cmd_vel
  -> /robot/ackermann_drive_controller
```

`remote_control_node` 也会发布 `/robot/cmd_vel`，但只在手动摇杆、键盘或急停时发送。自动任务期间如果手动控制，会通过 `/task/command` 发布 `manual_override`，任务执行器应取消当前自动任务。

## 当前架构

### 仿真与传感器接口

- 外部控制接口仍保持 `/robot/cmd_vel`。
- 统一接口使用：
  - `/sensing/lidar/points`
  - `/sensing/imu/data`
  - `/sensing/gps/fix`
  - `/control/cmd_vel`
- 坐标链按项目约定保持：

  ```text
  map -> odom -> base_footprint -> base_link
  ```

- `/robot/ground_truth/odom` 只用于仿真评估，不进入正式定位和导航闭环。
- 轮速、IMU、LiDAR 当前是仿真传感器，接口设计上按真车可替换驱动预留。

### 定位与建图

- FAST-LIO2 兼容 ROS 2 前端输出 LIO 里程计和点云。
- Wheel-LIO 相关脚本把轮速约束作为里程计先验/辅助约束，降低纯 LiDAR 在长廊、重复结构、退化场景中的漂移风险。
- 当前用于 Nav2 的 2D 栅格地图主要通过 `scripts/export_odom_projected_map.py` 从点云和位姿投影生成。
- 旧的 `scripts/export_lio_map_to_occupancy.py` 仍保留，但当前测试主线优先使用 odom projected map。

### Nav2 与任务系统

- `scripts/start_full_stack.sh` 是当前推荐的一键启动脚本，会依次启动：
  - Gazebo 仿真
  - FAST-LIO2
  - Nav2
  - 任务执行相关节点
- `BUILD=auto` 会在关键脚本或配置比 `install/` 更新时自动执行：

  ```bash
  colcon build --packages-select robot_description
  ```

- Nav2 当前使用 Smac Hybrid A*，配置为 Dubins 前进优先模式：
  - `motion_model_for_search: "DUBIN"`
  - `minimum_turning_radius: 0.95`
  - `min_turning_radius: 0.95`
  - `reverse_penalty: 100.0`
  - `smooth_path: false`
  - controller 目标速度 `0.35 m/s`、较大 lookahead，避免贴墙急转。
- 2026-06-11 已用 `scripts/measure_turning_radius.py` 在仿真中实测左右转：
  - 左转拟合半径：`0.862 m`
  - 右转拟合半径：`0.862 m`
  - Nav2 使用 `0.95 m`，即实测值加约 10% 安全余量。
- 示教路线优先策略落点：
  - `config/navigate_through_poses_w_replanning_and_recovery_no_remove.xml`
    使用 `IsPathValid`，路径有效时不周期性重规划，路径失效才重规划。
  - `scripts/task_executor.py`
    负责将示教路线转为 Nav2 goals，并持续发布 `/task/active_path` 作为任务参考路径；当 forward-only 任务遇到 `direction: reverse` 标签时，会按路径切线重算 yaw，把路线转换成前进可执行版本。
- 任务执行器：
  - 订阅 `/task/command`
  - 发布 `/task/status`
  - 发布 `/task/current_goal`
  - 发布 `/task/active_path`
  - 支持 `start_task`、`pause_task`、`resume_task`、`cancel_task`、`manual_override`、`return_home`
  - `/task/status` 现在每秒重复发布最后状态，避免 Web 或命令行错过瞬时 `BLOCKED` / `RUNNING` 状态。
- 若启动任务时还没有 `/localization/global_odom`，任务执行器会进入：

  ```text
  WAITING_FOR_LOCALIZATION
  ```

  收到第一帧定位后自动继续启动任务。

### 2026-06-11 最新运行结论

用户反馈“直接执行 `daily_patrol.route min_route_002` 后车没有动”。根因已经确认：

```text
BLOCKED; reason=route min_route_002 violates motion profile
```

原因是 `maps/task_map.yaml` 中 `min_route_002` 含有多段 `direction: reverse`，但运动策略 `forward_only_safe.allow_reverse=false`。以前 `task_executor` 只发布一次状态，Web 很容易错过这个 `BLOCKED`，因此看起来像“没有任何反应”。

当前修复：

- `task_executor.py` 使用 `executable_goal_poses_for_task()` 获取运行时可执行路径。
- `task_map_core.py` 新增 `forward_executable_route_poses()`：
  - 保留路线点的 x / y。
  - 忽略 `direction: reverse` 标签。
  - 按相邻路径切线重算 yaw。
  - 将旧路线转换为 forward-only 可执行路线。
- 启动 `min_route_002` 时会进入 `RUNNING`，并在状态中带提示：

  ```text
  warning=route=min_route_002 reverse_tags_normalized_for_forward_only
  ```

注意：这是为了让当前最小测试能继续推进，不代表 `min_route_002` 是最终验收路线。最终仍建议重录一条全程前进、离墙更远、没有倒车标签的基准路线。

### Web 遥控器联动

`remote` 项目现在不只是摇杆遥控器，也承担任务示教和任务编辑入口：

- Web 地图支持手动上传 Nav2 地图：
  - `.yaml`
  - `.pgm` / `.png` / `.jpg`
- Teach 页面支持：
  - 标注当前位置为点位
  - 录制路线
  - 实时显示正在录制的路线
  - 点位和路线清单管理
- Task 页面已拆分：
  - 编辑任务草稿
  - 保存任务到任务清单
  - 从任务清单选择任务
  - 预览任务全貌
  - 执行、暂停、继续、取消、回起点
- 手动摇杆介入会发布 `manual_override`，优先取消自动任务，保证“自动驾驶时手动优先”。

## 当前可用启动方式

### 启动完整仿真栈

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
MAP=maps/min_test_map.yaml TASK_MAP=maps/task_map.yaml ./scripts/start_full_stack.sh
```

常用环境变量：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GUI` | `true` | 是否启动 Gazebo GUI |
| `RVIZ` | `true` | 是否启动 RViz |
| `MAP` | `maps/barn_corridor_sim_001.yaml` | Nav2 加载的静态地图 |
| `TASK_MAP` | `maps/task_map.yaml` | 任务、点位、路线数据 |
| `BUILD` | `auto` | 自动检测是否需要重新构建 |

### 启动遥控器

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
./start.sh
```

浏览器访问：

```text
http://localhost:8765
```

如果系统分配了其它端口，以 `start.sh` 输出为准。

## 手动验证流程

### 1. 启动系统

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
MAP=maps/min_test_map.yaml TASK_MAP=maps/task_map.yaml ./scripts/start_full_stack.sh
```

预期结果：

- Gazebo 中车辆生成。
- RViz 中能看到 TF、地图、车辆模型、激光/点云相关显示。
- 日志目录生成在 `log/full_stack/`。

### 2. 打开 Web 遥控器

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
./start.sh
```

预期结果：

- 页面显示手动控制、地图、Teach、Task 等区域。
- 手动摇杆或键盘可以驱动车辆。
- 自动任务执行中手动控制会触发任务取消。

### 3. Web 地图为空时手动上传地图

在 Web 页面地图区域选择：

- `maps/min_test_map.yaml`
- 对应图像 `maps/min_test_map.pgm`

预期结果：

- Web 平面地图显示上传的栅格地图。
- 上传记录写入 `maps/task_map.yaml` 的 `maps.uploaded_map`。

### 4. 设置 Home 点

在 Teach 页面：

1. 手动开到希望作为回起点的位置。
2. 新建或覆盖点位，ID 使用 `home`。
3. 点击“标注当前位置”。
4. 保存点位。

预期结果：

- `maps/task_map.yaml` 中 `waypoints` 存在 `id: home`。
- Task 页点击“回起点”时会使用该点。
- 如果车已经在 home 0.25 m 范围内，状态会显示 `HOME_REACHED`，车辆不会明显运动，这是正常结果。

### 5. 录制路线

在 Teach 页面：

1. 点击开始录制路线。
2. 手动驾驶车辆沿可通行路线前进。
3. 地图上应实时显示正在录制的轨迹。
4. 点击停止并保存路线。

预期结果：

- 路线清单出现新路线。
- 地图上能看到路线预览。
- `maps/task_map.yaml` 中 `recorded_routes` 增加对应路线。

注意：

- 当前默认运动策略是前进优先。正式任务建议录制“尽量全程前进”的路线。
- 现有 `min_route_002` 中间包含 `direction: reverse`；运行时会转换成前进可执行路线，但它不适合作为最终 forward-only 验收基准，后续仍建议重录。

### 6. 编辑并执行任务

在 Task 页面：

1. 切到编辑页。
2. 从点位或路线清单添加任务步骤。
3. 保存草稿为任务。
4. 切到执行页。
5. 从任务清单中选择刚保存的任务。
6. 预览任务全貌。
7. 点击执行。

预期结果：

- Web 显示当前选中的任务、目标点或路线。
- RViz 中可观察 `/task/active_path` 和 Nav2 规划路径。
- `/task/status` 进入 `RUNNING`、`COMPLETED` 或明确的 `BLOCKED` 原因。

## 关键调试命令

启动后检查节点和话题：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list
```

检查定位是否已输出：

```bash
ros2 topic echo /localization/global_odom --once
```

检查任务状态：

```bash
ros2 topic echo /task/status
ros2 topic echo /task/current_goal
ros2 topic echo /task/active_path --once
```

检查 Nav2 是否发出速度：

```bash
ros2 topic echo /control/cmd_vel --once
ros2 topic echo /robot/cmd_vel --once
```

检查当前运行时 Nav2 参数：

```bash
ros2 param get /planner_server GridBased.motion_model_for_search
ros2 param get /planner_server GridBased.minimum_turning_radius
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 param get /controller_server FollowPath.lookahead_dist
```

## 最近已完成的关键修改

- 修复 `start_full_stack.sh` 在 `set -u` 下 source ROS setup 时的 `AMENT_TRACE_SETUP_FILES: unbound variable` 问题。
- `start_full_stack.sh` 增加 `BUILD=auto`，避免源码更新但 `install/` 仍是旧版本。
- `task_executor.py`：
  - 增加 `/task/active_path`。
  - `/task/status` 每秒重发最后状态，避免 Web 错过阻塞原因。
  - 支持暂停、继续、取消、手动介入取消、回起点。
  - 无定位时进入等待状态，收到定位后自动开始任务。
  - 回起点时若已经接近 home，发布 `HOME_REACHED`，不再误以为车辆没响应。
  - 对示教路线做起点适配和稀疏化，减少密集点导致的阿克曼不可达路径。
  - 对包含 `direction: reverse` 的 forward-only 示教路线做运行时 yaw 归一化，避免 `daily_patrol -> min_route_002` 直接 `BLOCKED`。
- `task_map_core.py`：
  - 增加起点适配、距离计算等任务路线辅助逻辑。
  - 增加 forward-only 路线归一化辅助函数。
- `route_recorder.py`：
  - 保存路线时合并已有 `task_map.yaml`，避免覆盖 Web 端创建的点位、任务和上传地图记录。
- `navigation.yaml`：
  - Smac Hybrid 切到 Dubins 前进优先。
  - 转弯半径按仿真实测 `0.862 m` 加安全余量设置为 `0.95 m`。
  - 目标线速度提高到 `0.35 m/s`。
  - 放宽普通目标点朝向容差，减少“看起来到点后又绕一圈”的问题。
  - 保持较大膨胀半径和 controller lookahead。
- `remote`：
  - 地图上传。
  - Teach / Task 页面重构。
  - 任务草稿、任务清单、预览、显式选择后执行。
  - 修复“选中路线”总选错的问题。
  - 去掉点位标签，暂时用 ID 管理。

## 当前已知问题

### 1. 车辆仍可能撞墙或规划出不可执行小弯

原因可能有三类：

- 当前地图局部空间太窄，Nav2 只看栅格通行性，不知道真实车辆控制器是否能按该路径跟踪。
- 任务路线本身包含倒车段或急转段；当前运行时会做 forward-only yaw 归一化，但路线质量仍可能影响跟踪效果。
- 运行时加载的参数可能仍是旧 `install/`，需要确认 `BUILD=auto` 是否实际构建过，或手动 `BUILD=true ./scripts/start_full_stack.sh`。

下一步建议：

- 重新录制一条全程前进、离墙更远的测试路线。
- 实测车辆最小转弯半径后同步修改：
  - `config/navigation.yaml`
  - `maps/task_map.yaml` 中 `motion_profiles.min_turning_radius`
- 增加任务校验：forward-only 模式下对包含 reverse 段的路线给出明显提示，并推荐重录前进路线。

### 2. “回起点”看见路径但车辆不动

如果车辆距离 home 已小于 0.25 m，任务执行器会直接发布：

```text
HOME_REACHED; reason=home_already_within_tolerance
```

这不是故障。若希望明显看到回起点动作，需要先把 `home` 标到更远位置，或者把车开离 home 后再执行。

### 3. 重启后执行旧任务没反应

优先检查：

- Web Task 执行页是否已经显式选中某个任务。
- `/task/status` 是否显示 `WAITING_FOR_LOCALIZATION`。
- `/localization/global_odom` 是否有输出。
- 任务引用的路线是否存在于当前 `TASK_MAP`。
- 任务路线是否包含 reverse 段但当前配置不允许倒车。

### 4. Web 地图和真实车辆位置偏离

常见原因：

- Web 上传地图和 Nav2 实际加载地图不是同一张。
- 地图 origin / resolution 不一致。
- 建图时定位漂移，导致路线保存的坐标与地图错位。
- 车辆启动后 `map -> odom` 还没有稳定。

建议先用同一组文件：

```bash
MAP=maps/min_test_map.yaml TASK_MAP=maps/task_map.yaml ./scripts/start_full_stack.sh
```

并在 Web 中上传同一个 `min_test_map.yaml` 和 `min_test_map.pgm`。

## 已运行验证

最近一次验证结果：

```bash
cd remote
python3 -m unittest test_remote_control.py -v
```

结果：`39 tests OK`

```bash
cd ros2_robot_sim
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q
```

结果：`120 passed`

```bash
cd ros2_robot_sim
python3 -m py_compile scripts/task_executor.py scripts/task_map_core.py scripts/route_recorder.py scripts/measure_turning_radius.py
```

结果：通过

```bash
cd ros2_robot_sim
colcon build --packages-select robot_description
```

结果：通过

构建后已确认 `install/robot_description` 中包含最新：

- `task_executor.py`
- `task_map_core.py`
- `navigation.yaml`

关键安装检查：

- `install/robot_description/share/robot_description/config/navigation.yaml`
  - `desired_linear_vel: 0.35`
  - `regulated_linear_scaling_min_speed: 0.18`
  - `minimum_turning_radius: 0.95`
- `install/robot_description/lib/robot_description/task_executor.py`
  - `executable_goal_poses_for_task`
  - `republish_status`
  - `task command received`
- `install/robot_description/lib/robot_description/task_map_core.py`
  - `reverse_tags_normalized_for_forward_only`

## 推荐下一步

### 短期

1. 启动完整栈并确认运行时参数是最新值。
2. 直接执行 `daily_patrol` 验证 `min_route_002` 是否进入 `RUNNING`，并确认 Web 能持续显示 `/task/status`。
3. 在 Web 中上传和 Nav2 一致的地图。
4. 重新设置 `home`。
5. 录制一条全程前进、离墙更远的路线，作为替代 `min_route_002` 的正式基准路线。
6. 用这条新路线创建任务并执行。
7. 若仍撞墙或明显走大弯，抓取：
   - `/task/status`
   - `/task/active_path`
   - Nav2 global plan
   - `/control/cmd_vel`
   - `/robot/cmd_vel`
   - `log/full_stack/navigation.log`
   - 当前运行参数：

     ```bash
     ros2 param get /controller_server FollowPath.desired_linear_vel
     ros2 param get /controller_server FollowPath.regulated_linear_scaling_min_speed
     ros2 param get /planner_server GridBased.minimum_turning_radius
     ```

### 中期

1. 增加 Web 任务校验：
   - forward-only 路线包含 reverse 标签时提示“已运行时归一化，但建议重录”。
   - 路线点过密或急转时提示重录。
   - 起点距离当前车辆太近或朝向差太大时提示。
2. 增加地图编辑：
   - 手动擦除噪点。
   - 添加临时障碍物。
   - 添加禁行区和限速区。
3. 增加任务执行可视化：
   - 当前目标点高亮。
   - 已完成 / 正在执行 / 待执行路径分色。
   - Web 同步显示 Nav2 规划路径和任务示教路径差异。
4. 增加真车配置抽象：
   - 车辆尺寸。
   - 转弯半径。
   - 是否允许倒车。
   - 传感器外参。
   - 轮速编码器安装方向和比例系数。

### 长期真车迁移

1. 建立车端、远程服务器、平板客户端的网络拓扑：
   - 车端通过 4G / Wi-Fi 接入。
   - 本地平板可作为配置和近场控制入口。
   - 远程服务器用于任务下发、状态查看、日志回传。
2. 把仿真 Web 控制能力迁移为真车控制台：
   - 参数配置。
   - 地图上传和编辑。
   - 点位/路线示教。
   - 任务编辑和执行。
   - 实时定位和路径查看。
3. 增强感知：
   - 作业区域识别。
   - 障碍物识别。
   - 信标或视觉跟踪。
   - 动态障碍处理。
4. 增强定位：
   - GPS 好的地方做全局校准。
   - GPS 差或无 GPS 时依靠 LiDAR / IMU / 轮速。
   - 识别打滑并降低轮速权重。

## 提交注意事项

- 不提交：
  - `build/`
  - `install/`
  - `log/`
  - `.superpowers/`
  - 第三方源码目录 `src/third_party/`
- 可以提交用于复现问题和验证的轻量地图文件，但应避免把临时大文件无限累积。
- 改完脚本后至少运行：

  ```bash
  python3 -m py_compile scripts/task_executor.py scripts/task_map_core.py scripts/route_recorder.py
  python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q
  colcon build --packages-select robot_description
  ```

## 回交记录

### 2026-06-11：Codex

- 完成 Web Teach / Task 交互重构和任务执行器增强。
- 完成 Nav2 阿克曼前进优先参数初步收敛。
- 完成一键启动脚本和自动构建检查。
- 实测仿真车左右最小转弯半径约 `0.862 m`，Nav2 使用 `0.95 m`。
- 将 Nav2 目标速度提高到 `0.35 m/s`，将弯道最低调节速度提高到 `0.18 m/s`。
- 修复 `daily_patrol -> min_route_002` 因 reverse 标签被 forward-only 策略阻塞的问题：运行时会将 reverse 标签路线转换成前进可执行 yaw。
- 当前仍需用户手动录制一条新的 forward-only 基准路线验证最终导航效果。
