# 交接说明（Codex + Claude 协作）

## 交接时间

- 仿真栈主线最后更新：2026-06-11（Codex）
- remote 控制台重写最后更新：2026-06-13（Claude）
- 仓库：`/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim`
- 当前主线：仿真闭环验证 FAST-LIO2 / Wheel-LIO / Nav2 / 任务执行，并为后续真车迁移保留接口。
- 配套遥控器仓库：`/home/xavier/Workspace/ClaudeSpace/remote`

> **本文件分两块来源**：仿真栈（ROS 2 / Nav2 / 任务执行，Codex 维护）与 remote 控制台（Web 前后端，Claude 维护）。
> remote 已于 2026-06-13 从旧单文件 `remote_control.py` 完整重写为 FastAPI 后端 `server/` + Vue 3 前端 `web/`，详见下方
> 「remote 控制台（2026-06-13 重写后）」一节与文末「2026-06-13：Claude」回交记录。ROS 话题/服务契约保持不变。

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
  - `/remote_console`（重写后的 remote 后端 rclpy 节点；旧名为 `/remote_control_node`）

当前速度链路已经确认：

```text
controller_server / behavior_server
  -> /control/cmd_vel
  -> velocity_smoother
  -> /robot/cmd_vel
  -> /robot/ackermann_drive_controller
```

remote 后端节点（`/remote_console`）也会发布 `/robot/cmd_vel`，但只在手动摇杆、键盘或急停时发送。自动任务期间如果手动控制，会通过 `/task/command` 发布 `manual_override`，任务执行器应取消当前自动任务。后端还带 0.5s watchdog：WS 断流超 0.5s 自动发零速。

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

### remote 控制台（2026-06-13 重写后）

`remote` 已从旧单文件 `remote_control.py`（3594 行 rclpy + websockets + 内嵌 HTML）完整重写为前后端分离架构。**ROS 话题/服务契约完全不变**，仿真栈侧无需任何改动（唯一跨仓库改动是 `scripts/export_odom_projected_map.py`，见下）。

**目录与进程模型**

```
remote/
  server/                FastAPI 后端（单进程 :8765）
    config.py            ROBOT_SIM_DIR 路径解析（默认指向兄弟仓库 ros2_robot_sim）
    ros_bridge/          RosGateway 窄接口 + node.py（唯一 import rclpy，spin 在后台线程）
    services/            task_map 读写、地图库、建图编排、限速/急停/watchdog、实时状态枢纽
    api/                 REST 路由 + /ws WebSocket 网关 + 统一错误模型
  web/                   Vue 3 + Vite SPA（生产构建到 web/dist，由 FastAPI 静态伺服）
  tests/                 后端 pytest（95 个，FakeGateway 替身，无需 ROS 即可跑）
  docs/acceptance-checklist.md   全流程人工验收清单（7 条）
```

- uvicorn 主线程跑 asyncio（REST + WS），rclpy 在后台 daemon 线程 spin，经线程安全的 RealtimeHub 交接。
- 后端正常启动不依赖 ROS：栈未起时页面显示空态，依赖 ROS 的操作返回 503。
- 三端架构预留：`ros_bridge` 对 `services` 只暴露窄接口 `RosGateway`（未来可拆边缘/服务/客户端三端，API 契约不变）。

**五个页面（Web 全流程零命令行）**

| 页面 | 能力 |
|------|------|
| 驾驶 | 全屏地图 + 大摇杆/WASD 键盘 + 速度仪表；任务运行中提示手动介入将接管 |
| 建图 | 开始/完成/放弃建图、实时快照预览、采集统计、状态流水线、内嵌摇杆 |
| 地图 | 地图库卡片（缩略图/元数据/当前使用标记）、设为当前、删除、上传、打包下载 |
| 示教 | 标注点位、录制路线（实时轨迹）、点位/路线清单、内嵌摇杆 |
| 任务 | 任务编辑器（点位序列/示教路线）、校验、执行控制台（执行/暂停/继续/取消/回起点）+ 状态流水线 + 时间线 |

- 顶栏常驻：WS 连接状态、定位状态、当前地图名、任务状态徽章、**全局急停（Space 键也触发）**。
- `BLOCKED; reason=...` 等原始状态在前端翻译为人话提示 + 建议动作；reverse 路线告警提示「已自动转换为前进路线，建议重录」。
- 急停最高优先级；手动摇杆介入发 `manual_override`（1s 去抖），保证「自动驾驶时手动优先」。

**建图链路（新增，对应跨仓库改动）**

- 建图页「开始建图」→ 后端以 subprocess 运行 `ros2_robot_sim/scripts/export_odom_projected_map.py`。
- 该脚本新增 `--progress-jsonl`（stdout 输出 `{event:progress, elapsed, poses, clouds, snapshot_version}` 等 JSON 行）、`--snapshot-interval`（周期写预览快照 PGM，原子替换）、SIGINT/SIGTERM 落盘收尾。旧用法（不带新参数）行为不变。
- 「完成并保存」→ SIGINT 通知脚本落盘 → 产物从 `maps/_mapping/` 改名进 `maps/{name}.pgm/.yaml`，并写 `{name}_pose.json` sidecar（当时车辆位姿，供 `navigation.launch.py` 本 session 坐标连续）。
- 「放弃」→ kill 进程并清理工作目录。建图期间只允许一个 job，且不允许切换 active 地图。

**地图管理**

- 切换当前地图：`PUT /api/maps/active` → 后端调 `/map_server/load_map` 服务 → 成功后更新 `task_map.yaml` 的 `maps.nav2_map`；服务不可用 503、地图非法 400。
- 上传/保存统一落 `maps/`（废弃旧 `uploaded_maps/` 子目录）；地图名服务端校验 `[A-Za-z0-9_-]+` 防路径穿越。
- 打包下载：yaml + 图像 + pose sidecar 打包 zip（真车迁移取图用）。

**REST / WebSocket 端点速查**

- 状态：`GET /api/status`
- 地图：`GET /api/maps`、`GET/PUT /api/maps/active`、`GET /api/maps/{name}/image|download`、`POST /api/maps`（multipart name+map_yaml+image）、`DELETE /api/maps/{name}`
- 点位/路线/任务：`GET/PUT/DELETE /api/waypoints[/{id}]`、`GET/DELETE /api/routes[/{id}]`、`GET/POST/PUT/DELETE /api/tasks[/{id}]`、`POST /api/tasks/{id}/validate`
- 执行/示教：`POST /api/execution`（start/pause/resume/cancel/return_home）、`POST /api/teach/mark-waypoint`、`POST /api/teach/recording`
- 建图：`GET /api/mapping/status`、`POST /api/mapping/start|stop|cancel`、`GET /api/mapping/preview`
- WebSocket `/ws`：上行 `cmd_vel`/`estop`；下行 `hello`(快照) + 频道帧 `odom`/`pose`/`nav_path`/`active_path`/`task_status`/`teach_status`/`task_current_goal`/`localization_status`/`mapping_status`
- OpenAPI 文档：`http://localhost:8765/docs`

这些 REST/WS 名称即三端拆分的稳定契约，对应后端 `server/api/*.py`，修改需同步前端 `web/src/lib/api.js`。

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

### 2. 打开 Web 控制台

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
./start.sh
```

`start.sh` 会在 `web/dist` 缺失时自动 `npm install && npm run build`（失败则回落占位页，API 不受影响）。

预期结果：

- 浏览器 `http://localhost:8765` 打开后左侧导航有五页：驾驶 / 建图 / 地图 / 示教 / 任务。
- 顶栏显示 WS 连接、定位状态、当前地图、任务状态徽章、全局急停。
- 驾驶页摇杆或键盘 WASD 可驱动车辆，Space 急停。
- 自动任务执行中手动控制会触发任务取消（手动优先）。

> 新版完整全流程验收（建图→保存→选用→标点→录路线→建任务→执行→回起点→断线恢复）见
> `remote/docs/acceptance-checklist.md` 的 7 条清单，逐条人工勾选。下面 3-6 步是其中关键环节的速记。

### 3. 准备地图（二选一）

- **建图页现采**：建图页「开始建图」→ 用内嵌摇杆开车 → 预览快照与采集统计实时刷新 → 「完成并保存」并命名 → 地图页「设为当前地图」。
- **地图页上传**：地图页「上传地图」选 `maps/min_test_map.yaml` + 对应图像 `maps/min_test_map.pgm` 提交（yaml 与图像都必填）。

预期结果：

- 地图库出现该地图卡片，可「设为当前」。
- 设为当前后写入 `maps/task_map.yaml` 的 `maps.nav2_map` 并触发 `/map_server/load_map`。

### 4. 设置 Home 点

在示教页：

1. 手动开到希望作为回起点的位置。
2. 新建或覆盖点位，ID 使用 `home`。
3. 点击“标注当前位置”。
4. 保存点位。

预期结果：

- `maps/task_map.yaml` 中 `waypoints` 存在 `id: home`。
- Task 页点击“回起点”时会使用该点。
- 如果车已经在 home 0.25 m 范围内，状态会显示 `HOME_REACHED`，车辆不会明显运动，这是正常结果。

### 5. 录制路线

在示教页：

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

在任务页（三栏：任务清单 / 编辑器 + 预览 / 执行控制台）：

1. 「新建任务」，选类型（点位序列或示教路线），组合步骤。
2. 「保存任务」，可「校验」（缺引用报错、reverse 路线给告警）。
3. 在右侧执行控制台选中任务，点「执行」。
4. 用状态流水线 + 时间线观察 `WAITING_FOR_LOCALIZATION → RUNNING → COMPLETED/BLOCKED`。
5. 暂停/继续/取消/回起点各按钮均有反馈。

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

remote 后端（FastAPI，无需 ROS）：

```bash
cd remote
python3 -m pytest tests/ -q
```

结果：`95 passed`（旧 `test_remote_control.py` 已随单文件实现一并删除）

remote 前端（Vue，纯函数 + store 单测）：

```bash
cd remote/web
npx vitest run
npm run build
```

结果：`18 passed`，构建成功

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

1. 增加 Web 任务校验（**部分已完成**，2026-06-13 重写）：
   - 已完成：任务校验端点 `POST /api/tasks/{id}/validate`，缺引用报 error、forward-only 路线含 reverse 段报 warning 并人话化提示「已自动转换为前进路线，建议重录」。
   - 待办：路线点过密或急转时提示重录；起点距离当前车辆太近或朝向差太大时提示。
2. 增加地图编辑（待办）：
   - 手动擦除噪点。
   - 添加临时障碍物。
   - 添加禁行区和限速区。
   - 注：`task_map.yaml` 已预留 `map_overlays`（keepout_zones/speed_zones/temporary_obstacles/map_corrections）字段位，后端 store 会保留，前端编辑 UI 尚未做。
3. 增加任务执行可视化（**部分已完成**，2026-06-13 重写）：
   - 已完成：执行状态流水线 StatusPipeline + 状态变迁时间线；MapCanvas 分层渲染车辆/点位/路线/任务路径（`/task/active_path`）/Nav2 规划路径（`/plan`），可同时叠加观察差异。
   - 待办：当前目标点单独高亮；路径按已完成/执行中/待执行分色（现按 forward/reverse 分色）。
4. 增加真车配置抽象（待办）：
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

### 2026-06-13：Claude（remote 控制台重写）

把 `remote` 从旧单文件 `remote_control.py`（3594 行）完整重写为 FastAPI 后端 `server/` + Vue 3 前端 `web/`，分三个计划用 subagent-driven-development 逐任务实现（每任务规格审查 + 代码质量审查两道关）。**ROS 话题/服务契约不变，仿真栈侧无需改动。**

设计与计划文档（都在 `remote/docs/superpowers/`）：

- 规格：`specs/2026-06-11-remote-console-redesign-design.md`
- 计划 1 后端基础：`plans/2026-06-11-remote-backend-foundation.md`
- 计划 2 建图链路：`plans/2026-06-12-remote-mapping-pipeline.md`
- 计划 3 前端 SPA：`plans/2026-06-12-remote-frontend-spa.md`

完成内容：

- **计划 1（后端基础）**：FastAPI 三层（ros_bridge / services / api）、RosGateway 窄接口（三端拆分线，测试用 FakeGateway 无需 ROS）、REST + `/ws` WebSocket、阶梯限速 + 急停 + 0.5s watchdog + manual_override、地图库（枚举/渲染/上传/删除/切换 load_map）、task_map 校验合并持久化。
- **计划 2（建图链路）**：`scripts/export_odom_projected_map.py` 加 `--progress-jsonl` / `--snapshot-interval` / 信号收尾（这是本次唯一跨仓库改动，旧用法不变）；后端 MappingJobs subprocess 编排 + `/api/mapping`（start/stop/cancel/status/preview）+ `_pose.json` sidecar；地图打包下载。
- **计划 3（前端 SPA）**：Vue 3 + Vite，五页（驾驶/建图/地图/示教/任务），Liquid Glass 深色玻璃视觉，MapCanvas 分层画布 + 摇杆 + 状态流水线/时间线 + 顶栏常驻状态 + 全局急停 + WS 指数退避重连（重连后刷新列表）；删除旧 `remote_control.py` / `test_remote_control.py`；`start.sh` 自动构建 `web/dist`。

验证：后端 `python3 -m pytest tests/` = 95 passed；前端 `cd web && npx vitest run` = 18 passed，`npm run build` 成功；建图链路用真实脚本经后端 start→进度→cancel→清理冒烟通过。

**唯一剩余 = 人工全流程验收**：`remote/docs/acceptance-checklist.md` 的 7 条需启动仿真栈 + 浏览器逐条勾选（E2E 自动化按规格明确排除）。已知缩减项（本轮不做）：平板竖屏抽屉布局、跨 session 旧地图重定位对齐、地图编辑（噪点/障碍/禁行区）UI。

后续接手建议：

- 跑通 7 条人工验收清单，记录每条结果；遇到的 ROS 侧问题（撞墙/走大弯/定位漂移）仍按上文「当前已知问题」与「短期推荐」排查。
- 要扩 API 时改 `server/api/*.py` 并同步前端 `web/src/lib/api.js`（两端契约一一对应）；纯函数（状态解析 `web/src/lib/status.js`、坐标 `transform.js`）改动要补 `web/tests/` vitest。
- 三端架构真正拆分时，沿 `RosGateway` 窄接口切：`ros_bridge` 抽边缘进程、`services/api` 上服务器、`web` 独立部署，REST/WS 契约保持不变。

### 2026-06-15：Claude（物理漂移修复 + 雷达/几何回退）

接手排查「小车空闲自己漂移」，定位为之前「推料车改造」引入的回归并回退：

- **物理漂移根因 = 推料车几何**：`vehicle_geometry.yaml` body height 被 0.12→0.5 抬高了 CoM，配合圆柱轮接触，空闲时车被持续蠕动（验证：强制发零 `cmd_vel` 仍漂、把 `ackermann_drive_controller` 驱动全禁用仍漂 → 确认是接触/几何而非控制器）。已回退：`vehicle_geometry.yaml` 恢复 height `0.12`、删除 `auger`/`deck`；`src/robot_description/urdf/robot_base.urdf.xacro` 删 auger/deck link 与 property；`ackermann_drive_controller.cpp`、`worlds/corridor_tunnel.world` 回退到 HEAD（撤销排查期临时改动）。回退后空闲 20s 位移 < 0.2mm。
- **3D 雷达回退到「水平朝上、无桅杆」**：曾改朝下 FOV 想兼顾低矮障碍，但点云被地面主导 → 地面平面不约束 x/y/yaw → LIO 偏航退化、建图旋转糊团。最终回到原始配置：`config/sensor_mount.yaml` lidar `xyz [0,0,0.25]`（无桅杆）、`rpy [0,0,0]`、`±15°` 对称、`v_samples 32`、360°；`config/fast_lio.yaml` `scan_line 32`、`extrinsic_T [0,0,0.17]`（= lidar−imu）；URDF 删 `lidar_mast_visual` 桅杆。低矮障碍与倒车避障改由 3 个毫米波（前 1 后 2，仅 2D 角度+距离、不出点云）兜底，**建图纯靠 L2**。
- `maps/min_test_map`（2026-06-10 用这套水平朝上雷达所建）回退后应重新兼容，定位时可直接复用。

环境备注：本机 Claude 的 shell 起不动 gazebo（被环境 SIGTERM，exit 144），所有运行期诊断靠读取用户已启动栈的 ROS 话题完成。
