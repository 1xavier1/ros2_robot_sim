# Autonomous Task Navigation Design

## 目标

在当前 ROS 2 仿真工程中，先完成可重复验证的自动任务导航闭环，同时为真实车辆、4G 远程服务器、车载屏、蓝牙/WiFi 配置、平板客户端、后续作业识别和算法优化预留清晰边界。

当前阶段只实现 P0：

```text
遥控示教
  -> 保存任务点/作业线路
  -> 生成 task_map.yaml
  -> 地图平面实时查看
  -> 自动执行示教任务
  -> 失败停车并上报状态
```

P1-P3 进入路线图，但不阻塞 P0 实现。

## 当前系统基线

已有能力：

- Gazebo 仿真车和 `/robot/cmd_vel` 控制入口。
- FAST-LIO2 兼容前端，输出 `/mapping/lio/odom` 和 `/mapping/lio/map_points`。
- wheel-LIO 鲁棒融合，输出 `/localization/wheel_lio_odom` 和 `/localization/wheel_lio_status`。
- `global_localization_backend.py`，输出 `/localization/global_odom`。
- Nav2 launch 和保存地图加载能力。
- `remote/remote_control.py`，提供 WebSocket 浏览器遥控、`/robot/cmd_vel` 发布、`/robot/odom` 反馈和 topic 监控。

生产链路不得使用 `/robot/ground_truth/odom`。它只能出现在诊断、评估和仿真验证工具中。

## 阶段边界

### P0：仿真任务导航闭环

P0 必须能在仿真中完成：

1. 启动仿真、FAST-LIO2、wheel-LIO、Nav2。
2. 使用现有 `remote` 手动遥控车辆。
3. 在当前位置标记任务点。
4. 录制一段作业线路。
5. 保存 `task_map.yaml`。
6. 在地图平面实时查看车辆、朝向、路线、目标点和状态。
7. 自动执行示教路线。
8. 失败时停车，并发布可诊断状态。

P0 不做：

- 真实 4G 云平台。
- 真实平板 App。
- 蓝牙/WiFi 配网。
- 自动作业区识别。
- 视觉或信标硬件接入。
- 复杂倒车恢复或三点掉头。
- 直接修改正式 `.pgm` 地图底图。

### P1：控制平面和本地平板模拟端

基于现有 `remote` 项目扩展，不重做遥控器。

P1 目标：

- 任务下发。
- 参数配置。
- 状态上报。
- 遥控接管。
- 心跳和命令超时。
- 地图 overlay 编辑。

### P2：真实车辆迁移

P2 目标：

- 真实 LiDAR、IMU、GPS/RTK、轮速和底盘控制接入。
- 标定车辆几何、传感器外参和 Nav2 footprint。
- 室外 GPS good 区域校准。
- 入口过渡区域平滑降低 GPS 影响。
- 室内无 GPS 区域依赖 LiDAR、IMU、轮速和 wheel-LIO 连续定位。
- 4G 服务器、车载屏、WiFi 热点和蓝牙配置都接 Vehicle Gateway。

### P3：算法优化和智能作业

P3 目标：

- 点云地图保存、版本管理、多次建图融合和地图重定位。
- 2D 地图分辨率、高度切片、滤波和区域地图切分优化。
- 长走廊导航稳定性、路线走廊约束、狭窄区域限速、动态障碍恢复。
- 作业区、禁行区、饲喂区、入口区、掉头区识别。
- 静态/动态障碍识别。
- UWB、AprilTag、ArUco、反光柱或视觉目标跟踪作为增强观测。

信标和视觉只作为增强观测，不替代基础 wheel-LIO/IMU/LiDAR 定位链路。

## 网络拓扑原则

真实车和仿真车的网络拓扑不需要一致，但控制平面接口必须一致。

推荐拓扑：

```text
车辆端 Vehicle Edge
  定位 / 建图 / Nav2 / 避障 / 急停 / 任务状态机

控制平面 Vehicle Gateway
  参数 / 任务 / 状态 / 遥控 / 心跳 / 权限 / 断线策略

远程交互层
  4G 服务器 / 车载屏 / 平板 / 蓝牙 / WiFi 热点
```

车辆端是权威执行端。远程服务器只下发任务、配置和受限遥控意图，不参与实时运动闭环。

断线策略：

- 已接收任务可以按配置继续、暂停、返航或停车。
- 新远程命令必须等待链路恢复。
- 急停和本地接管优先级高于自动任务。

## Remote Control Extension

现有 `remote/remote_control.py` 是 P0/P1 的本地客户端基础，不重做。

当前已有能力：

- WebSocket 浏览器面板。
- `/robot/cmd_vel` 发布。
- `/robot/odom` 反馈。
- topic 列表和订阅监控。
- `start.sh` / `stop.sh`。
- 局域网访问能力。

P0 在此基础上增量扩展：

```text
Teach 面板
  mark_waypoint
  start_recording
  stop_recording
  save_task_map

Task 面板
  start_task
  pause_task
  resume_task
  cancel_task
  return_home

Status 面板
  localization mode
  task state
  current goal
  wheel-LIO status
  Nav2 status

Map Monitor
  2D map
  vehicle pose
  taught route
  current goal
  overlays
```

`remote` 不直接实现定位、建图、任务执行和 Nav2 逻辑。它只通过 ROS topic/service/action 调用车辆端接口。

## 遥控示教采集

第一版不能要求用户手写准确坐标。任务点和作业线路必须支持遥控示教。

新增 `route_recorder` 概念：

输入：

- `/localization/global_odom`，优先定位源。
- `/localization/wheel_lio_odom`，fallback 定位源。
- `/robot/odom` 或命令反馈，用于判断前进/倒车方向。

命令：

- `mark_waypoint`
- `start_recording`
- `stop_recording`
- `save_task_map`

输出：

- `task_map.yaml`
- 当前录制状态。
- 当前采样点数量。
- 最近保存的 waypoint 或 route id。

采样策略：

- 按最小距离采样，默认 `0.3 m`。
- 按最小 yaw 变化补采样，默认 `0.25 rad`。
- 保存每个采样点的 pose 和 direction。

## task_map.yaml 数据契约

`task_map.yaml` 是站点任务层，不替代 Nav2 地图。

推荐结构：

```yaml
site:
  id: barn_sim_site
  map_frame: map

maps:
  nav2_map: maps/manual_wheel_lio_map_test.yaml
  pointcloud_map: maps/lio_cloud_map_manual.pcd
  source: simulation

motion_profiles:
  - id: forward_only_safe
    vehicle_model: ackermann
    allow_reverse: false
    min_turning_radius: 0.78
    max_forward_speed: 0.35
    max_reverse_speed: 0.0

regions:
  - id: outdoor_yard
    type: outdoor
    polygon: [[0.0, 0.0], [5.0, 0.0], [5.0, 4.0], [0.0, 4.0]]
    localization_mode: OUTDOOR
    gps_policy: prefer_gps
    speed_limit: 0.35

waypoints:
  - id: home
    pose: [0.52, 0.48, 0.01]
    role: docking_home
    source: taught

recorded_routes:
  - id: feed_lane_route
    motion_profile: forward_only_safe
    source: taught
    sample_policy:
      min_distance: 0.3
      min_yaw_change: 0.25
    path:
      - pose: [1.0, 1.0, 0.0]
        direction: forward
      - pose: [2.0, 1.1, 0.0]
        direction: forward

tasks:
  - id: daily_patrol
    type: taught_route
    route: feed_lane_route
    start: home
    finish: home
    failure_policy:
      nav_retry_count: 1
      on_failure: stop_and_wait

map_overlays:
  keepout_zones: []
  speed_zones: []
  temporary_obstacles: []
  map_corrections: []
```

P0 必须使用：

- `maps.nav2_map`
- `motion_profiles[].allow_reverse`
- `regions[].localization_mode`
- `waypoints[].pose`
- `recorded_routes[].path`
- `tasks[].route`
- `tasks[].failure_policy`

P0 可以只预留但不完整实现：

- `pointcloud_map`
- `map_overlays`
- `gps_policy`
- `role`

## 倒车和运动约束

阿克曼车不一定必须倒车，但能否倒车会影响路径规划、掉头、失败恢复和狭窄区域通行。

系统必须支持两种模式：

```text
allow_reverse: false
allow_reverse: true
```

P0 默认：

```text
allow_reverse: false
```

P0 行为：

- Nav2 `allow_reversing` 保持 false。
- 控制器不允许负速度。
- 示教路线按前进可达路线录制。
- 任务失败时停车等待人工处理，不自动倒车恢复。

后续 `allow_reverse: true` 时需要：

- Nav2 规划器允许倒车段。
- 控制器允许受限负速度。
- UI 明确显示倒车状态。
- 倒车速度更低。
- 倒车障碍检查更严格。
- 恢复行为必须可配置。

## 地图实时查看和编辑策略

P0 必须提供地图平面实时查看能力。可以先集成在现有 `remote` 页面中，也可以通过独立本地页面实现，但入口应保持在 `remote` 项目中。

P0 地图查看内容：

- 2D Nav2 地图。
- 车辆当前位置和朝向。
- 已示教路线。
- 当前目标点。
- 当前任务状态。
- 定位模式。
- wheel-LIO 状态。

地图编辑采用 overlay 优先策略。

原则：

- 正式 `.pgm/.yaml` 地图底图默认只读。
- 手动添加的禁行区、临时障碍、限速区、擦除建议保存到 `map_overlays`。
- overlay 可启用、禁用、回滚。
- 审核后才生成新的正式地图版本。

P1/P2 再实现图形化编辑：

- 点击添加禁行区。
- 点击添加临时障碍物。
- 绘制限速区。
- 拖动 waypoint。
- 编辑 recorded route。
- 生成地图修正建议。

## Task Executor

`task_executor` 负责把 `task_map.yaml` 中的任务转换为 Nav2 action。

输入：

- `task_map.yaml`
- 任务命令：start、pause、resume、cancel、return_home。
- 定位状态。
- Nav2 action 反馈。

输出：

- `/task/status`
- `/task/current_goal`
- `/task/active_route`
- `/task/failure_reason`

状态机：

```text
IDLE
TEACHING
READY
RUNNING
PAUSED
RETURNING_HOME
COMPLETED
BLOCKED
CANCELLED
```

P0 执行策略：

- `taught_route` 转为 Nav2 `NavigateThroughPoses` 或逐点 `NavigateToPose`。
- 当前目标失败后按 `nav_retry_count` 重试。
- 重试仍失败则停车并进入 `BLOCKED`。
- 不自动执行复杂恢复动作。

## Localization Mode Supervisor

P0 不重写融合算法，只增加任务/区域层的定位模式监督和可观测性。

模式：

```text
OUTDOOR
TRANSITION
INDOOR
DEGRADED
```

输入：

- 当前位姿。
- `task_map.yaml` 中的 region。
- GPS 质量。
- `/localization/wheel_lio_status`。
- Nav2 和任务状态。

行为：

- OUTDOOR：GPS good 时允许缓慢校准全局 offset。
- TRANSITION：入口区域降低 GPS 影响，不允许 map->odom 大跳变。
- INDOOR：GPS 不参与定位修正，依赖 wheel-LIO、LiDAR、IMU、轮速。
- DEGRADED：定位不可信时限速或停车。

## Vehicle Gateway 最小契约

P0 可先用 ROS topic/service/action 实现，不要求 HTTP 或云端协议。

命令：

```text
teach/mark_waypoint
teach/start_recording
teach/stop_recording
teach/save_task_map
task/start
task/pause
task/resume
task/cancel
task/return_home
safety/estop
manual/takeover
manual/release
```

状态：

```text
vehicle pose
localization mode
wheel-LIO state
task state
current goal
active route
Nav2 state
manual takeover state
network heartbeat state
```

后续远程服务器、车载屏、平板、蓝牙和 WiFi 都接入这层。

## 验收标准

P0 通过条件：

1. 使用 `remote` 遥控车辆完成一段示教。
2. 成功保存 `task_map.yaml`。
3. 地图界面能看到车辆在 2D 地图上运动。
4. 地图界面能看到示教路线和当前目标点。
5. `task_executor` 能自动执行示教路线。
6. `/task/status` 能显示 `RUNNING`、`COMPLETED` 或 `BLOCKED`。
7. `allow_reverse=false` 时不会生成或执行倒车段。
8. 失败时车辆停止，不继续盲目运动。
9. `/robot/ground_truth/odom` 不进入生产任务链路。

建议测试：

- 静态测试 `task_map.yaml` schema。
- route recorder 纯函数测试：采样、方向判断、保存结构。
- task executor 纯函数测试：route 到目标点序列转换。
- remote contract 测试：WebSocket 命令映射到 ROS 命令。
- Nav2 仿真 smoke：示教一条短路线并自动复现。

## 未来路线

### P1

- 基于现有 `remote` 扩展完整本地平板模拟端。
- Vehicle Gateway 状态聚合。
- overlay 图形化编辑。
- 参数版本和任务版本。
- 本地日志和任务回放。

### P2

- 真车传感器接入。
- 车辆和传感器标定流程。
- 真实车低速示教建图。
- 室外 GPS 校准。
- 室内无 GPS 导航。
- 4G、车载屏、WiFi、蓝牙接入 Vehicle Gateway。
- 断线、权限和远程急停策略。

### P3

- 建图优化。
- 导航优化。
- 作业区域识别。
- 障碍物识别。
- 信标和视觉跟踪。
- 多地图、多任务和地图版本管理。
