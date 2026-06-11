# ROS2 机器人仿真与固定任务导航

这是 ROS2 Humble 下的牛场/园区车辆仿真项目。当前主线已经从早期 LIO-SAM2 方案切换到 FAST-LIO2、Wheel-LIO、Nav2 和 Web 示教任务闭环。

项目优先保证仿真流程可验证，同时为后续真车迁移保留传感器、车辆参数、网络控制和任务系统接口。

## 当前能力

- Gazebo 四轮车辆仿真。
- 3D LiDAR、IMU、GPS、轮速编码器仿真话题。
- FAST-LIO2 前端接入。
- Wheel-LIO / 轮速辅助定位链路。
- 点云投影生成 Nav2 2D 栅格地图。
- Nav2 加载保存地图导航。
- 阿克曼/前进优先的 Nav2 初始参数。
- 任务地图 `task_map.yaml`：
  - 点位
  - 示教路线
  - 固定任务
  - Home 点
  - 区域和运动策略
- 任务执行器：
  - 执行任务
  - 暂停、继续、取消
  - 手动介入优先
  - 回起点
  - 发布当前任务路径
- 配合 `remote` 项目进行 Web 遥控、地图上传、点位标注、路线录制和任务执行。

## 目录结构

```text
ros2_robot_sim/
├── config/                     # FAST-LIO2、Nav2、传感器、车辆配置
├── docs/                       # 交接文档、验证记录、实施计划
├── launch/                     # 仿真、FAST-LIO2、导航等 launch 文件
├── maps/                       # Nav2 地图、任务地图、验证地图
├── rviz/                       # RViz 配置
├── scripts/                    # 建图、定位融合、任务执行、启动脚本
├── src/robot_description/      # ROS2 package、URDF、测试
├── worlds/                     # Gazebo world
├── start.sh
└── stop.sh
```

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- Nav2 Humble
- Python 3.10+
- `colcon`

常用依赖：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-tf2-ros ros-humble-rviz2
```

## 构建

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
source install/setup.bash
```

## 一键启动完整仿真栈

推荐使用：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
MAP=maps/min_test_map.yaml TASK_MAP=maps/task_map.yaml ./scripts/start_full_stack.sh
```

该脚本会启动：

1. Gazebo 仿真和传感器桥接。
2. FAST-LIO2。
3. Nav2 保存地图导航。
4. 任务执行相关节点。

常用参数：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GUI` | `true` | 是否启动 Gazebo GUI |
| `RVIZ` | `true` | 是否启动 RViz |
| `USE_SIM_TIME` | `true` | 是否使用仿真时间 |
| `MAP` | `maps/barn_corridor_sim_001.yaml` | Nav2 加载地图 |
| `TASK_MAP` | `maps/task_map.yaml` | 点位、路线和任务数据 |
| `BUILD` | `auto` | 检测到安装文件过期时自动构建 |

示例：

```bash
GUI=false RVIZ=true MAP=maps/min_test_map.yaml ./scripts/start_full_stack.sh
BUILD=true MAP=maps/min_test_map.yaml ./scripts/start_full_stack.sh
```

日志写入：

```text
log/full_stack/
```

## 建图

当前推荐脚本是：

```bash
python3 scripts/export_odom_projected_map.py \
  --pose-topic /localization/wheel_lio_odom \
  --output maps/min_test_map \
  --duration-sec 60 \
  --resolution 0.05
```

输出：

```text
maps/min_test_map.yaml
maps/min_test_map.pgm
maps/min_test_map.json
```

说明：

- `.yaml` 和 `.pgm` 用于 Nav2 静态地图。
- `.json` 保存建图统计信息，便于对比不同参数。
- 当前脚本依赖位姿和点云同步质量；如果地图呈放射状错位，优先检查定位话题、点云时间戳、TF 和车辆是否快速打滑。

旧脚本 `scripts/export_lio_map_to_occupancy.py` 仍保留，但当前测试主线优先使用 `export_odom_projected_map.py`。

## 加载地图导航

```bash
MAP=maps/min_test_map.yaml TASK_MAP=maps/task_map.yaml ./scripts/start_full_stack.sh
```

预期结果：

- RViz 中显示静态地图。
- 车辆定位点位于地图附近。
- Nav2 可以在地图上规划路径。
- `/task/active_path` 可显示任务执行器当前要走的任务路径。

如果 RViz 看不到地图：

1. 检查 `MAP` 是否为存在的 `.yaml` 文件。
2. 检查 `log/full_stack/navigation.log` 中 map_server 是否加载成功。
3. 检查 RViz Fixed Frame 是否为 `map`。
4. 检查 Map display 话题是否为 `/map`。

## 任务系统

任务系统使用 YAML 文件保存：

```text
maps/task_map.yaml
```

主要字段：

| 字段 | 说明 |
|------|------|
| `site` | 场地 ID 和 map frame |
| `maps` | Nav2 地图、点云地图、Web 上传地图记录 |
| `motion_profiles` | 车辆运动策略，例如是否允许倒车、最小转弯半径 |
| `regions` | 室内/室外区域、GPS 策略、限速 |
| `waypoints` | 点位，`home` 是回起点使用的点 |
| `recorded_routes` | Web 或脚本录制的示教路线 |
| `tasks` | 可执行任务 |
| `map_overlays` | 预留禁行区、限速区、临时障碍物、地图修正 |

任务命令话题：

```text
/task/command
```

常用命令：

```bash
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'start_task id=daily_patrol'}"
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'pause_task'}"
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'resume_task'}"
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'cancel_task'}"
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'return_home'}"
```

任务状态：

```bash
ros2 topic echo /task/status
ros2 topic echo /task/current_goal
ros2 topic echo /task/active_path
```

## 与 remote 项目配合

启动完整仿真栈后，另开终端：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote
./start.sh
```

Web 页面中可以：

- 手动驾驶车辆。
- 上传地图到 Web 平面视图。
- 标注当前位置为点位。
- 录制路线。
- 编辑任务草稿。
- 保存任务到任务清单。
- 预览并执行任务。
- 暂停、继续、取消任务。
- 回到 `home` 点。

## 坐标系和话题约定

坐标链：

```text
map -> odom -> base_footprint -> base_link
```

关键话题：

| 功能 | 话题 |
|------|------|
| 外部控制输入 | `/robot/cmd_vel` |
| Nav2 控制输出 | `/control/cmd_vel` |
| LiDAR 统一输入 | `/sensing/lidar/points` |
| IMU 统一输入 | `/sensing/imu/data` |
| GPS 统一输入 | `/sensing/gps/fix` |
| Wheel-LIO odom | `/localization/wheel_lio_odom` |
| 全局定位 odom | `/localization/global_odom` |
| 任务命令 | `/task/command` |
| 任务状态 | `/task/status` |
| 当前任务路径 | `/task/active_path` |

## 阿克曼导航配置

当前 Nav2 配置在 `config/navigation.yaml`。

重点参数：

- Smac Hybrid A*。
- Dubins 前进优先搜索。
- 较大的最小转弯半径。
- 较低跟踪速度。
- 较大的 inflation radius。

如果实际车辆仍撞墙：

1. 先确认运行时加载的是最新参数：

   ```bash
   ros2 param get /planner_server GridBased.minimum_turning_radius
   ros2 param get /controller_server FollowPath.lookahead_dist
   ```

2. 重新录制离墙更远、全程前进的路线。
3. 根据真实车辆实测最小转弯半径更新：
   - `config/navigation.yaml`
   - `maps/task_map.yaml` 的 `motion_profiles.min_turning_radius`
4. 如果真车允许倒车，再单独设计 Reeds-Shepp / 倒车任务策略；不要和 forward-only 混用。

## 轮速编码器作用

当前方案中轮速编码器主要用于：

- 在 LiDAR 退化或重复结构场景中提供短时间运动约束。
- 辅助估计车辆前进距离和速度。
- 与 IMU、LiDAR 一起提高无 GPS 区域定位连续性。

局限：

- 车辆打滑时，轮速会高估或低估真实位移。
- 急转、湿滑、泥地、坡地会增加轮速误差。
- 真车需要做轮半径、编码器方向、比例系数、时间戳、左右轮安装误差标定。

后续优化方向：

- 通过 IMU 角速度、LiDAR 匹配残差识别打滑。
- 打滑时降低轮速权重。
- GPS 良好区域使用 GPS 校准轮速尺度和全局漂移。
- 在任务系统中记录低速、转弯、湿滑区域，动态调整速度和定位权重。

## 测试

常规验证：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
python3 -m py_compile scripts/task_executor.py scripts/task_map_core.py scripts/route_recorder.py
python3 -m pytest src/robot_description/test/test_wheel_encoder_integration.py -q
colcon build --packages-select robot_description
```

最近基线：

- `test_wheel_encoder_integration.py`：`119 passed`
- `py_compile`：通过
- `colcon build --packages-select robot_description`：通过

## 已知限制

- 当前任务文件中已有的 `min_route_002` 包含倒车段，不适合作为 forward-only 策略的最终验收路线。
- Web 地图和 Nav2 地图需要使用同一组 `.yaml` / `.pgm`，否则车辆位置和路线会看起来偏移。
- Nav2 能规划不代表真实阿克曼车一定能跟踪，需要结合车辆最小转弯半径、路线离墙距离和控制器参数验证。
- 当前还没有完整真车网络拓扑实现，后续需要接入 4G、远程服务器、平板客户端和车端配置管理。
