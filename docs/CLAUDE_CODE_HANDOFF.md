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
  - `minimum_turning_radius: 1.6`
  - `min_turning_radius: 1.6`
  - `reverse_penalty: 100.0`
  - `smooth_path: false`
  - controller 低速、较大 lookahead，避免贴墙小半径急转。
- 任务执行器：
  - 订阅 `/task/command`
  - 发布 `/task/status`
  - 发布 `/task/current_goal`
  - 发布 `/task/active_path`
  - 支持 `start_task`、`pause_task`、`resume_task`、`cancel_task`、`manual_override`、`return_home`
- 若启动任务时还没有 `/localization/global_odom`，任务执行器会进入：

  ```text
  WAITING_FOR_LOCALIZATION
  ```

  收到第一帧定位后自动继续启动任务。

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
- 现有 `min_route_002` 中间包含 `direction: reverse`，它不适合作为当前 forward-only 策略下的基准任务。

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
  - 支持暂停、继续、取消、手动介入取消、回起点。
  - 无定位时进入等待状态，收到定位后自动开始任务。
  - 回起点时若已经接近 home，发布 `HOME_REACHED`，不再误以为车辆没响应。
  - 对示教路线做起点适配和稀疏化，减少密集点导致的阿克曼不可达路径。
- `task_map_core.py`：
  - 增加起点适配、距离计算等任务路线辅助逻辑。
- `route_recorder.py`：
  - 保存路线时合并已有 `task_map.yaml`，避免覆盖 Web 端创建的点位、任务和上传地图记录。
- `navigation.yaml`：
  - Smac Hybrid 切到 Dubins 前进优先。
  - 增大转弯半径和膨胀半径。
  - 降低速度并增大 controller lookahead。
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
- 任务路线本身包含倒车段或急转段，而当前配置按 forward-only 执行。
- 运行时加载的参数可能仍是旧 `install/`，需要确认 `BUILD=auto` 是否实际构建过，或手动 `BUILD=true ./scripts/start_full_stack.sh`。

下一步建议：

- 重新录制一条全程前进、离墙更远的测试路线。
- 实测车辆最小转弯半径后同步修改：
  - `config/navigation.yaml`
  - `maps/task_map.yaml` 中 `motion_profiles.min_turning_radius`
- 增加任务校验：forward-only 模式下拒绝包含 reverse 段的路线。

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

结果：`119 passed`

```bash
cd ros2_robot_sim
python3 -m py_compile scripts/task_executor.py scripts/task_map_core.py scripts/route_recorder.py
```

结果：通过

```bash
cd ros2_robot_sim
colcon build --packages-select robot_description
```

结果：通过

构建后已确认 `install/robot_description` 中包含最新：

- `task_executor.py`
- `navigation.yaml`

## 推荐下一步

### 短期

1. 启动完整栈并确认运行时参数是最新值。
2. 在 Web 中上传和 Nav2 一致的地图。
3. 重新设置 `home`。
4. 录制一条全程前进、离墙更远的路线。
5. 用这条路线新建任务并执行。
6. 若仍撞墙，抓取：
   - `/task/status`
   - `/task/active_path`
   - Nav2 global plan
   - `/control/cmd_vel`
   - `/robot/cmd_vel`
   - `log/full_stack/navigation.log`

### 中期

1. 增加 Web 任务校验：
   - forward-only 不允许 reverse 路线。
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
- 当前仍需用户手动录制一条新的 forward-only 基准路线验证导航效果。
