# ROS2 Robot Sim 强化学习应用设计

## 概述

在现有 `ros2_robot_sim`（ROS2 Humble + Gazebo 阿克曼四驱机器人仿真）基础上，分三阶段引入强化学习，逐步实现从端到端导航到 sim-to-real 可迁移的底层控制补偿。

技术栈：Stable-Baselines3 + Gymnasium + Gazebo 11 + ROS2 Humble

## 架构原则

- **零侵入**：RL 作为独立 package `rl_controller` 新增，`robot_description` 一行不改
- **ROS2 通信**：RL 策略通过订阅 LiDAR/odom、发布 cmd_vel 与仿真交互
- **三阶段共享核心模块**（ros2_bridge、base_env、obs processor），每阶段只实现差异化逻辑

## 目录结构

```
ros2_robot_sim/src/rl_controller/
├── package.xml
├── setup.py
├── config/
│   ├── phase1_ppo.yaml
│   ├── phase2_ppo.yaml
│   └── phase3_ppo.yaml
├── rl_controller/
│   ├── __init__.py
│   ├── bridge/
│   │   └── ros2_bridge.py          # 三阶段共享：话题订阅/发布、Gazebo 重置
│   ├── envs/
│   │   ├── base_env.py             # 共享基类：reset/step/reward 框架
│   │   ├── phase1_nav_env.py
│   │   ├── phase2_planner_env.py
│   │   └── phase3_track_env.py
│   ├── obs/
│   │   └── processor.py            # LiDAR 投影、路径点编码、归一化
│   ├── rewards/
│   │   └── reward_functions.py
│   ├── scripts/
│   │   ├── train_phase1.py
│   │   ├── train_phase2.py
│   │   ├── train_phase3.py
│   │   ├── evaluate.py
│   │   └── obstacle_spawner.py     # 阶段2 动态障碍物
│   └── inference/
│       └── rl_controller_node.py   # 推理节点，加载模型发布 cmd_vel
├── models/                          # 训练产出（.zip / .onnx）
└── launch/
    └── rl_inference.launch.py
```

## 阶段1：端到端无地图导航

### 目标

机器人从随机起点到达随机目标点，仅靠 LiDAR 感知自主避开静态障碍物，不依赖地图或全局路径。

### 观察空间（96维）

| 来源 | 内容 | 维度 | 处理 |
|------|------|------|------|
| LiDAR | 360° 水平投影 | 90 | 16 线取 min 压成 1 圈，每 4° 一个 bin |
| 目标 | 相对极坐标 | 2 | (distance, angle) |
| 自车速度 | 当前 cmd_vel | 2 | (linear_x, angular_z) |
| 目标朝向差 | 车头 vs 目标方向 | 2 | (sin, cos) |

### 动作空间（连续2维）

| 动作 | 范围 | 映射 |
|------|------|------|
| 线速度 | [-1, 1] | [-1.0, 5.0] m/s |
| 转向角速度 | [-1, 1] | [-0.5236, 0.5236] rad |

### 奖励函数

| 事件 | 值 | 权重 |
|------|------|------|
| 生存惩罚 | -0.01/step | 鼓励快速到达 |
| 接近目标 | +Δdistance | 正向引导 |
| 到达目标 | +10 | 距离 < 0.3m |
| 碰撞 | -10 (terminal) | LiDAR 最近距 < 0.2m |
| 倒退惩罚 | -0.02 * |v| v < 0 时 |
| 角速度平滑 | -0.005 * |ω| |

### 训练配置

- 算法：PPO（SB3 MlpPolicy）
- 总步数：500,000 ~ 2,000,000
- Episode 长度：500 steps（约 50s @ 10Hz）
- 重置策略：随机起点 + 随机目标点
- 环境：corridor_tunnel.world（先简单后换复杂 world）

## 阶段2：局部规划与动态避障

### 目标

RL 策略替代 Nav2 DWB Controller，跟踪全局路径的同时躲避动态障碍物。

### 观察空间（207维）

| 来源 | 内容 | 维度 | 处理 |
|------|------|------|------|
| LiDAR | 360° 投影 | 90 | 同阶段1 |
| 全局路径 | 前方最近 10 个路径点 | 20 | 相对极坐标 (dist, angle) |
| 最终目标 | 极坐标 | 2 | 保持全局方向感 |
| 自车速度 | 当前速度 | 2 | (v_x, ω_z) |
| 自车朝向 | (sin, cos) | 2 | |
| 障碍物运动 | LiDAR 时序差分 | 90 | 连续两帧点云差值 |
| 路径偏差 | 横向误差 | 1 | odom vs 路径投影 |

### 动作空间

同阶段1：连续2维 (v_x, ω_z)

### 奖励函数

| 事件 | 值 | 说明 |
|------|------|------|
| 沿路径前进 | +Δprogress | 沿路径投影距离 |
| 偏离路径 | -0.01 * lateral_error² | 横向偏差 |
| 靠近障碍 | -0.1 / d_min | 距离越近惩罚越大 |
| 碰撞 | -20 (terminal) | 有路径引导不应撞 |
| 到达终点 | +20 | |
| 动作平滑 | -0.01 * |a_t - a_{t-1}| |

### 动态障碍物

`obstacle_spawner.py` 通过 `/gazebo/set_model_state` 服务控制 2-4 个移动障碍物，支持匀速直线、随机游走、横穿路径三种运动模式。

### 集成方式

作为 Nav2 插件注册或替换 local controller 节点，接收全局路径发布 cmd_vel。

### 评估指标

- 成功率（到达终点且未碰撞）
- 平均速度 vs DWB baseline
- 最小障碍距离
- 路径平滑度

## 阶段3：阿克曼轨迹跟踪与漂移补偿

### 目标

RL 策略以残差形式修正开环控制器输出，补偿阿克曼运动学模型与 Gazebo 物理（摩擦、侧滑、惯性）之间的误差，实现高精度轨迹跟踪。

### 观察空间（33维）

| 来源 | 内容 | 维度 | 处理 |
|------|------|------|------|
| 参考轨迹 | 未来 5 个路径点 | 10 | 相对车体 (x, y) |
| 目标速度 | 路径点期望速度 | 5 | v_target 序列 |
| 自车速度 | odom | 3 | (v_x, v_y, ω_z) |
| 轮速编码器 | 4 轮读数 | 4 | /robot/wheel_encoder/* |
| IMU | 加速度 + 角速度 | 6 | (a_x, a_y, a_z, ω_x, ω_y, ω_z) |
| 转向角 | 当前前轮转角 | 2 | left/right steering joint |
| 横向误差 | odom vs 参考 | 1 | |
| 朝向误差 | odom vs 参考 | 2 | (sin, cos) |

### 动作空间（残差修正）

| 动作 | 范围 | 含义 |
|------|------|------|
| 速度修正 | [-0.3, 0.3] | 加到前馈速度 |
| 转角修正 | [-0.1, 0.1] | 加到 Pure Pursuit 转角 |

动作以残差形式输出：`cmd_vel = open_loop_output + rl_correction`

### 训练轨迹

- 恒定曲率圆（不同半径、速度）
- U 型弯
- 蛇形曲线
- 变曲率路径（路口转弯）
- 紧急制动（突然减速）

每种轨迹在多种摩擦系数下训练（修改 Gazebo mu 参数）。

### 奖励函数

| 事件 | 值 | 说明 |
|------|------|------|
| 横向误差 | -1.0 * error_lat² | 核心指标 |
| 朝向误差 | -0.5 * error_heading² | |
| 速度跟踪 | -0.3 * (v_actual - v_ref)² | |
| 转向平滑 | -0.02 * Δsteering² | 防抖动 |
| 滑移惩罚 | -0.1 * |v_y| 侧向速度 |
| 完成轨迹 | +5 | |
| 严重偏离 | -10 (terminal) | 横向误差 > 0.5m |

### 部署

导出 ONNX 模型，推理节点运行在 50Hz。在真实小车部署时仅需将 Gazebo 话题映射到真实硬件话题。

## 数据流

```
Gazebo ──[LiDAR/odom/IMU]──> ros2_bridge ──[obs]──> env.step() ──> SB3 PPO
                                                           │
                                                     [action: cmd_vel]
                                                           │
Gazebo <──[cmd_vel]──── ros2_bridge <──────────────────────┘
```

## 与现有项目的关系

- `robot_description`：零改动
- RL 训练/推理与键盘控制、Nav2 导航互不冲突，可同时存在
- RL 推理节点可随时启用或关闭，不影响仿真原有功能

## 依赖

```bash
pip install stable-baselines3[extra] gymnasium numpy torch
# 已有：ROS2 Humble, Gazebo 11.10.2
```

## 预期产出

| 阶段 | 产出 | 可展示内容 |
|------|------|------|
| 1 | PPO 模型 + 推理节点 | 车从 A 到 B 自主避开障碍 |
| 2 | PPO 模型 + Nav2 插件 | 沿导航路径走，躲避移动障碍 |
| 3 | ONNX 模型 + 补偿节点 | 轨迹跟踪可视化对比（有/无补偿） |
