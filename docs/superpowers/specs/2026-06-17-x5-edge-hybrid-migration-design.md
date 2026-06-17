# X5 主控混合仿真迁移设计

## 背景

当前项目以 Gazebo 为仿真底座，GPS、LiDAR、IMU、毫米波雷达、底盘和车体都由仿真提供。现在已经具备一块地平线 X5 主控板，目标是在真实主控上先运行机器人边缘计算，再逐步替换仿真部件。

第一阶段不能破坏本机现有仿真流程。本机仍需要能完整运行 `ros2_robot_sim` 和 remote 控制台。X5 先作为真实主控雏形接入当前仿真系统：本机提供仿真传感器和底盘执行，X5 负责本地 ROS 计算、任务执行和建图计算。

## 第一阶段目标

第一阶段采用增强 B+ 范围：**X5 主控闭环 + remote 建图全流程**。

阶段完成后应满足：

- X5 能作为 ROS 主控计算端运行定位、Nav2、任务执行和安全监控。
- 本机 Gazebo 仍提供仿真传感器、仿真底盘和仿真 world。
- 本机 remote 仍作为临时服务器和页面入口。
- remote 页面上的建图功能必须完整触发 X5 上的建图任务。
- X5 本地地图作为运行权威，本机 remote 保存同步副本用于 UI、下载和备份。
- 本机原 `ros2_robot_sim` 仿真流程保持可用。

## 非目标

第一阶段不做以下事项：

- 公网云端正式部署。
- 用户登录、权限、车队管理。
- 真传感器驱动接入。
- 真底盘驱动接入。
- 地图自动同步 GitHub 或云对象存储。
- X5 systemd 开机自启动。
- 将 remote 整体部署到 X5。
- 将密码、固定 IP、机器私有 `.env` 提交到仓库。

## 总体架构

```text
客户端浏览器
  -> 本机 remote 临时服务器
  -> X5 edge agent
  -> X5 ROS 主控计算层
  -> /robot/cmd_vel
  -> 本机 Gazebo 仿真车体

本机 Gazebo
  -> /sensing/* /robot/odom /tf /clock
  -> X5 定位 / Nav2 / 任务执行 / 建图计算
```

本机职责：

- 运行 Gazebo world、仿真车体、仿真传感器和底盘控制器。
- 发布 `/sensing/*`、`/robot/odom`、`/tf`、`/tf_static`、`/clock`。
- 接收 X5 发布的 `/robot/cmd_vel` 并驱动仿真车辆。
- 运行 remote 临时服务器和 Web 页面。
- 保存 X5 同步回来的地图副本。

X5 职责：

- 运行定位、Nav2、任务执行、安全监控、建图计算。
- 订阅本机仿真传感器、里程计、TF 和仿真时钟。
- 发布 `/localization/*`、`/task/*`、`/plan`、`/push/active`、`/robot/cmd_vel`。
- 运行轻量 edge agent，供 remote 后端调用建图和地图同步能力。
- 保存运行权威地图，供 X5 本地 Nav2 加载。

remote 第一阶段职责：

- 仍部署在本机，不部署到 X5。
- 页面功能保持完整，尤其是建图页面。
- 后端通过 X5 edge agent 启动、停止、取消、查询、保存建图任务。
- 不通过 SSH 执行建图命令。
- 不直接读写 X5 文件系统。

## 仓库策略

第一阶段采用“先复制，后拆分”的仓库策略。

源仓库：

```text
/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
```

X5 目标目录：

```text
/home/sunrise/Workspace/robot
```

X5 云端仓库：

```text
https://github.com/1xavier1/robot_x5.git
```

策略：

- `robot_x5` 第一阶段先完整复制当前 `ros2_robot_sim`。
- 在 `robot_x5` 中新增 `sim_host` 和 `x5_edge` profile。
- `ros2_robot_sim` 继续作为本机仿真验证主仓。
- `robot_x5` 不直接替代 `ros2_robot_sim`。
- X5 相关部署脚本、环境检查脚本、edge agent 设计进入 `robot_x5`。
- 未来 X5 闭环稳定后，再将 Gazebo world、仿真模型、RViz 等仿真资产从 `robot_x5` 拆出。

## 网络与 ROS 边界

第一阶段使用同一 ROS 2 DDS 域的局域网多机通信，不引入云端消息队列或自研实时转发协议。

本机 `sim_host` 发布或提供：

```text
/sensing/lidar/points
/sensing/imu/data
/sensing/gps/fix
/sensing/mmwave/*
/robot/odom
/tf
/tf_static
/clock
/robot/cmd_vel 接收端
```

X5 `x5_edge` 发布或提供：

```text
/localization/*
/plan
/task/*
/push/active
/robot/cmd_vel
Nav2 action/service
X5 edge agent HTTP API
```

配置要求：

- 本机与 X5 使用同一个 `ROS_DOMAIN_ID`。
- 两端设置 `ROS_LOCALHOST_ONLY=0`。
- 固定使用一种 RMW 实现，避免两端 DDS 发现行为不一致。
- 两端启动脚本必须打印主机名、IP、`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 和 profile。
- 使用 Gazebo 仿真时，X5 节点必须统一 `use_sim_time=true`。
- TF 权威来源必须唯一，避免本机和 X5 同时发布重复 `map -> odom` 或 `odom -> base_footprint`。

## 启动 Profile

第一阶段新增两个 profile。

### sim_host

运行位置：本机。

启动内容：

- Gazebo world。
- 仿真车体和传感器。
- 底盘控制器。
- `/sensing/*`、`/robot/odom`、`/tf`、`/clock`。
- remote 临时服务器。
- 必要的 RViz 和调试工具。

不启动：

- Nav2。
- task executor。
- localization fusion。
- global localization backend。
- 建图导出计算链路。

### x5_edge

运行位置：X5。

启动内容：

- FAST-LIO / Wheel-LIO / fused localization / global localization。
- localization mode manager。
- Nav2。
- route recorder。
- task executor。
- proximity safety monitor。
- X5 edge agent。
- 建图导出脚本依赖和运行入口。

不启动：

- Gazebo。
- 仿真 world。
- remote Web 服务。
- 与本机重复的 TF 发布者。

建议启动入口：

```bash
./scripts/start_sim_host.sh
./scripts/start_x5_edge.sh
```

也可以保留兼容入口：

```bash
./start.sh --profile sim_host
./start.sh --profile x5_edge
```

## X5 Edge Agent

为了让 remote 页面完整控制 X5 建图，第一阶段需要新增轻量车端 agent。

Agent 运行在 X5，职责是：

- 管理 X5 上的建图进程。
- 暴露建图 start / stop / cancel / status / preview / save 能力。
- 将建图错误、进度、预览路径返回给本机 remote。
- 保存地图到 X5 本地权威地图库。
- 在保存成功后向本机 remote 提供地图文件同步。
- 防止并发启动多个建图任务。

Agent 不负责：

- 用户登录。
- 云端车队管理。
- 长期地图版本治理。
- 实时底盘控制闭环。

Agent 与 remote 后端之间第一阶段使用局域网 HTTP API 即可。后续云端化时，可将该协议演进为车端长连接或云端消息通道。

## Remote 建图全流程

remote 页面现有建图能力必须保持完整，但执行位置从本机迁移到 X5。

页面操作链路：

```text
开始建图
  -> remote 后端
  -> X5 edge agent
  -> X5 建图进程

状态/预览
  -> remote 后端轮询或订阅 X5 agent
  -> remote 页面展示

保存地图
  -> X5 保存权威地图
  -> remote 拉取同步副本
  -> remote 地图库展示地图
```

建图保存必须生成：

```text
.pgm
.yaml
_pose.json
元数据
```

保存后：

- X5 本地地图是 Nav2 加载的权威地图。
- 本机 remote 地图副本用于 UI 展示、下载和备份。
- 如果 X5 保存成功但本机同步失败，remote 必须显示同步失败。
- 如果 X5 保存失败，remote 不能伪装成本机保存成功。
- 第一阶段冲突处理规则是 X5 版本优先。

## 只读环境盘点

迁移前先只读 SSH 到 X5 盘点，不安装、不改网络、不写 systemd。

盘点项：

- OS / 内核 / CPU 架构。
- ROS 2 是否安装及版本。
- colcon / Python / pip / git 状态。
- `/home/sunrise/Workspace/robot` 是否存在且可写。
- X5 与本机网络连通性。
- GitHub 访问能力。
- `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`ROS_LOCALHOST_ONLY` 当前状态。
- 磁盘剩余空间。
- 需要安装或补齐的依赖清单。

盘点结果写入文档时必须脱敏，不记录密码。

## 部署流程

只读盘点通过后进入部署阶段。

部署步骤：

1. 在本机准备 `robot_x5` 仓库。
2. 将当前 `ros2_robot_sim` 作为初始迁移副本。
3. 添加 X5 profile、edge agent、环境检查脚本和部署说明。
4. 推送到 `https://github.com/1xavier1/robot_x5.git`。
5. 在 X5 上创建或更新 `/home/sunrise/Workspace/robot`。
6. 从 GitHub clone 或 pull `robot_x5`。
7. 安装缺失依赖。
8. `colcon build`。
9. 配置不提交的本地 `.env`。
10. 运行 ROS 网络检查。
11. 启动 `x5_edge`。
12. 在本机启动 `sim_host` 和 remote。

## 验收标准

### 验收 0：X5 只读盘点完成

输出环境盘点结果，包含系统、ROS、依赖、网络、磁盘、GitHub 访问能力和依赖缺口。

### 验收 1：本机仿真不被破坏

本机原有流程仍可运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
./start.sh
```

remote 本机临时服务仍能打开。

### 验收 2：ROS 多机通信

X5 能看到本机仿真话题：

```text
/sensing/lidar/points
/sensing/imu/data
/sensing/gps/fix
/robot/odom
/tf
/clock
```

本机能看到 X5 发布的话题：

```text
/localization/*
/task/status
/plan
/robot/cmd_vel
```

### 验收 3：X5 控制 Gazebo 小车

X5 发布 `/robot/cmd_vel` 后，本机 Gazebo 小车移动；停止命令后小车停止。

### 验收 4：X5 主控闭环

X5 启动定位、Nav2、任务执行和安全监控。remote 从本机下发控制或任务指令，X5 接收并执行，Gazebo 小车响应。

### 验收 5：Remote 页面完整建图

remote 页面必须能：

- 点击开始建图，实际启动 X5 上的建图任务。
- 显示 X5 建图状态、进度和错误。
- 查看建图预览。
- 停止或取消建图。
- 保存地图。
- 生成并同步 `.pgm`、`.yaml`、`_pose.json` 和元数据。
- 保存后本机 remote 地图库能看到地图。
- X5 本地也保留地图，供 Nav2 后续加载。

### 验收 6：地图权威与同步

- X5 本地地图是运行权威。
- 本机 remote 同步副本用于 UI 和备份。
- 第一阶段如果两边冲突，优先 X5 版本。
- 地图同步失败时，remote 页面必须显示失败。

## 失败排查顺序

迁移问题按层排查：

```text
网络/DDS
-> ROS topic
-> /clock 和 use_sim_time
-> TF 权威来源
-> 定位
-> Nav2
-> task executor
-> X5 edge agent
-> remote 页面
```

第一阶段不要先改算法参数。优先证明部署、网络、时间、TF 和进程边界正确。

## 规格自检

- 没有未定义占位符或未完成章节。
- 第一阶段范围聚焦在 X5 混合仿真主控迁移和 remote 建图全流程。
- X5、本机、remote、edge agent 的职责边界互不冲突。
- 地图权威规则明确为 X5 优先，本机 remote 保存同步副本。
- 不包含密码、固定私有 IP 或敏感 `.env` 内容。
