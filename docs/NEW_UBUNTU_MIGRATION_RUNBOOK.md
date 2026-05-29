# 新 Ubuntu 双系统迁移与开箱验证手册

本文用于把当前虚拟机开发环境迁移到本机 Ubuntu 双系统，并恢复两个项目：

- 仿真项目：`ros2_robot_sim`
- 遥控器项目：`remote`（远端仓库名：`ros2_remote`）

目标状态：新 Ubuntu 系统安装完成后，可以拉取代码、恢复必要配置、安装 ROS2 Humble、安装 Claude Code 和 Codex、配置代理，并完成仿真与遥控器运行验证。

---

## 0. 当前基线

迁移前已经完成：

- `ros2_robot_sim` 已推送到 GitHub：
  - 仓库：`https://github.com/1xavier1/ros2_robot_sim.git`
  - 基线提交：`4d8b23e feat: add migration navigation baseline`
- `remote` 已推送到 GitHub：
  - 仓库：`https://github.com/1xavier1/ros2_remote.git`
  - 基线提交：`b79fb42 chore: ignore remote runtime pid`
- 本机备份包：
  - 路径：`/home/xavier/claudespace-backup-2026-05-29.tar.gz`
  - 大小：约 `162M`
  - 包含：项目目录、两个 Git 仓库元数据、`~/.ssh`、`~/.gitconfig`、`~/.claude`、`~/.codex`
  - 已排除：`build`、`install`、`log`、`.codex/*.sqlite*` 运行状态库

新系统优先从 GitHub 拉代码，备份包用于恢复密钥、配置、Claude/Codex 资料，以及在网络不可用时兜底恢复项目。

---

## 1. 双系统安装注意事项

推荐系统：

- Ubuntu `22.04 LTS`
- 不建议用 Ubuntu 24.04 做本项目主开发环境，因为 ROS2 Humble 官方目标平台是 Ubuntu 22.04 Jammy。

安装前在 Windows 中处理：

1. 关闭快速启动：
   - 控制面板 → 电源选项 → 选择电源按钮功能 → 取消「启用快速启动」
2. 如果启用了 BitLocker，先暂停或解锁确认恢复密钥可用。
3. 在 Windows 磁盘管理中压缩出 Ubuntu 空间：
   - 建议至少 `100 GB`
   - 如果长期运行 Gazebo、RViz、地图和日志，建议 `150 GB+`
4. 安装 Ubuntu 时选择「与 Windows 共存」或手动分区。

手动分区建议：

| 挂载点 | 大小 | 文件系统 | 说明 |
|--------|------|----------|------|
| `/` | 80 GB+ | ext4 | 系统、ROS2、开发工具 |
| `/home` | 剩余空间 | ext4 | 项目、缓存、配置 |
| EFI | 使用现有 EFI 分区 | FAT32 | 不要格式化 Windows EFI |

安装完成后先进入 Ubuntu，确认：

```bash
lsb_release -a
df -h
sudo apt update
```

---

## 2. 把备份包复制到新 Ubuntu

任选一种方式。

### 方式 A：U 盘或移动硬盘

在旧虚拟机中把文件复制出去：

```bash
cp /home/xavier/claudespace-backup-2026-05-29.tar.gz /media/$USER/<U盘名>/
```

在新 Ubuntu 中复制到 home：

```bash
cp /media/$USER/<U盘名>/claudespace-backup-2026-05-29.tar.gz ~/
```

### 方式 B：局域网 scp

旧虚拟机作为源，新 Ubuntu 执行：

```bash
scp xavier@<旧虚拟机IP>:/home/xavier/claudespace-backup-2026-05-29.tar.gz ~/
```

### 方式 C：网盘或临时文件服务器

只要最终文件在新 Ubuntu 的 `~/claudespace-backup-2026-05-29.tar.gz` 即可。

校验备份包能读取：

```bash
ls -lh ~/claudespace-backup-2026-05-29.tar.gz
tar -tzf ~/claudespace-backup-2026-05-29.tar.gz | sed -n '1,20p'
```

---

## 3. 恢复备份中的配置

先安装基础工具：

```bash
sudo apt update
sudo apt install -y git curl wget ca-certificates gnupg lsb-release \
  build-essential cmake python3-pip python3-venv unzip
```

创建工作目录：

```bash
mkdir -p ~/Workspace/ClaudeSpace
```

### 3.1 推荐恢复方式：只恢复配置

不要直接覆盖整个 home，先把备份解到临时目录：

```bash
mkdir -p ~/migration-restore
tar -xzf ~/claudespace-backup-2026-05-29.tar.gz -C ~/migration-restore
```

恢复 Git 配置：

```bash
cp ~/migration-restore/home/xavier/.gitconfig ~/.gitconfig
```

恢复 SSH：

```bash
mkdir -p ~/.ssh
cp -a ~/migration-restore/home/xavier/.ssh/. ~/.ssh/
chmod 700 ~/.ssh
chmod 600 ~/.ssh/* 2>/dev/null || true
chmod 644 ~/.ssh/*.pub 2>/dev/null || true
```

恢复 Claude Code 配置：

```bash
mkdir -p ~/.claude
cp -a ~/migration-restore/home/xavier/.claude/. ~/.claude/
```

恢复 Codex 配置：

```bash
mkdir -p ~/.codex
cp -a ~/migration-restore/home/xavier/.codex/. ~/.codex/
```

注意：

- `~/.ssh` 权限必须正确，否则 GitHub SSH 可能拒绝使用密钥。
- 如果新系统用户名不是 `xavier`，上面的备份源路径仍然是 `~/migration-restore/home/xavier/...`，这是正常的。
- Claude Code 和 Codex 的登录态可能因机器变化失效。恢复配置后仍可能需要重新登录。

### 3.2 离线兜底：恢复项目目录

如果 GitHub 暂时无法访问，可以先恢复备份里的项目：

```bash
mkdir -p ~/Workspace
cp -a ~/migration-restore/home/xavier/Workspace/ClaudeSpace ~/Workspace/
```

恢复后检查两个仓库：

```bash
git -C ~/Workspace/ClaudeSpace/ros2_robot_sim status --short --branch
git -C ~/Workspace/ClaudeSpace/remote status --short --branch
```

只要网络恢复，仍建议执行：

```bash
git -C ~/Workspace/ClaudeSpace/ros2_robot_sim pull
git -C ~/Workspace/ClaudeSpace/remote pull
```

---

## 4. 优先从 GitHub 拉取两个项目

如果 SSH 可用：

```bash
mkdir -p ~/Workspace/ClaudeSpace
cd ~/Workspace/ClaudeSpace

git clone git@github.com:1xavier1/ros2_robot_sim.git
git clone git@github.com:1xavier1/ros2_remote.git remote
```

如果 SSH 暂时不可用，用 HTTPS：

```bash
mkdir -p ~/Workspace/ClaudeSpace
cd ~/Workspace/ClaudeSpace

git clone https://github.com/1xavier1/ros2_robot_sim.git
git clone https://github.com/1xavier1/ros2_remote.git remote
```

确认基线提交：

```bash
git -C ~/Workspace/ClaudeSpace/ros2_robot_sim log --oneline --decorate --max-count=3
git -C ~/Workspace/ClaudeSpace/remote log --oneline --decorate --max-count=3
```

期望看到：

- `ros2_robot_sim` 顶部包含 `4d8b23e`
- `remote` 顶部包含 `b79fb42`

---

## 5. 配置代理（vmess）

Ubuntu 下推荐两种方案：

- 图形界面优先：v2rayA
- 更适合长期规则管理：sing-box

如果你已有 vmess 订阅或节点，先确保本机代理端口可用。常见端口：

- HTTP：`127.0.0.1:7890`
- SOCKS5：`127.0.0.1:7891` 或 `127.0.0.1:7890`

临时让当前终端走代理：

```bash
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
export all_proxy=socks5://127.0.0.1:7890
```

写入 `~/.bashrc`：

```bash
cat >> ~/.bashrc <<'EOF'

# Proxy for development tools
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
export all_proxy=socks5://127.0.0.1:7890
EOF
```

验证：

```bash
curl -I https://github.com
git ls-remote https://github.com/1xavier1/ros2_robot_sim.git
```

如果代理客户端端口不是 `7890`，替换成实际端口。

---

## 6. 安装 ROS2 Humble 和项目依赖

进入仿真仓库：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
```

执行项目自带安装脚本：

```bash
chmod +x scripts/install_ros2_humble.sh scripts/install_dependencies.sh
./scripts/install_ros2_humble.sh
./scripts/install_dependencies.sh
```

如果 `sudo rosdep init` 提示已经初始化，继续执行：

```bash
rosdep update
```

把 ROS2 环境写入 shell：

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

安装当前工作区依赖：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
rosdep install --from-paths src --ignore-src -r -y
```

构建：

```bash
colcon build --packages-select robot_description
source install/setup.bash
```

---

## 7. 安装 Claude Code 和 Codex

先安装 Node.js 18+。Ubuntu 22.04 默认 Node.js 可能偏旧，推荐安装 Node.js 20：

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

node --version
npm --version
```

配置 npm 全局安装目录，避免使用 `sudo npm install -g`：

```bash
mkdir -p ~/.npm-global
npm config set prefix ~/.npm-global

grep -qxF 'export PATH="$HOME/.npm-global/bin:$PATH"' ~/.bashrc || \
  echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> ~/.bashrc

source ~/.bashrc
```

安装 Claude Code：

```bash
npm install -g @anthropic-ai/claude-code
claude --version
claude doctor
```

启动并登录：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
claude
```

安装 Codex：

```bash
npm install -g @openai/codex
codex --version
```

启动并登录：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
codex
```

说明：

- Claude Code 官方要求 Node.js 18+，并建议不要用 `sudo npm install -g`。
- Codex CLI 官方支持 `npm install -g @openai/codex`。
- 备份恢复了 `~/.claude` 和 `~/.codex`，但新机器上仍可能要求重新认证。

---

## 8. 验证仿真项目

### 8.1 无 GUI 快速验证

终端 1：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
./start.sh --no-rviz
```

如果需要完全无窗口验证，可直接用 launch 参数：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
source install/setup.bash

ros2 topic list
ros2 topic echo /robot/odom --once
ros2 topic echo /robot/imu/data --once
ros2 topic echo /robot/velodyne_points --once --field width
```

期望：

- `ros2 topic list` 中包含 `/robot/odom`、`/robot/imu/data`、`/robot/velodyne_points`、`/robot/cmd_vel`
- `/robot/odom` 能输出 `nav_msgs/msg/Odometry`
- `/robot/imu/data` 能输出 `sensor_msgs/msg/Imu`
- `/robot/velodyne_points` 的 `width` 有有效数值

### 8.2 发送速度指令验证

终端 2 执行：

```bash
ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

再检查里程计：

```bash
ros2 topic echo /robot/odom --once
```

### 8.3 项目自带检查脚本

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/verify_runtime_topics.sh
./scripts/verify_navigation_precheck.sh
./scripts/verify_saved_map_nav2_precheck.sh
```

如果检查脚本提示缺少 frame、plugin 或 topic，优先看当前终端中 Gazebo 和 ROS2 launch 日志。

---

## 9. 验证遥控器项目

安装 Python 依赖：

```bash
python3 -m pip install --user websockets
```

先启动仿真，再启动遥控器：

```bash
cd ~/Workspace/ClaudeSpace/remote
./start.sh
```

浏览器访问：

```text
http://localhost:8765
```

局域网设备访问：

```bash
hostname -I
```

用输出的 IP 访问：

```text
http://<Ubuntu局域网IP>:8765
```

验证点：

- 页面显示在线状态。
- 点击或拖动摇杆后，仿真中的 `/robot/cmd_vel` 有输出。
- 遥控器能读取 `/robot/odom` 并显示反馈。

停止遥控器：

```bash
cd ~/Workspace/ClaudeSpace/remote
./stop.sh
```

停止仿真：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
./stop.sh
```

---

## 10. 常见问题

### 10.1 `ros-humble-*` 包找不到

检查系统版本：

```bash
lsb_release -a
```

必须是 Ubuntu 22.04。然后检查 ROS2 apt 源：

```bash
cat /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### 10.2 `rosdep init` 已经执行过

这是正常情况，直接执行：

```bash
rosdep update
```

### 10.3 GitHub 访问失败

先验证代理：

```bash
curl -I https://github.com
git ls-remote https://github.com/1xavier1/ros2_robot_sim.git
```

如果代理有效但 Git 不走代理：

```bash
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890
```

取消 Git 代理：

```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

### 10.4 SSH 密钥权限错误

执行：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/* 2>/dev/null || true
chmod 644 ~/.ssh/*.pub 2>/dev/null || true
ssh -T git@github.com
```

### 10.5 Gazebo 启动失败或端口残留

执行：

```bash
cd ~/Workspace/ClaudeSpace/ros2_robot_sim
./stop.sh -f || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
rm -f /tmp/gazebo-* 2>/dev/null || true
```

再重新启动：

```bash
./start.sh --no-rviz
```

### 10.6 Claude Code 或 Codex 命令找不到

检查 npm 全局路径：

```bash
npm config get prefix
echo "$PATH"
ls ~/.npm-global/bin
```

修复：

```bash
grep -qxF 'export PATH="$HOME/.npm-global/bin:$PATH"' ~/.bashrc || \
  echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

---

## 11. 最终验收清单

全部通过后，新 Ubuntu 环境可以继续开发：

- [ ] `lsb_release -a` 显示 Ubuntu 22.04。
- [ ] `git -C ~/Workspace/ClaudeSpace/ros2_robot_sim status --short --branch` 干净。
- [ ] `git -C ~/Workspace/ClaudeSpace/remote status --short --branch` 干净。
- [ ] `ros2 --help` 和 `ros2 topic list` 可执行。
- [ ] `colcon build --packages-select robot_description` 成功。
- [ ] `ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false` 可以启动。
- [ ] `/robot/odom`、`/robot/imu/data`、`/robot/velodyne_points` 有数据。
- [ ] `/robot/cmd_vel` 可以接收速度指令。
- [ ] `remote/start.sh` 可以启动 Web 遥控器。
- [ ] 浏览器可以访问 `http://localhost:8765`。
- [ ] `claude --version` 正常。
- [ ] `codex --version` 正常。
- [ ] 代理下 `curl -I https://github.com` 正常。

---

## 12. 参考链接

- ROS2 Humble 官方安装页：<https://docs.ros.org/en/humble/Installation.html>
- Claude Code 官方安装页：<https://docs.anthropic.com/en/docs/claude-code/getting-started>
- Codex CLI 官方说明：<https://help.openai.com/en/articles/11096431>
