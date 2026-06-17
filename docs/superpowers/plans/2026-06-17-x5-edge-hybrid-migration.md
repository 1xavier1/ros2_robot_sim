# X5 主控混合仿真迁移实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前仿真系统迁移出第一阶段 X5 主控闭环：本机继续运行 Gazebo 和 remote，X5 运行 ROS 主控计算，并由 remote 页面完整触发 X5 建图全流程。

**架构：** 第一阶段保留本机 `ros2_robot_sim` 作为仿真主仓，新建 `robot_x5` 作为 X5 主控仓库，先复制当前仿真仓再增加 `sim_host` / `x5_edge` profile。remote 后端通过 X5 edge agent 调用车端建图能力，X5 地图为运行权威，本机 remote 保存同步副本。

**技术栈：** ROS 2 Humble、Gazebo、Nav2、Python 3、stdlib `http.server`、FastAPI remote 后端、Vue remote 前端、pytest、colcon。

---

## 关联规格

- 规格：`docs/superpowers/specs/2026-06-17-x5-edge-hybrid-migration-design.md`
- 本机仿真仓库：`/home/xavier/Workspace/ClaudeSpace/ros2_robot_sim`
- remote worktree：`/home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench`
- X5 本地开发仓库：`/home/xavier/Workspace/ClaudeSpace/robot_x5`
- X5 目标运行目录：`/home/sunrise/Workspace/robot`

## 文件结构

### robot_x5 仓库

从 `ros2_robot_sim` 复制创建，新增和修改文件：

- 创建：`scripts/env_profile.sh`  
  集中加载 ROS 环境、打印主机名/IP/profile/RMW/ROS_DOMAIN_ID。
- 创建：`scripts/check_x5_env.sh`  
  只读环境盘点脚本，不安装依赖、不改系统配置。
- 创建：`scripts/check_ros_network.sh`  
  多机 ROS topic 可见性检查脚本。
- 创建：`scripts/start_sim_host.sh`  
  本机仿真端启动入口，只启动 Gazebo、仿真传感器、底盘和 remote 提示。
- 创建：`scripts/start_x5_edge.sh`  
  X5 主控端启动入口，启动 FAST-LIO、Nav2、任务执行、安全监控和 edge agent。
- 创建：`scripts/x5_mapping_agent.py`  
  车端 HTTP agent，管理 X5 上的建图进程和地图文件同步。
- 创建：`launch/x5_edge.launch.py`  
  X5 主控 launch，组合定位、Nav2、任务执行和安全节点。
- 创建：`src/robot_description/test/test_x5_edge_scripts.py`  
  源码级测试 profile 脚本和 launch 约束。
- 创建：`src/robot_description/test/test_x5_mapping_agent.py`  
  测试 X5 mapping agent 的状态机、预览、保存和 bundle 下载。
- 修改：`start.sh`  
  增加 `--profile sim_host|x5_edge` 兼容入口。
- 修改：`src/robot_description/CMakeLists.txt`  
  确保新增脚本和 launch 文件安装。
- 创建：`docs/x5_environment_check.md`  
  盘点输出模板和脱敏记录格式。
- 创建：`docs/x5_deployment.md`  
  部署、构建、运行、验收命令。

### remote worktree

- 修改：`server/config.py`  
  增加 `mapping_backend`、`edge_agent_url`、`edge_request_timeout` 配置。
- 创建：`server/services/edge_mapping_client.py`  
  调用 X5 agent 的 HTTP 客户端，负责 JSON 请求、PNG 预览、地图 bundle 下载。
- 创建：`server/services/remote_mapping_jobs.py`  
  实现与 `MappingJobs` 同样接口的 X5 代理建图后端。
- 修改：`server/main.py`  
  根据配置选择本地 `MappingJobs` 或 X5 `RemoteMappingJobs`。
- 修改：`server/api/mapping.py`  
  保持 REST API 不变，只补充 X5 同步失败的错误码映射。
- 测试：`tests/test_edge_mapping_client.py`
- 测试：`tests/test_remote_mapping_jobs.py`
- 修改：`tests/test_api_mapping.py`
- 修改：`tests/test_config.py`

### ros2_robot_sim 仓库

- 修改：`docs/CLAUDE_CODE_HANDOFF.md`  
  记录 X5 迁移计划、仓库关系和 remote 建图代理边界。
- 修改：`docs/superpowers/plans/2026-06-17-x5-edge-hybrid-migration.md`  
  执行过程中按复选框更新进度。

## 任务 1：准备 robot_x5 仓库副本

**文件：**
- 创建目录：`/home/xavier/Workspace/ClaudeSpace/robot_x5`
- 创建：`/home/xavier/Workspace/ClaudeSpace/robot_x5/.git`
- 修改：`/home/xavier/Workspace/ClaudeSpace/robot_x5/.gitignore`
- 测试：命令行检查

- [ ] **步骤 1：确认源仓库干净**

运行：

```bash
git -C /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim status --short
```

预期：无输出。

- [ ] **步骤 2：创建迁移副本**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace
mkdir -p robot_x5
rsync -a \
  --exclude .git \
  --exclude build \
  --exclude install \
  --exclude log \
  --exclude .pytest_cache \
  ros2_robot_sim/ robot_x5/
```

预期：`robot_x5/src/robot_description/CMakeLists.txt` 存在。

- [ ] **步骤 3：初始化 robot_x5 Git 仓库**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
git init
git branch -M main
git remote add origin https://github.com/1xavier1/robot_x5.git
```

预期：`git remote -v` 显示 `origin https://github.com/1xavier1/robot_x5.git`。

- [ ] **步骤 4：确认 .gitignore 屏蔽构建产物**

检查 `.gitignore`，确保包含：

```gitignore
build/
install/
log/
.pytest_cache/
__pycache__/
*.pyc
.env
.env.*
```

如果缺少，补齐这些条目。

- [ ] **步骤 5：初始提交**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
git add .
git commit -m "chore(仓库): 初始化 X5 主控迁移副本"
```

预期：提交成功，`git status --short` 无输出。

## 任务 2：添加环境盘点和 ROS 网络检查脚本

**文件：**
- 创建：`robot_x5/scripts/env_profile.sh`
- 创建：`robot_x5/scripts/check_x5_env.sh`
- 创建：`robot_x5/scripts/check_ros_network.sh`
- 创建：`robot_x5/docs/x5_environment_check.md`
- 测试：`robot_x5/src/robot_description/test/test_x5_edge_scripts.py`

- [ ] **步骤 1：编写失败的源码级测试**

创建 `src/robot_description/test/test_x5_edge_scripts.py`：

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def test_x5_environment_scripts_exist_and_are_non_destructive():
    env_profile = read("scripts/env_profile.sh")
    check_env = read("scripts/check_x5_env.sh")
    network = read("scripts/check_ros_network.sh")

    assert "ROS_DOMAIN_ID" in env_profile
    assert "RMW_IMPLEMENTATION" in env_profile
    assert "ROS_LOCALHOST_ONLY" in env_profile
    assert "PROFILE_NAME" in env_profile

    assert "apt install" not in check_env
    assert "sudo " not in check_env
    assert "systemctl" not in check_env
    assert "uname -a" in check_env
    assert "df -h" in check_env
    assert "ros2 --version" in check_env

    assert "ros2 topic list" in network
    assert "/sensing/lidar/points" in network
    assert "/robot/odom" in network
    assert "/clock" in network
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
python3 -m pytest src/robot_description/test/test_x5_edge_scripts.py -q
```

预期：FAIL，报错包含 `FileNotFoundError`。

- [ ] **步骤 3：创建 env_profile.sh**

创建 `scripts/env_profile.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

PROFILE_NAME="${PROFILE_NAME:-unknown}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$(pwd)/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source_if_exists() {
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        set +u
        source "$setup_file"
        set -u
    fi
}

first_ip() {
    hostname -I 2>/dev/null | awk '{print $1}'
}

source_if_exists "$ROS_SETUP"
source_if_exists "$WORKSPACE_SETUP"

printf '[env] profile=%s\n' "$PROFILE_NAME"
printf '[env] host=%s\n' "$(hostname)"
printf '[env] ip=%s\n' "$(first_ip)"
printf '[env] ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf '[env] ROS_LOCALHOST_ONLY=%s\n' "$ROS_LOCALHOST_ONLY"
printf '[env] RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"
```

- [ ] **步骤 4：创建 check_x5_env.sh**

创建 `scripts/check_x5_env.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE_NAME="${PROFILE_NAME:-x5_env_check}"
cd "$ROOT_DIR"
source "$ROOT_DIR/scripts/env_profile.sh"

section() {
    printf '\n[x5-check] %s\n' "$1"
}

section "system"
uname -a
lsb_release -a 2>/dev/null || cat /etc/os-release

section "cpu"
lscpu | sed -n '1,20p'

section "disk"
df -h "$ROOT_DIR" "$HOME" 2>/dev/null || df -h

section "tools"
command -v git || true
git --version || true
command -v python3 || true
python3 --version || true
command -v pip3 || true
pip3 --version || true
command -v colcon || true
colcon version-check 2>/dev/null || true

section "ros"
command -v ros2 || true
ros2 --version || true
printenv | grep -E '^(ROS_|RMW_)' || true

section "workspace"
pwd
test -w "$ROOT_DIR" && echo "workspace_writable=yes" || echo "workspace_writable=no"

section "github"
git ls-remote https://github.com/1xavier1/robot_x5.git HEAD >/dev/null 2>&1 \
    && echo "github_access=yes" \
    || echo "github_access=no"
```

- [ ] **步骤 5：创建 check_ros_network.sh**

创建 `scripts/check_ros_network.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE_NAME="${PROFILE_NAME:-ros_network_check}"
cd "$ROOT_DIR"
source "$ROOT_DIR/scripts/env_profile.sh"

TIMEOUT_SEC="${TIMEOUT_SEC:-8}"
REQUIRED_TOPICS=(
  /sensing/lidar/points
  /sensing/imu/data
  /sensing/gps/fix
  /robot/odom
  /tf
  /clock
)

topics="$(timeout "$TIMEOUT_SEC" ros2 topic list || true)"
printf '%s\n' "$topics"

missing=0
for topic in "${REQUIRED_TOPICS[@]}"; do
    if ! grep -qx "$topic" <<<"$topics"; then
        printf '[network] missing topic: %s\n' "$topic" >&2
        missing=1
    fi
done

exit "$missing"
```

- [ ] **步骤 6：创建盘点记录模板**

创建 `docs/x5_environment_check.md`：

```markdown
# X5 环境盘点记录

## 使用方式

在 X5 上运行：

```bash
cd /home/sunrise/Workspace/robot
./scripts/check_x5_env.sh
```

## 记录规则

- 不记录密码。
- 不记录 SSH 命令中的账号和地址。
- 可以记录系统版本、架构、ROS 版本、磁盘和依赖缺口。

## 盘点结果

```text
将 check_x5_env.sh 输出粘贴到这里，删除敏感信息。
```
```

- [ ] **步骤 7：运行测试验证通过**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
chmod +x scripts/env_profile.sh scripts/check_x5_env.sh scripts/check_ros_network.sh
python3 -m pytest src/robot_description/test/test_x5_edge_scripts.py -q
```

预期：`1 passed`。

- [ ] **步骤 8：Commit**

运行：

```bash
git add scripts/env_profile.sh scripts/check_x5_env.sh scripts/check_ros_network.sh docs/x5_environment_check.md src/robot_description/test/test_x5_edge_scripts.py
git commit -m "feat(X5): 添加环境盘点与网络检查脚本"
```

## 任务 3：拆分 sim_host 与 x5_edge 启动入口

**文件：**
- 创建：`robot_x5/scripts/start_sim_host.sh`
- 创建：`robot_x5/scripts/start_x5_edge.sh`
- 创建：`robot_x5/launch/x5_edge.launch.py`
- 修改：`robot_x5/start.sh`
- 修改：`robot_x5/src/robot_description/test/test_x5_edge_scripts.py`

- [ ] **步骤 1：扩展失败测试**

追加到 `src/robot_description/test/test_x5_edge_scripts.py`：

```python
def test_profile_start_scripts_split_sim_host_and_x5_edge():
    sim_host = read("scripts/start_sim_host.sh")
    x5_edge = read("scripts/start_x5_edge.sh")
    launch = read("launch/x5_edge.launch.py")
    start = read("start.sh")

    assert "robot_simulation.launch.py" in sim_host
    assert "sensing_bridge:=true" in sim_host
    assert "navigation.launch.py" not in sim_host
    assert "fast_lio2.launch.py" not in sim_host

    assert "fast_lio2.launch.py" in x5_edge
    assert "x5_edge.launch.py" in x5_edge
    assert "robot_simulation.launch.py" not in x5_edge

    assert "navigation.launch.py" in launch
    assert "enable_task_navigation" in launch
    assert "x5_mapping_agent.py" in launch
    assert "DeclareLaunchArgument('map'" in launch

    assert "--profile" in start
    assert "sim_host" in start
    assert "x5_edge" in start
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_x5_edge_scripts.py -q
```

预期：FAIL，缺少启动脚本或 launch 文件。

- [ ] **步骤 3：创建 start_sim_host.sh**

创建 `scripts/start_sim_host.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
PROFILE_NAME=sim_host source "$ROOT_DIR/scripts/env_profile.sh"

GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"
WORLD="${WORLD:-worlds/cattle_barn.world}"
BUILD="${BUILD:-auto}"

if [[ "$BUILD" = "true" ]] || [[ ! -f install/setup.bash ]]; then
    colcon build --packages-select robot_description
    PROFILE_NAME=sim_host source "$ROOT_DIR/scripts/env_profile.sh"
fi

ros2 launch robot_description robot_simulation.launch.py \
    gui:="$GUI" \
    rviz:="$RVIZ" \
    use_sim_time:=true \
    world:="$ROOT_DIR/$WORLD" \
    sensing_bridge:=true
```

- [ ] **步骤 4：创建 x5_edge.launch.py**

创建 `launch/x5_edge.launch.py`：

```python
#!/usr/bin/env python3
"""Launch the X5 edge computation stack without Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml = LaunchConfiguration("map")
    task_map = LaunchConfiguration("task_map")
    agent_host = LaunchConfiguration("agent_host", default="0.0.0.0")
    agent_port = LaunchConfiguration("agent_port", default="8790")

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "fast_lio2.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "task_map": task_map,
            "enable_task_navigation": "true",
        }.items(),
    )
    agent = Node(
        package="robot_description",
        executable="x5_mapping_agent.py",
        name="x5_mapping_agent",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--host", agent_host,
            "--port", agent_port,
            "--maps-dir", "maps",
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(pkg_share, "maps", "barn_corridor_sim_001.yaml"),
        ),
        DeclareLaunchArgument(
            "task_map",
            default_value=os.path.join(pkg_share, "config", "task_map.example.yaml"),
        ),
        DeclareLaunchArgument("agent_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("agent_port", default_value="8790"),
        fast_lio,
        navigation,
        agent,
    ])
```

- [ ] **步骤 5：创建 start_x5_edge.sh**

创建 `scripts/start_x5_edge.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
PROFILE_NAME=x5_edge source "$ROOT_DIR/scripts/env_profile.sh"

BUILD="${BUILD:-auto}"
MAP="${MAP:-maps/barn_corridor_sim_001.yaml}"
TASK_MAP="${TASK_MAP:-maps/task_map.yaml}"
AGENT_HOST="${AGENT_HOST:-0.0.0.0}"
AGENT_PORT="${AGENT_PORT:-8790}"

if [[ "$BUILD" = "true" ]] || [[ ! -f install/setup.bash ]]; then
    colcon build --packages-select robot_description
    PROFILE_NAME=x5_edge source "$ROOT_DIR/scripts/env_profile.sh"
fi

ros2 launch robot_description x5_edge.launch.py \
    use_sim_time:=true \
    map:="$ROOT_DIR/$MAP" \
    task_map:="$ROOT_DIR/$TASK_MAP" \
    agent_host:="$AGENT_HOST" \
    agent_port:="$AGENT_PORT"
```

- [ ] **步骤 6：修改 start.sh 增加 profile 分发**

在 `start.sh` 参数解析前增加 `PROFILE=""`，参数解析中增加：

```bash
        --profile)
            PROFILE="$2"
            shift 2
            ;;
```

在原仿真启动逻辑前增加：

```bash
if [[ "${PROFILE:-}" = "sim_host" ]]; then
    exec "$SCRIPT_DIR/scripts/start_sim_host.sh"
fi
if [[ "${PROFILE:-}" = "x5_edge" ]]; then
    exec "$SCRIPT_DIR/scripts/start_x5_edge.sh"
fi
if [[ -n "${PROFILE:-}" ]]; then
    log_error "未知 profile: $PROFILE"
    exit 1
fi
```

- [ ] **步骤 7：运行测试和语法检查**

运行：

```bash
chmod +x scripts/start_sim_host.sh scripts/start_x5_edge.sh
python3 -m pytest src/robot_description/test/test_x5_edge_scripts.py -q
python3 -m py_compile launch/x5_edge.launch.py
bash -n start.sh scripts/start_sim_host.sh scripts/start_x5_edge.sh
```

预期：pytest 通过，`py_compile` 无输出，`bash -n` 无输出。

- [ ] **步骤 8：Commit**

运行：

```bash
git add start.sh scripts/start_sim_host.sh scripts/start_x5_edge.sh launch/x5_edge.launch.py src/robot_description/test/test_x5_edge_scripts.py
git commit -m "feat(X5): 拆分仿真端与主控端启动入口"
```

## 任务 4：实现 X5 Mapping Agent

**文件：**
- 创建：`robot_x5/scripts/x5_mapping_agent.py`
- 创建：`robot_x5/src/robot_description/test/test_x5_mapping_agent.py`
- 修改：`robot_x5/src/robot_description/CMakeLists.txt`

- [ ] **步骤 1：编写失败测试**

创建 `src/robot_description/test/test_x5_mapping_agent.py`：

```python
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
AGENT = ROOT / "scripts" / "x5_mapping_agent.py"


def request_json(url, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"content-type": "application/json"},
        method="POST" if body is not None else "GET",
    )
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode("utf-8"))


def test_agent_lifecycle_with_fake_mapper(tmp_path):
    mapper = tmp_path / "fake_mapper.py"
    mapper.write_text(
        "import json, signal, sys, time\n"
        "from pathlib import Path\n"
        "out=Path(sys.argv[1]); out.parent.mkdir(parents=True, exist_ok=True)\n"
        "stop=[]; signal.signal(signal.SIGINT, lambda *_: stop.append(True))\n"
        "i=0\n"
        "while not stop:\n"
        " i+=1\n"
        " (out.parent/(out.name+'_snapshot.pgm')).write_bytes(b'P5\\n2 2\\n255\\n'+bytes(4))\n"
        " print(json.dumps({'event':'progress','clouds':i,'poses':i,'snapshot_version':i}), flush=True)\n"
        " time.sleep(0.05)\n"
        "out.with_suffix('.pgm').write_bytes(b'P5\\n2 2\\n255\\n'+bytes(4))\n"
        "out.with_suffix('.yaml').write_text('image: job.pgm\\nresolution: 0.1\\norigin: [0,0,0]\\n')\n",
        encoding="utf-8",
    )
    maps = tmp_path / "maps"
    cmd = [
        sys.executable, str(AGENT),
        "--host", "127.0.0.1",
        "--port", "18790",
        "--maps-dir", str(maps),
        "--mapper-command", f"{sys.executable} {mapper} {{output}}",
    ]
    proc = subprocess.Popen(cmd)
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                if request_json("http://127.0.0.1:18790/health")["ok"]:
                    break
            except Exception:
                time.sleep(0.05)
        assert request_json("http://127.0.0.1:18790/api/mapping/start", {})["state"] == "RUNNING"
        assert request_json("http://127.0.0.1:18790/api/mapping/status")["state"] == "RUNNING"
        with urllib.request.urlopen("http://127.0.0.1:18790/api/mapping/preview", timeout=3) as resp:
            assert resp.read().startswith(b"P5")
        result = request_json("http://127.0.0.1:18790/api/mapping/stop", {"name": "x5_map"})
        assert result["state"] == "DONE"
        assert (maps / "x5_map.pgm").exists()
        assert (maps / "x5_map.yaml").exists()
        with urllib.request.urlopen("http://127.0.0.1:18790/api/maps/x5_map/bundle", timeout=3) as resp:
            assert resp.headers["content-type"] == "application/zip"
            assert resp.read().startswith(b"PK")
    finally:
        proc.terminate()
        proc.wait(timeout=5)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m pytest src/robot_description/test/test_x5_mapping_agent.py -q
```

预期：FAIL，报错显示 `x5_mapping_agent.py` 不存在或服务无法启动。

- [ ] **步骤 3：实现 x5_mapping_agent.py**

创建 `scripts/x5_mapping_agent.py`，使用 `ThreadingHTTPServer`。必须包含这些接口：

```python
ROUTES = {
    "GET /health": "return {'ok': True}",
    "GET /api/mapping/status": "return current status",
    "POST /api/mapping/start": "start mapper process",
    "POST /api/mapping/stop": "stop mapper, publish named map",
    "POST /api/mapping/cancel": "kill mapper and clean workdir",
    "GET /api/mapping/preview": "return current snapshot PGM bytes",
    "GET /api/maps/{name}/bundle": "return zip with pgm/yaml/pose/meta files",
}
```

核心状态字段与 remote 当前接口保持一致：

```python
{
    "state": "IDLE",
    "progress": {},
    "snapshot_version": 0,
    "error": "",
    "result_name": "",
}
```

`stop` 保存文件时必须生成：

```text
<name>.pgm
<name>.yaml
<name>_pose.json
<name>_meta.json
```

`mapper-command` 支持 `{output}` 占位，默认命令为：

```python
[
    "python3", "scripts/export_odom_projected_map.py",
    "--output", "<maps>/_mapping/job",
    "--duration-sec", "86400",
    "--snapshot-interval", "2.0",
    "--progress-jsonl",
]
```

- [ ] **步骤 4：确保脚本安装**

确认 `src/robot_description/CMakeLists.txt` 已安装 `${PARENT_DIR}/scripts/` 到 `lib/${PROJECT_NAME}`。如果任务 1 复制后的文件已经包含该规则，只运行检查：

```bash
rg -n "install\\(DIRECTORY \\$\\{PARENT_DIR\\}/scripts/" src/robot_description/CMakeLists.txt
```

预期：输出安装规则。

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
chmod +x scripts/x5_mapping_agent.py
python3 -m pytest src/robot_description/test/test_x5_mapping_agent.py -q
python3 -m py_compile scripts/x5_mapping_agent.py
```

预期：pytest 通过，`py_compile` 无输出。

- [ ] **步骤 6：Commit**

运行：

```bash
git add scripts/x5_mapping_agent.py src/robot_description/test/test_x5_mapping_agent.py src/robot_description/CMakeLists.txt
git commit -m "feat(X5): 添加车端建图代理"
```

## 任务 5：remote 增加 X5 建图客户端和后端选择

**文件：**
- 修改：`remote/.worktrees/glass-workbench/server/config.py`
- 创建：`remote/.worktrees/glass-workbench/server/services/edge_mapping_client.py`
- 创建：`remote/.worktrees/glass-workbench/server/services/remote_mapping_jobs.py`
- 修改：`remote/.worktrees/glass-workbench/server/main.py`
- 测试：`remote/.worktrees/glass-workbench/tests/test_config.py`
- 测试：`remote/.worktrees/glass-workbench/tests/test_edge_mapping_client.py`
- 测试：`remote/.worktrees/glass-workbench/tests/test_remote_mapping_jobs.py`

- [ ] **步骤 1：配置测试先失败**

扩展 `tests/test_config.py`：

```python
def test_mapping_backend_settings(monkeypatch):
    monkeypatch.setenv("MAPPING_BACKEND", "x5")
    monkeypatch.setenv("EDGE_AGENT_URL", "http://edge.local:8790")
    from server.config import load_settings

    settings = load_settings()
    assert settings.mapping_backend == "x5"
    assert settings.edge_agent_url == "http://edge.local:8790"
    assert settings.edge_request_timeout == 10.0
```

运行：

```bash
python3 -m pytest tests/test_config.py::test_mapping_backend_settings -q
```

预期：FAIL，`Settings` 缺少字段。

- [ ] **步骤 2：修改 server/config.py**

将 `Settings` 扩展为：

```python
@dataclass(frozen=True)
class Settings:
    robot_sim_dir: Path
    mapping_backend: str = "local"
    edge_agent_url: str = ""
    edge_request_timeout: float = 10.0
```

`load_settings()` 返回：

```python
return Settings(
    robot_sim_dir=Path(os.environ.get("ROBOT_SIM_DIR", default_dir)),
    mapping_backend=os.environ.get("MAPPING_BACKEND", "local"),
    edge_agent_url=os.environ.get("EDGE_AGENT_URL", ""),
    edge_request_timeout=float(os.environ.get("EDGE_REQUEST_TIMEOUT", "10.0")),
)
```

- [ ] **步骤 3：编写 EdgeMappingClient 测试**

创建 `tests/test_edge_mapping_client.py`：

```python
import io
import json
import zipfile

import pytest

from server.services.edge_mapping_client import EdgeAgentError, EdgeMappingClient


class FakeResponse:
    def __init__(self, body=b"{}", status=200, content_type="application/json"):
        self.body = body
        self.status = status
        self.headers = {"content-type": content_type}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return self.body


def test_edge_client_status_and_start(monkeypatch):
    calls = []

    def fake_urlopen(req, timeout):
        calls.append((req.full_url, req.get_method(), timeout))
        return FakeResponse(json.dumps({"state": "RUNNING"}).encode())

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    client = EdgeMappingClient("http://x5:8790", timeout=3)
    assert client.start()["state"] == "RUNNING"
    assert client.status()["state"] == "RUNNING"
    assert calls[0] == ("http://x5:8790/api/mapping/start", "POST", 3)


def test_edge_client_download_bundle(monkeypatch, tmp_path):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as zf:
        zf.writestr("m1.pgm", b"P5\n")
        zf.writestr("m1.yaml", "image: m1.pgm\n")

    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda req, timeout: FakeResponse(buffer.getvalue(), content_type="application/zip"),
    )
    client = EdgeMappingClient("http://x5:8790")
    files = client.download_bundle("m1", tmp_path)
    assert sorted(path.name for path in files) == ["m1.pgm", "m1.yaml"]


def test_edge_client_error_response(monkeypatch):
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda req, timeout: FakeResponse(b'{"error":"bad"}', status=500),
    )
    client = EdgeMappingClient("http://x5:8790")
    with pytest.raises(EdgeAgentError):
        client.status()
```

- [ ] **步骤 4：实现 edge_mapping_client.py**

创建 `server/services/edge_mapping_client.py`：

```python
class EdgeAgentError(RuntimeError):
    pass


class EdgeMappingClient:
    def __init__(self, base_url: str, timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def status(self) -> dict: ...
    def start(self) -> dict: ...
    def stop(self, name: str) -> dict: ...
    def cancel(self) -> dict: ...
    def preview_png(self) -> bytes: ...
    def download_bundle(self, name: str, target_dir: Path) -> list[Path]: ...
```

实现约束：

- 使用 `urllib.request`，不新增 HTTP 依赖。
- 非 2xx 响应抛 `EdgeAgentError`。
- `download_bundle()` 解压 zip 时拒绝绝对路径和包含 `..` 的成员名。

- [ ] **步骤 5：编写 RemoteMappingJobs 测试**

创建 `tests/test_remote_mapping_jobs.py`：

```python
from server.ros_bridge.gateway import RosEvent
from server.services.remote_mapping_jobs import RemoteMappingJobs


class FakeEdgeClient:
    def __init__(self):
        self.calls = []
        self.preview = b"\x89PNG"

    def status(self):
        return {"state": "IDLE", "progress": {}, "snapshot_version": 0, "error": "", "result_name": ""}

    def start(self):
        self.calls.append("start")
        return {"state": "RUNNING", "progress": {}, "snapshot_version": 0, "error": "", "result_name": ""}

    def stop(self, name):
        self.calls.append(f"stop:{name}")
        return {"state": "DONE", "progress": {}, "snapshot_version": 0, "error": "", "result_name": name}

    def cancel(self):
        self.calls.append("cancel")
        return {"state": "CANCELLED", "progress": {}, "snapshot_version": 0, "error": "", "result_name": ""}

    def preview_png(self):
        return self.preview

    def download_bundle(self, name, target_dir):
        (target_dir / f"{name}.pgm").write_bytes(b"P5\n")
        (target_dir / f"{name}.yaml").write_text(f"image: {name}.pgm\n", encoding="utf-8")
        return [target_dir / f"{name}.pgm", target_dir / f"{name}.yaml"]


def test_remote_mapping_jobs_delegates_and_syncs(settings, hub):
    client = FakeEdgeClient()
    jobs = RemoteMappingJobs(settings.maps_dir, hub, client)
    assert jobs.start()["state"] == "RUNNING"
    result = jobs.stop("edge_map")
    assert result["state"] == "DONE"
    assert client.calls == ["start", "stop:edge_map"]
    assert (settings.maps_dir / "edge_map.pgm").exists()
    assert hub.last("mapping_status")["state"] == "DONE"
    assert jobs.preview_png() == b"\x89PNG"
```

- [ ] **步骤 6：实现 remote_mapping_jobs.py**

创建 `server/services/remote_mapping_jobs.py`，对外方法与 `MappingJobs` 一致：

```python
class RemoteMappingJobs:
    def __init__(self, maps_dir, hub, edge_client):
        self._maps_dir = Path(maps_dir)
        self._hub = hub
        self._edge = edge_client

    def status(self) -> dict:
        return self._edge.status()

    def start(self) -> dict:
        status = self._edge.start()
        self._emit_status(status)
        return status

    def stop(self, name: str) -> dict:
        validate_map_name(name)
        if (self._maps_dir / f"{name}.yaml").exists():
            raise FileExistsError(f"map already exists: {name}")
        status = self._edge.stop(name)
        self._edge.download_bundle(name, self._maps_dir)
        self._emit_status(status)
        return status
```

`cancel()` 和 `preview_png()` 直接代理。`_emit_status()` 发布 `RosEvent("mapping_status", status)`。

- [ ] **步骤 7：修改 main.py 选择建图后端**

在 `server/main.py` 引入：

```python
from server.services.edge_mapping_client import EdgeMappingClient
from server.services.remote_mapping_jobs import RemoteMappingJobs
```

增加工厂函数：

```python
def create_mapping_jobs(settings: Settings, hub: RealtimeHub):
    if settings.mapping_backend == "x5":
        if not settings.edge_agent_url:
            raise RuntimeError("EDGE_AGENT_URL is required when MAPPING_BACKEND=x5")
        return RemoteMappingJobs(
            settings.maps_dir,
            hub,
            EdgeMappingClient(settings.edge_agent_url, settings.edge_request_timeout),
        )
    return MappingJobs(settings.maps_dir, hub)
```

替换：

```python
app.state.mapping_jobs = create_mapping_jobs(settings, hub)
```

- [ ] **步骤 8：运行 remote 后端测试**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
python3 -m pytest tests/test_config.py tests/test_edge_mapping_client.py tests/test_remote_mapping_jobs.py tests/test_api_mapping.py -q
```

预期：全部通过。

- [ ] **步骤 9：Commit**

运行：

```bash
git add server/config.py server/main.py server/services/edge_mapping_client.py server/services/remote_mapping_jobs.py tests/test_config.py tests/test_edge_mapping_client.py tests/test_remote_mapping_jobs.py tests/test_api_mapping.py
git commit -m "feat(建图): 支持代理到 X5 车端建图"
```

## 任务 6：保持 remote 页面建图接口不变并补充前端验证

**文件：**
- 修改：`remote/.worktrees/glass-workbench/web/src/stores/mapping.js`
- 修改：`remote/.worktrees/glass-workbench/web/src/views/MappingView.vue`
- 测试：`remote/.worktrees/glass-workbench/web/tests/mapping.test.js`

- [ ] **步骤 1：确认现有前端是否已经只依赖 `/api/mapping/*`**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
rg -n "mapping|/api/mapping|preview" web/src web/tests
```

预期：建图页面只调用 remote 后端 API，不直接访问本地文件路径。

- [ ] **步骤 2：编写前端测试**

如果还没有 mapping store 测试，创建 `web/tests/mapping.test.js`：

```javascript
import { describe, expect, it, vi } from 'vitest'
import { startMapping, stopMapping, cancelMapping, getMappingStatus } from '../src/lib/api.js'

describe('mapping API client', () => {
  it('uses stable remote mapping endpoints', async () => {
    const calls = []
    global.fetch = vi.fn(async (url, options = {}) => {
      calls.push([url, options.method || 'GET'])
      return {
        ok: true,
        headers: { get: () => 'application/json' },
        json: async () => ({ state: 'RUNNING' }),
      }
    })

    await getMappingStatus()
    await startMapping()
    await stopMapping('edge_map')
    await cancelMapping()

    expect(calls).toEqual([
      ['/api/mapping/status', 'GET'],
      ['/api/mapping/start', 'POST'],
      ['/api/mapping/stop', 'POST'],
      ['/api/mapping/cancel', 'POST'],
    ])
  })
})
```

- [ ] **步骤 3：修改前端显示同步失败错误**

在 `MappingView.vue` 的错误提示逻辑中确保显示 API 返回的 `message` 和 `code`。如果当前已经显示，则只保留测试。错误对象应渲染为：

```javascript
const message = error?.body?.message || error?.message || '建图操作失败'
```

- [ ] **步骤 4：运行前端测试和构建**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench/web
npx vitest run
npm run build
```

预期：Vitest 通过，Vite build 通过。

- [ ] **步骤 5：Commit**

运行：

```bash
git add web/src/lib/api.js web/src/views/MappingView.vue web/tests/mapping.test.js
git commit -m "test(建图): 固定 remote 建图页面 API 契约"
```

## 任务 7：本机与 X5 只读环境盘点

**文件：**
- 修改：`robot_x5/docs/x5_environment_check.md`

- [ ] **步骤 1：本机准备 SSH 目标环境变量**

在当前终端设置，不写入仓库：

```bash
export X5_SSH_TARGET='user@host'
```

实际值由操作者在终端输入。不要把账号、地址或密码写入提交。

- [ ] **步骤 2：只读检查 X5 基础环境**

运行：

```bash
ssh "$X5_SSH_TARGET" 'bash -lc "
set -e
uname -a
lsb_release -a 2>/dev/null || cat /etc/os-release
lscpu | sed -n \"1,20p\"
df -h \$HOME
command -v git || true
git --version || true
command -v python3 || true
python3 --version || true
command -v colcon || true
command -v ros2 || true
ros2 --version || true
printenv | grep -E \"^(ROS_|RMW_)\" || true
"'
```

预期：命令退出码为 0。输出中允许某些工具不存在；缺失项写入盘点记录。

- [ ] **步骤 3：记录脱敏盘点结果**

将步骤 2 的输出整理到 `docs/x5_environment_check.md`。删除账号、地址和任何密钥路径，只保留：

```markdown
## 盘点结果

- OS:
- 架构:
- Python:
- ROS 2:
- colcon:
- git:
- 磁盘:
- 依赖缺口:
```

- [ ] **步骤 4：Commit**

运行：

```bash
git add docs/x5_environment_check.md
git commit -m "docs(X5): 记录主控环境盘点结果"
```

## 任务 8：部署到 X5 并验证 ROS 多机通信

**文件：**
- 修改：`robot_x5/docs/x5_deployment.md`
- 修改：`ros2_robot_sim/docs/CLAUDE_CODE_HANDOFF.md`

- [ ] **步骤 1：创建部署文档**

在 `robot_x5/docs/x5_deployment.md` 写入：

```markdown
# X5 第一阶段部署说明

## 本机 sim_host

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
./scripts/start_sim_host.sh
```

## X5 x5_edge

```bash
cd /home/sunrise/Workspace/robot
./scripts/start_x5_edge.sh
```

## Remote 使用 X5 建图

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
MAPPING_BACKEND=x5 EDGE_AGENT_URL=http://X5_HOST:8790 ./start.sh --host 0.0.0.0 --port 8765
```

`X5_HOST` 在本地 shell 中替换，不提交到仓库。
```

- [ ] **步骤 2：推送 robot_x5 仓库**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
git push -u origin main
```

预期：推送成功。

- [ ] **步骤 3：在 X5 上 clone 或更新代码**

运行：

```bash
ssh "$X5_SSH_TARGET" 'bash -lc "
mkdir -p /home/sunrise/Workspace
cd /home/sunrise/Workspace
if [ -d robot/.git ]; then
  cd robot && git pull --ff-only
else
  git clone https://github.com/1xavier1/robot_x5.git robot
fi
"'
```

预期：`/home/sunrise/Workspace/robot` 是 Git 仓库。

- [ ] **步骤 4：X5 构建**

运行：

```bash
ssh "$X5_SSH_TARGET" 'bash -lc "
cd /home/sunrise/Workspace/robot
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description
"'
```

预期：`Summary: 1 package finished`。

- [ ] **步骤 5：启动本机 sim_host**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
./scripts/start_sim_host.sh
```

预期：Gazebo 启动，`/sensing/lidar/points`、`/robot/odom`、`/clock` 发布。

- [ ] **步骤 6：X5 运行网络检查**

运行：

```bash
ssh "$X5_SSH_TARGET" 'bash -lc "
cd /home/sunrise/Workspace/robot
./scripts/check_ros_network.sh
"'
```

预期：脚本退出码为 0，列出必需仿真话题。

- [ ] **步骤 7：X5 启动 x5_edge**

运行：

```bash
ssh "$X5_SSH_TARGET" 'bash -lc "
cd /home/sunrise/Workspace/robot
./scripts/start_x5_edge.sh
"'
```

预期：`x5_mapping_agent`、Nav2、定位节点启动。

- [ ] **步骤 8：本机确认 X5 发布主控话题**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 10s ros2 topic list
```

预期：输出包含 `/localization/global_odom`、`/task/status`、`/plan`、`/robot/cmd_vel`。

- [ ] **步骤 9：更新交接文档**

在 `ros2_robot_sim/docs/CLAUDE_CODE_HANDOFF.md` 增加 X5 第一阶段状态：

```markdown
### 2026-06-17：X5 主控混合仿真迁移

- 规格：`docs/superpowers/specs/2026-06-17-x5-edge-hybrid-migration-design.md`
- 计划：`docs/superpowers/plans/2026-06-17-x5-edge-hybrid-migration.md`
- robot_x5 仓库：`https://github.com/1xavier1/robot_x5.git`
- 第一阶段采用本机 sim_host + X5 x5_edge。
- remote 建图通过 X5 edge agent 执行，X5 地图为运行权威。
```

- [ ] **步骤 10：Commit 文档**

在相关仓库分别提交：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
git add docs/x5_deployment.md
git commit -m "docs(X5): 添加第一阶段部署说明"

cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
git add docs/CLAUDE_CODE_HANDOFF.md
git commit -m "docs(交接): 记录 X5 混合仿真迁移状态"
```

## 任务 9：Remote 页面触发 X5 建图验收

**文件：**
- 修改：`remote/.worktrees/glass-workbench/docs/acceptance-checklist.md`
- 修改：`ros2_robot_sim/docs/CLAUDE_CODE_HANDOFF.md`

- [ ] **步骤 1：启动 remote 的 X5 建图模式**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
MAPPING_BACKEND=x5 EDGE_AGENT_URL=http://X5_HOST:8790 ./start.sh --host 0.0.0.0 --port 8765
```

`X5_HOST` 在 shell 中替换，不提交到仓库。

- [ ] **步骤 2：API 冒烟**

运行：

```bash
curl -s http://localhost:8765/api/mapping/status
curl -s -X POST http://localhost:8765/api/mapping/start
sleep 3
curl -s http://localhost:8765/api/mapping/status
curl -s -o /tmp/x5_mapping_preview.png http://localhost:8765/api/mapping/preview
curl -s -X POST http://localhost:8765/api/mapping/stop \
  -H 'content-type: application/json' \
  -d '{"name":"x5_remote_smoke"}'
```

预期：

- `start` 返回 `state=RUNNING`。
- `preview` 生成非空 PNG。
- `stop` 返回 `state=DONE`。
- 本机 remote maps 目录出现 `x5_remote_smoke.pgm` 和 `x5_remote_smoke.yaml`。
- X5 maps 目录也出现同名地图。

- [ ] **步骤 3：页面人工验收**

浏览器打开：

```text
http://localhost:8765
```

在建图页完成：

```text
开始建图 -> 看到进度 -> 看到预览 -> 保存地图 -> 地图库出现新地图
```

- [ ] **步骤 4：记录验收清单**

在 `remote/.worktrees/glass-workbench/docs/acceptance-checklist.md` 增加：

```markdown
## X5 建图代理验收

- [ ] remote 以 `MAPPING_BACKEND=x5` 启动。
- [ ] `/api/mapping/start` 启动 X5 建图任务。
- [ ] `/api/mapping/preview` 返回预览图。
- [ ] `/api/mapping/stop` 在 X5 和本机各保存一份地图。
- [ ] 页面建图流程完整可用。
```

- [ ] **步骤 5：Commit**

运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
git add docs/acceptance-checklist.md
git commit -m "docs(验收): 添加 X5 建图代理检查项"
```

## 总体验证

完成全部任务后运行：

```bash
cd /home/xavier/Workspace/ClaudeSpace/robot_x5
python3 -m pytest src/robot_description/test/test_x5_edge_scripts.py src/robot_description/test/test_x5_mapping_agent.py -q
python3 -m py_compile scripts/x5_mapping_agent.py launch/x5_edge.launch.py
bash -n scripts/env_profile.sh scripts/check_x5_env.sh scripts/check_ros_network.sh scripts/start_sim_host.sh scripts/start_x5_edge.sh start.sh
colcon build --packages-select robot_description

cd /home/xavier/Workspace/ClaudeSpace/remote/.worktrees/glass-workbench
python3 -m pytest tests/test_config.py tests/test_edge_mapping_client.py tests/test_remote_mapping_jobs.py tests/test_api_mapping.py -q
cd web && npx vitest run && npm run build
```

预期：

- `robot_x5` pytest 通过。
- Python 编译和 shell 语法检查无输出。
- `robot_x5` colcon build 成功。
- remote 后端测试通过。
- remote 前端测试和构建成功。

## 计划自检

- 规格中的 X5 主控、remote 建图全流程、地图双写、只读盘点、多机 ROS 通信和本机仿真不破坏均有对应任务。
- 任务中没有未完成章节、占位实现或敏感连接信息。
- `MappingJobs` 与 `RemoteMappingJobs` 对外接口一致：`status()`、`start()`、`stop(name)`、`cancel()`、`preview_png()`。
- 地图同步规则在 X5 agent、remote client、验收步骤中保持一致：X5 为运行权威，本机 remote 为同步副本。
