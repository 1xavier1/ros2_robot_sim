#!/bin/bash
# ============================================================
# ROS2 四轮差速驱动机器人仿真 — 停止脚本
# 用法: ./stop.sh [-f|--force]
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_FILE="$SCRIPT_DIR/.sim_pid"
ROS2_SOURCE="/opt/ros/humble/setup.bash"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[!]${NC} $1"; }
log_step()  { echo -e "${CYAN}[*]${NC} $1"; }

FORCE=false
[ "$1" = "-f" ] || [ "$1" = "--force" ] && FORCE=true

KILL_SIG="-TERM"
[ "$FORCE" = "true" ] && KILL_SIG="-KILL"

STOPPED_ANY=false

# --- 1. 通过 PID 文件停止 ros2 launch 主进程 ---
if [ -f "$PID_FILE" ]; then
    PID=$(cat "$PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        log_step "停止仿真主进程 (PID: $PID)..."
        # 先杀进程组
        kill -TERM -- -$PID 2>/dev/null || true
        sleep 2
        # 如果还活着，强制杀
        kill -0 "$PID" 2>/dev/null && kill -KILL -- -$PID 2>/dev/null || true
        STOPPED_ANY=true
        log_info "仿真主进程已停止"
    else
        log_warn "PID 文件中的进程 ($PID) 已不存在"
    fi
    rm -f "$PID_FILE"
fi

# --- 2. 按进程名清理 ---
PROCS=(
    "gzserver"
    "gzclient"
    "gz sim"
    "rviz2"
    "robot_state_publisher"
    "spawn_entity.py"
)

for proc in "${PROCS[@]}"; do
    # 精确匹配和模糊匹配都尝试
    PIDS=$(pgrep -f "$proc" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        log_step "停止: $proc (PID: $(echo $PIDS | tr '\n' ' '))"
        echo "$PIDS" | xargs kill $KILL_SIG 2>/dev/null || true
        STOPPED_ANY=true
    fi
done

# 等进程退出
sleep 1

# 二次检查，强制杀残留
STILL_ALIVE=$(pgrep -f "gzserver\|gzclient\|rviz2" 2>/dev/null || true)
if [ -n "$STILL_ALIVE" ]; then
    log_step "强制终止顽固进程..."
    echo "$STILL_ALIVE" | xargs kill -9 2>/dev/null || true
    sleep 0.5
fi

# --- 3. 停止 ROS2 daemon ---
if [ -f "$ROS2_SOURCE" ]; then
    source "$ROS2_SOURCE" 2>/dev/null
    ros2 daemon stop 2>/dev/null && log_info "ROS2 daemon 已停止" || true
fi

# --- 4. 释放 Gazebo 端口 ---
if fuser 11345/tcp &>/dev/null 2>&1; then
    log_step "释放 Gazebo master 端口 11345..."
    fuser -k 11345/tcp 2>/dev/null || true
    STOPPED_ANY=true
    log_info "Gazebo master 端口已释放"
fi

# --- 5. 清理锁文件 ---
for lock in /tmp/gazebo-*; do
    if [ -e "$lock" ]; then
        rm -rf "$lock" 2>/dev/null
        log_info "已清理锁文件: $lock"
    fi
done

# --- 6. 总结 ---
echo ""
if [ "$STOPPED_ANY" = "true" ]; then
    log_info "仿真系统已完全停止"
else
    log_warn "未发现正在运行的仿真进程"
fi
