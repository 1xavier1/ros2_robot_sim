#!/bin/bash
# ============================================================
# ROS2 四轮差速驱动机器人仿真 — 启动脚本
# 用法: ./start.sh [-h|--help] [--no-rviz] [--no-build]
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_SOURCE="/opt/ros/humble/setup.bash"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[!]${NC} $1"; }
log_error() { echo -e "${RED}[✗]${NC} $1"; }
log_step()  { echo -e "${CYAN}[*]${NC} $1"; }

usage() {
    echo "用法: ./start.sh [选项]"
    echo ""
    echo "选项:"
    echo "  --no-rviz     不启动 RViz2 可视化"
    echo "  --no-build    跳过 colcon build"
    echo "  -h, --help    显示帮助"
    exit 0
}

# --- 参数解析 ---
NO_RVIZ="false"
NO_BUILD="false"
ROS_ARGS=""

for arg in "$@"; do
    case $arg in
        --no-rviz)  NO_RVIZ="true";  ROS_ARGS="$ROS_ARGS rviz:=false" ;;
        --no-build) NO_BUILD="true" ;;
        -h|--help)  usage ;;
    esac
done

echo "================================================"
echo "  ROS2 四轮差速驱动机器人仿真系统"
echo "  4-Wheel Differential Drive Robot Simulator"
echo "================================================"
echo ""

# --- 1. 预清理：确保没有残留进程干扰 ---
log_step "检查残留进程..."
STALE=$(pgrep -f "gzserver\|gzclient\|rviz2\|robot_state_publisher" 2>/dev/null || true)
if [ -n "$STALE" ]; then
    log_warn "发现残留仿真进程，正在清理..."
    "$SCRIPT_DIR/stop.sh" -f
    sleep 2
    # 二次确认
    if pgrep -f "gzserver" &>/dev/null; then
        log_warn "强制终止 gzserver..."
        pkill -9 -f "gzserver" 2>/dev/null || true
    fi
    # 停止 ROS2 daemon
    source "$ROS2_SOURCE" 2>/dev/null && ros2 daemon stop 2>/dev/null || true
    log_info "残留进程已清理"
else
    log_info "无残留进程"
fi

# 清理锁文件
rm -f /tmp/gazebo-* 2>/dev/null || true

# --- 2. 加载 ROS2 环境 ---
if [ -f "$ROS2_SOURCE" ]; then
    log_step "加载 ROS2 Humble 环境..."
    source "$ROS2_SOURCE"
else
    log_error "未找到 $ROS2_SOURCE"
    log_error "请安装 ROS2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# --- 3. 设置 Gazebo 环境变量 ---
GAZEBO_MODEL_PATHS=(
    "$SCRIPT_DIR/src/robot_description/models"
    "$SCRIPT_DIR/install/robot_description/share/robot_description/models"
    "$HOME/.gazebo/models"
    "/usr/share/gazebo-11/models"
)
for p in "${GAZEBO_MODEL_PATHS[@]}"; do
    if [ -d "$p" ]; then
        export GAZEBO_MODEL_PATH="$p${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
    fi
done

# 设置插件路径
PLUGIN_LIB="$SCRIPT_DIR/install/robot_description/lib/robot_description"
if [ -d "$PLUGIN_LIB" ]; then
    export GAZEBO_PLUGIN_PATH="$PLUGIN_LIB${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
fi

# --- 4. 构建（如需要） ---
if [ "$NO_BUILD" = "false" ]; then
    if [ ! -d "$SCRIPT_DIR/install/robot_description" ] || [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
        log_step "需要构建，正在 colcon build..."
        cd "$SCRIPT_DIR"
        colcon build --packages-select robot_description
        if [ $? -ne 0 ]; then
            log_error "构建失败，请检查编译错误"
            exit 1
        fi
        log_info "构建完成"
    else
        log_info "已构建，跳过编译 (使用 --no-build 跳过检查)"
    fi
else
    log_warn "跳过构建步骤"
fi

# --- 5. 加载项目环境 ---
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    log_step "加载项目 install 环境..."
    source "$SCRIPT_DIR/install/setup.bash"
else
    log_error "未找到 install/setup.bash，请先构建: colcon build --packages-select robot_description"
    exit 1
fi

# --- 6. 启动仿真 ---
echo ""
log_step "启动仿真..."
log_info "  世界文件: corridor_tunnel.world"
log_info "  机器人:   ackermann_robot"
if [ "$NO_RVIZ" = "false" ]; then
    log_info "  RViz2:    已启用"
else
    log_info "  RViz2:    已禁用"
fi
echo ""
log_warn "Gazebo 和 RViz2 窗口将打开，按 Ctrl+C 停止"
echo ""

# 信号处理 — 确保彻底清理
cleanup() {
    echo ""
    log_step "正在关闭仿真系统..."
    # 先杀 ros2 launch 进程组
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -TERM -- -$LAUNCH_PID 2>/dev/null || true
        sleep 2
        kill -KILL -- -$LAUNCH_PID 2>/dev/null || true
    fi
    # 再按名称清理
    for p in gzserver gzclient rviz2 robot_state_publisher; do
        pkill -f "$p" 2>/dev/null || true
    done
    # 释放 Gazebo master 端口
    fuser -k 11345/tcp 2>/dev/null || true
    # 清理锁文件
    rm -f /tmp/gazebo-* 2>/dev/null || true
    log_info "仿真系统已关闭"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 前台运行 — 用户直接看输出，Ctrl+C 停止
cd "$SCRIPT_DIR"
ros2 launch robot_description robot_simulation.launch.py $ROS_ARGS &
LAUNCH_PID=$!

# 等待子进程
wait $LAUNCH_PID
