#!/usr/bin/env bash
set -euo pipefail

# Simple full-stack starter for manual validation.
# Edit these defaults directly, or override them from the shell:
#   GUI=false MAP=maps/my_map.yaml ./scripts/start_full_stack.sh
GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
ENABLE_TASK_NAVIGATION="${ENABLE_TASK_NAVIGATION:-true}"

MAP="${MAP:-maps/barn_corridor_sim_001.yaml}"
RUNTIME_TASK_MAP="${RUNTIME_TASK_MAP:-maps/task_map.yaml}"
TASK_MAP="${TASK_MAP:-$RUNTIME_TASK_MAP}"

SIM_WAIT_SECONDS="${SIM_WAIT_SECONDS:-8}"
FAST_LIO_WAIT_SECONDS="${FAST_LIO_WAIT_SECONDS:-6}"
BUILD="${BUILD:-auto}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/log/full_stack}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

PIDS=()
CLEANED_UP=false

log() {
    printf '[full-stack] %s\n' "$*"
}

resolve_path() {
    local path="$1"
    if [[ "$path" = /* ]]; then
        printf '%s\n' "$path"
    else
        printf '%s/%s\n' "$ROOT_DIR" "$path"
    fi
}

source_setup() {
    local setup_file="$1"
    set +u
    # ROS setup scripts read optional environment variables such as
    # AMENT_TRACE_SETUP_FILES, so nounset must be disabled while sourcing.
    source "$setup_file"
    set -u
}

start_launch() {
    local name="$1"
    shift
    local log_file="$LOG_DIR/${name}.log"

    log "starting $name"
    log "  log: $log_file"
    (
        cd "$ROOT_DIR"
        ros2 launch "$@"
    ) >"$log_file" 2>&1 &
    PIDS+=("$!")
}

install_is_stale() {
    local relative_path
    local install_share="$ROOT_DIR/install/robot_description/share/robot_description"
    for relative_path in \
        config/navigation.yaml \
        launch/navigation.launch.py \
        scripts/task_executor.py \
        scripts/task_map_core.py \
        scripts/route_recorder.py; do
        if [[ ! -f "$install_share/$relative_path" ]]; then
            return 0
        fi
        if [[ "$ROOT_DIR/$relative_path" -nt "$install_share/$relative_path" ]]; then
            return 0
        fi
    done
    return 1
}

cleanup() {
    if [[ "$CLEANED_UP" = "true" ]]; then
        return
    fi
    CLEANED_UP=true

    log "stopping launched processes"
    for ((idx=${#PIDS[@]}-1; idx>=0; idx--)); do
        pid="${PIDS[$idx]}"
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    sleep 2

    for ((idx=${#PIDS[@]}-1; idx>=0; idx--)); do
        pid="${PIDS[$idx]}"
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup SIGINT SIGTERM EXIT

if [[ ! -f "$ROS_SETUP" ]]; then
    log "missing ROS setup: $ROS_SETUP"
    exit 1
fi

source_setup "$ROS_SETUP"

if [[ "$BUILD" = "true" ]] || { [[ "$BUILD" = "auto" ]] && install_is_stale; }; then
    log "building robot_description"
    cd "$ROOT_DIR"
    colcon build --packages-select robot_description
fi

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
    log "missing install/setup.bash; run: colcon build --packages-select robot_description"
    exit 1
fi

source_setup "$ROOT_DIR/install/setup.bash"

mkdir -p "$LOG_DIR"

MAP="$(resolve_path "$MAP")"
TASK_MAP="$(resolve_path "$TASK_MAP")"
if [[ ! -f "$TASK_MAP" ]]; then
    log "runtime task map missing, creating from config/task_map.example.yaml"
    mkdir -p "$(dirname "$TASK_MAP")"
    cp "$ROOT_DIR/config/task_map.example.yaml" "$TASK_MAP"
fi

log "configuration"
log "  GUI=$GUI"
log "  RVIZ=$RVIZ"
log "  USE_SIM_TIME=$USE_SIM_TIME"
log "  ENABLE_TASK_NAVIGATION=$ENABLE_TASK_NAVIGATION"
log "  MAP=$MAP"
log "  TASK_MAP=$TASK_MAP"
log "  LOG_DIR=$LOG_DIR"

start_launch simulation \
    robot_description robot_simulation.launch.py \
    gui:="$GUI" \
    rviz:="$RVIZ" \
    use_sim_time:="$USE_SIM_TIME" \
    sensing_bridge:=true

log "waiting ${SIM_WAIT_SECONDS}s for simulation and sensing topics"
sleep "$SIM_WAIT_SECONDS"

start_launch fast_lio2 \
    robot_description fast_lio2.launch.py \
    use_sim_time:="$USE_SIM_TIME"

log "waiting ${FAST_LIO_WAIT_SECONDS}s for FAST-LIO2"
sleep "$FAST_LIO_WAIT_SECONDS"

start_launch navigation \
    robot_description navigation.launch.py \
    use_sim_time:="$USE_SIM_TIME" \
    map:="$MAP" \
    enable_task_navigation:=$ENABLE_TASK_NAVIGATION \
    task_map:="$TASK_MAP"

log "started. Useful checks:"
log "  ros2 topic echo /localization/wheel_lio_status"
log "  ros2 topic echo /task/status"
log "  ros2 topic echo /task/current_goal"
log "  cd ../remote && ./start.sh"
log "press Ctrl+C here to stop this full stack"

wait
