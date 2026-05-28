#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-8s}"
TF_TIMEOUT="${TF_TIMEOUT:-8s}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

check_topic_once() {
    local topic="$1"
    local label="$2"
    if ! timeout "$TOPIC_TIMEOUT" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        echo "no message received on $topic ($label)" >&2
        exit 1
    fi
    echo "ok: $topic ($label)"
}

check_topic_once "/mapping/lio/odom" "FAST-LIO odometry"
check_topic_once "/robot/odom" "wheel odometry"
check_topic_once "/localization/fused_odom" "FAST-LIO + wheel/GPS fused odom"
check_topic_once "/localization/global_odom" "global localization backend odom"
check_topic_once "/localization/fusion_status" "fusion input status"
check_topic_once "/localization/backend_status" "global backend status"
check_topic_once "/localization/loop_closure_status" "loop closure status"

TF_OUTPUT="$(
    timeout "$TF_TIMEOUT" ros2 run tf2_ros tf2_echo map base_link \
        --ros-args -p use_sim_time:=true 2>&1 || true
)"
echo "$TF_OUTPUT"
if ! grep -Eq "At time|Translation:" <<< "$TF_OUTPUT"; then
    echo "map -> base_link TF is not available" >&2
    exit 1
fi

if grep -R "Robot is out of bounds of the costmap" "$ROS_LOG_DIR" >/dev/null 2>&1; then
    echo "Nav2 reported: Robot is out of bounds of the costmap" >&2
    exit 1
fi

echo "global localization runtime verification passed"
