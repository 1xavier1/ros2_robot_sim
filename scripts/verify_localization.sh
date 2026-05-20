#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_SOURCE="/opt/ros/humble/setup.bash"
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source "$ROS2_SOURCE"
source "$SETUP_FILE"
set -u

echo "starting localization.launch.py"
timeout 20s ros2 launch robot_description localization.launch.py &
LAUNCH_PID=$!

cleanup() {
    kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 4

if ! timeout 8s ros2 topic echo /odometry/filtered --once >/dev/null; then
    echo "no message received on /odometry/filtered" >&2
    exit 1
fi

echo "localization verification passed"
