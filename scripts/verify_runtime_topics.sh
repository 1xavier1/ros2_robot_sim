#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_SOURCE="/opt/ros/humble/setup.bash"
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

if [ ! -f "$ROS2_SOURCE" ]; then
    echo "missing ROS2 setup: $ROS2_SOURCE" >&2
    exit 1
fi

set +u
source "$ROS2_SOURCE"
set -u

if [ -f "$SETUP_FILE" ]; then
    set +u
    source "$SETUP_FILE"
    set -u
else
    echo "missing workspace setup: $SETUP_FILE" >&2
    echo "run: colcon build --packages-select robot_description" >&2
    exit 1
fi

require_topic() {
    local topic="$1"
    if ! timeout 8s ros2 topic list | grep -Fxq "$topic"; then
        echo "missing topic: $topic" >&2
        exit 1
    fi
    echo "topic ok: $topic"
}

require_echo_once() {
    local topic="$1"
    if ! timeout 8s ros2 topic echo "$topic" --once >/dev/null; then
        echo "no message received on: $topic" >&2
        exit 1
    fi
    echo "message ok: $topic"
}

require_topic "/tf"
require_topic "/robot/odom"
require_topic "/robot/ground_truth/odom"
require_topic "/robot/imu/data"
require_topic "/robot/velodyne_points"
require_topic "/robot/wheel_encoder/rear_average"
require_topic "/sensing/lidar/points"
require_topic "/sensing/imu/data"
require_topic "/sensing/wheel/speed"
require_topic "/sensing/gps/fix"

require_echo_once "/tf"
require_echo_once "/robot/odom"
require_echo_once "/robot/ground_truth/odom"
require_echo_once "/robot/imu/data"
require_echo_once "/robot/velodyne_points"
require_echo_once "/robot/wheel_encoder/rear_average"
require_echo_once "/sensing/lidar/points"
require_echo_once "/sensing/imu/data"
require_echo_once "/sensing/wheel/speed"
require_echo_once "/sensing/gps/fix"

echo "runtime topic verification passed"
