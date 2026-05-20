#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE_DIR/log/ros}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

OUTPUT="$(timeout 15s ros2 launch robot_description navigation.launch.py 2>&1 || true)"
echo "$OUTPUT"

if echo "$OUTPUT" | grep -q "Navigation2 precheck failed"; then
    echo "navigation precheck reported missing Nav2 dependencies"
    exit 0
fi

if timeout 8s ros2 node list | grep -Eq "controller_server|bt_navigator"; then
    echo "navigation nodes are present"
    exit 0
fi

echo "navigation precheck did not report missing dependencies and Nav2 nodes were not found" >&2
exit 1
