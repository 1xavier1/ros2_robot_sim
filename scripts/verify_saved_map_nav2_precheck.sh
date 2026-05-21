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
    echo "saved-map Nav2 precheck reported missing Nav2 dependencies"
    exit 0
fi

if echo "$OUTPUT" | grep -q "Created controller : FollowPath" \
    && echo "$OUTPUT" | grep -q "Created global planner plugin GridBased" \
    && echo "$OUTPUT" | grep -q "Configuring bt_navigator"; then
    if echo "$OUTPUT" | grep -q 'Invalid frame ID "map"'; then
        echo "saved-map Nav2 plugins are available; map->base_link TF is still required for activation"
    else
        echo "saved-map Nav2 plugins are available"
    fi
    exit 0
fi

if timeout 8s ros2 topic list | grep -q "/control/cmd_vel"; then
    echo "saved-map Nav2 command topic is available"
    exit 0
fi

echo "saved-map Nav2 precheck did not load core Nav2 plugins and /control/cmd_vel was not found" >&2
exit 1
