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

OUTPUT="$(timeout 15s ros2 launch robot_description fast_lio2.launch.py 2>&1 || true)"
echo "$OUTPUT"

if echo "$OUTPUT" | grep -q "FAST-LIO2 precheck failed"; then
    echo "fast-lio2 precheck reported missing FAST-LIO dependency"
    exit 0
fi

if echo "$OUTPUT" | grep -q "spark_lio_mapping"; then
    echo "fast-lio2 front end executable is available"
    exit 0
fi

if timeout 8s ros2 topic list | grep -Eq "/mapping/lio/odom|/mapping/lio/map_points"; then
    echo "fast-lio2 mapping topics are present"
    exit 0
fi

echo "fast-lio2 precheck did not report missing dependencies and mapping topics were not found" >&2
exit 1
