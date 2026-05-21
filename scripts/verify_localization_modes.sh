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

if ! timeout 8s ros2 topic echo /localization/mode --once >/dev/null; then
    echo "no localization mode received" >&2
    exit 1
fi

if ! timeout 8s ros2 topic echo /localization/fusion_weights --once >/dev/null; then
    echo "no localization fusion weights received" >&2
    exit 1
fi

echo "localization mode verification passed"
