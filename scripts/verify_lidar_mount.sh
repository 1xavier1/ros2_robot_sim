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

if ! timeout 8s ros2 run tf2_ros tf2_echo base_link laser_link >/dev/null; then
    echo "missing TF: base_link -> laser_link" >&2
    exit 1
fi

if ! timeout 8s ros2 topic echo /sensing/lidar/points_filtered --once >/dev/null; then
    echo "no filtered LiDAR point cloud received" >&2
    exit 1
fi

FRAME_ID="$(timeout 8s ros2 topic echo /sensing/lidar/points_filtered --once --field header.frame_id || true)"
if [ -z "$FRAME_ID" ]; then
    echo "filtered LiDAR frame_id is empty" >&2
    exit 1
fi

echo "lidar mount verification passed"
