#!/bin/bash
# Install a ROS 2 compatible FAST-LIO front end into this workspace.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# FAST_LIO2_REPO_URL: ROS 2 compatible FAST-LIO repository URL.
FAST_LIO2_REPO_URL="${FAST_LIO2_REPO_URL:-https://github.com/AIC-Robotics/fast_lio.git}"

# FAST_LIO2_BRANCH: source branch to checkout after clone.
FAST_LIO2_BRANCH="${FAST_LIO2_BRANCH:-ros2}"

# FAST_LIO2_SOURCE_DIR: local source checkout directory inside this workspace.
FAST_LIO2_SOURCE_DIR="${FAST_LIO2_SOURCE_DIR:-$WORKSPACE_DIR/src/third_party/fast_lio}"

echo "=========================================="
echo "安装 FAST-LIO2 / FAST-LIO ROS 2 前端源码"
echo "=========================================="
echo "repo:   $FAST_LIO2_REPO_URL"
echo "branch: $FAST_LIO2_BRANCH"
echo "dir:    $FAST_LIO2_SOURCE_DIR"

mkdir -p "$(dirname "$FAST_LIO2_SOURCE_DIR")"

if [ ! -d "$FAST_LIO2_SOURCE_DIR/.git" ]; then
    git clone "$FAST_LIO2_REPO_URL" "$FAST_LIO2_SOURCE_DIR"
fi

cd "$FAST_LIO2_SOURCE_DIR"
git fetch origin "$FAST_LIO2_BRANCH"
git checkout "$FAST_LIO2_BRANCH"

cd "$WORKSPACE_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --packages-select fast_lio

echo "=========================================="
echo "FAST-LIO2 源码接入完成"
echo "=========================================="
