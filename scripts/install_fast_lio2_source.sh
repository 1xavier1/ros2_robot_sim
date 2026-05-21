#!/bin/bash
# Install a ROS 2 compatible FAST-LIO2/LIO front end into this workspace.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# LIO_FRONTEND_REPO_URL: ROS 2 compatible FAST-LIO2/LIO repository URL.
LIO_FRONTEND_REPO_URL="${LIO_FRONTEND_REPO_URL:-${FAST_LIO2_REPO_URL:-https://github.com/MIT-SPARK/spark-fast-lio.git}}"

# LIO_FRONTEND_BRANCH: source branch to checkout after clone.
LIO_FRONTEND_BRANCH="${LIO_FRONTEND_BRANCH:-${FAST_LIO2_BRANCH:-main}}"

# LIO_FRONTEND_SOURCE_DIR: local source checkout directory inside this workspace.
LIO_FRONTEND_SOURCE_DIR="${LIO_FRONTEND_SOURCE_DIR:-${FAST_LIO2_SOURCE_DIR:-$WORKSPACE_DIR/src/third_party/spark-fast-lio}}"

# LIO_FRONTEND_PACKAGE_NAME: colcon package name to build.
LIO_FRONTEND_PACKAGE_NAME="${LIO_FRONTEND_PACKAGE_NAME:-${FAST_LIO2_PACKAGE_NAME:-spark_fast_lio}}"

echo "=========================================="
echo "安装 FAST-LIO2 / FAST-LIO ROS 2 前端源码"
echo "=========================================="
echo "repo:    $LIO_FRONTEND_REPO_URL"
echo "branch:  $LIO_FRONTEND_BRANCH"
echo "dir:     $LIO_FRONTEND_SOURCE_DIR"
echo "package: $LIO_FRONTEND_PACKAGE_NAME"

mkdir -p "$(dirname "$LIO_FRONTEND_SOURCE_DIR")"

if [ ! -d "$LIO_FRONTEND_SOURCE_DIR/.git" ]; then
    git clone "$LIO_FRONTEND_REPO_URL" "$LIO_FRONTEND_SOURCE_DIR"
fi

cd "$LIO_FRONTEND_SOURCE_DIR"
git fetch origin "$LIO_FRONTEND_BRANCH"
git checkout "$LIO_FRONTEND_BRANCH"

cd "$WORKSPACE_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --packages-up-to "$LIO_FRONTEND_PACKAGE_NAME"

echo "=========================================="
echo "FAST-LIO2 源码接入完成"
echo "=========================================="
