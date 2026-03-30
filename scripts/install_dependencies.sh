#!/bin/bash
# ROS2 Humble 依赖安装脚本

set -e

echo "=========================================="
echo "安装 ROS2 Humble 仿真依赖"
echo "=========================================="

# 更新apt
sudo apt update

# ROS2 核心包
echo "安装 ROS2 核心包..."
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-velodyne \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-nmea-navsat-driver \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs

# Gazebo Fortress (ROS2 Humble默认使用的Gazebo版本)
echo "安装 Gazebo Fortress..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null
sudo apt update
sudo apt install -y gz-fortress

# 额外工具
echo "安装工具包..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-ros \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep

# 安装FastLIO2作为备选SLAM方案
echo "安装 FastLIO2..."
cd /tmp
if [ ! -d "fast_lio" ]; then
    git clone https://github.com/AIC-Robotics/fast_lio.git
    cd fast_lio
    git checkout ros2
    colcon build --packages-select fast_lio
fi

echo "=========================================="
echo "依赖安装完成!"
echo "=========================================="
