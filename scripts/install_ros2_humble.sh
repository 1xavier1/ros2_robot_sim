#!/bin/bash
# ROS2 Humble + Gazebo Fortress 完整安装脚本
# 适用于 Ubuntu 22.04

set -e

echo "=========================================="
echo "ROS2 Humble 安装脚本"
echo "=========================================="

# 检查Ubuntu版本
if [ "$(lsb_release -rs)" != "22.04" ]; then
    echo "错误: 此脚本仅适用于 Ubuntu 22.04，当前版本: $(lsb_release -ds)"
    exit 1
fi

# 设置locale
echo "设置locale..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS2仓库
echo "添加ROS2仓库..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Humble
echo "安装ROS2 Humble..."
sudo apt update
sudo apt install -y ros-humble-desktop

# 安装ROS2工具
echo "安装ROS2工具..."
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep
sudo rosdep init
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 安装Gazebo Fortress
echo "安装Gazebo Fortress..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null
sudo apt update
sudo apt install -y gz-fortress

# ROS2 Gazebo桥接
echo "安装ROS2 Gazebo桥接..."
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# 仿真依赖包
echo "安装仿真依赖包..."
sudo apt install -y \
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
    ros-humble-pcl-msgs \
    ros-humble-tf-transformations

echo "=========================================="
echo "安装完成!"
echo "=========================================="
echo "请执行以下命令加载环境:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "或者将其添加到 ~/.bashrc:"
echo "  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
