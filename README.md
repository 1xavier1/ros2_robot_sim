# ROS2 Humble 四轮差速驱动机器人仿真系统

## 系统概述

基于ROS2 Humble的四轮差速驱动小车仿真系统，支持：
- **3D激光雷达** (Velodyne VLP-16仿真)
- **IMU传感器** (仿真)
- **RTK GPS** (仿真)
- **轮速编码器** (仿真)
- **LIO-SAM2 3D SLAM建图**
- **Nav2导航定位**

## 目录结构

```
ros2_robot_sim/
├── src/robot_description/     # 机器人模型和描述
│   ├── urdf/                   # URDF/XACRO模型
│   ├── src/                    # 插件源码 (wheel_encoder)
│   ├── CMakeLists.txt
│   └── package.xml
├── worlds/                     # Gazebo仿真世界
│   └── office_campus.world
├── config/                     # 配置文件
│   ├── lio_sam.yaml           # LIO-SAM2配置
│   ├── localization.yaml      # EKF定位配置
│   └── navigation.yaml        # Nav2导航配置
├── launch/                     # Launch文件
│   ├── robot_simulation.launch.py  # 仿真启动
│   ├── lio_sam2.launch.py          # 建图启动
│   └── navigation.launch.py       # 导航启动
├── rviz/                       # RViz配置
│   └── robot_config.rviz
└── maps/                       # 建图输出目录
```

## 依赖安装

```bash
# ROS2 Humble 核心包
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-velodyne-*
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
sudo apt install ros-humble-nmea-navsat-driver
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-rviz2

# LIO-SAM2 (需要单独编译)
# git clone https://github.com/TonyRobotics/LIO-SAM2.git
# 或使用 ROS2 版本的 lio_sam
```

## 构建

```bash
cd ~/workspace/ros2_robot_sim
colcon build --packages-select robot_description
source install/setup.bash
```

## 运行

### 1. 启动仿真

```bash
ros2 launch robot_description robot_simulation.launch.py
```

这将启动：
- Gazebo仿真世界
- 机器人模型
- 传感器仿真 (LiDAR, IMU, RTK, Wheel Encoder)
- 机器人状态发布器
- EKF定位节点
- RViz2

### 2. 启动LIO-SAM2建图

```bash
ros2 launch robot_description lio_sam2.launch.py
```

### 3. 启动导航

```bash
ros2 launch robot_description navigation.launch.py
```

## 传感器话题

| 传感器 | 话题 | 类型 |
|--------|------|------|
| 激光雷达 | `/robot/velodyne_points` | sensor_msgs/PointCloud2 |
| IMU | `/robot/imu/data` | sensor_msgs/Imu |
| RTK GPS | `/robot/rtk_gps/fix` | sensor_msgs/NavSatFix |
| 里程计 | `/robot/odom` | nav_msgs/Odometry |
| 轮速编码器 | `/robot/wheel_encoder/*` | std_msgs/Float64 |

## 控制话题

| 功能 | 话题 | 类型 |
|------|------|------|
| 速度控制 | `/robot/cmd_vel` | geometry_msgs/Twist |

## 坐标系

```
map -> odom -> base_footprint -> base_link
                         -> imu_link
                         -> velodyne_link
                         -> rtk_link
                         -> front/rear_axle -> *_*_wheel
```

## 仿真参数

- **机器人尺寸**: 0.55m x 0.38m x 0.12m
- **轮子半径**: 0.07m
- **轮间距**: 0.45m (wheelbase), 0.35m (track)
- **激光雷达高度**: 0.25m
- **IMU高度**: 0.08m
- **RTK高度**: 0.30m

## 真实小车移植

1. 修改launch文件中的话题名称（从仿真话题切换到真实设备话题）
2. 修改URDF中的传感器配置为真实硬件
3. 配置外参标定参数
4. 参考config/localization.yaml配置EKF参数

## 已知问题

1. LIO-SAM2需要针对ROS2 Humble进行适配，建议使用`fast_lio2`作为备选方案
2. RTK GPS在室内环境无法使用，仅室外场景有效
3. 轮速编码器插件需要在真实硬件上替换为实际驱动
