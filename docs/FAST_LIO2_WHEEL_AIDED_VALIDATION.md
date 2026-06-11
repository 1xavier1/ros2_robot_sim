# FAST-LIO2 Wheel-Aided Validation

## Rule

`/robot/ground_truth/odom` is evaluation only. It is never a production localization input.

## Terminals

Terminal 1:

```bash
cd /home/xavier/Workspace/ClaudeSpace/ros2_robot_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description fast_lio2.launch.py
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/wheel_lio_fusion.py
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/fast_lio_drift_diagnostic.py \
  --duration-sec 60 \
  --output maps/fast_lio_drift_check.json \
  --reference-topic /robot/ground_truth/odom
```

Terminal 5:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros python3 scripts/export_odom_projected_map.py \
  --output maps/wheel_lio_map_check \
  --duration-sec 60 \
  --pose-topic /localization/wheel_lio_odom \
  --reference-topic /robot/ground_truth/odom
```

## Correctness

- `/localization/wheel_lio_odom` is published continuously.
- Drift diagnostic shows wheel-LIO translation scale closer to reference than raw LIO.
- The generated occupancy map is less warped than raw FAST-LIO map accumulation.
- Ground truth appears only in diagnostic/reference arguments.
