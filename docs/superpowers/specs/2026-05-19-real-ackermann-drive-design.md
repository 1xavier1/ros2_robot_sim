# Real Ackermann Drive Controller Design

## Background

The current robot simulation is intended to behave like an Ackermann steering car, but the active control path is still split across three pieces:

- `cmd_vel_bridge.py` subscribes to `/robot/cmd_vel`.
- `libgazebo_ros_diff_drive.so` drives the rear wheels through `/robot/rear/cmd_vel`.
- `libsteering_controller.so` drives the front steering joints through `/robot/steering_cmd`.

This makes the robot fragile and physically inconsistent. The rear drive is still a differential-drive plugin, while front steering is controlled by a separate plugin. Odom is published by the differential-drive plugin, so its model of motion is not owned by the same controller that changes the front wheel steering angles.

The observed symptom is that the robot can move forward and backward but does not reliably follow Ackermann turning behavior. Static inspection also shows that existing tests already expect a unified Ackermann controller, while the active URDF and launch files still use the old split chain.

## Goals

- Keep the external command interface as `/robot/cmd_vel` with `geometry_msgs/msg/Twist`.
- Make the simulated vehicle follow real Ackermann steering geometry.
- Use one Gazebo ModelPlugin to own steering, rear wheel drive, odometry, and TF.
- Preserve existing sensor topics, wheel encoder topics, joint state publishing, and the `base_footprint` frame convention.
- Make the simulation behavior useful as a baseline for a future real Ackermann vehicle.
- Provide static and dynamic tests that prove the vehicle turns by Ackermann geometry, not by differential-drive yaw.

## Non-Goals

- Do not migrate from Gazebo Classic to a newer Gazebo stack in this change.
- Do not introduce `gazebo_ros2_control` or `ackermann_steering_controller` for this fix.
- Do not change remote control, Nav2, or manual publishers that already send `/robot/cmd_vel`.
- Do not implement real hardware drivers.

## Recommended Architecture

Add a single Gazebo ModelPlugin named `libackermann_drive_controller.so`.

Data flow:

```text
remote / Nav2 / manual publisher
        |
        v
/robot/cmd_vel
        |
        v
libackermann_drive_controller.so
        |-- left_steering_joint / right_steering_joint
        |-- rear_left_joint / rear_right_joint
        |-- /robot/odom
        `-- odom -> base_footprint TF
```

The old split control path is removed:

- Remove `libgazebo_ros_diff_drive.so` from the URDF.
- Remove `libsteering_controller.so` from the URDF.
- Stop launching `cmd_vel_bridge.py`.
- Remove `/robot/rear/cmd_vel` and `/robot/steering_cmd` as required control links.

## Plugin Interface

The URDF should configure the new plugin under the `/robot` namespace:

```xml
<plugin name="ackermann_drive_controller" filename="libackermann_drive_controller.so">
  <ros>
    <namespace>/robot</namespace>
  </ros>

  <cmd_vel_topic>cmd_vel</cmd_vel_topic>
  <odom_topic>odom</odom_topic>
  <odom_frame>odom</odom_frame>
  <base_frame>base_footprint</base_frame>

  <left_steering_joint>left_steering_joint</left_steering_joint>
  <right_steering_joint>right_steering_joint</right_steering_joint>
  <rear_left_wheel_joint>rear_left_joint</rear_left_wheel_joint>
  <rear_right_wheel_joint>rear_right_joint</rear_right_wheel_joint>

  <wheelbase>0.45</wheelbase>
  <track_width>0.35</track_width>
  <wheel_radius>0.07</wheel_radius>
  <max_steering_angle>0.5236</max_steering_angle>
  <cmd_timeout>0.5</cmd_timeout>
  <update_rate>50.0</update_rate>

  <steering_p>50.0</steering_p>
  <steering_i>0.0</steering_i>
  <steering_d>5.0</steering_d>
  <max_wheel_torque>50.0</max_wheel_torque>
</plugin>
```

With the `/robot` namespace, `cmd_vel_topic=cmd_vel` resolves to `/robot/cmd_vel` and `odom_topic=odom` resolves to `/robot/odom`.

## Control Behavior

The controller consumes `geometry_msgs/msg/Twist`:

- `linear.x` is the desired vehicle longitudinal speed.
- `angular.z` is the desired yaw rate.
- Other fields are ignored.

For moving commands where `abs(linear.x)` is above a small threshold:

1. Compute the virtual center steering angle:

   ```text
   steer = atan(wheelbase * angular_z / linear_x)
   ```

2. Clamp `steer` to `[-max_steering_angle, max_steering_angle]`.
3. Compute turn radius:

   ```text
   radius = wheelbase / tan(abs(steer))
   ```

4. Compute front wheel angles from Ackermann geometry:

   ```text
   inner_angle = atan(wheelbase / (radius - track_width / 2))
   outer_angle = atan(wheelbase / (radius + track_width / 2))
   ```

5. For a left turn, assign the larger angle to the left front wheel. For a right turn, assign the larger magnitude to the right front wheel.
6. Split rear wheel speeds by turn radius:

   ```text
   inner_rear_linear = abs(linear_x) * (radius - track_width / 2) / radius
   outer_rear_linear = abs(linear_x) * (radius + track_width / 2) / radius
   wheel_angular_velocity = wheel_linear_velocity / wheel_radius
   ```

7. Preserve reverse motion by applying the sign of `linear.x` to rear wheel velocities.

For near-zero `linear.x`:

- Rear wheel velocity is zero.
- The controller may allow static pre-steering from `angular.z`.
- The model must not rotate in place, because a real Ackermann vehicle cannot yaw like a differential-drive robot at zero forward speed.

For stale commands:

- If no command arrives within `cmd_timeout`, rear wheel velocities go to zero.
- Steering returns to center unless a later design explicitly requires holding the last steering angle.

## Joint Control

- Front steering joints use Gazebo `JointController` position PID.
- Rear wheel joints use velocity targets with a force or torque limit.
- The plugin must fail loudly during load if any required joint is missing.
- The plugin owns all drivetrain control in one update loop so steering targets, wheel targets, odom, and TF remain consistent.

## Odometry And TF

The new controller publishes `/robot/odom` from the Gazebo model's actual world pose and velocity, not from a differential-drive approximation.

Odometry rules:

- `header.frame_id = odom`
- `child_frame_id = base_footprint`
- Pose comes from the model's Gazebo pose.
- Twist comes from the model's Gazebo velocity, expressed according to `nav_msgs/Odometry` semantics.

TF rules:

- Publish exactly one `odom -> base_footprint` transform.
- Do not keep `libgazebo_ros_diff_drive.so`, because it would publish another odom and TF source.

## File Changes

Expected implementation changes:

- Add `src/robot_description/src/ackermann_drive_controller.cpp`.
- Update `src/robot_description/CMakeLists.txt` to build and install `ackermann_drive_controller`.
- Update `src/robot_description/package.xml` with required dependencies such as `geometry_msgs`, `nav_msgs`, and `tf2_ros`.
- Update `src/robot_description/urdf/robot_base.urdf.xacro` to remove the old drive and steering plugins and add the new plugin.
- Update `launch/robot_simulation.launch.py` to stop launching `cmd_vel_bridge.py`.
- Update static tests in `src/robot_description/test/test_wheel_encoder_integration.py`.
- Keep or update Ackermann math tests in `src/robot_description/test/test_ackermann_kinematics.py`.

Expected cleanup:

- Delete or clearly deprecate `scripts/cmd_vel_bridge.py`.
- Delete or clearly deprecate `src/robot_description/src/steering_controller.cpp`.
- Keep `config/ackermann_robot_controllers.yaml` only if it remains useful as documentation; otherwise mark it deprecated or remove it in a cleanup commit.

## Validation

Static validation:

- URDF contains `libackermann_drive_controller.so`.
- URDF does not contain `libgazebo_ros_diff_drive.so`.
- URDF does not contain `libsteering_controller.so`.
- Launch file does not start `cmd_vel_bridge.py`.
- CMake builds and installs the new plugin.
- Steering joints remain `revolute`.
- Wheel roll joints remain free to rotate.

Build validation:

```bash
colcon build --packages-select robot_description
colcon test --packages-select robot_description
colcon test-result --verbose
```

Dynamic validation:

Start simulation:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=$PWD/log/ros ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
```

Publish a sustained turn:

```bash
ros2 topic pub -r 20 /robot/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

Expected results:

- `/joint_states` shows `left_steering_joint` and `right_steering_joint` are nonzero.
- Left and right front steering angles are different.
- During a left turn, the left front angle is larger than the right front angle.
- During a right turn, the right front angle magnitude is larger than the left front angle magnitude.
- Rear wheel velocities are nonzero and different while turning.
- The outside rear wheel is faster than the inside rear wheel.
- `/robot/odom` yaw changes continuously during the turn.
- Gazebo shows the vehicle following an arc.
- ROS graph no longer requires `/robot/rear/cmd_vel` or `/robot/steering_cmd`.

## Risks

- Gazebo Classic physics and simple tire contacts can still allow slip. Publishing odom from actual Gazebo pose makes this visible instead of hiding it behind an idealized kinematic odom.
- A custom plugin adds maintenance cost. The benefit is a single explicit control boundary that matches the simulated vehicle and can be tested directly.
- Future migration to `ros2_control` may still be desirable. This plugin should serve as the behavior baseline for that migration.

