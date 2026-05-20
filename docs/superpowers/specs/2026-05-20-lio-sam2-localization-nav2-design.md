# LIO-SAM2 Mapping, Localization, And Nav2 Design

## Goal

Build a simulation-first mapping, localization, and navigation stack for the Ackermann robot in `ros2_robot_sim`, while reserving clean interfaces for future real vehicle deployment on RK3588 or Horizon X5 with LiDAR options such as Unitree L1 or Livox MID-360.

The implementation priority is:

1. Record and preserve the current verified simulation baseline.
2. Run LIO-SAM2 mapping with LiDAR, IMU, wheel encoder, and weak GPS constraints.
3. Provide localization that remains usable when GPS is weak or unavailable.
4. Run Nav2 first on a saved map, then later support mapping while navigating.
5. Keep the real vehicle migration boundary explicit.

## Current Baseline

The current `ros2_robot_sim` baseline is:

- The robot uses a unified Gazebo `ackermann_drive_controller` plugin.
- `/robot/cmd_vel` directly drives front steering angles and rear wheel speeds.
- `/robot/odom` is published for runtime odometry.
- `/robot/ground_truth/odom` is published separately for simulation evaluation.
- `/tf`, `/robot/imu/data`, `/robot/velodyne_points`, and `/robot/wheel_encoder/rear_average` have been dynamically verified.
- `scripts/verify_runtime_topics.sh` verifies required runtime topics and messages.
- `localization.launch.py` can start `robot_localization`, and `/odometry/filtered` can publish `odom -> base_footprint`.
- `navigation.launch.py` has a working missing-package precheck, but the current environment lacks required Nav2 packages.
- `lio_sam2.launch.py` has a missing-package precheck, but LIO-SAM2 mapping has not yet been run.

The system must not treat `/robot/ground_truth/odom` as an input to production localization or navigation. It is only for simulation comparison, regression testing, and drift evaluation.

## Recommended Architecture

Use a staged architecture that gets a working simulation loop quickly while preserving clean replacement points for real sensors:

```text
Gazebo sensors / real sensor drivers
        |
        v
sensor_adapter
        |
        v
unified sensing topics
        |
        +--> LIO-SAM2 mapping / map localization
        |
        +--> wheel + IMU + optional GPS localization fusion
        |
        v
map -> odom -> base_footprint
        |
        v
Nav2
        |
        v
/control/cmd_vel
        |
        v
simulation remap to /robot/cmd_vel or real chassis bridge
```

The first implementation may use direct remaps from current simulation topics to the unified sensing topics. A dedicated adapter node can be introduced when real LiDAR or real chassis interfaces are added.

## Unified Interfaces

The stack should converge on these internal topics:

| Type | Unified topic | Simulation source | Real vehicle source |
|---|---|---|---|
| LiDAR point cloud | `/sensing/lidar/points` | `/robot/velodyne_points` | Unitree L1 or Livox MID-360 driver |
| IMU | `/sensing/imu/data` | `/robot/imu/data` | onboard or external IMU |
| Wheel speed | `/sensing/wheel/speed` | `/robot/wheel_encoder/rear_average` | MCU, CAN, or UART chassis interface |
| GPS | `/sensing/gps/fix` | `/robot/rtk_gps/fix` | GNSS or RTK receiver |
| Navigation command | `/control/cmd_vel` | remap to `/robot/cmd_vel` | real chassis command bridge |

Future real vehicle work should replace only the sensor driver side and the chassis command bridge side. LIO-SAM2, localization fusion, and Nav2 should continue to consume the unified topics.

## LIO-SAM2 Mapping

LIO-SAM2 is the mapping mainline.

Inputs:

- `/sensing/lidar/points`
- `/sensing/imu/data`
- `/sensing/wheel/speed`
- `/sensing/gps/fix`

The first simulation mapping milestone should use:

- `/robot/velodyne_points` remapped to `/sensing/lidar/points`
- `/robot/imu/data` remapped to `/sensing/imu/data`
- `/robot/wheel_encoder/rear_average` remapped to `/sensing/wheel/speed`
- `/robot/rtk_gps/fix` remapped to `/sensing/gps/fix`

LIO-SAM2 outputs:

- A map representation usable for later localization and Nav2.
- A trajectory estimate for evaluation.
- A transform or pose estimate that can contribute to `map -> odom`.

Mapping should support two modes:

1. Saved-map mapping: manually drive the robot through the environment, build the map, and save it.
2. Online mapping while navigating: reserved for the second phase after saved-map navigation is stable.

GPS is a weak global constraint. It may reduce long-term drift when quality is good, but it must not directly pull the map or localization state through sudden jumps.

## Localization Fusion

Localization must work in GPS-good and GPS-poor conditions.

The TF contract is:

```text
map -> odom -> base_footprint -> base_link -> sensors
```

Responsibilities:

- `odom -> base_footprint`: continuous local motion estimate.
- `map -> odom`: global correction from LIO-SAM2 and optional GPS.
- `/robot/ground_truth/odom`: simulation-only evaluation source.

### Local Continuous Localization

The initial local estimate can continue using `robot_localization` EKF. It currently fuses `/robot/odom` and `/robot/imu/data`.

Future improvement:

- Replace direct Gazebo truth-like odometry input with wheel-derived Ackermann odometry.
- Use `/sensing/wheel/speed` and steering state to estimate short-term odom.
- Keep IMU yaw rate and acceleration as stabilizing inputs.

### LiDAR-Inertial Localization

LIO-SAM2 provides the map-frame correction source. It should reduce long-term drift from wheel and IMU-only localization.

### GPS Weak Constraint And Degradation

GPS integration uses quality gates:

- Good GPS: valid fix, acceptable covariance, and no sudden position jump. It may help correct global drift.
- Weak GPS: high covariance or degraded fix status. Its weight is reduced or ignored.
- GPS outage: localization continues using LiDAR, IMU, and wheel speed.
- GPS recovery: GPS is reintroduced smoothly and must not reset the current pose abruptly.

## Nav2 Navigation

Nav2 is implemented in two stages.

### Stage 1: Navigation On A Saved Map

Flow:

```text
LIO-SAM2 saved map
        |
        v
map server / map loader
        |
        v
localization publishes map -> odom -> base_footprint
        |
        v
Nav2 planner and controller
        |
        v
/control/cmd_vel -> /robot/cmd_vel
```

Success criteria:

- A saved map can be loaded.
- A start pose and goal can be set.
- Nav2 plans a path.
- Nav2 controller output reaches `/robot/cmd_vel`.
- The robot follows a path in simulation.
- GPS-good and GPS-off modes can both complete a short navigation task.
- Ground-truth odom is used only for evaluation.

### Stage 2: Mapping While Navigating

After saved-map navigation is stable, support online map updates:

```text
LIO-SAM2 updates map continuously
        |
        v
Nav2 consumes updated map or updated costmaps
        |
        v
goal navigation or exploration
```

This stage is reserved and should not block the first working navigation loop.

### Ackermann Constraints

Nav2 must respect the robot as a non-holonomic Ackermann vehicle:

- `vy` must remain zero.
- Reverse motion may be allowed but should be more expensive than forward motion.
- The configured minimum turning radius must match the vehicle geometry.
- With `wheelbase = 0.45m` and `max_steering_angle = 0.5236rad`, the minimum turning radius is about `0.78m`.
- Existing `navigation.yaml` currently uses `min_turning_radius: 0.20`; this must be corrected before Nav2 path validation.
- The robot must not be expected to rotate in place like a differential-drive base.

## Real Vehicle Reservation

The real vehicle target is an Ackermann robot using RK3588 or Horizon X5 as the compute platform.

Likely LiDAR choices:

- Unitree L1
- Livox MID-360

Real deployment should preserve the same internal topics:

- Sensor drivers publish into `/sensing/...`.
- Navigation publishes `/control/cmd_vel`.
- A chassis bridge converts `/control/cmd_vel` to the vehicle protocol.

The LiDAR adapter must reserve fields for:

- frame id
- timestamp policy
- sensor extrinsics
- point cloud format conversion
- motion compensation or deskew support

This is important because Velodyne-style simulation clouds are regular, while Unitree L1 and Livox MID-360 may require non-repetitive scan handling or Livox-specific preprocessing.

## Testing And Acceptance

### Static Checks

Required commands:

```bash
colcon build --packages-select robot_description
colcon test --packages-select robot_description
```

Static checks should verify:

- LIO-SAM2 launch reports missing dependencies clearly when LIO-SAM2 is absent.
- Nav2 launch reports missing dependencies clearly when Nav2 is absent.
- Nav2 configuration uses `vy = 0`.
- Nav2 `min_turning_radius` is at least `0.78`.
- LIO-SAM2 configuration consumes unified `/sensing/...` topics.
- `/robot/ground_truth/odom` is not used as a production localization input.

### Runtime Topic Checks

The runtime verification script should check:

- `/tf`
- `/robot/odom`
- `/robot/ground_truth/odom`
- `/robot/imu/data`
- `/robot/velodyne_points`
- `/robot/wheel_encoder/rear_average`
- Later, unified `/sensing/...` topics

### Mapping Acceptance

Mapping acceptance requires:

- LIO-SAM2 starts and subscribes to LiDAR, IMU, wheel speed, and optional GPS.
- Manual driving can produce a usable map.
- The map can be saved.
- GPS-off mapping still runs.
- GPS-on mapping has lower global drift than GPS-off mapping in outdoor-like worlds.
- Ground truth is used only to measure drift.

### Localization Acceptance

Localization acceptance requires:

- `/odometry/filtered` publishes continuously.
- `map -> odom -> base_footprint` remains connected.
- GPS-good mode is stable.
- GPS-off mode continues localizing.
- GPS jump simulation does not instantly pull the pose.

### Nav2 Acceptance

Saved-map Nav2 acceptance requires:

- A saved map loads.
- Nav2 lifecycle nodes activate.
- A goal can be sent.
- `/robot/cmd_vel` receives commands.
- The vehicle reaches a goal in GPS-good and GPS-off modes.
- The planned path and driven behavior respect Ackermann constraints.

Online mapping navigation acceptance is deferred until saved-map navigation passes.

## Out Of Scope For The First Implementation

- Full real hardware drivers.
- Production-ready Unitree L1 or Livox MID-360 deskew.
- Autonomous exploration.
- Online map updates during navigation.
- Replacing Nav2's local controller with a custom Ackermann controller plugin.
- Reinforcement learning controllers.

## Implementation Order

Recommended implementation order:

1. Update documentation and progress records.
2. Add unified sensing remaps and tests.
3. Update LIO-SAM2 config to use unified topics.
4. Add wheel-speed to odometry adapter design or minimal node.
5. Add GPS quality gate configuration.
6. Run LIO-SAM2 dependency and launch validation.
7. Build and save a map in simulation.
8. Correct Nav2 Ackermann constraints.
9. Install or validate Nav2 dependencies.
10. Run saved-map Nav2 navigation.
11. Add GPS-off and GPS-jump test scenarios.
12. Defer online mapping navigation to the next phase.

## Phase 2 Boundary

After saved-map navigation passes, the next design cycle should cover online mapping while navigating. That phase should decide how LIO-SAM2 map updates are converted into Nav2 map or costmap updates, and whether autonomous exploration is included.
