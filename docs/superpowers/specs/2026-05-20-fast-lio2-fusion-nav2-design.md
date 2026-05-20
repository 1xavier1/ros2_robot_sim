# FAST-LIO2 Fusion Localization And Nav2 Design

## Goal

Build a simulation-first 3D LiDAR mapping, localization, and saved-map navigation stack for the Ackermann robot. The main line is FAST-LIO2 or a compatible FAST-LIO ROS 2 front end, plus GPS, wheel encoder, IMU, and LiDAR fusion localization, followed by Nav2 saved-map navigation.

The system must support the full operating pattern:

- Start from an outdoor charging area.
- Navigate outdoors where GPS or RTK is available.
- Enter long barn-like structures where GPS is unavailable.
- Continue stable localization and navigation inside barns using 3D LiDAR, IMU, and wheel encoder.
- Reserve later work for edge-following near feed alleys and dynamic cattle handling.

## Current Baseline

The current project already has:

- Unified sensing topics under `/sensing/...`.
- Simulation LiDAR bridged to `/sensing/lidar/points`.
- Simulation IMU bridged to `/sensing/imu/data`.
- Rear wheel speed bridged to `/sensing/wheel/speed`.
- GPS fix bridged to `/sensing/gps/fix`.
- `/control/cmd_vel` as the internal navigation command interface.
- `/robot/cmd_vel` as the Gazebo control input.
- `/robot/ground_truth/odom` for simulation evaluation only.
- A Nav2 launch precheck that reports missing Nav2 packages.
- A previous LIO-SAM2-oriented plan that should be replaced by this FAST-LIO2-oriented design.

`/robot/ground_truth/odom` must never be used as a production localization or navigation input.

## Recommended Architecture

Use a layered architecture:

```text
3D LiDAR + IMU
  -> FAST-LIO2 / FAST-LIO ROS 2 front end
  -> /mapping/lio/odom
  -> /mapping/lio/map_points

Wheel encoder
  -> Ackermann wheel odometry
  -> /localization/wheel/odom

GPS / RTK
  -> GPS quality gate
  -> /localization/gps/gated

Fusion localization
  -> OUTDOOR: GPS weight is high
  -> TRANSITION: GPS weight decreases, LiDAR and wheel odom take over
  -> BARN: GPS is disabled or given very low weight
  -> /odometry/filtered
  -> map -> odom -> base_footprint

Nav2
  -> saved-map navigation
  -> /control/cmd_vel
  -> /robot/cmd_vel or real chassis bridge
```

FAST-LIO2 is responsible for LiDAR-inertial odometry and 3D mapping. It is not responsible for all sensor fusion. GPS and wheel encoder integration should remain outside the FAST-LIO2 front end so that GPS-good and GPS-outage modes can be switched safely.

## Operating Modes

### OUTDOOR

OUTDOOR mode is used near the charging area and open outdoor routes.

- GPS or RTK is expected to be available.
- GPS has high global correction weight when quality is good.
- LiDAR, IMU, and wheel odometry still run continuously.
- GPS is rejected if covariance is too high or if a position jump is detected.

### TRANSITION

TRANSITION mode is used around barn entrances.

- GPS weight decreases gradually.
- FAST-LIO2 and wheel odometry become the dominant localization sources.
- `map -> odom` must not jump when GPS quality changes.
- The mode can be triggered by geofence, GPS covariance, GPS outage, or LiDAR localization quality.

### BARN

BARN mode is used inside long barn structures.

- GPS is assumed unavailable.
- LiDAR, IMU, and wheel encoder are the main localization sources.
- Wheel encoder is important for longitudinal stability in 1 km tunnel-like barns.
- Localization should prioritize continuity and stable lateral/heading behavior over GPS absolute position.

## Mode Manager

Add a `localization_mode_manager` concept.

Inputs:

- `/sensing/gps/fix`
- `/mapping/lio/odom`
- `/localization/wheel/odom`
- optional geofence or route segment metadata
- `/robot/ground_truth/odom` only in simulation evaluation tools

Outputs:

- `/localization/mode`
- `/localization/gps/gated`
- `/localization/fusion_weights`

The first implementation may publish mode and gated GPS information without implementing a full factor graph. Later versions can feed the same outputs into a graph-based backend.

## Mapping And Saved Maps

Mapping flow:

```text
manual or scripted driving
  -> FAST-LIO2 builds 3D point cloud map
  -> save trajectory, keyframes, and point cloud map
  -> split large site into map regions
  -> generate 2D occupancy map for Nav2
```

Map layers:

- 3D point cloud map: primary map for LiDAR localization and future real vehicle deployment.
- 2D occupancy map: Nav2 saved-map navigation input generated from 3D points.
- Region metadata: charging area, outdoor yard, barn entrance, barn lane, and other route sections.

Large barns should use regional maps:

```text
outdoor_yard
charging_area
barn_01_entrance
barn_01_lane_a
barn_01_lane_b
barn_02_entrance
```

Regional maps reduce ambiguity in long repeated corridors, allow single-barn map updates, and make route-based map loading easier.

## Vehicle Geometry Configuration

Add `config/vehicle_geometry.yaml` as the main geometry and kinematics configuration.

All key parameters must have comments that explain:

- the physical part represented by the value
- unit
- coordinate frame
- positive direction
- affected modules

Recommended structure:

```yaml
vehicle:
  # wheelbase: distance between front axle center and rear axle center, unit m.
  # Affects Ackermann kinematics, wheel odometry, and Nav2 minimum turning radius.
  wheelbase: 0.45

  # track_width: lateral distance between left and right wheel centers, unit m.
  # Affects URDF wheel placement and wheel-speed geometry.
  track_width: 0.35

  # wheel_radius: wheel radius, unit m.
  # Affects wheel encoder linear speed conversion.
  wheel_radius: 0.07

  # max_steering_angle: maximum front-wheel steering angle, unit rad.
  # Affects min_turning_radius and Ackermann command limits.
  max_steering_angle: 0.5236

  # min_turning_radius: minimum feasible turning radius, unit m.
  # Should equal wheelbase / tan(max_steering_angle), about 0.78 m for current robot.
  min_turning_radius: 0.78

  body:
    # length: vehicle body envelope along base_link +X forward direction, unit m.
    # Used by URDF, Nav2 footprint, and LiDAR self filtering.
    length: 0.55

    # width: vehicle body envelope along base_link Y axis, unit m.
    # +Y is left, -Y is right.
    width: 0.38

    # height: vehicle body envelope along base_link Z axis, unit m.
    # +Z is upward.
    height: 0.12

  footprint:
    # polygon: 2D vehicle footprint in base_link frame, unit m.
    # Points are [x, y], X forward positive, Y left positive.
    polygon:
      - [0.275, 0.19]
      - [0.275, -0.19]
      - [-0.275, -0.19]
      - [-0.275, 0.19]

  self_filter:
    # box_min / box_max: 3D vehicle self-filter box in base_link frame, unit m.
    # Points inside this box are treated as vehicle body returns and removed from LiDAR clouds.
    box_min: [-0.35, -0.25, -0.05]
    box_max: [0.35, 0.25, 0.45]
```

URDF, Nav2 footprint, Ackermann kinematics, wheel odometry, and LiDAR self-filtering should use or be checked against this file.

## Sensor Mount Configuration

Add `config/sensor_mount.yaml` as the main sensor extrinsic and effective point cloud configuration.

Recommended structure:

```yaml
lidar:
  # parent_frame: frame that LiDAR extrinsics are relative to.
  parent_frame: base_link

  # frame: LiDAR point cloud frame_id.
  frame: laser_link

  # xyz: LiDAR origin relative to parent_frame, unit m.
  # X forward positive, Y left positive, Z upward positive.
  xyz: [0.0, 0.0, 0.18]

  # rpy: LiDAR roll, pitch, yaw relative to parent_frame, unit rad.
  # Right-hand rule: roll about X, pitch about Y, yaw about Z.
  rpy: [0.0, 0.524, 0.0]

  # min_range / max_range: accepted LiDAR point distance, unit m.
  min_range: 0.1
  max_range: 100.0

  # horizontal_fov / vertical_fov: expected useful field of view, unit rad.
  # Used for simulation sanity checks and later real LiDAR validation.
  horizontal_fov: 6.28318
  vertical_fov: 0.5236
```

The current simulation has `laser_link` pitched about 0.524 rad relative to `base_link`. That value must become configurable and easy to replace when using Unitree L1, Livox MID-360, or another 3D LiDAR.

## LiDAR Point Cloud Filtering

Algorithm-facing LiDAR should use filtered, valid point clouds.

Topic contract:

```text
/sensing/lidar/points_raw        raw or directly bridged point cloud
/sensing/lidar/points_filtered   self-filtered and range-filtered point cloud
/sensing/lidar/points            default algorithm input, should point to filtered data
```

Filtering stages:

1. Transform points to `base_link`.
2. Remove points inside `vehicle.self_filter`.
3. Remove points outside LiDAR min/max range.
4. Optionally downsample for FAST-LIO2 or Nav2 costmaps.
5. Publish filtered point cloud with a stable frame.

Sanity checks:

- `base_link -> laser_link` TF exists.
- `/sensing/lidar/points` has stable `frame_id`.
- The point cloud is not dominated by vehicle body points.
- Forward, left, right, ground, and wall structures are visible in barn-like worlds.
- FAST-LIO2 LiDAR-IMU extrinsics match URDF/TF and `sensor_mount.yaml`.

## Localization Fusion

First-stage fusion should remain pragmatic:

- Use FAST-LIO2 odometry as LiDAR-inertial motion input.
- Use wheel odometry to stabilize longitudinal motion.
- Use IMU yaw rate and acceleration where appropriate.
- Use GPS only through the quality gate.
- Publish `/odometry/filtered` and preserve `map -> odom -> base_footprint`.

GPS rules:

- Good GPS is used heavily outdoors.
- GPS is gradually down-weighted at barn entrances.
- GPS is disabled or near-zero weight inside barns.
- GPS recovery must be smooth and must not cause pose jumps.

Later upgrades may replace the fusion backend with a graph-based optimizer, but the external topic contracts should remain the same.

## Nav2 Saved-Map Navigation

The first Nav2 target is saved-map navigation, not online mapping while navigating.

Closed-loop validation flow:

```text
1. Start simulation.
2. Start sensing bridge and LiDAR filtering.
3. Start FAST-LIO2 front end.
4. Manually or script-drive mapping.
5. Save 3D point cloud map.
6. Generate 2D occupancy map.
7. Start fusion localization.
8. Start Nav2.
9. Send goal.
10. Verify /control/cmd_vel reaches /robot/cmd_vel.
11. Verify the robot reaches the goal region.
```

Validation scenarios:

- GPS-good outdoor-like route.
- GPS-outage barn-like route.

Acceptance criteria:

- `/sensing/lidar/points`, `/sensing/imu/data`, `/sensing/wheel/speed`, and `/sensing/gps/fix` publish.
- `/sensing/lidar/points_filtered` publishes valid filtered points.
- FAST-LIO2 starts and subscribes to LiDAR and IMU.
- `/mapping/lio/odom` publishes.
- `/localization/mode` switches between `OUTDOOR` and `BARN`.
- `/odometry/filtered` publishes continuously.
- Nav2 lifecycle nodes start when Nav2 dependencies are installed.
- `/control/cmd_vel` publishes and is bridged to `/robot/cmd_vel`.
- A short saved-map goal completes in simulation.

## Cloud And Client Extension Boundary

Later system versions need a cloud service and a client application. The client may be used for vehicle management, connection setup, map creation, job design, route setup, and remote task triggering. Vehicles may use 4G, and the cloud side may have normal internet connectivity.

This stage only reserves the boundary. It does not implement cloud services, client UI, 4G communication, remote task dispatch, authentication, or fleet management.

Future high-level architecture:

```text
client / web console
  -> vehicle management
  -> connection configuration
  -> map creation and review
  -> job design
  -> route setup
  -> remote task trigger

cloud service
  -> vehicle registry
  -> task dispatch
  -> map, route, and job configuration storage
  -> status, logs, and telemetry collection
  -> OTA and version-management reservation

vehicle mission agent
  -> receives cloud tasks
  -> validates task and configuration versions
  -> converts tasks into local ROS 2 actions, services, or topics
  -> starts mapping, localization, navigation, or job workflows
  -> reports local execution state
```

Reserved namespaces:

```text
/mission/*  later local mission state, commands, and progress
/maps/*     later map upload, download, selection, and versioning
/fleet/*    later vehicle state, heartbeat, and remote configuration
/config/*   later validated vehicle, sensor, localization, and navigation configuration
```

Design rules:

- FAST-LIO2, localization, and Nav2 must not depend on the cloud service.
- The vehicle must keep local localization and navigation running when 4G or cloud connectivity is unavailable.
- The cloud must not be part of the real-time control loop.
- Remote task triggers must be converted into a local vehicle mission state machine before they affect ROS 2 navigation.
- Maps, routes, jobs, and vehicle configuration must be versioned before remote management is introduced.
- `vehicle_geometry.yaml` and `sensor_mount.yaml` may later be edited through a client, but the vehicle must validate them before applying them.
- Safety, authentication, remote operation permissions, emergency stop, and geofencing require a separate design cycle.

## Out Of Scope For This Stage

The following work is explicitly deferred:

- Feed alley or trough edge extraction.
- `/barn/edge_reference`.
- Dynamic cattle classification or tracking.
- Edge-following local controller.
- Semantic barn maps.
- Multi-barn mission scheduling.
- Online mapping while navigating.
- Real vehicle hardware drivers.
- Cloud service and client application.
- 4G communication and remote task triggering.
- Authentication, fleet management, and OTA.

Reserve these topic names for later stages without implementing them now:

```text
/barn/edge_reference
/barn/dynamic_objects
/barn/work_mode
```

## Plan Migration Requirements

The current LIO-SAM2 plan should be replaced or superseded by a FAST-LIO2 plan.

Required plan changes:

- Rename LIO-SAM2 milestones to FAST-LIO2 / FAST-LIO ROS 2 front-end milestones.
- Add `vehicle_geometry.yaml` with documented units and coordinate frames.
- Add `sensor_mount.yaml` with documented LiDAR extrinsics and point cloud validity parameters.
- Add LiDAR point cloud self-filtering before algorithm consumption.
- Add LiDAR mount and point cloud sanity checks early in the plan.
- Keep unified `/sensing/...` and `/control/cmd_vel` interfaces.
- Keep Nav2 saved-map navigation as the first navigation target.
- Keep edge-following and dynamic cattle handling out of the current implementation scope.
- Reserve mission, map, fleet, and remote configuration namespaces for later cloud/client integration.
