# Wheel-LIO Robust Fusion Design

## Goal

Improve the current wheel-LIO mapping pose against wheel slip, sharp Ackermann turns, and short FAST-LIO inconsistency without replacing the existing pipeline.

The production mapping pose remains:

```text
/mapping/lio/odom + /robot/odom
        -> scripts/wheel_lio_fusion.py
        -> /localization/wheel_lio_odom
        -> scripts/export_odom_projected_map.py
```

`/robot/ground_truth/odom` remains evaluation only. It must not become a production localization input.

## Current Problem

The current `wheel_lio_fusion.py` uses wheel odometry for translation scale and fresh FAST-LIO yaw for heading. This improves short mapping, but it has no explicit trust model:

- If wheels slip, wheel odometry can pull the fused pose too far.
- If FAST-LIO jumps in a degenerate corridor, LIO can pull the fused pose suddenly.
- During sharp Ackermann turns, wheel and LIO deltas can differ briefly even when both are usable.

The next change should add robust consistency detection and conservative dynamic weighting, not a full EKF or full Ackermann model.

## Chosen Approach

Use a state-machine-style robust fusion layer inside `scripts/wheel_lio_fusion.py`.

The first implementation should add pure functions for motion comparison and state selection:

```text
compute_motion_delta(previous_pose, current_pose)
compare_wheel_lio_motion(wheel_delta, lio_delta, thresholds)
classify_fusion_state(...)
```

These functions must be testable without ROS runtime.

## Motion Consistency Metrics

The fusion node compares short-window motion from `/robot/odom` and `/mapping/lio/odom`.

Metrics:

- Translation distance difference:
  ```text
  abs(wheel_distance - lio_distance)
  ```
- Motion direction difference:
  ```text
  heading(wheel_delta_xy) vs heading(lio_delta_xy)
  ```
- Yaw delta difference:
  ```text
  abs(wheel_yaw_delta - lio_yaw_delta)
  ```
- Short-time speed ratio:
  ```text
  wheel_speed vs lio_speed
  ```

The first version does not consume steering angle and does not solve full Ackermann kinematics. It uses existing odometry outputs and leaves steering-angle integration for a later design.

## Fusion States

The node publishes one of these states in `/localization/wheel_lio_status`:

```text
normal
turning_caution
wheel_suspect
lio_suspect
degraded
```

### normal

Wheel and LIO deltas agree within thresholds.

Action:

- Use wheel odometry to preserve translation scale.
- Use fresh FAST-LIO yaw.
- Allow normal fused output.

### turning_caution

Yaw delta or motion direction differs moderately, but translation distance is not severely inconsistent.

Typical cause:

- Sharp Ackermann turn.
- Short transient turn where wheel and LIO observations disagree slightly.

Action:

- Continue using wheel translation, but reduce wheel translation weight.
- Do not refresh anchors aggressively during the turn.
- Publish status with `state=turning_caution`.

### wheel_suspect

Wheel distance or speed is much larger than LIO motion.

Typical cause:

- Mud, manure, wet ground, or wheel spin.

Action:

- Reduce wheel translation weight strongly.
- Pull fused position closer to FAST-LIO.
- Avoid refreshing the wheel anchor for the suspect frame.
- Publish status with `state=wheel_suspect`.

### lio_suspect

FAST-LIO jumps or reports motion much larger than wheel odometry.

Typical cause:

- Corridor degeneracy.
- Sparse or repetitive LiDAR geometry.
- Temporary point-cloud registration jump.

Action:

- Preserve wheel translation continuity.
- Keep using LIO yaw only if fresh and not clearly unstable.
- Avoid immediate anchor reset from the suspect LIO sample.
- Publish status with `state=lio_suspect`.

### degraded

Inputs are stale, or several consecutive frames are severely inconsistent.

Action:

- Publish status with `state=degraded`.
- Do not refresh anchors.
- Hold the last trusted pose when available.
- If no trusted pose has ever been published, publish only status and do not publish a new odometry sample.

## Dynamic Translation Weighting

The fused position should blend wheel-projected pose and LIO pose:

```text
fused_xy = wheel_weight * wheel_projected_xy + (1 - wheel_weight) * lio_xy
```

Initial weights:

```text
normal: 1.0
turning_caution: 0.8
wheel_suspect: 0.2
lio_suspect: 1.0
degraded: hold_last_trusted_pose
```

Yaw remains:

- Fresh FAST-LIO yaw by default.
- Wheel yaw fallback only when LIO yaw is stale or marked unusable.

## Parameters

Add ROS parameters to `wheel_lio_fusion.py`:

```text
motion_window_min_distance: 0.05
wheel_lio_distance_warn: 0.15
wheel_lio_distance_error: 0.35
wheel_lio_speed_ratio_warn: 1.8
wheel_lio_speed_ratio_error: 3.0
yaw_delta_warn: 0.25
yaw_delta_error: 0.60
turning_yaw_rate_threshold: 0.25
wheel_weight_normal: 1.0
wheel_weight_turning: 0.8
wheel_weight_wheel_suspect: 0.2
wheel_weight_lio_suspect: 1.0
wheel_weight_degraded: 0.0
max_consecutive_bad_frames: 5
```

Thresholds are conservative defaults for simulation. They must remain configurable because real cattle-yard surfaces and tire behavior will differ.

## Topics

Do not rename existing public topics:

```text
/localization/wheel_lio_odom
/localization/wheel_lio_status
```

Enhance status text with diagnostic fields:

```text
state=normal; wheel_weight=1.00; lio_yaw=fresh; wheel=fresh; gps=none
state=wheel_suspect; wheel_weight=0.20; reason=wheel_distance_high; gps=none
state=turning_caution; wheel_weight=0.80; reason=yaw_rate_high; gps=none
state=lio_suspect; wheel_weight=1.00; reason=lio_jump; gps=none
state=degraded; wheel_weight=0.00; reason=stale_inputs; gps=none
```

## Testing

### Pure Function Tests

Add tests for:

- Normal straight motion produces `normal`.
- Wheel distance much greater than LIO distance produces `wheel_suspect`.
- LIO distance much greater than wheel distance produces `lio_suspect`.
- Large yaw delta with consistent distance produces `turning_caution`.
- Consecutive bad frames produce `degraded`.
- Weight blending moves fused XY toward LIO when wheel is suspect.

### Static Contract Tests

Add or update tests to verify:

- `wheel_lio_fusion.py` declares all new parameters.
- Status strings include `state=` and `wheel_weight=`.
- `/localization/wheel_lio_odom` remains the output topic.
- `export_odom_projected_map.py` still defaults to `/localization/wheel_lio_odom`.
- `/robot/ground_truth/odom` is not used by production fusion.

### Runtime Validation

Use the existing validation flow:

```bash
ros2 launch robot_description robot_simulation.launch.py gui:=false rviz:=false
ros2 launch robot_description fast_lio2.launch.py
python3 scripts/wheel_lio_fusion.py
```

Expected normal simulation behavior:

- `/localization/wheel_lio_odom` publishes continuously.
- `/localization/wheel_lio_status` mostly reports `state=normal`.
- During tighter turns it may report `state=turning_caution`.
- Drift diagnostic still includes `/mapping/lio/odom` and `/localization/wheel_lio_odom`.
- Mapping still uses:
  ```bash
  --pose-topic /localization/wheel_lio_odom
  ```

Synthetic pure-function tests cover slip and LIO jump cases. Runtime slip simulation is out of scope for the first implementation.

## Out of Scope

- Full EKF or factor graph.
- Direct steering-angle sensor integration.
- Full Ackermann kinematic fusion model.
- Real cattle-yard GPS tuning.
- Changing map exporter output format.
- Using `/robot/ground_truth/odom` as a production input.

## Success Criteria

- Existing tests still pass.
- New pure-function tests pass.
- `colcon build --packages-select robot_description` passes.
- In normal simulation, wheel-LIO status is readable and mostly `normal`.
- Generated maps still use `/localization/wheel_lio_odom`.
- The system can identify wheel-suspect and LIO-suspect cases in tests and adjust translation weight accordingly.
