# GPS Degradation Test Scenarios

## Purpose

Verify that localization and Nav2 remain usable when GPS quality changes.

## Scenario 1: GPS good

- `/sensing/gps/fix` publishes a valid fix with low covariance.
- Localization should remain stable.
- Ground-truth comparison uses `/robot/ground_truth/odom`.
- Expected result: map-frame drift is lower than GPS-off mode.

## Scenario 2: GPS outage

- `/sensing/gps/fix` is absent or ignored.
- Localization continues with LiDAR, IMU, and wheel speed.
- Expected result: `/odometry/filtered` continues publishing and Nav2 can complete a short saved-map goal.

## Scenario 3: GPS jump

- `/sensing/gps/fix` changes abruptly by more than `max_position_jump`.
- GPS quality gate should reject or down-weight the jump.
- Expected result: `map -> odom -> base_footprint` does not jump abruptly.

## Metrics

- Drift against `/robot/ground_truth/odom`.
- Goal success rate.
- Maximum pose jump during GPS recovery.
- Time until localization recovers after GPS outage.
