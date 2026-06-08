# FAST-LIO2 Wheel-Aided Optimization Design

## Goal

Improve mapping and localization robustness without depending on simulation ground truth in the production algorithm.

The current FAST-LIO2 front end publishes `/mapping/lio/odom` and `/mapping/lio/map_points`, but tunnel driving shows clear longitudinal translation under-estimation after turns. Wheel odometry, GPS, and backend localization currently exist downstream, but they do not correct FAST-LIO2 mapping or the pose used to accumulate maps.

The design goal is to make the localization and map-building path robust enough for barn-like corridors while keeping a clean migration path to the real vehicle.

## Ground Truth Rule

`/robot/ground_truth/odom` is allowed only in simulation evaluation tools.

It must not be used by:

- FAST-LIO2.
- Wheel/LIO production fusion.
- Nav2 runtime localization.
- Saved map generation in production mode.

Production inputs are limited to:

- LiDAR.
- IMU.
- Wheel odometry or wheel encoder-derived odometry.
- GPS or RTK when available and quality-gated.

## Current Problem

Observed behavior:

- `/mapping/lio/odom` and `/robot/odom` begin with different absolute origins, which is acceptable.
- Short straight motion has some local consistency.
- After turning or circling, FAST-LIO2 translation diverges, especially in the forward direction.
- Yaw often remains closer than translation.
- Maps generated from `/mapping/lio/map_points` become sparse, warped, or dominated by incorrect tunnel structure.
- The odom-projected occupancy exporter produces a more coherent baseline, proving that the main issue is not only 2D map filtering.

Architectural cause:

- FAST-LIO2 is currently a LiDAR+IMU front end only.
- Wheel/GPS fusion is downstream and does not feed back into FAST-LIO2 or map accumulation.
- `accumulate_lio_map.py` and FAST-LIO map outputs trust FAST-LIO pose quality directly.

## Design Direction

Use a staged approach:

1. Keep FAST-LIO2 running as the LiDAR/IMU front end.
2. Add measurement and scoring tools to quantify FAST-LIO drift against wheel odom and simulation ground truth.
3. Add a wheel-aided LIO pose layer outside FAST-LIO2.
4. Use the wheel-aided pose for map accumulation and Nav2 localization.
5. Tune FAST-LIO2 input and parameters after the diagnostic baseline is measurable.
6. Reserve internal FAST-LIO factor modification for later only if external fusion is insufficient.

## Recommended Architecture

```text
LiDAR + IMU
  -> FAST-LIO2
  -> /mapping/lio/odom
  -> /mapping/lio/map_points

Wheel odometry
  -> /robot/odom

GPS / RTK
  -> quality gate
  -> /localization/gps/gated

Wheel/LIO pose fusion
  inputs:
    /mapping/lio/odom
    /robot/odom
    optional /localization/gps/gated
  output:
    /localization/wheel_lio_odom
    /localization/wheel_lio_status

Map generation
  inputs:
    /sensing/lidar/points
    /localization/wheel_lio_odom
  optional evaluation only:
    /robot/ground_truth/odom
  output:
    Nav2 occupancy map

Nav2 global localization
  input:
    /localization/wheel_lio_odom
  output:
    map -> odom -> base_footprint -> base_link
```

## Wheel/LIO Fusion Behavior

The first production fusion layer should be conservative and explainable.

Inputs:

- `/mapping/lio/odom`: FAST-LIO pose and yaw.
- `/robot/odom`: wheel odometry pose and twist.
- `/localization/gps/gated`: optional global anchor when fresh.

Output:

- `/localization/wheel_lio_odom`.
- `/localization/wheel_lio_status`.

Pose policy:

- Translation scale comes primarily from wheel odometry relative motion.
- Yaw uses FAST-LIO when fresh and consistent.
- Wheel yaw is fallback when FAST-LIO is stale or divergent.
- The fused pose starts in the FAST-LIO map frame by anchoring the first FAST-LIO pose and applying wheel relative deltas from that anchor.
- If FAST-LIO and wheel diverge beyond configured thresholds, the node lowers FAST-LIO influence and reports degraded status.

This is intentionally not a full factor graph. It is a bounded external fusion layer that can be tested before touching third-party FAST-LIO internals.

## Diagnostics

Add a runtime diagnostic tool or node that compares:

- `/mapping/lio/odom`
- `/robot/odom`
- `/localization/wheel_lio_odom`
- `/robot/ground_truth/odom` in simulation only

Metrics:

- Translation delta.
- Yaw delta.
- Translation scale ratio.
- Drift per meter traveled.
- Topic freshness.
- Point cloud freshness.
- FAST-LIO map point freshness.
- Divergence threshold crossings.

Expected output:

- JSON summary per run.
- Periodic console status.
- Optional CSV for plotting.

The diagnostic result must show whether wheel-aided fusion improves drift compared with raw FAST-LIO.

## Map Generation

The current odom-projected exporter should evolve into a pose-source configurable map exporter.

Required options:

```text
--pose-topic /localization/wheel_lio_odom
--reference-topic /robot/ground_truth/odom
--cloud-topic /sensing/lidar/points
```

Rules:

- Default production pose source is `/localization/wheel_lio_odom`.
- Ground truth is optional and writes evaluation metrics only.
- 2D map filtering may use `min-z`, `max-z`, `max-range`, self-filtering, and free-space evidence.
- FAST-LIO should continue receiving reliable 3D input. 2D roof filtering should not be confused with FAST-LIO front-end input filtering.

## FAST-LIO2 Parameter And Input Governance

FAST-LIO2 tuning should be treated as a separate controlled layer.

Initial parameter areas:

- `det_range`: reduce from 100 m to a corridor-appropriate range, such as 15-30 m.
- `blind`: keep close returns out of the estimator.
- `filter_size_map`: balance stability and detail.
- `timestamp_unit`, `scan_rate`, `ring`, and `time`: verify against simulated point cloud fields.
- `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov`: tune only after drift metrics are available.

Input checks:

- `/sensing/lidar/points` must publish continuously.
- `/sensing/imu/data` must publish continuously.
- FAST-LIO should not report sustained `No point`.
- `/mapping/lio/odom` and `/mapping/lio/map_points` must have fresh timestamps.

## GPS And Loop Closure

GPS should not be used to hide local FAST-LIO drift inside the barn.

GPS role:

- Outdoor or entrance global anchoring.
- Slow correction when quality is gated and fresh.
- No hard jump in `map -> odom`.

Loop closure and scan matching are later-stage work:

- ICP or scan matching against saved maps.
- Pose graph or loop closure.
- Internal FAST-LIO wheel factor fork.

These should wait until wheel-aided fusion and diagnostics prove their limits.

## Success Criteria

Simulation success:

- Raw FAST-LIO drift is measured over repeatable straight and turning trajectories.
- Wheel-aided LIO reduces translation scale error compared with raw FAST-LIO.
- Map generated from `/localization/wheel_lio_odom` is less warped than map generated from raw FAST-LIO.
- Ground truth evaluation confirms improvement but is not used by the production pose.
- Topic freshness diagnostics can identify LiDAR, FAST-LIO, and odom stalls.

Production-readiness success:

- The algorithm can run without `/robot/ground_truth/odom`.
- Wheel/LIO fusion status reports degraded states instead of silently trusting bad FAST-LIO.
- GPS can be enabled as a bounded anchor without creating TF jumps.

## Non-Goals

This design does not include:

- Modifying third-party FAST-LIO internals in the first implementation.
- Building a full pose graph.
- Building scan matching against saved maps.
- Using ground truth as a runtime localization input.
- Optimizing Nav2 controllers.

## Implementation Phases

### Phase 1: Diagnostics

Create a FAST-LIO drift diagnostic that compares LIO, wheel odom, fused odom, and optional ground truth.

### Phase 2: Wheel/LIO Fusion

Create `/localization/wheel_lio_odom` as an external fusion layer with status reporting and divergence guards.

### Phase 3: Map Exporter Integration

Make map generation use configurable pose sources and default to `/localization/wheel_lio_odom`.

### Phase 4: FAST-LIO Input And Parameter Tuning

Tune range, timestamps, point fields, and noise parameters using diagnostics from Phase 1 and Phase 2.

### Phase 5: Long-Term Enhancements

Evaluate GPS anchoring, scan matching, loop closure, or an internal FAST-LIO wheel factor only after the external fusion layer has been measured.
