# Phase 2J 2D Relocalization

## Goal

Phase 2J adds a replaceable 2D global-localization backend without waiting for
Phase 2I real mapping validation or requiring a 3D PCD. The first backend is
Nav2 AMCL using an occupancy map and a planar LaserScan.

This phase does not change FAST-LIO, Nav2 control, serial, referee, mission, or
competition BT behavior. It does not make AMCL a canonical TF owner.

## Architecture

```text
occupancy map + LaserScan + canonical odometry
  -> AMCL (tf_broadcast=false)
  -> /localization/amcl_pose_raw
  -> amcl_pose_gate
  -> /localization/global_pose
  -> map_odom_from_global_pose
  -> canonical map -> odom
```

`lio_adapter` remains the only `odom -> base_link` owner. In AMCL mode,
`map_odom_stub` is absent and `map_odom_from_global_pose` remains the only
`map -> odom` owner.

The pose gate rejects wrong-frame, stale, future, non-finite, non-planar, and
excessive-covariance results. It publishes backend-specific validity on
`/localization/amcl_backend_valid`; the Phase 2C bridge separately publishes
canonical correction validity on `/localization/global_localization_valid`.

## Backend Selection

The generic navigation entry exposes:

```text
relocalization_backend:=none | amcl_2d | gicp_3d
```

`none` is the safe default. `amcl_2d` requires both:

```text
global_localization_mode:=external_pose
use_map_server:=true
```

Invalid combinations are rejected before runtime nodes start. The parallel
`gicp_3d` backend uses the same `/localization/global_pose` boundary but requires
an `occupancy_with_pcd` bundle. See `docs/phase2j_3d_relocalization.md`.

## Map Contract

Map bundle schema version 2 adds explicit map types:

```yaml
schema_version: 2
map_type: occupancy_only
alignment:
  occupancy_origin_reviewed: true
```

An `occupancy_only` bundle contains occupancy YAML and image artifacts and must
not contain a fake PCD. An approved bundle requires human review of the map
origin and yaw. Schema version 1 remains backward compatible and means
`occupancy_with_pcd`.

The occupancy map can come from Phase 2I, an official field asset, CAD/manual
conversion, or another reviewed mapping workflow. Phase 2J depends on the
approved artifact contract, not on how the map was created.

## Scan Input

AMCL consumes `/localization/scan`. A native planar scanner or bag may publish
that topic directly. For MID360 use, the optional projection path is:

```text
filtered PointCloud2
  -> pointcloud_to_laserscan_node
  -> /localization/scan
```

The projection is independently switchable and rate-limited to reduce AMCL
load. Localization scan filtering is a separate concern from local costmap
obstacle processing; changing it must not silently change the costmap input.

### Experimental High-Spin Candidate

Field evidence showed that the baseline Omni motion model can inject excessive
translation noise during high-speed stationary rotation, while a frame-level
pointcloud projection also leaves scan motion distortion. The isolated
candidate combines:

```text
AMCL Omni alpha4: 0.2 -> 0.02
filtered PointCloud2 with preserved per-point timestamps
strict per-point SE(3) deskew from canonical /odometry/lio
```

The candidate is selected only by
`old_car_2026_amcl_spin_candidate.launch.py`. The normal AMCL and competition
launches remain unchanged. Deskew failure drops the frame and reports inactive;
there is no raw-scan fallback disguised as a valid deskew result.

Deterministic replay and no-hardware short-bag smoke passed, but real static,
translation, ordinary turn, Nav2 spin and high-speed-spin acceptance remain
open. Do not make this candidate the competition default based only on replay
seeds. See
`docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`.

ROS 2 Humble AMCL must use a positive `save_pose_rate`; `0.0` overflows the
timer-period conversion in the released implementation. The baseline uses
`0.5 Hz`, which is the upstream-style low-rate persistence behavior and has
negligible runtime cost. Keep `tf_broadcast: false` independently of this
setting.

## No-Hardware Test

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_localization_adapters \
  rm_relocalization_bridge rm_map_tools rm_navigation_bringup
source install/setup.bash

colcon test --packages-select rm_relocalization_bridge rm_map_tools
colcon test-result --verbose

ros2 launch rm_navigation_bringup phase2j_2d_relocalization_test.launch.py
```

The test uses a synthetic occupancy map, synthetic LaserScan, and fake raw LIO
odometry. It starts real AMCL and the real Phase 2C bridge, but no driver,
Nav2, serial, referee, mission, or competition BT.

Inspect:

```bash
ros2 topic echo --once /localization/amcl_pose_raw
ros2 topic echo --once /localization/global_pose
ros2 topic echo --once /localization/amcl_backend_valid
ros2 topic echo --once /localization/global_localization_valid
ros2 run tf2_ros tf2_echo map odom
ros2 topic info /tf -v
```

The Linux smoke test has passed with real Humble AMCL, a synthetic occupancy
map, synthetic scan, fake canonical odometry, and the real Phase 2C bridge.
Verified boundaries include `tf_broadcast: false`, `save_pose_rate: 0.5`, no
`map_odom_stub`, `/localization/global_pose` output, and a single canonical
`map -> odom` owner. This is interface and ownership evidence only; real-map
AMCL stability remains a separate acceptance step.

## Real Runtime Shape

After an occupancy-only bundle and scan source are approved:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  global_localization_mode:=external_pose \
  relocalization_backend:=amcl_2d \
  use_map_server:=true \
  map_bundle_manifest:=/maps/field/field.bundle.yaml \
  use_relocalization_pointcloud_projection:=true \
  relocalization_pointcloud_topic:=/livox/left/pointcloud_filtered
```

Real hardware, LIO, Nav2, and chassis transport remain separate explicit
switches. An operator must provide a credible initial pose until global
localization behavior is separately validated.

For manual startup, publish that pose with RViz `2D Pose Estimate` (the standard
`/initialpose` topic). This is an initialization input, not another TF owner.

## Acceptance

1. Build and tests pass.
2. AMCL publishes no TF.
3. `map_odom_stub` and `map_odom_from_global_pose` never coexist.
4. Only `map_odom_from_global_pose` publishes `map -> odom`.
5. Only `lio_adapter` publishes `odom -> base_link`.
6. Wrong frame, invalid values, stale data, future data, and excessive
   covariance do not reach `/localization/global_pose`.
7. No map or no scan cannot produce a false valid result.
8. Occupancy-only deployment requires no PCD and no fake PCD artifact.

## Current Status And Next Evidence

The backend and no-hardware chain are functionally complete. The next bounded
test is a no-motion old-car run using a reviewed candidate occupancy map:

- start driver, LIO, AMCL, and `map_odom_from_global_pose`;
- keep Nav2 and real serial disabled;
- verify stable `/localization/global_pose` and canonical `map -> odom`;
- verify AMCL still publishes no TF and the stub remains absent;
- compare estimated pose with measured laboratory landmarks.

A candidate PGM may be used for this validation, but it remains unsuitable for
formal navigation deployment until human landmark and origin review passes.
