# Phase 2J 3D Relocalization

## Scope

Phase 2J provides `gicp_3d` in parallel with `amcl_2d`. It aligns a current 3D
cloud in `odom` against a prior PCD in `map`, starting from `/initialpose`.

```text
occupancy_with_pcd bundle + current PointCloud2 + /initialpose
  -> rm_gicp_relocalization (PCL GICP, no TF)
  -> /localization/gicp_pose_raw
  -> global_pose_gate
  -> /localization/global_pose
  -> map_odom_from_global_pose
  -> canonical map -> odom
```

The package is independent of AMCL. The generic navigation entry selects one
backend:

```text
relocalization_backend:=none | amcl_2d | gicp_3d
```

Both real backends require `global_localization_mode:=external_pose` and
`use_map_server:=true` in the navigation profile. They cannot run together.

## Why A PCD Is Not Sufficient By Itself

A PCD is required, but successful registration also depends on:

1. a correct common `map` origin and sensor/base extrinsics;
2. enough geometric overlap and non-degenerate structure;
3. a seed within the GICP convergence basin;
4. suitable downsampling and correspondence thresholds;
5. bounded motion distortion and timestamped `odom -> base_link`;
6. fitness/jump rejection and CPU latency on the target minipc.

The first implementation deliberately requires `/initialpose`. It does not
claim full-field place recognition or brute-force global search. A descriptor,
multi-hypothesis, referee, or other coarse prior can be added later upstream of
the same seed interface.

## Map Location

The production interface is an approved map bundle, not a hard-coded PCD path.
The launch resolves the PCD from:

```text
<deployment-root>/<map-id>/<map-id>.bundle.yaml
```

The manifest must use `map_type: occupancy_with_pcd`. Occupancy-only bundles
are rejected explicitly. The PCD and occupancy map must share the reviewed
origin/yaw recorded by the bundle.

## Validation

```bash
colcon build --symlink-install --packages-select \
  rm_gicp_relocalization rm_relocalization_bridge rm_localization_adapters \
  rm_map_tools rm_navigation_bringup
source install/setup.bash

colcon test --packages-select \
  rm_gicp_relocalization rm_relocalization_bridge rm_map_tools
colcon test-result --verbose

ros2 launch rm_navigation_bringup phase2j_3d_relocalization_test.launch.py
```

The synthetic test uses the real PCL GICP algorithm, an asymmetric 522-point
PCD/current cloud pair, fake canonical odometry, the real pose gate, and the
real Phase 2C bridge. It starts no hardware, Nav2, serial, referee, mission, or
competition BT.

Inspect:

```bash
ros2 topic echo --once /localization/gicp_fitness_score
ros2 topic echo --once /localization/gicp_pose_raw
ros2 topic echo --once /localization/global_pose
ros2 topic echo --once /localization/global_localization_valid
ros2 run tf2_ros tf2_echo map odom
ros2 topic info /tf -v
```

Real acceptance additionally requires multiple initial-pose errors, repeated
field structures, partial overlap, occlusion, collision-induced displacement,
long runs, and target-minipc CPU/latency tests.

The first pose covariance is a conservative fitness-derived placeholder. It is
useful for gating but is not a calibrated probabilistic uncertainty model; real
bags must be used before a strategy layer relies on its numeric value.
