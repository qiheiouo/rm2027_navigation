# Phase 2K Old-Car AMCL Integration

## Goal

This profile connects the verified old-car driver/LIO boundary to the Phase 2J
AMCL backend without starting Nav2 or either serial transport. It is a
no-motion real-map validation entry, not a competition navigation launch.

```text
left MID360 -> old-car FAST-LIO -> /odometry/lio
filtered PointCloud2 -> /localization/scan
candidate or approved occupancy map -> map_server -> AMCL (no TF)
AMCL pose gate -> /localization/global_pose
map_odom_from_global_pose -> canonical map -> odom
```

`lio_adapter` remains the only `odom -> base_link` owner. AMCL keeps
`tf_broadcast: false`; `map_odom_from_global_pose` is the only `map -> odom`
owner while relocalization is enabled.

## Safety Boundary

`old_car_2026_amcl_relocalization.launch.py` hard-disables:

- Nav2;
- real serial transport;
- serial dry-run;
- managed mapping.

The launch defaults to `enable_relocalization:=false`, starts no real driver by
default, and retains the old-car stub only while the backend is disabled.
`allow_test` assets are rejected when the real MID360 driver is enabled.

## Candidate Map Experiment

The map may remain `candidate` while PGM cleanup is pending. Structural checks,
hashes and canonical frames are still mandatory:

```bash
ros2 launch rm_navigation_bringup old_car_2026_amcl_relocalization.launch.py \
  enable_relocalization:=true \
  use_driver:=true \
  use_lio_backend:=true \
  use_rviz:=true \
  selected_side:=left \
  map_bundle_manifest:=/data/rm27_maps/field/revision/field.bundle.yaml \
  map_acceptance_policy:=allow_candidate
```

Use RViz `2D Pose Estimate` or the explicit initial-pose launch arguments. Do
not send a navigation goal from this profile.

## Acceptance

1. `/map` is active and matches the selected manifest revision.
2. `/localization/scan` has finite data in `base_link`.
3. AMCL is active and publishes no TF.
4. `/localization/global_pose` becomes valid after initialization.
5. `map_odom_stub` is absent while the external bridge is active.
6. Only the bridge publishes `map -> odom`.
7. Nav2, `/cmd_vel`, real serial and serial dry-run remain absent.
8. Candidate use validates the chain but does not approve the map.
