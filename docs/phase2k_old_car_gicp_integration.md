# Phase 2K Old-Car GICP Integration

## Goal

This profile connects the old-car LIO chain to the Phase 2J seeded GICP backend
without starting Nav2 or either serial transport.

```text
left MID360 -> old-car FAST-LIO
  -> /odometry/lio
  -> /lio/cloud_registered_transformed

candidate or approved occupancy_with_pcd bundle
  -> map_server + GICP (no TF)
  -> /localization/global_pose
  -> map_odom_from_global_pose
  -> canonical map -> odom
```

The source cloud uses the same mapping-verified old-car basis correction as the
exported PCD. Using uncorrected `/lio/cloud_registered` against that PCD would
mix coordinate semantics and is intentionally avoided.

## Safety Boundary

`old_car_2026_gicp_relocalization.launch.py` hard-disables Nav2, managed
mapping, real serial and serial dry-run. Safe defaults start neither the real
driver nor GICP. When relocalization is enabled, the stub is absent and only
`map_odom_from_global_pose` may own canonical `map -> odom`.

The upstream FAST-LIO parameter named `publish_tf_results` is used only to
publish the corrected registered cloud and optional backend visualization pose;
backend `/tf` remains quarantined and canonical TF ownership does not change.

## Candidate PCD Experiment

```bash
ros2 launch rm_navigation_bringup old_car_2026_gicp_relocalization.launch.py \
  enable_relocalization:=true \
  use_driver:=true \
  use_lio_backend:=true \
  use_rviz:=true \
  selected_side:=left \
  map_bundle_manifest:=/data/rm27_maps/field/revision/field.bundle.yaml \
  map_acceptance_policy:=allow_candidate
```

Set the seed with RViz `2D Pose Estimate`; GICP consumes the standard
`/initialpose` message. Do not send a navigation goal from this profile.

## Acceptance

1. The bundle resolves as `occupancy_with_pcd` and reports the expected map ID.
2. `/lio/cloud_registered_transformed` has finite data in `odom`.
3. GICP publishes no TF and reports bounded fitness/correction diagnostics.
4. Accepted registration reaches `/localization/global_pose`.
5. `map_odom_stub` is absent while the external bridge is active.
6. Only the bridge publishes `map -> odom`.
7. Nav2, `/cmd_vel`, real serial and serial dry-run remain absent.
8. Candidate use validates registration behavior but does not approve the map.
