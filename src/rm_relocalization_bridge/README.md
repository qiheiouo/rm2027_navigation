# rm_relocalization_bridge

This package owns the 2027 global-localization boundary. It converts a global
robot pose and canonical LIO odometry into the canonical dynamic transform:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

Inputs:

- `/localization/global_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/odometry/lio` (`nav_msgs/Odometry`, `odom -> base_link` semantics)

Outputs:

- dynamic `map -> odom` on `/tf`
- `/localization/map_to_odom` for inspection
- `/localization/global_localization_valid`
- `/localization/reset_map_to_odom` reset service

The node never publishes an identity fallback. `map_odom_stub` and this package
must never run together. A future small_gicp, scan-to-map, or NDT backend must
publish `/localization/global_pose`; it must not publish canonical TF directly.

The bridge also has an optional correction-innovation fail-safe. It compares a
new `map -> odom` candidate with the last accepted correction, invalidates
global localization, and withholds TF when either the planar translation or
shortest yaw step exceeds its configured limit. The generic profile keeps this
gate disabled for compatibility. Old-car launch profiles select
`config/map_odom_from_global_pose_old_car_2026.yaml`, which enables 0.35 m and
0.35 rad limits after a field-observed high-spin AMCL failure produced a
low-covariance wrong pose. Publishing `/initialpose` or calling
`/localization/reset_map_to_odom` deliberately resets the accepted baseline;
the next timestamp-matched correction is then allowed to establish a new one.
This is a downstream containment boundary, not a replacement for AMCL/LIO root
cause correction.

`fake_global_pose_publisher` is test-only. It derives synchronized fake global
poses from `/odometry/lio` and a configured correction, and publishes no TF.
The Phase 2C test launch uses one publication so reset behavior remains
observable. The test publisher uses transient-local QoS and supports a startup
delay; this is a test convenience, not the required QoS of a real backend.

Phase 2J adds `amcl_2d_backend.launch.py`. AMCL runs with `tf_broadcast=false`
and publishes backend-private `/localization/amcl_pose_raw`. The
`global_pose_gate_node` validates frame, timestamp, finite values, planar pose,
and covariance before publishing `/localization/global_pose`. The optional
PointCloud2 projection and AMCL itself remain replaceable upstream components.

The normal 2D profile remains `config/amcl_2d.yaml`. The separate
`config/amcl_2d_spin_robust_candidate.yaml` changes only the documented
high-speed-spin motion term (`alpha4=0.02`) and is paired with
`config/pointcloud_to_scan_2d_spin_robust_candidate.yaml`. It is experimental,
not a competition default, and must be launched through the no-motion
`old_car_2026_amcl_spin_candidate.launch.py` wrapper until field acceptance is
complete.
