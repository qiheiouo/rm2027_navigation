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
low-covariance wrong pose.

The old-car profile also enables autonomous recovery. The first rejected jump
latches global localization invalid so canonical TF cannot flicker between
AMCL modes. After canonical LIO reports that the chassis has remained
stationary, the bridge computes a trusted predicted pose from the last accepted
`map -> odom` correction and the current `odom -> base_link`, then publishes it
to `/initialpose` to locally reseed AMCL. Canonical TF resumes only after five
consecutive corrections agree with the trusted baseline. Failed recovery is
rate-limited and retried; state is published on
`/localization/correction_recovery_state`. An operator `/initialpose` or
`/localization/reset_map_to_odom` still deliberately clears the baseline.

This is a downstream containment and recovery boundary, not a replacement for
AMCL/LIO root-cause correction. In particular, it assumes canonical LIO remains
reliable while AMCL is rejected.

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

The generic 2D profile remains `config/amcl_2d.yaml`. The separate
`config/amcl_2d_spin_robust_candidate.yaml` changes only the documented
high-speed-spin motion term (`alpha4=0.02`) and is paired with
`config/pointcloud_to_scan_2d_spin_robust_candidate.yaml`. Old-car full
navigation currently selects this field profile explicitly; generic and
new-car defaults do not. Its residual high-spin failure mode and correction
recovery still require the documented old-car field acceptance.
