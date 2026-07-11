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
