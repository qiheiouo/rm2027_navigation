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
Because the old-car LIO adapter derives twist by finite difference, isolated
over-threshold speed samples do not reset the stationary hold. Motion must stay
over threshold for `recovery_motion_confirmation_sec` before it is confirmed;
the old-car profile additionally requires a bounded one-second odometry pose
window before reseeding.

This is a downstream containment and recovery boundary, not a replacement for
AMCL/LIO root-cause correction. The generic profile still assumes canonical LIO
remains reliable and keeps odometry rebasing disabled. The old-car profile has
an additional fail-closed fallback for a settled LIO position jump: when the
trusted correction would put the robot at least 1 m from its last accepted
global pose, the bridge seeds AMCL at the last accepted global XY instead of in
the displaced LIO region. It retains the current settled yaw, uses wider seed
covariance, and accepts the new `map -> odom` baseline only after ten global
poses remain within both the absolute anchor gate and the correction-consistency
gate. A wrong but internally stable AMCL result therefore remains rejected.

The old-car retry period is 5 s and the sequence stops after six unsuccessful
seeds. During a valid consistency streak the timer cannot reset AMCL. Exhaustion
publishes `latched_reseed_attempts_exhausted` and continues withholding canonical
TF until an explicit initial pose or reset. A rebase attempt publishes
`rebase_reseeded_waiting_for_consistency`; normal recovery retains
`reseeded_waiting_for_consistency`. These states are diagnostics, not movement
authority.

This fallback intentionally does not guess vehicle displacement after a fault.
If the chassis actually moves more than the absolute anchor tolerance, if LIO
keeps changing, or if global localization cannot converge near the anchor, it
stays invalid. Real-car acceptance is required before this old-car-only profile
can be called a completed fix.

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
