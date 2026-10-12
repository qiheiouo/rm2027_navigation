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

The normal 2D profile remains `config/amcl_2d.yaml`. The separate
`config/amcl_2d_spin_robust_candidate.yaml` changes only the documented
high-speed-spin motion term (`alpha4=0.02`) and is paired with
`config/pointcloud_to_scan_2d_spin_robust_candidate.yaml`. It is experimental,
not a competition default, and must be launched through the no-motion
`old_car_2026_amcl_spin_candidate.launch.py` wrapper until field acceptance is
complete.

## Localization validity lease (AUD-001)

The selected gate publishes reliable **volatile** `LocalizationStatus` on
`<valid_topic>/status` at least every 100 ms while alive. The header is emission
time; `producer_id`, `started_at` and increasing `sequence` identify an instance.
`pose` is the last accepted map/base measurement, with its original timestamp.
`valid_until` is the source deadline upper bound: the existing 1.0 s AMCL /
1.5 s GICP input timeout plus the existing 0.1 s timeout timer period. The
steady-clock source timeout itself is unchanged. A heartbeat does not extend that deadline or manufacture a new
measurement. Fresh stamped observations with an unchanged pose remain legal.

When `upstream_valid_topic` is configured, the canonical bridge consumes that
status **including the pose**, rather than independently pairing a cached Bool
and an unqualified pose. It still exposes `/localization/global_pose` upstream
for inspection, but that topic cannot bypass the selected gate. The bridge is
still the only dynamic map/odom owner. It publishes `/localization/status`
with its own instance identity and the last matched pose; correction math and
backend quality thresholds are unchanged.

The bridge and localization consumers require source validity, fresh emission
time, increasing sequence/epoch (equal emission timestamps are legal within one
simulated clock tick; reversed time is rejected) and a steady-clock receipt lease (default
`status_timeout_sec` / `localization_status_timeout_sec`: 1.0 s). This budget
reuses the AMCL stream budget and permits ten gate heartbeat periods, rather
than requiring each correction to change. The accepted source window is bounded
by the existing GICP 1.5 s budget plus the 0.1 s timeout timer period.
Old/pre-start messages, repeated or reversed timestamps, an expired source or
a silent producer close validity. Gate/bridge restart, reset and fault recovery
require a fresh acceptable source timestamp newer than the accepted-source
high-water mark; startup and explicit reset also impose their own time cutoff.
This admits fresh in-flight measurements rather than rejecting them merely
because a timeout callback ran before delivery.
A ROS clock reversal fails closed; restart in the new clock epoch is required
if the clock does not catch up. Simulated startup waits for positive ROS time.

`/localization/global_localization_valid` and backend Bool topics remain latched
**diagnostic snapshots**, with no independent lifetime semantics. Mission and
readiness now consume `/localization/status`; bridge death closes their
localization input within one lease plus their timer period (0.1/0.2 s).
Other Mission inputs are unchanged. Third-party Bool-only consumers must migrate
before relying on localization health. This change grants no serial authority
and does not prove a chassis stop.

Invalidation retains the last correction/pose in memory for diagnosis but
publishes false status and stops new TF/transform timestamps. Already cached TF
may still be visible to TF lookup; that is not localization health. Pending
poses are cleared on invalidation and their existing 0.2 s wait/0.5 s source-age
limits are checked before matching, so late odometry cannot reopen validity.

An unconfigured standalone external-pose bridge has no producer heartbeat; each
new pose provides only a finite 1.0 s lease. A one-shot Phase 2C fake pose now
expires automatically. For AMCL/GICP use the selected backend status chain.
