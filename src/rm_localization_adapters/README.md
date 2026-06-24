# rm_localization_adapters

Phase 1 localization boundary package.

Responsibilities:

- `map_odom_stub`: temporary Phase 1 `global_localization` placeholder. It publishes identity `map -> odom` as a dynamic TF on `/tf`.
- `lio_adapter`: converts backend LIO odometry into canonical `/odometry/lio` and publishes canonical `odom -> base_link`.
- `gimbal_state_adapter`: publishes `gimbal_yaw_joint` to `/joint_states`. Phase 1 defaults to a zero-yaw placeholder; real hardware must feed timestamped gimbal yaw through the adapter.
- `imu_frame_adapter`: converts selected MID360 raw IMU messages from `/livox/lio_imu_raw` into canonical `/livox/lio_imu` by rewriting only `header.frame_id` to `lio_imu_link`.
- `fake_lio_odom_publisher`: test helper that publishes fake raw LIO odometry for adapter validation only.
- `canonical_odometry`: shared sensor-to-base pose conversion and optional
  timestamped pose-difference twist estimation.

Non-goals:

- No real global relocalization in Phase 1.
- No `small_gicp`, `scan_to_map`, or NDT in Phase 1.
- No direct dependency on a specific LIO backend.

Important rule:

`lio_adapter` must not hide a backend `body`, `lio_imu_link`, or `mid360_*_frame` by only changing `child_frame_id` to `base_link`.

For the 2027 gimbal-mounted MID360 layout, `lio_adapter` computes `odom -> base_link` from the LIO sensor pose and the current TF from `base_link` to the input sensor frame:

```text
T_odom_base = T_odom_sensor * inverse(T_base_sensor)
```

`T_base_sensor` is expected to come from `robot_state_publisher`, measured sensor extrinsics, and `gimbal_yaw_joint`.

The Phase 2A FAST-LIO Multi integration is a narrowly documented exception to
the raw frame-name rule. Upstream hard-codes `child_frame_id=body` for its IMU
state pose. The Phase 2A config may alias that backend-private `body` semantic
to `lio_imu_link`, but never to `base_link`; the adapter then performs the full
timestamped sensor-to-base transform above. The alias is disabled in the
general Phase 1 config.

`lio_adapter` supports two twist modes:

- `passthrough`: preserves a valid canonical input twist for Phase 1 fake and
  simulation inputs.
- `finite_difference`: estimates twist from consecutive canonical base poses
  for backends such as FAST-LIO Multi that leave `Odometry.twist` empty.

Finite difference runs after gimbal compensation. It rejects first samples,
invalid time intervals, and configured speed outliers rather than publishing a
fabricated zero velocity. Its covariance values remain conservative
placeholders until real trajectories are measured.

Timestamped sensor TF is handled by a bounded FIFO. Raw odometry waits for its
exact transform instead of falling back to the latest gimbal yaw. Queue size,
maximum wait, and retry rate are parameters; timeout or overflow drops data and
resets finite-difference history so stale samples cannot corrupt twist.

`gimbal_state_adapter` does not publish localization TF, odometry, navigation goals, or serial packets. It only provides the gimbal yaw joint state needed by `robot_state_publisher` to produce the sensor TF subtree.

`imu_frame_adapter` does not publish TF, filter IMU data, or modify measurement values. It exists because the inspected Livox ROS driver hard-codes IMU `header.frame_id` as `livox_frame`; the canonical public IMU frame for LIO remains `lio_imu_link`.
