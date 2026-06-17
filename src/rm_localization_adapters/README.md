# rm_localization_adapters

Phase 1 localization boundary package.

Responsibilities:

- `map_odom_stub`: temporary Phase 1 `global_localization` placeholder. It publishes identity `map -> odom` as a dynamic TF on `/tf`.
- `lio_adapter`: converts backend LIO odometry into canonical `/odometry/lio` and publishes canonical `odom -> base_link`.
- `gimbal_state_adapter`: publishes `gimbal_yaw_joint` to `/joint_states`. Phase 1 defaults to a zero-yaw placeholder; real hardware must feed timestamped gimbal yaw through the adapter.
- `fake_lio_odom_publisher`: test helper that publishes fake raw LIO odometry for adapter validation only.

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

`gimbal_state_adapter` does not publish localization TF, odometry, navigation goals, or serial packets. It only provides the gimbal yaw joint state needed by `robot_state_publisher` to produce the sensor TF subtree.
