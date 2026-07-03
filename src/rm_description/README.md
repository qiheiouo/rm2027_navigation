# rm_description

Phase 1 robot description package.

The public robot body frame is `base_link`.

Phase 1 defines only:

- `base_link`
- `gimbal_yaw_link`
- `mid360_left_frame`
- `mid360_right_frame`
- `lio_imu_link`
- optional `base_imu_link`

The sensor extrinsics in this package are placeholders. They are not the final 2027 real-robot calibration and must be measured again during the hardware stage.

The current `base_link -> gimbal_yaw_link` transform is a zero-yaw Phase 1 placeholder. Real hardware must replace it with a dynamic gimbal yaw state before validating localization with gimbal-mounted MID360 data.

`description.launch.py` sets the `robot_state_publisher` dynamic-TF ceiling to
`100 Hz` by default. This removes its upstream 20 Hz throttle while leaving the
actual update rate controlled by `/joint_states` (currently 50 Hz). It does not
interpolate or invent extra gimbal samples.

`rm_old_car_2026.urdf.xacro` is an experiment-only model for validating the
2027 stack on the available 2026 chassis. Its lidar/IMU offset is copied from
the old 2026 URDF as a temporary placeholder and must not be treated as the
2027 robot calibration.
