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
