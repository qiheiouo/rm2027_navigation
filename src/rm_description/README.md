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

The default `base_link -> gimbal_yaw_link` input remains a zero-yaw Phase 1
placeholder. The dog-hole simulation supplies a timestamped non-zero joint
state and commands the matching Gazebo joint, so simulated lidar direction can
differ from chassis direction. Real hardware must still replace the simulation
source with lower-controller gimbal yaw before validating localization.

`description.launch.py` sets the `robot_state_publisher` dynamic-TF ceiling to
`100 Hz` by default. This removes its upstream 20 Hz throttle while leaving the
actual update rate controlled by `/joint_states` (currently 50 Hz). It does not
interpolate or invent extra gimbal samples.

`rm_old_car_2026.urdf.xacro` is an experiment-only model for validating the
2027 stack on the available 2026 chassis. Its left MID360 transform uses the
measured 2026 old-car L1 extrinsic from
`rm2026_navigation/src/robot_msgs/urdf/robot.urdf` and
`rm2026_navigation/radar_multi_notice.md`: source left-handed
`(193.5, -175.3, 210.2) mm` becomes ROS `xyz=(0.1935, 0.1753,
0.2102)`, with `rpy=(0.332345, 0.287549, 1.668370)`. The old-car
`lio_imu_link` additionally applies the measured MID360 internal IMU-to-lidar
offset and sits at `xyz=(0.157178, 0.172624, 0.174374)` with the same rpy.
This is not the 2027 robot calibration, and the 2027 MID360/gimbal/IMU
extrinsics still need fresh measurement on the final chassis.
