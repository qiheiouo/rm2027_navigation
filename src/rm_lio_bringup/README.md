# rm_lio_bringup

Phase 2A integration boundary for external LIO backends.

The first backend is the GPL-2.0 `fast_lio_multi` submodule pinned in
`src/fast_lio_multi`. This package owns only launch, parameter, topic, and TF
isolation. It does not copy or modify the backend algorithm.

## Safety Boundary

- `use_backend` defaults to `false`.
- Backend `/Odometry` is remapped to `/odometry/fast_lio_raw`.
- Backend `/tf` and `/tf_static` are remapped to quarantine topics.
- `lio_adapter` remains the only canonical `odom -> base_link` publisher.
- Backend frame `body` is accepted only as a code-inspected alias for its IMU
  state frame, `lio_imu_link`; it is never aliased directly to `base_link`.
- No MID360 driver, serial, referee, Nav2, or competition BT is started here.

The inspected backend does not currently fill odometry `twist`. Phase 2B adds
an adapter-side finite-difference estimate after canonical base-pose
conversion. Its filtering and fixed covariance are no-hardware defaults and
still require real trajectory validation. Upstream pose covariance remains
unaccepted.

FAST-LIO input uses a bounded exact-timestamp TF queue. This avoids losing
odometry when the equally fast gimbal TF publisher arrives a few milliseconds
after the sensor message, without weakening the contract to latest-TF lookup.

## Configurations

- `fast_lio_multi_single_mid360.yaml`: Phase 2A default, using the left MID360
  topic and selected internal IMU. Launch argument `selected_side:=right`
  overrides the input topic for the right-side fallback.
- `fast_lio_multi_dual_mid360.yaml`: dual-input placeholder. It must not be
  used on the real robot until both lidar-to-IMU extrinsics and timing have
  been measured.
- `lio_adapter_fast_lio_multi.yaml`: controlled conversion from backend
  `odom -> body` semantics to canonical `odom -> base_link` through the
  timestamped `base_link -> lio_imu_link` TF.
- `fast_lio_multi_old_car_2026.yaml`: experiment-only single-MID360 backend
  profile for validating the 2027 architecture on the available 2026 chassis.
- `lio_adapter_old_car_2026.yaml`: matching adapter profile using the old-car
  placeholder `base_link -> lio_imu_link` transform. It is not 2027
  calibration.

The zero extrinsics in these files are deliberate no-hardware placeholders,
not 2027 calibration results.

## Build-Only Launch

```bash
ros2 launch rm_lio_bringup fast_lio_multi_phase2a.launch.py
```

This only prints the boundary contract. To start the backend later on Linux:

```bash
ros2 launch rm_lio_bringup fast_lio_multi_phase2a.launch.py \
  use_backend:=true sensor_mode:=single update_method:=bundle
```

Starting the backend without valid point cloud, IMU, timing, and extrinsic
inputs is not a localization validation.
