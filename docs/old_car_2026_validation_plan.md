# 2026 Old-Car Validation Plan

This document describes the temporary branch profile for validating parts of
the 2027 navigation stack on the available 2026 sentry chassis.

The goal is to gain hardware confidence before the 2027 robot is built. This
profile is not a shortcut for changing the 2027 canonical architecture.

## Scope

Useful old-car checks:

- Linux/Docker deployment on a real minipc-like environment.
- MID360 driver startup, topics, and raw data rates if the old car can carry the
  sensor safely.
- FAST-LIO backend startup and adapter conversion into canonical
  `/odometry/lio`.
- Canonical TF ownership:
  `map -> odom -> base_link -> gimbal_yaw_link -> lio_imu_link`.
- Nav2 MPPI obstacle avoidance with conservative speed limits.
- `/cmd_vel` generation and serial dry-run frame encoding.
- RViz visualization and operator debugging workflow.

Out of scope:

- Proving 2027 final extrinsics.
- Proving high-speed chassis spin behavior.
- Proving the 2027 dual-MID360 gimbal-mounted layout.
- Proving the final four-omni-wheel lower-controller protocol.
- Copying the old `serial_task` node into the 2027 navigation mainline.

## Added Experiment Profile

The branch adds:

- `rm_description/urdf/rm_old_car_2026.urdf.xacro`
- `rm_description/launch/old_car_2026_description.launch.py`
- `rm_lio_bringup/config/fast_lio_multi_old_car_2026.yaml`
- `rm_lio_bringup/config/lio_adapter_old_car_2026.yaml`
- `rm_navigation_bringup/launch/old_car_2026_validation.launch.py`

Safe defaults:

- `use_driver:=false`
- `use_lio_backend:=false`
- `use_nav2:=false`
- `use_serial_dry_run:=false`
- `use_real_serial:=false`

Therefore the default launch starts no real MID360 driver, no FAST-LIO backend,
no Nav2 controller, and no serial transport.

## Temporary Extrinsics

The old-car model uses the 2026 URDF laser joint as an experiment-only
placeholder:

```text
base_link -> gimbal_yaw_link: fixed xyz = 0 0 0, rpy = 0 0 0
gimbal_yaw_link -> lio_imu_link: xyz = 0.15 0.14 0.24, rpy = 0 0 0
gimbal_yaw_link -> mid360_left_frame: xyz = 0.15 0.14 0.24, rpy = 0 0 0
```

Source reference:

```text
rm2026_navigation/src/robot_msgs/urdf/car.urdf.xacro
joint_laser_x = 0.15
joint_laser_y = 0.14
joint_laser_z = 0.24
```

These values are not accepted as 2027 calibration. They are also not guaranteed
to match the old car exactly if the sensor is re-mounted.

## Serial Boundary

The old 2026 upper-computer `serial_task` is not copied into this project. It
mixed useful packet knowledge with forbidden responsibilities:

- publishing odometry;
- publishing or preparing navigation goals;
- using old glue topics;
- containing debug overrides;
- mixing referee, chassis, and navigation state.

The experiment launch may enable `rm_serial_driver` dry-run:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_serial_dry_run:=true serial_protocol_profile:=legacy_v1_no_crc
```

This only publishes mock encoded bytes on `/serial/mock_tx`. It does not open a
real serial device.

The old car may also use the gated real serial transport after dry-run passes:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_real_serial:=true serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 serial_baudrate:=115200
```

This is only for off-ground validation. The wheels must be lifted, the remote
must be able to switch from auto to manual to remove chassis force, and a person
must be ready to stop the robot. Do not enable `use_serial_dry_run` and
`use_real_serial` at the same time.

Real 2027 serial transport remains blocked until the 2027 firmware profile and
device permissions are confirmed.

## Incremental Validation Order

1. Description only:

   ```bash
   ros2 launch rm_description old_car_2026_description.launch.py
   ```

   Check TF:

   ```bash
   ros2 run tf2_ros tf2_echo base_link gimbal_yaw_link
   ros2 run tf2_ros tf2_echo gimbal_yaw_link lio_imu_link
   ```

   The old-car `gimbal_yaw_link` is fixed at zero yaw, so this description-only
   launch does not require a `/joint_states` publisher.

2. Safe integrated skeleton:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py
   ```

   Expected: no real driver, no FAST-LIO, no Nav2, no serial IO.

3. MID360 driver only:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
     use_driver:=true selected_side:=left
   ```

   Check raw topics and rates before enabling FAST-LIO.

4. FAST-LIO backend plus adapter:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
     use_driver:=true use_lio_backend:=true selected_side:=left
   ```

   Check `/odometry/fast_lio_raw`, `/odometry/lio`, and `odom -> base_link`.

5. Nav2 only after localization is stable:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
     use_driver:=true use_lio_backend:=true use_nav2:=true
   ```

   This requires a usable map and obstacle input. If the old car only provides
   raw MID360 point clouds, use the old-car Nav2 profile first. Its costmaps
   subscribe to `/livox/left/pointcloud` and require the fixed
   `mid360_left_frame` placeholder TF. This validates topic plumbing only; it
   does not prove final obstacle filtering or 2027 extrinsics.

6. Serial dry-run:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
     use_serial_dry_run:=true
   ```

   Check `/serial/mock_tx`; do not connect real serial from this profile yet.

7. Real serial off-ground test:

   ```bash
   ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
     use_real_serial:=true serial_protocol_profile:=legacy_v1_no_crc \
     serial_device:=/dev/ttyACM0
   ```

   Run only with wheels off-ground and remote/manual stop ready. Start with
   zero commands and verify watchdog behavior before sending any non-zero
   velocity.

## Acceptance Checks

Minimum checks for each run:

- No `/Pose_pub`, `/my_set_goal`, or `/nav_result`.
- `/tf` publishers are only expected canonical publishers.
- `map -> odom` has one owner.
- `odom -> base_link` has one owner.
- No `body` frame enters the public TF tree.
- No serial device is opened unless a later hardware-specific profile explicitly
  says so.
- Real serial tests must use the legacy no-CRC old-car profile unless captured
  packets prove otherwise.
- If Nav2 is enabled, `/cmd_vel` remains in `base_link` semantics.

## Exit Criteria

This branch is useful if it proves that the new architecture can run on real
hardware conditions without relying on the final 2027 chassis. It should be
retired or merged selectively once the 2027 robot exists.

Any old-car-only parameter that survives into main must be marked as temporary
and must have a corresponding item in
`docs/real_hardware_confirmation_checklist.md`.
