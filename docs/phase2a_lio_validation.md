# Phase 2A LIO Integration Validation

This gate validates dependency completeness, buildability, launch safety, raw
odometry adaptation, and TF ownership. It does not claim real MID360 or LIO
accuracy without recorded sensor data.

This Phase 2A gate does not accept upstream odometry `twist` or covariance for
Nav2. The inspected backend leaves `twist` empty and publishes before
refreshing pose covariance. Phase 2B adds a separately tested adapter-side
twist estimate; real covariance and full Nav2 acceptance remain deferred.

## Clone And Dependencies

```bash
git clone --recurse-submodules https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
git submodule update --init --recursive
git submodule status --recursive
```

Expected external commits include:

```text
e7864a63a7e2a1ad62ec8fd75fcb9149db08e321 src/fast_lio_multi
0438b0daa6ccfaaad9f65f45a3addc318b19ae7a src/fast_lio_multi/include/ikd-Tree
2a2029a6e62a2196b280be6ec00bb2418065b8e0 src/livox_ros_driver2_humble
```

Inside the project Docker:

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon list
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`colcon list` must include `fast_lio_multi` and `rm_lio_bringup` in addition to
the existing packages. All packages must build before runtime tests continue.

## Safe-Default Launch

```bash
ros2 launch rm_lio_bringup fast_lio_multi_phase2a.launch.py
```

Expected: the launch exits normally after reporting `use_backend:=false` and
does not start `laserMapping_bundle`, `laserMapping_async`, or
`laserMapping_adaptive`.

```bash
ros2 launch rm_navigation_bringup phase2a_lio_bringup.launch.py \
  use_driver:=false use_lio_backend:=false use_rviz:=false
```

Expected running nodes include robot description, `map_odom_stub`,
`lio_adapter`, `gimbal_state_adapter`, and `imu_frame_adapter`. No MID360
driver, FAST-LIO, Nav2, serial, referee, or competition BT is started.

## Adapter Alias Test Without Hardware

With the safe Phase 2A bringup running, publish one synthetic upstream message.
Its pose equals the current placeholder `base_link -> lio_imu_link` transform:

```bash
ros2 topic pub --once /odometry/fast_lio_raw nav_msgs/msg/Odometry \
"{header: {frame_id: odom}, child_frame_id: body, pose: {pose: {position: {x: 0.0, y: 0.12, z: 0.35}, orientation: {w: 1.0}}}}"
```

Then check:

```bash
ros2 topic echo --once /odometry/lio
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link gimbal_yaw_link
ros2 run tf2_ros tf2_echo gimbal_yaw_link lio_imu_link
```

Expected `/odometry/lio`:

```text
header.frame_id: odom
child_frame_id: base_link
position approximately [0, 0, 0]
orientation identity
```

This proves the adapter used
`T_odom_base = T_odom_lio_imu * inverse(T_base_lio_imu)`. Merely replacing
`child_frame_id` would incorrectly leave the input translation in the output.

Also publish a message with an unexpected parent frame and confirm it is
rejected:

```bash
ros2 topic pub --once /odometry/fast_lio_raw nav_msgs/msg/Odometry \
"{header: {frame_id: map}, child_frame_id: body, pose: {pose: {orientation: {w: 1.0}}}}"
```

No new `/odometry/lio` sample may be produced from that invalid message.

## TF Ownership And Quarantine

```bash
ros2 run tf2_tools view_frames
ros2 topic info /tf --verbose
ros2 topic list | grep fast_lio
```

Acceptance:

1. `map_odom_stub` is the only `map -> odom` publisher.
2. `lio_adapter` is the only `odom -> base_link` publisher after valid raw
   odometry arrives.
3. `body` is absent from the canonical TF tree.
4. No node consumes `/fast_lio/_quarantine/tf` or
   `/fast_lio/_quarantine/tf_static`.
5. Description TF retains its documented owners.

## Optional Backend Startup Without Sensor Data

This checks process startup only; it is not required to publish odometry:

```bash
ros2 launch rm_lio_bringup fast_lio_multi_phase2a.launch.py \
  use_backend:=true sensor_mode:=single update_method:=bundle
```

Expected: `fast_lio_multi_backend` starts and waits for `/livox/left/lidar`
and `/livox/lio_imu`. It must not publish on canonical `/tf`. Stop it after the
startup check rather than treating the lack of odometry as a failure.

For the right-side single-lidar fallback, add `selected_side:=right` and verify
the backend subscribes to `/livox/right/lidar`, not the left topic.

## Deferred Runtime Gate

When a recorded bag or real MID360 is available, add evidence for:

- input message types, rates, timestamps, and frame conventions;
- non-empty `/odometry/fast_lio_raw` with `frame_id=odom` and backend `body`;
- canonical `/odometry/lio` and unique `odom -> base_link`;
- point-cloud/IMU synchronization and no sustained drop;
- gimbal-yaw timestamp interpolation during motion;
- CPU, memory, drift, relocalization behavior, and failure recovery.
- validated base-frame linear/angular velocity, covariance, and their latency;
  see the separate Phase 2B gate for the no-hardware estimator baseline.

Do not enable the dual config until the two-lidar extrinsics and synchronization
items in `real_hardware_confirmation_checklist.md` are closed.

## Recorded Result

Commit `f8128ac` passed the Linux Docker gate: all eleven packages built in
Release mode, all three nested external revisions were complete, safe defaults
started no hardware/backend nodes, nonzero-timestamp `body` odometry converted
to canonical `base_link`, invalid parent frames were rejected, backend TF was
quarantined, and left/right single-lidar subscriptions were correct. No source
files were changed during validation.

Phase 2A is therefore complete for build and interface boundaries. Real sensor
data, upstream pose covariance, and full Nav2 acceptance remain outside that
result.
