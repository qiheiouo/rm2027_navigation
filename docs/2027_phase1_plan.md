# 2027 Phase 1 Plan

## Goal

Phase 1 is not a complete competition system. Its goal is the minimum canonical navigation loop:

```text
LiDAR/IMU or bag/sim input
  -> LIO odom
  -> canonical TF
  -> Nav2
  -> /cmd_vel
  -> chassis_interface
```

## Main Chain

1. LIO backend produces localization output.
2. `lio_adapter` normalizes the output and publishes `/odometry/lio`.
3. `/odometry/lio` uses `header.frame_id=odom` and `child_frame_id=base_link`.
4. `lio_adapter` is the only external publisher of canonical `odom -> base_link`.
5. `map_odom_stub` publishes identity `map -> odom` as a dynamic TF.
6. Nav2 consumes canonical TF and `/odometry/lio`.
7. Nav2 publishes `/cmd_vel`.
8. `rm_chassis_interface` consumes `/cmd_vel`.
9. Phase 1 uses a stub or simulator output for chassis behavior.

## map_odom_stub

Phase 1 uses exactly one `map_odom_stub` node to publish identity `map -> odom`.

`map_odom_stub` is a temporary `global_localization` placeholder. It publishes dynamic TF on `/tf`, not `/tf_static`.

When Phase 2 connects `small_gicp`, `scan_to_map`, NDT, or another global localization backend, `map_odom_stub` must be removed.

## LIO Backend

Phase 1 baseline should start from the locally available `FAST_LIO_MULTI_ROS2` backend and wrap it with `lio_adapter`.

Point-LIO is an experimental reference only. It must not enter the main line unless GPL-related license policy is confirmed.

If a LIO backend publishes `odom -> body`, `odom -> base_link`, or any equivalent odometry TF itself, the adapter must disable, intercept, remap, or replace that TF. The backend and `lio_adapter` must never publish the same canonical transform at the same time.

## Nav2 Controller

Phase 1 default controller should be official Nav2 DWB with holonomic configuration.

`pb_omni_pid_pursuit_controller` is not the Phase 1 default controller. It may be introduced in Phase 1.5 as an A/B comparison against DWB or MPPI.

## Serial, Referee, And BT

Phase 1 does not connect to real serial hardware.

Phase 1 may include compile-only migration of old serial protocol parser and encoder code. Unit tests may cover packet format, CRC, `vx/vy/wz` control packets, chassis feedback, and referee field parsing.

Phase 2 connects to real serial hardware and should keep the lower-controller protocol compatible where possible.

Phase 1 does not connect referee.

Phase 1 does not connect competition BT.

Phase 3 may connect mission or BT only through standard Nav2 action interfaces.

## Explicit Non-Goals

1. Do not connect real hardware serial in Phase 1.
2. Do not connect referee in Phase 1.
3. Do not connect competition BT in Phase 1.
4. Do not connect `small_gicp` in Phase 1.
5. Do not use `pb_omni_pid_pursuit_controller` as the default Phase 1 controller.
6. Do not introduce Point-LIO into the main line.
7. Do not use PolarBear as a whole-repository baseline.
8. Do not use wheel-end odometry as the primary localization source.
9. Do not restore old temporary topic glue such as `/Pose_pub`, `/my_set_goal`, or `/nav_result`.

## Acceptance Criteria

1. TF tree is `map -> odom -> base_link -> livox_frame/imu_link`.
2. There is no duplicate TF publication.
3. `/odometry/lio` exists and uses `frame_id=odom`, `child_frame_id=base_link`.
4. `map_odom_stub` is the only `map -> odom` publisher.
5. Nav2 accepts `NavigateToPose` goals.
6. Nav2 publishes `/cmd_vel`.
7. `rm_chassis_interface` consumes `vx`, `vy`, and `wz` in `base_link`.
8. `serial`, `chassis`, and `referee` modules do not publish localization TF, do not publish navigation goals, and do not call Nav2 directly.
9. Phase 1 does not connect BT.
10. Phase 3 mission or BT, if added later, may call navigation only through standard Nav2 action interfaces.
11. Old topic glue such as `/Pose_pub`, `/my_set_goal`, and `/nav_result` is not restored.
12. Windows stage can complete code review and protocol unit tests. Linux stage can complete build, simulation, or bag replay. Real hardware stage validates MID360, serial, latency, and robustness.
