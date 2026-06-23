# 2027 Phase 1 Plan

## Goal

Phase 1 is not a complete competition system. Its goal is the minimum canonical navigation loop:

```text
Gimbal-mounted MID360 LiDAR/IMU or bag/sim input
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

The real 2027 sensor layout is expected to be dual MID360 on the gimbal. Phase 1 may use a zero-yaw gimbal placeholder for build, RViz, and early bag tests, but rotating-gimbal hardware validation requires a real `base_link -> gimbal_yaw_link` state.

## map_odom_stub

Phase 1 uses exactly one `map_odom_stub` node to publish identity `map -> odom`.

`map_odom_stub` is a temporary `global_localization` placeholder. It publishes dynamic TF on `/tf`, not `/tf_static`.

When Phase 2 connects `small_gicp`, `scan_to_map`, NDT, or another global localization backend, `map_odom_stub` must be removed.

## LIO Backend

Phase 1 baseline should start from the locally available `FAST_LIO_MULTI_ROS2` backend and wrap it with `lio_adapter`.

Point-LIO is an experimental reference only. It must not enter the main line unless GPL-related license policy is confirmed.

If a LIO backend publishes `odom -> body`, `odom -> base_link`, or any equivalent odometry TF itself, the adapter must disable, intercept, remap, or replace that TF. The backend and `lio_adapter` must never publish the same canonical transform at the same time.

For the gimbal-mounted MID360 layout, the preferred LIO IMU is the internal IMU of one selected MID360. `lio_adapter` must not convert `odom -> lio_imu_link` or `odom -> mid360_*_frame` into `odom -> base_link` by renaming `child_frame_id`. It must use the gimbal yaw and measured sensor extrinsics, or explicitly stay in zero-yaw placeholder mode.

## Nav2 Controller

Phase 1 default controller should be official Nav2 DWB with holonomic configuration.

`pb_omni_pid_pursuit_controller` is not the Phase 1 default controller. It may be introduced in Phase 1.5 as an A/B comparison against DWB or MPPI.

## Serial, Referee, And BT

Phase 1 does not connect to real serial hardware.

Phase 1C includes compile-only migration of the known legacy 19-byte `vx/vy/wz` command encoder and length-based stream parser. Unit tests cover byte layout, finite values, fragmentation, noise, and resynchronization. The inspected legacy protocol has no CRC, so Phase 1C does not add one.

The old receive path does not provide four clearly identified wheel encoder values. A 2027 uplink extension is required if the upper computer will calculate four-omni-wheel feedback. The wire layout remains blocked on wheel order, units, gear ratio, encoder resolution, timing, and sign conventions from the electrical and mechanical teams.

Phase 2 connects to real serial hardware and should keep the lower-controller protocol compatible where possible.

Phase 1 does not connect referee.

Phase 1 does not connect competition BT.

Phase 3 may connect mission or BT only through standard Nav2 action interfaces.

Phase 1 does not require a real gimbal serial connection, but it must reserve the interface for gimbal yaw angle, validity, and timestamp or sequence information. Phase 2 hardware validation must confirm that this state is fresh enough for rotating-gimbal localization.

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

1. TF tree follows `map -> odom -> base_link -> gimbal_yaw_link -> mid360_left_frame/mid360_right_frame/lio_imu_link`, with optional `base_link -> base_imu_link`.
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
12. Phase 1 zero-yaw gimbal placeholders are clearly marked and are not accepted as final real-robot extrinsics.
13. Windows stage can complete code review and protocol unit tests. Linux stage can complete build, simulation, or bag replay. Real hardware stage validates MID360, gimbal yaw timing, serial, latency, and robustness.
14. `rm_serial_driver` legacy codec unit tests pass without opening a serial device.

## Phase 1.5 Gazebo Increment

After Phase 1A, 1B, and 1C pass, Phase 1.5 replaces the fake motion integrator
with a Gazebo Fortress physics loop. Its first increment validates only the
placeholder holonomic chassis, canonical odometry adapter, Nav2, command
boundary, and optional RViz.

Simulation ground truth is published on `/simulation/ground_truth/odom` and is
consumed as a raw test input by `lio_adapter`. Gazebo TF is not bridged to ROS.
The first increment does not claim obstacle avoidance, MID360 simulation,
FAST-LIO behavior, final chassis dynamics, or Sim2Real accuracy.

Phase 1.5A basic physics passed on Linux. Phase 1.5B adds a simulation-only
planar scan, obstacle costmaps, and a blocking test object so DWB planning and
control can be verified against sensed obstacles. It does not represent the
MID360 scan pattern or final 3D perception chain.

Phase 1.5B established the DWB obstacle-avoidance baseline. Reducing the DWB
sampling load removed controller-loop overruns, but bounded critic and
inflation searches did not produce one profile that was both repeatably smooth
and safely clear of the rectangular robot footprint. Those experimental
profiles remain uncommitted; DWB remains the Phase 1 baseline rather than an
accepted competition controller.

Phase 1.5C compares the official Nav2 Humble MPPI controller using its `Omni`
motion model and full-footprint cost critic. The first profile is deliberately
limited to 10 Hz, 300 trajectories, 30 time steps, one optimization iteration,
and disabled visualization for the low-power target. It must pass CPU timing,
obstacle clearance, recovery, and repeatability gates before any default
controller decision. The PolarBear omni PID pursuit controller remains the
next comparison candidate if MPPI is too expensive or insufficiently robust.
