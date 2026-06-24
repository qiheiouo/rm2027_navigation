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

Phase 1C includes compile-only migration of the known legacy 19-byte `vx/vy/wz` command encoder and length-based stream parser. Unit tests cover byte layout, finite values, fragmentation, noise, and resynchronization. Phase 2D separately records the discovered HPM 21-byte payload-CRC profile; it does not alter the legacy no-CRC bytes or open a device.

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

Phase 1.5C compared the official Nav2 Humble MPPI controller using its `Omni`
motion model and full-footprint cost critic. The low-power profile uses 10 Hz,
300 trajectories, 30 time steps, one optimization iteration, and disabled
visualization. At commit `281dfaa`, it passed ten of ten navigation actions
with no collision, padding intrusion, progress failure, recovery, or control
loop miss. Its median action time was `13.029 s` and minimum physical clearance
was `0.231 m` on the Phase 1.5 obstacle world.

MPPI is now the baseline controller for later Phase 1.5 simulation increments.
DWB remains the Phase 1 minimum-loop baseline and fallback. Final real-robot
selection remains provisional until LIO, dual MID360 input, serial latency, and
real chassis dynamics share the target minipc. The PolarBear omni PID pursuit
controller remains a fallback comparison if that full-stack test exceeds the
MPPI CPU budget.

Phase 1.5D extends only the simulation test surface. It reuses the accepted
MPPI profile and original Gazebo world, then spawns a 0.8 m static passage and
an optional obstacle moving laterally across the route. The moving-obstacle
controller owns only a simulation joint target and never publishes TF,
odometry, chassis commands, or navigation goals. This increment validates
costmap clearing, narrow-passage tracking, dynamic avoidance, and repeated
replanning; it is not a competition behavior or a claim of MID360 fidelity.

Phase 1.5D clearing and static-passage checks passed at commit `3c6fd62`.
Dynamic safety did not pass: all actions completed, but only two of ten met the
strict clearance gate and five runs contained scan-confirmed collision. Further
sine-obstacle parameter tuning is deferred until real perception, latency,
localization, chassis, and safety-layer evidence exists.

## Phase 2A LIO Integration

Phase 2A begins with build-only and bag-ready FAST-LIO Multi integration. The
backend is pinned as an external GPL-2.0 submodule and stays disabled by
default. `rm_lio_bringup` remaps backend `/Odometry` to
`/odometry/fast_lio_raw` and quarantines backend TF. `lio_adapter` remains the
only canonical `odom -> base_link` publisher.

Single MID360 is the first runtime profile; dual input is structurally present
but blocked on measured extrinsics and timing. Phase 2A does not yet claim real
MID360, rotating-gimbal, serial, global relocalization, or Nav2 full-stack
acceptance.

The inspected backend odometry does not yet provide accepted base-frame twist
or covariance. Phase 2A may validate pose and TF boundaries, but real Nav2
closure is blocked until velocity and covariance semantics are implemented and
measured.

## Phase 2B Canonical Twist

Phase 2B implements a backend-independent finite-difference velocity path in
`lio_adapter`. It differentiates consecutive canonical `odom -> base_link`
poses, not raw sensor poses, so a moving gimbal is removed before chassis twist
is calculated. Phase 1 fake/simulation inputs keep passthrough mode.

No-hardware tests cover holonomic translation, yaw rate, timestamp rejection,
outlier recovery, smoothing, and dynamic-gimbal cancellation. Fixed twist
covariance is a conservative interface placeholder. Real Nav2 acceptance still
requires measured velocity error, latency, covariance, and full-stack CPU.

Raw odometry waits in a bounded FIFO when its exact dynamic sensor TF has not
arrived yet. This preserves timestamp ordering and avoids latest-yaw fallback.
Queue timeout or overflow drops the affected sample and resets twist history.
