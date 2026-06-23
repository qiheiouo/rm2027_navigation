# 2027 Architecture Decision

## Decision

The 2027 sentry navigation system will use a self-owned canonical skeleton plus selected open source modules.

This is not a plan to fully reimplement LIO, Nav2, controllers, serial protocol, or simulation from scratch. The team owns the top-level architecture, TF tree, topic contracts, chassis contract, launch boundaries, and acceptance criteria. Mature open source modules may be absorbed only after their interfaces, licenses, commits, and local changes are recorded.

## Recommended Main Chain

```text
LiDAR/IMU or bag/sim input
  -> LIO backend
  -> lio_adapter
  -> canonical TF
  -> Nav2
  -> /cmd_vel
  -> rm_chassis_interface
  -> sim/stub or serial hardware
```

## Gimbal-Mounted MID360 Decision

The 2027 robot is expected to use two MID360 LiDARs mounted on the gimbal, facing left and right, with an upward installation angle near 45 degrees. If the dual-LiDAR route proves too risky late in the season, the architecture may keep both sensors installed while using only one MID360 in software.

The main LIO IMU should be the internal IMU of the selected MID360. This keeps the LiDAR and IMU rigidly attached inside the same gimbal-mounted sensor group. A chassis-mounted IMU may be installed under the rotation axis as an auxiliary sensor, but it should be used for diagnostics, slip checks, latency checks, or future low-weight fusion rather than as the main IMU for gimbal-mounted LIO.

The canonical public TF remains `map -> odom -> base_link`. The sensor subtree changes to include the gimbal:

```text
base_link -> gimbal_yaw_link -> mid360_left_frame
                              -> mid360_right_frame
                              -> lio_imu_link
          -> base_imu_link
```

On the real robot, `base_link -> gimbal_yaw_link` is dynamic and depends on lower-controller gimbal yaw. Static direct `base_link -> mid360_*_frame` transforms are allowed only as Phase 1 zero-yaw placeholders for build, RViz, and early bag tests.

This decision adds a hard interface requirement for the lower controller: the upper computer needs gimbal yaw angle, validity, and timing information. Without that state, the system can estimate the gimbal sensor pose but cannot safely convert it into a validated `odom -> base_link` localization output during gimbal rotation.

## Why Not Fork PolarBear As The Main System

PolarBear remains the first reference object, but `pb2025_sentry_nav` should not be forked as the main 2027 system.

Reasons:

1. Its TF design uses `chassis`, `gimbal_yaw`, `gimbal_yaw_fake`, and `lidar_odom`, which is not our target `map -> odom -> base_link` canonical architecture with a documented gimbal-mounted sensor subtree.
2. It contains velocity and frame glue such as `fake_vel_transform`, `cmd_vel_nav2_result`, and `cmd_vel_controller`.
3. Many design choices are bound to its own gimbal, chassis, sensor layout, and season-specific engineering adaptations.
4. Point-LIO and IKFoM introduce license complexity, especially around GPL components.
5. Forking the whole repository would inherit its historical adaptations and submodule complexity.

## What To Absorb From PolarBear

The recommended route is not "fully self-developed". It is "self-owned canonical skeleton plus open source module absorption".

Priority references:

1. `rmu_gazebo_simulator`: RM fields, MID360/IMU simulation, chassis/referee simulation, Sim2Real workflow.
2. `small_gicp_relocalization`: Phase 2 candidate for global relocalization and `map -> odom` publication.
3. `pb_omni_pid_pursuit_controller`: holonomic Nav2 controller candidate for comparison with DWB/MPPI.
4. `pb_nav2_plugins`: IntensityVoxelLayer and BackUpFreeSpace for later RM-specific costmap/recovery work.
5. PolarBear map workflow: 2D occupancy map for Nav2 and 3D PCD map for relocalization.

## Nav2 Controller Decision

DWB remains the official Phase 1 minimum-loop controller and a lightweight
fallback. Phase 1.5 obstacle experiments showed that bounded DWB critic and
inflation searches did not provide both repeatable smoothness and rectangular
footprint clearance in the test world.

The official Nav2 Humble MPPI controller with its `Omni` motion model passed the
Phase 1.5C CPU, footprint-clearance, and ten-action repeatability gates at
commit `281dfaa`. It is the controller baseline for subsequent simulation work.
This is not final competition acceptance: the decision must be reviewed after
FAST-LIO, dual MID360 input, real serial latency, and real chassis dynamics are
measured together on the target minipc.

`pb_omni_pid_pursuit_controller` remains an Apache-2.0 fallback comparison. It
is not vendored or selected while the official MPPI controller satisfies the
current performance and safety gates.

Phase 1.5D confirmed costmap marking and clearing but did not establish safe
dynamic-obstacle avoidance. MPPI remains a navigation controller, not the sole
dynamic collision-safety layer. Real-robot work must add measured perception
latency and an independent stop/slow safety boundary before dynamic acceptance.

## Phase 2A LIO Backend Decision

The first backend integration uses `Draxran/FAST_LIO_MULTI_ROS2` pinned as a
separate GPL-2.0 submodule. We do not migrate the modified 2026 copy and do not
change upstream source in this increment.

`rm_lio_bringup` owns parameters, output namespace, and TF quarantine. Upstream
hard-codes an IMU-state child frame named `body`; the adapter may map that
backend-private semantic to `lio_imu_link`, then must calculate
`odom -> base_link` with the timestamped gimbal transform. Direct `body` to
`base_link` renaming remains forbidden.

## Serial And Hardware Protocol Decision

The 2027 system should not redesign the lower-controller serial protocol without a concrete reason.

The old `serial_task` contains hardware assets worth migrating:

1. Serial open, read, and write logic.
2. Packet format.
3. Existing frame envelope and validation behavior. The inspected legacy frame has no CRC.
4. `vx/vy/wz` or chassis-control packets.
5. Referee-system field parsing.
6. Existing agreements with the lower controller.

These assets should be migrated into:

1. `rm_serial_driver` for serial IO, packet framing, validation, and protocol statistics. A CRC may be added only through a coordinated versioned protocol extension.
2. `rm_chassis_interface` for chassis command encoding and chassis feedback parsing.
3. `rm_referee_interface` for referee-system parsing.

The old `serial_task` must not be attached back into the navigation chain as a whole. The following behaviors must be removed or isolated:

1. Publishing odometry.
2. Publishing TF.
3. Publishing navigation goals.
4. Calling Nav2 directly.
5. Mixing with BT or strategy logic.
6. Debug hardcoding and temporary competition fixes.

## Phase Policy

Phase 1 focuses on the minimum canonical loop. It does not connect to real serial hardware, referee, or competition BT.

Phase 1C may include compile-only migration of the known legacy command encoder and length-based frame parser, with unit tests. It preserves the legacy no-CRC wire format.

The old receive path does not define four individual wheel encoder values. If 2027 wheel-derived chassis velocity is calculated on the upper computer, Phase 2 requires an agreed lower-controller uplink containing four signed wheel samples, timing, sequence, units, and validity information.

Phase 2 connects to real serial hardware while keeping the existing protocol as compatible as possible.

Phase 3 adds referee, mission/BT, recovery behavior, and match robustness.
