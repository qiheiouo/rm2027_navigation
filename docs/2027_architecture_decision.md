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

## Why Not Fork PolarBear As The Main System

PolarBear remains the first reference object, but `pb2025_sentry_nav` should not be forked as the main 2027 system.

Reasons:

1. Its TF design uses `chassis`, `gimbal_yaw`, `gimbal_yaw_fake`, and `lidar_odom`, which is not our target `map -> odom -> base_link -> livox_frame/imu_link` canonical architecture.
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

## Serial And Hardware Protocol Decision

The 2027 system should not redesign the lower-controller serial protocol without a concrete reason.

The old `serial_task` contains hardware assets worth migrating:

1. Serial open, read, and write logic.
2. Packet format.
3. CRC and validation.
4. `vx/vy/wz` or chassis-control packets.
5. Referee-system field parsing.
6. Existing agreements with the lower controller.

These assets should be migrated into:

1. `rm_serial_driver` for serial IO, packet framing, and CRC.
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

Phase 1 may include compile-only migration of old serial parser/encoder code and unit tests for packet format, CRC, chassis command packets, and referee parsing.

Phase 2 connects to real serial hardware while keeping the existing protocol as compatible as possible.

Phase 3 adds referee, mission/BT, recovery behavior, and match robustness.
