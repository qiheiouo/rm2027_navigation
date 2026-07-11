# Pre-Hardware Freeze Status

## Purpose

This document summarizes the navigation stack state before the 2027 robot,
MID360 installation, measured extrinsics, lower-controller firmware and real
field maps are available.

The current repository is not a complete competition system, but the main
software boundaries are now in place and no longer blocked by old 2026
architecture coupling.

## Frozen Baseline Scope

The pre-hardware baseline includes:

- canonical TF ownership and adapters;
- gimbal-mounted single/dual MID360 frame layout placeholders;
- Livox driver and FAST-LIO Multi boundaries, disabled by default;
- canonical `/odometry/lio` output and `odom -> base_link` ownership;
- MPPI as the current Nav2 controller baseline;
- Gazebo holonomic chassis simulation;
- LaserScan and PointCloud2 obstacle boundary tests;
- map-bundle validation and `nav2_map_server` deployment gate;
- compile/test-only serial protocol profiles;
- serial dry-run `/cmd_vel -> /serial/mock_tx` runtime path;
- no-hardware launch profiles for navigation, simulation, bag replay, mapping
  guard, map deployment and serial dry-run.

## Validated Without Real Hardware

The following have passed Linux/Docker validation:

1. workspace build and package discovery;
2. canonical static/dynamic TF skeleton;
3. fake LIO odometry through `lio_adapter`;
4. Nav2 no-hardware `/cmd_vel` loop;
5. Gazebo holonomic movement;
6. MPPI static-obstacle simulation profile;
7. map bundle validation and map-server launch gate;
8. PointCloud2 obstacle input through Nav2 VoxelLayer;
9. legacy and HPM CRC command-frame codecs;
10. serial dry-run mock frame output and watchdog behavior;
11. safety boundaries preventing serial/referee/mission/BT from publishing
    localization TF or navigation goals.

## Explicitly Not Accepted Yet

The following are not accepted before real hardware:

- real MID360 IP/network/PTP behavior;
- dual-MID360 synchronization and bandwidth;
- measured `gimbal_yaw_link -> mid360_*_frame` extrinsics;
- selected MID360 internal IMU axis convention;
- gimbal yaw timestamp, delay and high-speed motion behavior;
- FAST-LIO performance with real point clouds and IMU;
- real obstacle filtering, self-hit removal and occlusion masks;
- real serial device permissions, baud rate and firmware profile;
- four-omni-wheel feedback packet layout;
- chassis sign conventions and velocity limits;
- real map PCD/occupancy alignment;
- global relocalization backend convergence;
- referee data and competition strategy behavior.

## Current Runtime Entry Points

Safe inspection:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py
```

Simulation:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py scenario:=basic
ros2 launch rm_navigation_bringup simulation.launch.py scenario:=pointcloud
```

Map deployment gate:

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  map_bundle_manifest:=/path/to/approved.bundle.yaml
```

Serial dry-run:

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py \
  protocol_profile:=legacy_v1_no_crc
```

Mapping safe default:

```bash
ros2 launch rm_navigation_bringup mapping.launch.py
```

The default still starts no mapper. Phase 2I now provides an explicit
`enable_mapping:=true` path for controlled candidate PCD and occupancy-map
export. See `docs/phase2i_managed_mapping.md`; it does not weaken the deployment
approval gate.

## Freeze Rules

Until hardware arrives:

1. do not rename canonical frames;
2. do not add a second `map -> odom` publisher;
3. do not add a second `odom -> base_link` publisher;
4. do not make serial/referee/mission publish localization TF or nav goals;
5. do not enable real drivers by default;
6. do not mark synthetic maps as deployable;
7. do not tune around fake simulation artifacts as if they were real robot
   behavior.

## Next Real-Hardware Gates

The first hardware sessions should prove, in order:

1. Docker and ROS2 environment on the minipc;
2. MID360 driver-only topics, rates and frames;
3. PTP/time synchronization;
4. gimbal yaw stream timing and sign;
5. static TF/extrinsic sanity in RViz;
6. FAST-LIO backend quarantined output;
7. `lio_adapter` canonical `/odometry/lio`;
8. map deployment and global-localization backend;
9. Nav2 `/cmd_vel` with chassis disconnected or wheels lifted;
10. serial transport and chassis motion limits;
11. complete low-speed closed loop;
12. match-specific behavior and referee integration.

Only after these gates pass should competition strategy or chase behavior be
treated as the main blocker.
