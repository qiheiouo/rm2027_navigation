# rm_mid360_driver_bridge

Phase 1 MID360 driver bridge for the 2027 gimbal-mounted LiDAR layout.

This package does not vendor, copy, or replace `livox_ros_driver2`. It only keeps the local bridge boundary: placeholder Livox config files, canonical topic naming, and launch skeletons that can start `livox_ros_driver2_node` when the real driver is installed in the Linux workspace.

## Contract

Expected canonical topics:

- `/livox/left/lidar`: left MID360 Livox custom point cloud.
- `/livox/right/lidar`: right MID360 Livox custom point cloud.
- `/livox/left/pointcloud`: left MID360 PointCloud2, if that driver mode is enabled.
- `/livox/right/pointcloud`: right MID360 PointCloud2, if that driver mode is enabled.
- `/livox/lio_imu`: selected MID360 internal IMU used by the LIO backend.

This package must not publish localization TF, odometry, navigation goals, serial packets, referee data, or behavior-tree commands.

## Driver Policy

Phase 1 keeps `use_driver:=false` as the safe default so that the workspace builds and launch files can be inspected without real hardware or `livox_ros_driver2`.

When the official driver is installed and sourced on Linux:

```bash
ros2 launch rm_mid360_driver_bridge dual_mid360_driver.launch.py use_driver:=true
```

Single-device fallback:

```bash
ros2 launch rm_mid360_driver_bridge single_mid360_driver.launch.py side:=left use_driver:=true
```

## Placeholder Values

The JSON files contain placeholder host IP, LiDAR IP, ports, and timing settings. They are not 2027 final hardware configuration. The real values must be checked after MID360 network setup.

The `extrinsic_parameter` values intentionally stay zero in the Livox config. The canonical robot/sensor extrinsics belong in `rm_description` and the TF contract, not in scattered driver config.

## Notes for Dual MID360

The dual launch starts separate left/right driver nodes when `use_driver:=true`. This lets each side use its own frame id and topic remaps. If the selected `livox_ros_driver2` version behaves better with one multi-lidar node, use `dual_mid360_config.json` as the starting point and document the changed launch behavior before merging.
