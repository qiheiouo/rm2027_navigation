# rm_mid360_driver_bridge

Phase 1 MID360 driver bridge for the 2027 gimbal-mounted LiDAR layout.

This package does not copy or modify `livox_ros_driver2`. It only keeps the local bridge boundary: placeholder Livox config files, canonical topic naming, and launch skeletons that can start `livox_ros_driver2_node`.

The ROS2 Humble driver itself is tracked as the external submodule `src/livox_ros_driver2_humble`. See `docs/external/livox_ros_driver2_humble.md` for URL, commit, license, SDK install commands, and submodule notes.

## Contract

Expected canonical topics:

- `/livox/left/lidar`: left MID360 Livox custom point cloud.
- `/livox/right/lidar`: right MID360 Livox custom point cloud.
- `/livox/left/pointcloud`: left MID360 PointCloud2, if that driver mode is enabled.
- `/livox/right/pointcloud`: right MID360 PointCloud2, if that driver mode is enabled.
- `/livox/lio_imu_raw`: selected MID360 internal IMU directly remapped from the driver.
- `/livox/lio_imu`: canonical LIO IMU after `imu_frame_adapter` rewrites `header.frame_id` to `lio_imu_link`.
- `/livox/left/pointcloud_filtered`: old-car local-costmap PointCloud2 after bridge-side self filtering.

This package must not publish localization TF, odometry, navigation goals, serial packets, referee data, or behavior-tree commands.

## Old-Car PointCloud2 Filter

`pointcloud_self_filter_node` is an experiment-only bridge for the 2026 old car.
It removes points that fall inside a configurable self box in `base_link` before
Nav2 local costmap marking. The node keeps the published cloud in the original
sensor frame, so it does not take ownership of TF or localization data.
The old-car defaults live in `config/old_car_pointcloud_filter.yaml`.

Default old-car chain:

```text
/livox/left/pointcloud
  -> pointcloud_self_filter_node
  -> /livox/left/pointcloud_filtered
  -> local costmap voxel_layer
```

The filter can be disabled without changing costmap YAML:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true use_lio_backend:=true use_nav2:=true \
  pointcloud_filter_enabled:=false
```

With `pointcloud_filter_enabled:=false`, the node republishes the raw input
cloud to the filtered topic for A/B comparison. Full rollback is to point the
old-car local costmap topic back to `/livox/left/pointcloud`.

## Driver Policy

Phase 1 keeps `use_driver:=false` as the safe default so that the workspace builds and launch files can be inspected without real hardware or `livox_ros_driver2`.

When the driver submodule and Livox-SDK2 are installed and sourced on Linux:

```bash
ros2 launch rm_mid360_driver_bridge dual_mid360_driver.launch.py use_driver:=true
```

Single-device fallback:

```bash
ros2 launch rm_mid360_driver_bridge single_mid360_driver.launch.py side:=left use_driver:=true
```

The default `xfer_format` is `4`, matching the locally inspected Livox ROS driver behavior that publishes both Livox `CustomMsg` and `PointCloud2`. LIO backends usually need `CustomMsg` for per-point timing, while `PointCloud2` is useful for debugging, perception, and future costmap tools.

## Placeholder Values

The JSON files contain placeholder host IP, LiDAR IP, ports, and timing settings. They are not 2027 final hardware configuration. The current placeholder LiDAR IPs follow the 2026 historical pair `192.168.1.166` and `192.168.1.3`; the real values must be checked after MID360 network setup.

The `extrinsic_parameter` values intentionally stay zero in the Livox config. The canonical robot/sensor extrinsics belong in `rm_description` and the TF contract, not in scattered driver config.

The locally inspected Livox ROS driver hard-codes IMU `header.frame_id` as `livox_frame`, while point cloud messages use the launch `frame_id` parameter. The bridge therefore remaps selected driver IMU data to `/livox/lio_imu_raw`; `rm_localization_adapters/imu_frame_adapter` publishes canonical `/livox/lio_imu` with `header.frame_id=lio_imu_link`.

## Notes for Dual MID360

The dual launch starts separate left/right driver nodes when `use_driver:=true`. This lets each side use its own frame id and topic remaps. If the selected `livox_ros_driver2` version behaves better with one multi-lidar node, use `dual_mid360_config.json` as the starting point and document the changed launch behavior before merging.
