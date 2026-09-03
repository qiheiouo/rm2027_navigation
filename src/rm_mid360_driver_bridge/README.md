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
- `/local_scan`: optional old-car LaserScan projection from the filtered left MID360 point cloud for local costmap clearing experiments.

This package must not publish localization TF, odometry, navigation goals, serial packets, referee data, or behavior-tree commands.

## Old-Car PointCloud2 Filter

`pointcloud_self_filter_node` is an experiment-only bridge for the 2026 old car.
It removes points that fall inside a configurable self box in `base_link` before
Nav2 local costmap marking. The node keeps the published cloud in the original
sensor frame, so it does not take ownership of TF or localization data.
The old-car defaults live in `config/old_car_pointcloud_filter.yaml`.

The filter copies complete retained point records instead of reconstructing an
XYZ-only cloud. This preserves `intensity`, `tag`, `line`, the Livox
`timestamp` field, offsets, datatypes, `point_step` and endianness. Output is
compacted to `height=1`, but bytes inside each retained point record are not
reinterpreted. Downstream localization deskew depends on this preservation;
costmap consumers may continue to read only XYZ.

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

## Old-Car PointCloud2 To LaserScan Projection

`pointcloud_to_laserscan_node` is an experiment-only clearing aid for the 2026
old car. It projects `/livox/left/pointcloud_filtered` into `/local_scan` in
`base_link`. Empty angular bins are published as `+inf`, allowing Nav2
`ObstacleLayer` with `inf_is_valid: true` to raytrace free space after a person
or other dynamic obstacle leaves.

This is a switchable alternative to the VoxelLayer path above; it does not
replace the raw driver topic or take ownership of TF. Defaults live in
`config/old_car_pointcloud_to_laserscan.yaml`.

## Experimental Localization Scan Deskew

The same executable has an explicit strict-deskew profile for AMCL localization
only. It is separate from `/local_scan` costmap clearing:

```text
/livox/left/pointcloud_filtered + /odometry/lio + timestamped sensor TF
  -> per-point SE(3) compensation to the cloud header time
  -> /localization/scan in base_link
```

The verified old-car PointCloud2 convention is `timestamp` as `FLOAT64`
absolute nanoseconds, with the cloud header representing the first-point/base
time. The profile does not guess alternate field names or units.

If timing, odometry coverage, interpolation, frame semantics or timestamped TF
is invalid, the entire scan frame is dropped. Diagnostics are published on
`/localization/scan_deskew/active` and `/localization/scan_deskew/status`.
There is no degraded raw-scan fallback while active is reported true.

The strict profile is selected by the isolated
`old_car_2026_amcl_spin_candidate.launch.py` wrapper. It is not selected by the
normal old-car or competition launches and has not yet passed real high-speed
rotation acceptance. See
`docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`.

## Dual PointCloud2 Obstacle Fusion

`dual_pointcloud_obstacle_fusion.launch.py` filters the left and right streams
independently, transforms accepted clouds at their own timestamps, and
publishes `/points/obstacles_fused` in `base_link`. It is obstacle perception
only; the selected left MID360 remains the default single LIO source.

The runtime regression covers left-only, right-only, dual-stream and stale-input
degradation. It also checks the launch-time string-array parameter boundary so
the fusion process cannot silently exit while the two filter nodes remain up.

## Map-Bound Ramp Plane Filter

`ramp_plane_filter_node` is an inactive, shadow-output prototype for testing a
field-map-aware way to keep known traversable ramp surfaces out of obstacle
point clouds. It preserves the full PointCloud2 record for retained points and
publishes diagnostics with input, removed, and retained counts.

Removal requires all of the following:

- a timestamped transform from the cloud frame to `map`;
- an exact expected/active map ID and revision match;
- a point inside a configured ramp polygon;
- a point height within the region's expected plane tolerance.

Missing TF, malformed clouds, disabled filtering, missing regions, or a map
contract mismatch all pass the original cloud through. Returns above the
plane tolerance remain as obstacles. The supplied configuration is synthetic
and publishes only `/simulation/ramp/points_filtered_shadow`; it is not a
Nav2 input and is not approved for the real field.

`ramp_laserscan_filter_node` is the simulation-only active counterpart used by
`field_geometry_simulation.launch.py`. It evaluates finite LaserScan endpoints
with the same timestamped TF, map-contract, polygon, and expected-plane gates;
matching ranges become infinity before the scan is published on `/scan`.
Contract mismatch or invalid input is fail-closed passthrough. Its supplied
configuration is bound to `synthetic_competition_ramps/candidate_11_15_v1` and
must not be reused on a real map.

`automatic_ramp_filter_node` is the unlabelled counterpart. It downsamples the
lowest return in robot-relative XY cells, fits only gravity-frame planes within
the configured traversable slope range, checks minimum width/length/height,
and requires the same odom-frame plane for consecutive clouds. Until confirmed,
or after its short missed-frame TTL expires, it publishes the original cloud.
Only returns close to the confirmed plane are removed, so protruding obstacles
remain. Its default launch integration is shadow-only; active use still needs a
clear-ramp A/B test and a PGM whose corresponding corridor is navigable.

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

The single-device fallback keeps `xfer_format=4`, which publishes the driver's
native Livox `CustomMsg` and PointCloud2 together. The dual-device launch cannot
use that mode safely with this driver: its per-device publisher bookkeeping
does not create two differently typed publishers for the same device topic.

The dual-device default is therefore `lio_input_mode:=native_custom`. The
driver publishes its native CustomMsg (`xfer_format=1`) per device; the bridge
passes `timebase` and every `offset_time` unchanged to FAST-LIO and derives a
PointCloud2 copy for obstacle perception. This avoids reconstructing LIO timing
from the PointCloud2 `FLOAT64` absolute timestamp. The previous conversion
direction remains available as `lio_input_mode:=reconstructed_custom` only for
controlled A/B tests.

## Placeholder Values

The JSON files contain placeholder host IP, LiDAR IP, ports, and timing settings. They are not 2027 final hardware configuration. The current old-car assignment is left/L1 `192.168.1.3` and right/L2 `192.168.1.166`; the real values must still be checked after MID360 network setup.

The `extrinsic_parameter` values intentionally stay zero in the Livox config. The canonical robot/sensor extrinsics belong in `rm_description` and the TF contract, not in scattered driver config.

The locally inspected Livox ROS driver hard-codes IMU `header.frame_id` as `livox_frame`, while point cloud messages use the launch `frame_id` parameter. The bridge therefore remaps selected driver IMU data to `/livox/lio_imu_raw`; `rm_localization_adapters/imu_frame_adapter` publishes canonical `/livox/lio_imu` with `header.frame_id=lio_imu_link`.

## Notes for Dual MID360

The dual launch starts one multi-device `livox_ros_driver2` process. Two SDK
instances are not safe on the old car: each instance discovers both MID360s and
the later process redirects both devices to its own UDP ports. The single
process loads `dual_mid360_config.json`. By default,
`livox_custom_adapter_node` preserves each IP-specific native CustomMsg for LIO
and builds the corresponding canonical PointCloud2 stream for perception. The
legacy `livox_pointcloud_adapter_node` path is opt-in. The left MID360 remains
the default LIO and IMU source; the right MID360 is obstacle perception only.
