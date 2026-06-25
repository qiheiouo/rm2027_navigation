# Phase 2G PointCloud2 Obstacle Boundary

## Goal

Phase 2G checks that Nav2 can consume a `sensor_msgs/PointCloud2` obstacle
stream before real MID360 hardware is available.

The previous obstacle simulations used `/scan`. Real MID360 integration will
primarily provide point clouds, so this phase adds a narrow no-hardware test:

```text
fake PointCloud2 obstacle -> Nav2 VoxelLayer -> costmaps -> MPPI
```

This phase does not model the real MID360 scan pattern, Livox packet timing,
dual-lidar overlap, self-occlusion, or gimbal rotation noise.

## Added Boundary

`rm_simulation fake_pointcloud_obstacle_publisher` publishes:

- topic: `/points/obstacles`
- type: `sensor_msgs/msg/PointCloud2`
- frame: `sim_lidar_link`

It deliberately does not publish:

- `/tf` or `/tf_static`
- odometry
- `/cmd_vel`
- navigation goals
- serial/referee/mission data

`sim_lidar_link` is still a simulation-only frame enabled by the description
launch. It is not a real MID360 frame.

## Nav2 Profile

`rm_nav_config/config/nav2_phase2g_pointcloud.yaml` keeps the accepted MPPI
baseline and replaces the LaserScan obstacle layers with VoxelLayer inputs:

```text
local_costmap/global_costmap -> voxel_layer -> /points/obstacles
```

The profile intentionally does not subscribe to `/scan`.

## Launch

Direct launch:

```bash
ros2 launch rm_simulation phase2g_pointcloud_obstacle.launch.py
```

Top-level simulation profile:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py scenario:=pointcloud
```

Both start a Gazebo holonomic chassis, canonical localization adapters, Nav2,
the chassis stub, and the fake point-cloud obstacle publisher. The scan frame
adapter is disabled in this profile.

## Expected Checks

1. `colcon build` succeeds.
2. `/points/obstacles` publishes `PointCloud2` with `frame_id=sim_lidar_link`.
3. `/scan` is not consumed by costmaps in this profile.
4. local/global costmaps subscribe to `/points/obstacles`.
5. costmaps show lethal/inflated cells near the fake obstacle.
6. A small `NavigateToPose` target around the obstacle succeeds or, if it
   fails, the failure is classified by planner/controller/costmap logs rather
   than missing point-cloud input.
7. `/tf` publishers remain the canonical owners only:
   `robot_state_publisher`, `map_odom_stub`, and `lio_adapter`.
8. No real MID360 driver, FAST-LIO backend, serial, referee, or competition BT
   starts.

## Real-Hardware Follow-Up

After MID360 is available, replace only the source topic and frame boundary:

- left/right MID360 raw cloud topics from `livox_ros_driver2`;
- selected or fused obstacle cloud topic;
- measured `mid360_*_frame` extrinsics;
- gimbal yaw timing;
- filtering/downsampling policy;
- self-hit removal and chassis/gimbal occlusion masks.

Do not change the canonical TF ownership to make the costmap look right. If
obstacles appear rotated or mirrored, fix the sensor frame/extrinsic contract
first.
