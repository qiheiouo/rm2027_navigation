# rm_nav_config

Phase 1 navigation configuration package.

This package stores placeholder Nav2 configuration and test maps. It does not contain final competition parameters.

`config/nav2_phase1.yaml` remains the no-sensor Phase 1 baseline.
`config/nav2_phase1_5_gazebo.yaml` is simulation-only and adds `/scan`
obstacle layers, a placeholder rectangular footprint, and the DWB
`BaseObstacle` critic. It is not final MID360 or competition tuning.
`config/nav2_phase1_5_mppi.yaml` is the accepted Phase 1.5 simulation baseline.
It keeps DWB as the Phase 1 fallback and uses a conservative 10 Hz, `300 x 30`
CPU budget for the Nav2 Humble omnidirectional model. Real-robot acceptance is
still blocked on the complete LIO, sensor, serial, and chassis workload.
`config/nav2_phase2f_deployment.yaml` keeps the MPPI baseline but expects an
approved map bundle to be loaded through `nav2_map_server`. Its global costmap
uses a static map layer plus the current obstacle layer. It is still not final
competition tuning because real MID360 point-cloud obstacle input is not wired
yet.
`config/nav2_phase2g_pointcloud.yaml` is a simulation-only boundary profile for
PointCloud2 obstacle input. It uses Nav2 VoxelLayer on `/points/obstacles` and
does not consume `/scan`.
`config/nav2_old_car_2026_left.yaml` is an experiment-only old-car profile for
real left MID360 PointCloud2 input. Its local costmap subscribes to
`/livox/left/pointcloud_filtered`, produced by `rm_mid360_driver_bridge` from
the raw `/livox/left/pointcloud`, so chassis/self returns can be filtered before
VoxelLayer marking. It aligns Nav2 output limits with the gated real serial
transport clamp. Its current limits are an old-car landing-debug profile chosen
to stay above the observed chassis dead zone while remaining below
competition-speed tuning. Its rolling costmaps are intentionally wider than the
first dry-run profile to keep the old-car MID360 sensor origin inside the
window, uses wall time instead of simulation time, and gives VoxelLayer a
generous vertical span for the approximate old-car mounting. It is not 2027
competition tuning.
`config/nav2_old_car_2026_left_local_scan.yaml` is an experiment-only
alternative for dynamic-obstacle clearing. It keeps the same old-car Nav2
limits and global costmap policy, but local costmap consumes `/local_scan`
through `nav2_costmap_2d::ObstacleLayer` with `inf_is_valid: true`. The scan is
projected from `/livox/left/pointcloud_filtered` by
`rm_mid360_driver_bridge/pointcloud_to_laserscan_node`.
`config/nav2_old_car_2026_left_stvl.yaml` is an experiment-only profile for
dynamic-obstacle residuals. It keeps the default old-car controller, planner,
serial, behavior-tree, footprint, inflation, and global-costmap policy, but
replaces the local VoxelLayer with
`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`. It is enabled only by
explicitly passing that file through `nav2_params:=...`. The project Docker
image installs the Humble binary package
`ros-humble-spatio-temporal-voxel-layer=2.3.4-1jammy.20260607.082704` and its
matching `ros-humble-openvdb-vendor` dependency. Do not replace this with a
floating source checkout from the upstream `ros2` branch; that branch currently
depends on newer packages such as `nav2_ros_common` and `point_cloud_transport`
that are not part of this old-car Humble baseline.
`config/nav2_old_car_2026_dual_stvl.yaml` is the explicit counterpart for the
optional `/points/obstacles_fused` stream. It keeps localization on the left
MID360 and is never selected automatically.
`rviz/old_car_2026.rviz` is the matching visualization profile for old-car
debugging. It shows TF, RobotModel, left MID360 point cloud, LIO odometry,
local/global costmaps, and Nav2 plans.
In this RViz profile, `Global Plan` is `/plan` from `planner_server` and the
global costmap. It may cross obstacles that exist only in the local costmap.
Use `/local_plan` or MPPI visualization, when available, to inspect controller
local intent.
For the old-car profile, dynamic MID360 PointCloud2 is intentionally used only
by the local costmap, and it enters through the filtered topic above. The global
costmap does not subscribe to the raw point cloud because PointCloud2 does not
provide LaserScan-style max-range free rays, and feeding it globally caused
self/ground ghosts to persist while the robot moved.

Both Phase 1.5 LaserScan profiles enable `inf_is_valid` so Gazebo max-range
returns can clear cells previously occupied by moving simulated obstacles.

## Costmap Inspector

`costmap_inspector` is a read-only helper for old-car costmap debugging. It
subscribes to `/local_costmap/costmap_raw` (`nav2_msgs/Costmap`) or
`/local_costmap/costmap` (`nav_msgs/OccupancyGrid`), looks up `base_link` in
the costmap frame, and summarizes a square window around the robot.

Example one-shot checks:

```bash
ros2 run rm_nav_config costmap_inspector \
  --topic /local_costmap/costmap_raw \
  --window-size 2.0

ros2 run rm_nav_config costmap_inspector \
  --topic /local_costmap/costmap \
  --window-size 2.0 \
  --csv /tmp/local_costmap_window.csv
```

The summary separates `free`, `intermediate`, `inscribed`, `lethal`, and
`unknown` cells. Use it before and after dynamic obstacle tests to check
whether lethal or inscribed residuals actually disappear rather than relying on
RViz color alone.

Phase 1 constraints:

- `global_frame: map`
- `robot_base_frame: base_link`
- `odom_frame: odom`
- `odom_topic: /odometry/lio`
- default controller: DWB holonomic
- Phase 1.5 simulation controller: MPPI Omni
- Phase 2G point-cloud obstacle input: Nav2 VoxelLayer

`pb_omni_pid_pursuit_controller` remains a fallback comparison if the complete
real-robot workload exceeds the accepted MPPI profile's CPU budget.
