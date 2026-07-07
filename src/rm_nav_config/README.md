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
`rviz/old_car_2026.rviz` is the matching visualization profile for old-car
debugging. It shows TF, RobotModel, left MID360 point cloud, LIO odometry,
local/global costmaps, and Nav2 plans.
For the old-car profile, dynamic MID360 PointCloud2 is intentionally used only
by the local costmap, and it enters through the filtered topic above. The global
costmap does not subscribe to the raw point cloud because PointCloud2 does not
provide LaserScan-style max-range free rays, and feeding it globally caused
self/ground ghosts to persist while the robot moved.

Both Phase 1.5 LaserScan profiles enable `inf_is_valid` so Gazebo max-range
returns can clear cells previously occupied by moving simulated obstacles.

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
