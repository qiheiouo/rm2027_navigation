# rm_simulation

Phase 1.5 Gazebo Fortress simulation boundary for the 2027 sentry.

The first milestone intentionally contains only:

- one robot with placeholder four-wheel holonomic geometry;
- Gazebo `MecanumDrive` physics driven from ROS `/cmd_vel`;
- Gazebo ground-truth odometry bridged to `/simulation/ground_truth/odom`;
- the existing `lio_adapter` converting that input to canonical
  `/odometry/lio` and `odom -> base_link`;
- the existing Nav2 DWB and chassis-interface stub;
- optional RViz and Gazebo GUI.

Phase 1.5B adds a simulation-only planar GPU lidar, one static blocking
obstacle, a scan-frame adapter, and Nav2 obstacle layers. The planar scan is a
costmap test instrument, not a MID360 model and not a substitute for later
3D point-cloud validation.

Phase 1.5C adds an official Nav2 MPPI comparison profile for the same world.
It uses the omnidirectional motion model and full footprint collision scoring.
The initial profile passed the documented stability, safety, and CPU gates and
is the baseline for subsequent Phase 1.5 simulation. The Phase 1 DWB launch and
configuration remain available as a fallback and historical comparison.

Phase 1.5D uses `phase1_5_mppi_course.launch.py` to spawn two static walls that
form a 0.8 m passage plus an optional laterally moving obstacle. The movement
controller publishes only `/simulation/moving_obstacle/target`; a dedicated
one-way bridge sends that target to Gazebo without exposing model TF to ROS.

Phase 2G adds `phase2g_pointcloud_obstacle.launch.py`, which disables the
LaserScan adapter and publishes a synthetic `PointCloud2` obstacle on
`/points/obstacles` in `sim_lidar_link`. It validates the Nav2 VoxelLayer
boundary only; it is not a MID360 physics or timing simulation.

The new-car dog-hole candidate uses `dog_hole_sim.launch.py`. It generates a
parameterized `0.80 m` tunnel model from the single configuration in
`rm_dog_hole/config/dog_hole_sim.yaml`, derives a narrow-passage MPPI profile,
and runs the approach/align/centerline-cross/exit sequence. A simulation-only
gimbal source publishes the same angle to ROS joint state and Gazebo joint
control. The default `0.65 rad` yaw demonstrates that lidar direction is not
the chassis heading used by the tunnel controller.

It does not simulate MID360 point clouds, FAST-LIO, serial, referee, or the
competition mission tree. Those concerns remain separate milestones.

The wheel radius, wheelbase, track width, mass, inertia, and directional
friction in `worlds/phase1_omni.sdf` are placeholders. They are not accepted
2027 chassis parameters. The built-in Gazebo mecanum model is an initial
holonomic-equivalent test model; final wheel geometry and sign conventions
must be aligned with the mechanical and lower-controller teams.

Gazebo TF is deliberately not bridged into ROS. The canonical ROS TF owners
remain `map_odom_stub`, `lio_adapter`, and `robot_state_publisher`.

`scan_frame_adapter` rewrites only the simulation scan message frame to
`sim_lidar_link`. It does not publish TF. `sim_lidar_link` is enabled in the
robot description only by the simulation launch and is fixed below the dynamic
`gimbal_yaw_link`, not directly below `base_link`.

`fake_pointcloud_obstacle_publisher` publishes only PointCloud2 test data. It
does not publish TF, odometry, velocity commands, or navigation goals.
