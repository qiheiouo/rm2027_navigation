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

The `ramp_perception` scenario adds a shadow-only 3D return test for one
representative 11-degree ramp and one representative 15-degree ramp. A
map-fixed synthetic surface is transformed into the rotating
`sim_lidar_link`; the filter transforms it back at the cloud timestamp and
removes only points close to the configured expected planes. A 12 cm object
on the 15-degree surface remains in the filtered cloud. Neither the synthetic
input nor `/simulation/ramp/points_filtered_shadow` is connected to a Nav2
costmap. Gazebo also spawns blue 11-degree and orange 15-degree static ramp
fixtures at the same candidate positions so the geometry is visible in the
GUI; the points remain synthetic and do not come from the Gazebo GPU lidar.

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=ramp_perception headless:=false use_rviz:=true
```

RViz compares the red raw cloud with the green filtered cloud. Gazebo shows
the physical fixtures. This scenario does not create a dog hole; use
`scenario:=dog_hole` for the parameterized tunnel. The two ramp regions are
candidate geometry only: the four
competition ramp polygons, elevations, directions, and final map revision
must be entered from the accepted field map before active costmap evaluation.
Setting `ramp_active_map_revision` to any nonmatching value verifies that the
filter passes every point through unchanged.

The unified user entry `rm_navigation_launch/field_geometry_simulation.launch.py`
is a separate simulation integration test. Unlike the standalone shadow
scenario above, it routes the Gazebo planar scan through the map-bound
`ramp_laserscan_filter_node` before Nav2. The active contract names only the
synthetic candidate scene; it does not authorize activation on the field map.
The 1.60 m ramp width and finite secondary wheel friction are provisional
simulation values chosen to leave a footprint-safe center corridor and prevent
the ideal zero-friction roller model from sliding sideways on the 15-degree
surface.

The new-car dog-hole candidate uses `dog_hole_sim.launch.py`. It generates a
parameterized `0.80 m` tunnel model from the single configuration in
`rm_dog_hole/config/dog_hole_sim.yaml`, derives a narrow-passage MPPI profile,
and runs the approach/align/centerline-cross/exit sequence. Its default
localization path explicitly exercises the optional `chassis_heading_fusion`
mode. `sim_chassis_heading_lio_source` converts Gazebo base truth and the
configured gimbal motion into a synthetic FAST-LIO sensor trajectory plus
`/chassis/heading`. `lio_adapter` alone recovers `/odometry/lio`,
`odom -> base_link`, and `/gimbal/state_derived`; the derived state is the only
input allowed to drive `gimbal_yaw_joint`. The separate
`/simulation/gimbal/state_truth` topic is evaluation-only and has no TF
consumer. Fixed, sinusoidal, and continuous gimbal modes remain available.

The generated scene is spawned with the configured world `x/y/yaw` explicitly;
`ros_gz_sim create` otherwise replaces the SDF model pose with its zero-valued
CLI defaults. Optional deck and ramp geometry produces real 3D chassis motion,
and the `0.25 m` roof is a collision rather than a visual marker. The temporary
plan footprint is an octagon with alternating `0.382 m` and `0.126 m` edges.
The launch-time `deformed` / `undeformed` upper-envelope profiles are `0.22 m`
and `0.32 m`; they do not simulate the lower-controller transformation action.
Final CAD and mechanical parameters are still required before acceptance.

It does not simulate MID360 point clouds or run the real FAST-LIO backend. The
synthetic raw odometry exercises the localization boundary math and timing
contract, not scan matching, deskew, LIO drift, serial transport, referee data,
or the competition mission tree. Those concerns remain separate milestones.

The wheel radius, wheelbase, track width, mass, inertia, and directional
friction in `worlds/phase1_omni.sdf` are placeholders. They are not accepted
2027 chassis parameters. The built-in Gazebo mecanum model is an initial
holonomic-equivalent test model; final wheel geometry and sign conventions
must be aligned with the mechanical and lower-controller teams.

Gazebo TF is deliberately not bridged into ROS. The canonical ROS TF owners
remain `map_odom_stub`, `lio_adapter`, and `robot_state_publisher`.

Gazebo is started as a process directly owned by ROS launch. This avoids the
Humble `ros_gz_sim` shell wrapper leaving an orphaned simulator and a competing
`/clock` publisher after Ctrl+C.

`scan_frame_adapter` rewrites only the simulation scan message frame to
`sim_lidar_link`. It does not publish TF. `sim_lidar_link` is enabled in the
robot description only by the simulation launch and is fixed below the dynamic
`gimbal_yaw_link`, not directly below `base_link`.

`fake_pointcloud_obstacle_publisher` publishes only PointCloud2 test data. It
does not publish TF, odometry, velocity commands, or navigation goals.

`sim_chassis_heading_lio_source` also compares fused base pose and derived
gimbal yaw against timestamped Gazebo truth on
`simulation/chassis_heading_lio_fusion` diagnostics. A nonzero lower-controller
world-yaw offset verifies delta-only alignment. `heading_timestamp_offset_sec`
and `heading_publish_divider` inject asynchronous or sparse heading samples;
coverage below 95 percent is an ERROR even when the surviving samples are
mathematically exact.

The MPPI simulation launch also accepts
`heading_policy:=baseline|path_aligned`. The candidate keeps the holonomic
motion model but enables the built-in PathAngle, Twirling, and PreferForward
objectives so the base tends to face the local path while moving. The baseline
remains the default for reproducible A/B runs. See
`docs/chassis_heading_motion_policy.md` for the validation gate.
