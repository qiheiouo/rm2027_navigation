# New-Car Ramp Perception Shadow Validation

## Scope

The current field description has four traversable ramps: two approximately
11 degrees and two approximately 15 degrees, each shorter than about 1 m. The
laboratory foam-board observation is sufficient to reproduce the failure mode
but not to accept an active perception rule. Exact field-map coordinates,
ascent directions, base elevations, and real MID360 returns are still absent.

This stage therefore validates the geometry and failure boundaries in
simulation only. It does not change the old-car or new-car Nav2 costmap input.

## Method

`synthetic_ramp_pointcloud_publisher` creates one representative surface per
angle in `map`, adds a 12 cm obstacle to the 15-degree surface, and transforms
the returns into `sim_lidar_link` at the cloud timestamp. The simulated gimbal
rotates continuously at 0.60 rad/s by default.

`ramp_plane_filter_node` transforms each return back to `map`. A return is
removed only when it is inside the configured polygon and within 0.04 m of
the expected plane. Points above that tolerance remain. The output topic is
explicitly named `/simulation/ramp/points_filtered_shadow` and is not remapped
to a VoxelLayer, STVL source, or any other Nav2 input.

The filter also binds the plane regions to an exact map identity and revision.
Any mismatch, missing timestamped TF, malformed PointCloud2 record, disabled
filter, or empty region set causes the original cloud to be republished.

## Run

Build and source the workspace, then run:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=ramp_perception headless:=false use_rviz:=true
```

RViz uses red for `/simulation/ramp/points_raw` and green for
`/simulation/ramp/points_filtered_shadow`. The green result should retain the
raised object while removing both expected surfaces.

Headless diagnostics:

```bash
ros2 topic echo /diagnostics
```

Find the status named `rm_mid360_driver_bridge/ramp_plane_filter`. It reports
the map contract and input/removed/kept point counts.

Fail-closed map test:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=ramp_perception headless:=true use_rviz:=false \
  ramp_active_map_revision:=wrong_revision
```

Expected: `map_contract_valid=false`, `removed_surface_points=0`, and
`kept_points=input_points`.

## Recorded Linux Smoke Result

On ROS 2 Humble in the project container:

- normal candidate contract: input approximately 2383 points, removed
  approximately 2019 surface points, retained approximately 364 obstacle and
  boundary points;
- gimbal joint velocity: 0.60 rad/s, with changing timestamped
  `map -> sim_lidar_link` yaw;
- wrong map revision: 2383 input, 0 removed, 2383 retained;
- Ctrl+C: Gazebo and all ROS nodes exit without a residual Gazebo process.

Point counts can vary slightly on polygon boundaries because the surface is
transformed through single-precision PointCloud2 coordinates.

## Activation Gate

Do not connect the filtered topic to Nav2 until all of these are complete:

1. Enter all four accepted map polygons, elevations, ascent directions, and a
   version tied to the deployed map bundle.
2. Record real dual-MID360 data for approach, ascent, crest, descent, and an
   obstacle placed on each ramp type, with synchronized TF and odometry.
3. Measure false removal outside the polygons and obstacle retention on the
   ramp; include localization error and timestamp-offset sweeps.
4. Run shadow comparison first, then a reversible low-speed costmap A/B test
   with a safety operator and physical stop authority.
5. Revalidate after final new-car LiDAR extrinsics, ground clearance, footprint,
   and suspension/CAD dimensions are fixed.
