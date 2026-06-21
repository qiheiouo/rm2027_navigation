# Gazebo Fortress Systems Record

## Upstream

- Repository: `https://github.com/gazebosim/gz-sim`
- Version: tag `ignition-gazebo6_6.18.0`
- License: Apache-2.0
- Files consulted:
  - `examples/worlds/mecanum_drive.sdf`
  - `src/systems/mecanum_drive/MecanumDrive.cc`
  - `test/worlds/odometry_publisher_custom.sdf`
  - `src/systems/odometry_publisher/OdometryPublisher.cc`

## Local Use

`rm_simulation/worlds/phase1_omni.sdf` adapts the documented plugin names,
joint-role parameters, directional-friction pattern, and odometry publisher
parameters. The local robot geometry, names, dimensions, mass, inertia, topic
boundary, and launch composition are project-specific.

Gazebo Sim is installed as a system dependency in the ROS2 Humble Docker
image. No Gazebo source or binary is copied into this repository.

The built-in `MecanumDrive` model is used as an initial holonomic-equivalent
test model. It does not prove that the placeholder wheel arrangement matches
the final 2027 four-omni chassis. Final wheel type, roller direction, wheel
order, radius, wheelbase, track width, sign convention, and dynamics must be
confirmed with the mechanical and lower-controller teams.

The Fortress `MecanumDrive` plugin accepts velocity commands but its 6.18.0
implementation does not publish the advertised odometry. Therefore the local
world explicitly uses the separate Gazebo `OdometryPublisher` system for
ground-truth odometry.

## RoboMaster Reference

- Repository: `https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator`
- Commit read during research: `a92ec68b753b164e876520d62cd958dee5bbb13e`
- License: Apache-2.0

PolarBear informed the choice of ROS2 Humble, Gazebo Fortress, `ros_gz_sim`,
and `ros_gz_bridge`. The Phase 1.5 package does not copy its RM field,
referee, web, multi-robot, `rmoss`, or robot-description code.

## TF Boundary

Gazebo ground-truth pose topics are not bridged to ROS TF. The only ROS-side
canonical publishers remain:

- `map_odom_stub`: `map -> odom` during Phase 1;
- `lio_adapter`: `odom -> base_link`;
- `robot_state_publisher`: robot links below `base_link`.
