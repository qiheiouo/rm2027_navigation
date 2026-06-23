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
  - `examples/worlds/gpu_lidar_sensor.sdf`
  - `src/systems/sensors/Sensors.cc`
  - `examples/worlds/joint_position_controller.sdf`

- Repository: `https://github.com/gazebosim/ros_gz`
- Version: branch `humble`, commit `9d7f8c721c233a9ac8b43950129d51e67905523e`
- License: Apache-2.0
- Files consulted:
  - `ros_gz_bridge/src/convert/sensor_msgs.cpp`
  - `ros_gz_bridge/src/convert/utils.cpp`
  - `ros_gz_sim/src/create.cpp`

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

Phase 1.5B uses the official Fortress GPU lidar SDF structure. Because the
Gazebo LaserScan frame is a scoped simulator name and `ros_gz_bridge` only
normalizes `::` delimiters, a local simulation adapter rewrites the scan
header to the explicitly owned `sim_lidar_link` frame. The adapter does not
modify ranges or publish TF.

Phase 1.5D uses the documented Fortress `JointPositionController` topic
interface and the ROS Gazebo `create` executable to spawn project-authored
course models. The local wall geometry, moving-obstacle model, sinusoidal
target node, topics, and launch composition are original project code. No
Gazebo implementation source is copied or vendored.

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
