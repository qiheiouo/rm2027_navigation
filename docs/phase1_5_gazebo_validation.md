# Phase 1.5 Gazebo Validation

This document records the Phase 1.5A dynamics baseline. Phase 1.5A passed on
Linux at commit `c03cb190681a5fd4bf37ab1b1718d129d24720a4`. The current
simulation launch may also contain the Phase 1.5B scan and obstacle path;
validate that increment with `docs/phase1_5b_obstacle_validation.md`.

This validation replaces the Phase 1 fake motion integrator with Gazebo
Fortress physics while preserving the canonical ROS boundary:

```text
Gazebo ground truth
  -> /simulation/ground_truth/odom
  -> lio_adapter
  -> /odometry/lio + odom -> base_link
  -> Nav2
  -> /cmd_vel
  -> chassis_interface_stub limits/watchdog
  -> /simulation/chassis/cmd_vel
  -> Gazebo MecanumDrive
```

This is a no-hardware test. It does not start MID360, FAST-LIO, real serial,
referee, or competition mission/BT.

## Build

On Ubuntu 22.04 from the repository root:

```bash
git submodule update --init --recursive
USER_UID=$(id -u) USER_GID=$(id -g) sudo -E docker compose -f docker/docker-compose.yml build
USER_UID=$(id -u) USER_GID=$(id -g) sudo -E docker compose -f docker/docker-compose.yml run --rm rm2027_nav
```

Inside the container:

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon list
colcon build --symlink-install
source install/setup.bash
```

`colcon list` should include `rm_simulation` in addition to the previous
packages.

## Headless Launch

```bash
ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=true use_nav2:=true use_rviz:=false
```

In another shell in the same running container, source the workspace and
check:

```bash
ros2 topic echo --once /clock
ros2 topic echo --once /simulation/ground_truth/odom
ros2 topic echo --once /odometry/lio
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

Expected frames:

- `/simulation/ground_truth/odom`: `frame_id=odom`, `child_frame_id=base_link`;
- `/odometry/lio`: `frame_id=odom`, `child_frame_id=base_link`;
- `map -> odom`: identity from `map_odom_stub`;
- `odom -> base_link`: only from `lio_adapter`;
- no Gazebo pose or TF bridge publishes a competing ROS transform.

## Manual Holonomic Motion

With Nav2 disabled, verify forward, lateral, and yaw commands separately:

```bash
ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=true use_nav2:=false
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.3}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}"
```

For each command, `/simulation/ground_truth/odom` and `/odometry/lio` should
change in the expected axis. After the watchdog timeout, the relayed command
must return to zero.

## Nav2 Goal

With Nav2 enabled:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Expected behavior:

- Nav2 accepts the goal;
- `/cmd_vel` contains holonomic commands;
- `chassis_interface_stub` receives, limits, and relays the command;
- Gazebo moves the robot;
- the goal succeeds without the fake LIO motion integrator.

## GUI And RViz

On a Linux desktop with X11:

```bash
xhost +local:docker
ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=false use_nav2:=true use_rviz:=true
```

If the container has no usable display or GPU forwarding, keep the validation
headless. GUI failure alone does not invalidate physics, topic, or TF tests.

## Acceptance Criteria

1. Docker image builds with Gazebo Fortress and `ros_gz` dependencies.
2. `rm_simulation` is discovered and the workspace builds.
3. `/clock`, simulation odometry, and canonical LIO odometry publish.
4. Forward, lateral, and yaw commands move the simulated robot correctly.
5. A small Nav2 goal succeeds through the chassis-interface relay.
6. No duplicate `map -> odom` or `odom -> base_link` exists.
7. No real hardware, serial, referee, or competition mission node starts.

The first milestone does not validate obstacle avoidance or MID360 simulation.
Those are the next Phase 1.5 increments after basic dynamics pass.
