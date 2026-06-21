# Phase 1.5B Gazebo Obstacle Validation

This validation adds a simulation-only planar scan and obstacle layers to the
already accepted Phase 1.5A dynamics loop.

It does not start real MID360, FAST-LIO, serial, referee, or competition
mission/BT. The planar GPU lidar is a Nav2 costmap test instrument, not a
MID360 simulator.

## Build

Use the single-container Docker workflow from
`docs/phase1_5_gazebo_validation.md`, then run:

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Confirm the new executable exists:

```bash
ros2 pkg executables rm_simulation
```

Expected: `rm_simulation scan_frame_adapter`.

## Headless Sensor Check

```bash
ros2 launch rm_simulation phase1_5_gazebo.launch.py \
  headless:=true use_nav2:=true use_rviz:=false
```

Wait at least five seconds, then check:

```bash
ros2 topic echo --once /simulation/scan_raw
ros2 topic echo --once /scan
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link sim_lidar_link
ros2 node info /scan_frame_adapter
```

Expected:

- both scan topics contain finite ranges;
- `/scan.header.frame_id` is `sim_lidar_link`;
- scan rate is near 15 Hz;
- `base_link -> sim_lidar_link` is static and owned by
  `robot_state_publisher`;
- `scan_frame_adapter` publishes no TF, odometry, command, or navigation goal.

If Ogre2, EGL, or render-engine initialization fails, record the complete
Gazebo log. Do not replace the sensor with fake scan data during this test.

## Costmap Check

Confirm both costmaps subscribe to `/scan`:

```bash
ros2 node info /local_costmap/local_costmap
ros2 node info /global_costmap/global_costmap
ros2 topic echo --once /local_costmap/costmap
ros2 topic echo --once /global_costmap/costmap
```

The `center_block` at approximately `x=1.4, y=0.0` must appear as lethal or
inflated cost. The costmaps must not remain uniformly free.

## Avoidance Goal

Send a goal behind the blocking obstacle:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Inspect the plan and motion:

```bash
ros2 topic echo --once /plan
ros2 topic echo /cmd_vel
ros2 topic echo /simulation/ground_truth/odom
```

Acceptance requires:

1. The goal is accepted and reaches `SUCCEEDED`.
2. The global path deviates around the obstacle rather than crossing it.
3. The robot uses lateral and/or turning motion to avoid collision.
4. Ground-truth odometry never enters the obstacle footprint.
5. Local and global costmaps clear old observations as the robot moves.
6. The chassis watchdog and canonical odometry chain still work.

## TF And Pollution Check

```bash
ros2 run tf2_tools view_frames
ros2 node list
ros2 topic list
```

Required ownership remains:

- `map -> odom`: only `map_odom_stub`;
- `odom -> base_link`: only `lio_adapter`;
- `base_link -> sim_lidar_link`: only `robot_state_publisher`;
- Gazebo and `scan_frame_adapter` do not publish ROS TF.

No old `/Pose_pub`, `/my_set_goal`, or `/nav_result` topic may appear.

## Optional RViz

On a host with working X11 forwarding:

```bash
ros2 launch rm_simulation phase1_5_gazebo.launch.py \
  headless:=false use_nav2:=true use_rviz:=true
```

RViz should show the red `/scan` points, local/global costmaps, robot model,
TF, LIO odometry, and Nav2 plan without frame errors.
