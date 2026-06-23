# Phase 1.5D Multi-Obstacle Course Validation

This increment reuses the accepted Phase 1.5C MPPI profile and original
Gazebo world. It spawns two static walls forming a `0.8 m` passage and may add
one obstacle moving laterally across the route. It does not simulate a full RM
field, MID360 point clouds, competition strategy, or dynamic-opponent intent.

## Build And Static Checks

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rm_simulation
source install/setup.bash
ros2 pkg executables rm_simulation
ign sdf -k $(ros2 pkg prefix rm_simulation)/share/rm_simulation/models/course_wall.sdf
ign sdf -k $(ros2 pkg prefix rm_simulation)/share/rm_simulation/models/moving_obstacle.sdf
```

Expected executables include `scan_frame_adapter` and
`moving_obstacle_controller`. Both SDF files must validate.

## Static Passage Gate

```bash
export LIBGL_ALWAYS_SOFTWARE=true
ros2 launch rm_simulation phase1_5_mppi_course.launch.py \
  headless:=true use_rviz:=false moving_obstacle:=false
```

Wait for MPPI lifecycle activation, stable scan and odometry rates, and both
spawn processes to report successful entity creation. Confirm the static walls
are marked in both costmaps. Send the goal `(4.3, 0.0)` five times from clean
launches.

The static gate requires `5/5` success, no collision or footprint-padding
intrusion, at least `0.05 m` physical clearance, no repeated recovery, and no
sustained controller-loop overrun. Record whether the global plan traverses the
passage or safely chooses an exterior route; either is acceptable if the route
is stable and collision-free.

## Moving Obstacle Gate

```bash
ros2 launch rm_simulation phase1_5_mppi_course.launch.py \
  headless:=true use_rviz:=false moving_obstacle:=true \
  moving_amplitude:=0.9 moving_period:=8.0
```

Check the command boundary:

```bash
ros2 topic hz /simulation/moving_obstacle/target
ros2 topic echo --once /simulation/moving_obstacle/target
ros2 node info /moving_obstacle_controller
```

The target should update near 20 Hz and remain within `[-0.9, 0.9]`. The node
must not publish TF, odometry, velocity commands, or navigation goals.

Confirm physical movement independently from the ROS target by inspecting the
Gazebo pose stream:

```bash
ign topic -l | grep /world/phase1_omni/pose/info
timeout 10s ign topic -e -t /world/phase1_omni/pose/info
```

The `moving_obstacle` y position must change over simulation time. A changing
ROS target without observed Gazebo motion fails this gate.

After costmaps visibly mark and clear the moving obstacle, send `(5.6, 0.0)`
from ten clean launches with varied obstacle phase. Do not synchronize the goal
to an intentionally favorable opening.

The dynamic gate requires:

- `10/10` successful actions and no physical collision;
- no footprint-padding intrusion and at least `0.05 m` clearance;
- old moving-obstacle cells clear after the obstacle leaves;
- no sustained scan, odometry, TF, or controller-loop loss;
- no repeated recovery pattern;
- final yaw error within `0.20 rad`.

Waiting for a temporarily blocked route is acceptable. Driving through stale
free space, failing to clear ghost obstacles, or relying on Gazebo TF is not.

## Ownership And Pollution

The only new ROS topic is the simulation-only joint target documented in the
topic contract. The extra bridge is ROS-to-Gazebo only. `map -> odom`,
`odom -> base_link`, and all robot-description transforms retain their existing
owners. No Gazebo pose or TF topic may be bridged into ROS `/tf` or `/tf_static`.

## Decision

Phase 1.5D passes only after both static and moving gates pass. Failure must be
classified as spawn/joint control, scan and clearing, global planning, MPPI
local control, or simulation CPU load before changing navigation parameters.
