# Old-Car Self-Test Guide

This guide is for manual old-car experiments when Codex is not available.

The old-car profile is only a hardware confidence path for the 2027 navigation
stack. It is not the 2027 final calibration or competition-speed tuning.

## Safety Rules

- Keep the remote controller in hand.
- Test in an open, flat area.
- Start with zero command.
- Use short goals first.
- Cancel immediately if motion direction, speed, or stopping behavior is wrong.
- Do not test chassis self-rotation as evidence for the 2027 gimbal-mounted
  MID360 plan. The old car has the sensor fixed to the chassis.

## Start With RViz

## Start Docker With Serial Access

The default Docker profile does not expose host serial devices. For real
old-car serial tests, recreate the container with the serial override after
the host can see `/dev/ttyACM0`:

```bash
ls -l /dev/ttyACM0
export SERIAL_DEVICE=/dev/ttyACM0
export DIALOUT_GID=$(stat -c '%g' /dev/ttyACM0)
sudo -E docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  up -d --force-recreate rm2027_nav
sudo docker exec -it rm2027_navigation_humble bash
```

Inside the container, confirm the device is visible before launching real
serial:

```bash
ls -l /dev/ttyACM0
python3 - <<'PY'
import os
print(os.path.exists("/dev/ttyACM0"), os.access("/dev/ttyACM0", os.R_OK | os.W_OK))
PY
```

If the device is not visible in the container, do not continue to Nav2 or
motion tests.

If a display is available, launch RViz with the old-car profile:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_nav2:=true \
  use_real_serial:=true \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200
```

If RViz does not open from Docker, run this on the host desktop first:

```bash
xhost +local:docker
```

## What To Check In RViz

- Fixed frame is `map`.
- RobotModel appears at the odometry pose.
- `/livox/left/pointcloud` is around the robot and not rotated into a clearly
  wrong direction.
- `/livox/left/pointcloud_filtered` exists when the driver is enabled. This is
  the local-costmap input; it should contain fewer points inside the robot body
  than the raw cloud.
- `/odometry/lio` moves smoothly when the robot moves.
- `/local_costmap/costmap` does not show persistent false obstacles around the
  robot.
- `/global_costmap/costmap` should not accumulate a trail of dynamic point-cloud
  obstacles behind the robot in the old-car profile.
- `/plan` roughly points toward the goal before motion starts.

If the filtered costmap looks worse, restart the launch with pass-through
filtering for comparison:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_nav2:=true \
  use_real_serial:=true \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200 \
  pointcloud_filter_enabled:=false
```

This keeps the same costmap topic but republishes the raw cloud through the
filter node, so RViz can compare filtered vs pass-through behavior without
editing YAML.

## Dynamic Obstacle Clearing Profiles

The default old-car Nav2 file remains the VoxelLayer baseline:

```bash
nav2_params:=$(ros2 pkg prefix rm_nav_config)/share/rm_nav_config/config/nav2_old_car_2026_left.yaml
```

For the LaserScan clearing comparison, enable `/local_scan` and use:

```bash
local_scan_enabled:=true
nav2_params:=$(ros2 pkg prefix rm_nav_config)/share/rm_nav_config/config/nav2_old_car_2026_left_local_scan.yaml
```

For the STVL dynamic-obstacle decay experiment, keep `/local_scan` disabled and
explicitly use the STVL profile:

```bash
local_scan_enabled:=false
nav2_params:=$(ros2 pkg prefix rm_nav_config)/share/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml
```

This profile requires the external `spatio_temporal_voxel_layer` package to be
available in the ROS environment. It keeps the global costmap dynamic-obstacle
policy unchanged; only the local obstacle layer is changed for the experiment.

Before comparing RViz screenshots, record numeric costmap evidence:

```bash
ros2 run rm_nav_config costmap_inspector \
  --topic /local_costmap/costmap_raw \
  --window-size 2.0 \
  --csv /tmp/local_costmap_before.csv
```

After a person crosses and leaves the robot front, repeat at about 1 s, 3 s,
and 5 s. Lethal and inscribed counts near `base_link` should clearly drop in
the STVL profile while static obstacles that remain visible should stay marked.

`/plan` is the global plan. In local-only profiles it can cross obstacles that
exist only in `/local_costmap/costmap`; MPPI may still stop at those obstacles.
Do not treat `/plan` as the controller's actual local trajectory.

## Minimal Health Check

Run only the checks relevant to the current experiment:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info -v /cmd_vel
```

Check logs only for the key failures:

```bash
grep -c "voxel_grid: only support up to 16 z values" /tmp/YOUR_LOG.log
grep -c "Sensor origin.*out of map bounds" /tmp/YOUR_LOG.log
grep -c "Optimizer fail to compute path" /tmp/YOUR_LOG.log
grep -c "Lookup would require extrapolation" /tmp/YOUR_LOG.log
```

## Zero Command

Before every motion test:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The robot must not move.

## Short Translation Tests

Use short translation goals first. Observe for only a few seconds, then cancel
if anything looks wrong.

Forward:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

Lateral:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

Do not continue if the robot turns aggressively, drifts into obstacles, or does
not stop cleanly.

## Stop And Inspect

After a failed or suspicious run:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Then stop the launch. Confirm the graph is empty except normal ROS discovery:

```bash
ros2 node list
ros2 topic list
```

## When To Stop Testing

Stop the current test and do not proceed to the next one if any of these occur:

- `/cmd_vel.angular.z` saturates during a translation-only test.
- Nav2 runs spin recovery during translation validation.
- `Optimizer fail to compute path` appears repeatedly.
- The robot moves in a direction that does not match the expected command.
- The robot does not stop after cancel plus zero command.
- RViz shows the point cloud or costmap in a clearly wrong place.
- Local costmap shows a dense ring of obstacles tightly around the robot while
  the real area is clear.
- Global costmap keeps old dynamic point-cloud obstacles after the robot moves.

## What To Record

For each useful run, save:

- launch log path;
- branch and commit;
- command used to launch;
- whether RViz point cloud/costmap looked reasonable;
- maximum observed `/cmd_vel`;
- actual motion direction;
- whether cancel and zero command stopped the robot.
