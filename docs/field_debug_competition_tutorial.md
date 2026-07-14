# Old-Car Field Debug Competition Tutorial

This procedure validates home and patrol mission behavior with the real old-car
sensor, localization, Nav2 and serial chain. Referee state is synthetic and the
remote manual/automatic switch remains the physical chassis authority.

This is not a competition-ready profile. Do not use it without an operator at
the remote, clear floor space and a known working manual takeover.

## 1. Safety Rules

1. Power on with the remote in manual mode.
2. Keep the mission disabled until every readiness check passes.
3. Only switch to automatic immediately before one planned motion.
4. To stop, switch the remote to manual first, then disable the mission.
5. Before switching back to automatic, confirm the mission is disabled. The
   software cannot observe the old lower controller's remote mode, so an active
   Nav2 goal may resume when automatic mode is restored.
6. Do not enable target mocks, the all-in-one mission safety mock or dual lidar
   during this procedure.

## 2. Host Startup

Open a host terminal in the repository:

```bash
cd ~/rm2027_navigation
git status --short --branch
```

Verify the serial device and record its group ID:

```bash
ls -l /dev/ttyACM0
export SERIAL_DEVICE=/dev/ttyACM0
export DIALOUT_GID=$(stat -c '%g' "$SERIAL_DEVICE")
export USER_UID=$(id -u)
export USER_GID=$(id -g)
echo "serial=$SERIAL_DEVICE dialout_gid=$DIALOUT_GID uid=$USER_UID gid=$USER_GID"
```

If `/dev/ttyACM0` does not exist, stop. Reconnect or power-cycle the lower
controller and identify the actual device before continuing.

Allow the Docker RViz window to use the current X display when required:

```bash
xhost +local:docker
```

Recreate the container with the serial override. Merely entering an old
container does not add the device mapping:

```bash
sudo -E docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  up -d --build --force-recreate
```

If ordinary `docker info` works for the current user, `sudo -E` may be omitted.

Enter the container:

```bash
sudo docker exec -it rm2027_navigation_humble bash
```

## 3. Container Build And Device Check

Inside the container:

```bash
cd /workspace/rm2027_navigation
source /opt/ros/humble/setup.bash

ls -l /dev/ttyACM0
id
test -r /dev/ttyACM0 && test -w /dev/ttyACM0 && echo SERIAL_RW_OK
```

Stop if `SERIAL_RW_OK` is not printed.

Install dependencies and build the workspace. A full build is intentional for
the first run because the launch also needs description, Livox, LIO, map,
localization, Nav2 configuration and serial packages:

```bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

Confirm the new launch arguments are installed:

```bash
ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py --show-args \
  | grep -E 'allow_field_debug_inputs|use_operator_chassis_authority|referee_mock_current_hp'
```

## 4. Select And Validate The Map

Find available manifests:

```bash
find /data/rm27_maps -name '*.bundle.yaml' -type f -print
```

Set the real path returned by the command:

```bash
export MAP_MANIFEST=/data/rm27_maps/REPLACE_ME/field.bundle.yaml
```

Validate and resolve it. Use `allow_candidate` only while the PGM is still
under field review:

```bash
ros2 run rm_map_tools validate_map_bundle "$MAP_MANIFEST"
ros2 run rm_map_tools resolve_map_bundle "$MAP_MANIFEST" \
  --acceptance-policy allow_candidate
```

Do not use a synthetic `test_only` map on the robot. Dirty PGM cells may affect
both AMCL and the global planner. For this test, choose home and patrol points
only in a visually checked, correctly aligned region. Keep the bundle as
`candidate` until the PGM is cleaned and its origin is reviewed.

## 5. Prepare Mission Coordinates

Create a runtime copy outside Git:

```bash
mkdir -p /data/rm27_maps/runtime
cp "$(ros2 pkg prefix rm_competition_mission)/share/rm_competition_mission/config/mission_field_debug_template.yaml" \
  /data/rm27_maps/runtime/mission_old_car.yaml
export MISSION_CONFIG=/data/rm27_maps/runtime/mission_old_car.yaml
```

Initially leave `home_pose` and `patrol_waypoints` empty. The first launch is
used to verify localization and collect safe map-frame coordinates.

## 6. First Launch: Localization And Point Collection

Keep the remote in manual mode. In the first container terminal:

```bash
source /opt/ros/humble/setup.bash
source /workspace/rm2027_navigation/install/setup.bash

export MAP_MANIFEST=/data/rm27_maps/REPLACE_ME/field.bundle.yaml
export MISSION_CONFIG=/data/rm27_maps/runtime/mission_old_car.yaml
export NAV2_PARAMS=$(ros2 pkg prefix rm_nav_config)/share/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml

ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py \
  enable_competition_stack:=true \
  use_driver:=true \
  use_lio_backend:=true \
  use_map_server:=true \
  relocalization_backend:=amcl_2d \
  map_bundle_manifest:="$MAP_MANIFEST" \
  map_acceptance_policy:=allow_candidate \
  nav2_params:="$NAV2_PARAMS" \
  use_nav2:=true \
  use_real_serial:=true \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200 \
  use_rviz:=true \
  use_referee_interface:=true \
  use_referee_mock:=true \
  referee_mock_game_progress:=4 \
  referee_mock_current_hp:=400 \
  use_mission:=true \
  mission_startup_enabled:=false \
  mission_config:="$MISSION_CONFIG" \
  allow_field_debug_inputs:=true \
  use_operator_chassis_authority:=true \
  use_chassis_mode_interface:=false \
  use_mission_safety_mock:=false \
  use_pursuit:=false \
  use_target_mock:=false \
  use_dual_obstacle_fusion:=false \
  use_right_driver:=false
```

The log must contain `FIELD DEBUG INPUTS ENABLED` and
`REAL SERIAL TRANSPORT ENABLED`. The mission must still report disabled.

In RViz:

1. Set Fixed Frame to `map`.
2. Check that the occupancy map orientation and origin match the field.
3. Use **2D Pose Estimate** to provide the initial pose for AMCL.
4. Wait until the robot model, map, pointcloud and costmaps agree.
5. Do not send a goal yet.

## 7. Readiness Checks

Open a second container terminal and source the workspace:

```bash
sudo docker exec -it rm2027_navigation_humble bash
source /opt/ros/humble/setup.bash
source /workspace/rm2027_navigation/install/setup.bash
```

Run every check:

```bash
ros2 node info /serial_transport_node
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

ros2 topic echo /referee/state --once
ros2 topic echo /referee/state_valid --once
ros2 topic echo /localization/global_localization_valid --once
ros2 topic echo /system/readiness --once
ros2 topic echo /mission/state --once

ros2 run tf2_ros tf2_echo map base_link
```

Required results:

- `serial_transport_node` subscribes to `/cmd_vel`;
- Nav2 lifecycle nodes are `active [3]`;
- referee state has `valid: true` and `game_progress: 4`;
- global localization validity is `true`;
- `map -> base_link` is continuously available;
- mission state has `enabled: false`;
- readiness does not list chassis authority as required in this operator-waiver
  profile.

## 8. Verify And Record Home/Patrol Points

Use RViz to send one ordinary navigation goal at a time. For each goal:

1. Keep the mission disabled.
2. Confirm the planned route does not cross dirty PGM cells or obstacles.
3. Switch the remote to automatic.
4. Send the RViz goal.
5. Keep a hand on manual takeover.
6. After arrival, switch back to manual.

At each desired home or patrol location, record the current map pose:

```bash
timeout 2 ros2 run tf2_ros tf2_echo map base_link
```

Record translation X/Y and RPY yaw. YAML uses metres and radians.

Edit the runtime config:

```bash
nano /data/rm27_maps/runtime/mission_old_car.yaml
```

Example structure only:

```yaml
home_pose: [1.20, -0.80, 0.0]
patrol_waypoints: [
  1.20, -0.80, 0.0,
  2.00, -0.80, 0.0,
  2.00,  0.20, 1.5708
]
```

Replace every example number. A useful functional test has one fixed home pose
and at least two patrol poses. A field-acceptance test should use three points
and complete two patrol loops.

Parameters are loaded only at node startup. With the remote in manual mode,
disable the mission, stop the launch with Ctrl+C, and start it again with the
same command after editing the YAML. Repeat the readiness checks.

## 9. Home Test

Place the robot at a safe location different from home. Keep the remote in
manual until ready, then switch to automatic and enable the home mission:

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: true, mode: home}"
```

Monitor:

```bash
ros2 topic echo /mission/state
ros2 topic echo /cmd_vel
```

Acceptance:

- `active_branch` becomes `home`;
- one Nav2 goal is issued;
- the robot reaches the configured fixed pose;
- repeat from a second starting location;
- no repeated goal resend occurs after success.

Stop the mission after each run:

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: false, mode: hold}"
```

Then switch the remote to manual.

## 10. Patrol Test

Switch to automatic only when the route is clear, then enable patrol:

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: true, mode: patrol}"
```

Acceptance:

- `active_branch` remains `patrol`;
- waypoints advance only after Nav2 reports success;
- at least two full loops complete;
- disabling the mission cancels the current goal and returns to hold;
- remote manual takeover stops physical motion.

## 11. Low-HP Automatic Return Test

This test requires restarting the launch because mock fields are launch
parameters. Keep the remote manual, stop the launch, and restart it with:

```text
referee_mock_current_hp:=100
```

Keep the requested mission mode as `patrol` or `auto`. Once enabled and safe,
the tree should prioritize `home` because 100 is below the default threshold
150. Do not perform this test until the explicit home test already passes.

## 12. Emergency And Normal Shutdown

Immediate physical stop order:

1. Switch the remote to manual/disable chassis power.
2. Call mission disable:

   ```bash
   ros2 service call /mission/set_mode \
     rm_competition_interfaces/srv/SetMissionMode \
     "{enable: false, mode: hold}"
   ```

3. Confirm `/cmd_vel` returns to zero.
4. Stop the launch with Ctrl+C.

After shutdown:

```bash
ros2 node list
```

Then leave the container and stop it if the test is complete:

```bash
exit
sudo docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  down
```

## 13. What This Proves

Passing this procedure proves the real localization/Nav2/serial chain and the
home/patrol BehaviorTree branches work with operator-controlled authority. It
does not prove:

- real referee decoding;
- software knowledge of remote manual/automatic mode;
- approved map quality;
- right-lidar calibration;
- pursuit target communication;
- match-duration reliability.

The next field steps are map cleanup and approval, right-lidar cloud alignment,
and a lower-controller or auto-aim adapter that publishes validated
`/perception/target_track` messages.
