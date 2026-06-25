# Minipc Hardware Bringup Sequence

## Principle

Bringup must progress from passive observation to controlled motion. Each step
has one new risk source. If a step fails, stop and fix that layer instead of
enabling later modules.

Use one ROS domain per robot test. Do not run simulation and real hardware
profiles in the same domain.

## 0. Environment

```bash
git clone https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
git submodule update --init --recursive
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm rm2027_navigation
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Acceptance:

- all packages build;
- `ros2 pkg list | grep rm_` shows the project packages;
- no real hardware launch has been started yet.

## 1. Passive Driver Check

Goal: prove MID360 topics without starting LIO or Nav2.

```bash
ros2 launch rm_mid360_driver_bridge dual_mid360_driver.launch.py \
  use_driver:=true
```

Record:

- left/right point-cloud topics;
- IMU topic selected for LIO;
- frame_id values;
- topic frequency;
- packet drops or driver warnings;
- CPU and network load.

Stop if the frames or topics do not match the contract. Do not fix this by
renaming canonical TF.

## 2. Time Sync And Gimbal State

Verify:

- PTP grandmaster/slave role;
- `ptp4l` and `phc2sys` status;
- system time drift;
- gimbal yaw topic frequency;
- yaw zero, positive direction and wrap behavior;
- yaw timestamp delay.

Do not validate rotating-gimbal LIO until this is understood.

## 3. Description And TF

Start description and localization adapters without backend motion:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  use_driver:=false use_lio_backend:=false use_nav2:=false
```

Check:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link gimbal_yaw_link
ros2 run tf2_ros tf2_echo gimbal_yaw_link mid360_left_frame
ros2 run tf2_ros tf2_echo gimbal_yaw_link mid360_right_frame
```

Acceptance:

- no duplicate TF parent;
- placeholder extrinsics are replaced before real localization acceptance;
- no serial/referee/mission node publishes TF.

## 4. LIO Boundary

Start driver and backend only after passive topics are correct:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  sensor_mode:=single selected_side:=left \
  use_driver:=true use_lio_backend:=true \
  use_nav2:=false use_chassis_stub:=true
```

Check:

```bash
ros2 topic echo --once /odometry/lio
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /odometry/lio
```

Acceptance:

- `/odometry/lio.header.frame_id == odom`;
- `/odometry/lio.child_frame_id == base_link`;
- `lio_adapter` is the only `odom -> base_link` publisher;
- backend TF remains quarantined.

## 5. Map And Global Localization

Use only approved bundles for deployment:

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  map_bundle_manifest:=/maps/approved_field.bundle.yaml
```

Then test the selected global-localization backend through
`/localization/global_pose`. The backend must not publish canonical TF.

Acceptance:

- map server publishes `/map`;
- exactly one `map -> odom` owner exists;
- reset/validity behavior is observable.

## 6. Nav2 Without Real Motion

With wheels lifted or chassis command disabled:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  use_driver:=true use_lio_backend:=true \
  use_map_server:=true use_nav2:=true \
  use_chassis_stub:=true \
  map_bundle_manifest:=/maps/approved_field.bundle.yaml
```

Send a short goal and verify:

- `/cmd_vel` appears;
- chassis stub receives commands;
- no real serial device is open;
- MPPI CPU is acceptable with driver, LIO and RViz/logging load.

## 7. Serial Dry-Run

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py \
  protocol_profile:=legacy_v1_no_crc
```

Check `/serial/mock_tx` for frame length, sequence and watchdog behavior. Only
after the electrical team confirms the firmware profile should real serial IO
be enabled in a future transport node.

## 8. First Controlled Motion

Before enabling real chassis motion:

- confirm emergency stop;
- set conservative velocity and acceleration limits;
- lift wheels for first command;
- confirm x forward, y left and yaw counterclockwise;
- verify watchdog zero command;
- keep Nav2 goals short and low speed.

Acceptance:

- `/cmd_vel` sign matches chassis motion;
- no uncontrolled rotation or lateral inversion;
- serial feedback is logged;
- wheel-derived feedback does not publish localization TF.

## 9. Full Closed Loop

Only after the previous gates pass:

1. enable low-speed Nav2 motion;
2. validate obstacle detection with real point clouds;
3. test relocalization reset;
4. test gimbal high-speed transient while stationary;
5. test chassis rotation and gimbal rotation separately;
6. test combined motion at low speed;
7. increase limits gradually.

## Stop Conditions

Stop the test immediately if:

- two nodes publish the same TF edge;
- `/odometry/lio` frame ids are not canonical;
- serial/referee/mission publishes nav goals or TF;
- MID360 PTP is unstable;
- gimbal yaw timestamps are stale or discontinuous;
- chassis sign convention is wrong;
- obstacle cloud appears mirrored or rotated in RViz;
- CPU saturation causes Nav2 loop misses or LIO instability.
