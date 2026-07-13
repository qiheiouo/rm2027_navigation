# Phase 3 Competition Software Validation

## Build And Unit Tests

Run in the ROS 2 Humble container before testing hardware:

```bash
colcon build --symlink-install --packages-select \
  rm_competition_interfaces rm_referee_interface rm_pursuit \
  rm_competition_mission rm_system_monitor rm_chassis_interface \
  rm_mid360_driver_bridge rm_navigation_bringup
source install/setup.bash

colcon test --packages-select \
  rm_referee_interface rm_pursuit rm_system_monitor
colcon test-result --verbose
```

Also parse all new top-level launches:

```bash
ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py --show-args
ros2 launch rm_navigation_bringup competition_no_hardware_test.launch.py --show-args
```

## No-Hardware Mission Smoke Test

```bash
ros2 launch rm_navigation_bringup competition_no_hardware_test.launch.py \
  enable_test:=true headless:=true use_rviz:=false
```

Expected before enabling the mission:

- Gazebo/Nav2 runs with simulation odometry and scan only.
- Mock referee, target and safety inputs are clearly logged.
- `/mission/state.enabled` is false.
- No real MID360 driver or serial transport exists.
- `/system/readiness.ready_for_mission` becomes true only after every mock/data
  input is present.

Enable patrol explicitly:

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: true, mode: patrol}"
```

Then verify `/navigate_to_pose` goals advance through the example route. Test
`mode: pursuit`, target timeout, localization-valid false and referee-valid
false. Each invalidation must cancel the active goal and return the mission to
hold. These are software tests, not field-coordinate or opponent-tracking
acceptance.

## Hardware-Gated Acceptance

The following cannot be claimed from the no-hardware profile:

1. referee byte decoding and field correctness;
2. lower-controller autonomous/manual/emergency state freshness;
3. auto-aim target frame, latency, covariance and confidence calibration;
4. right MID360 extrinsic and dual-stream alignment;
5. candidate/approved map quality and match field coordinates;
6. pursuit safety at competition speed.

Real serial must never be combined with any mock input. Start a real mission
disabled, inspect `/system/readiness`, then enable it through the service only
after the operator confirms automatic-mode takeover and stop behavior.
