# Phase 1 Linux Validation

This document describes the first validation pass after cloning the project onto Linux, ROS2 Humble, or the target mini PC.

Phase 1 validation does not require a real MID360, real serial hardware, referee system, FAST-LIO integration, or competition BT.

## Clone

```bash
git clone https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
```

## Environment Assumptions

- Ubuntu 22.04 or the target ROS2 Humble environment.
- ROS2 Humble is installed and sourced.
- `colcon` is installed.
- `rosdep` is installed and initialized.
- No real MID360 is required for this stage.
- No real serial device is required for this stage.

## Install Dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Package Discovery

```bash
colcon list
```

Expected packages:

- `rm_localization_adapters`
- `rm_chassis_interface`
- `rm_description`
- `rm_nav_config`
- `rm_navigation_bringup`

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Minimal Runtime Checks

Run each launch file in separate terminals after sourcing the workspace.

```bash
source install/setup.bash
ros2 launch rm_description description.launch.py
```

```bash
source install/setup.bash
ros2 launch rm_localization_adapters localization_adapters.launch.py
```

```bash
source install/setup.bash
ros2 launch rm_chassis_interface chassis_interface_stub.launch.py
```

```bash
source install/setup.bash
ros2 launch rm_navigation_bringup phase1_bringup.launch.py
```

Inspect topics and TF:

```bash
ros2 topic list
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

Inspect command and odometry topics:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odometry/lio
```

`/odometry/lio` is expected to be silent if no raw LIO odometry is being published into `lio_adapter`. That is acceptable for this skeleton validation. The topic should appear only after `lio_adapter` receives raw odometry and publishes adapted output.

## Acceptance Criteria

- `colcon list` recognizes all five Phase 1 packages.
- `colcon build --symlink-install` completes successfully.
- The launch files do not crash immediately due to missing package dependencies.
- `map_odom_stub` is the only `map -> odom` publisher.
- `lio_adapter` is the only `odom -> base_link` publisher.
- `chassis_interface_stub` does not publish TF.
- `chassis_interface_stub` does not publish odometry.
- `chassis_interface_stub` does not publish navigation goals.
- No serial, referee, FAST-LIO, or BT integration is required for this validation.

## Common Failures

### Missing colcon

Install colcon:

```bash
sudo apt install python3-colcon-common-extensions
```

### rosdep Cannot Resolve Dependencies

Make sure ROS2 Humble is sourced and `rosdep update` has completed:

```bash
source /opt/ros/humble/setup.bash
rosdep update
```

If a dependency is still unresolved, check the relevant `package.xml`.

### Missing package.xml Dependencies

Symptoms include build errors such as missing `rclcpp`, `tf2_ros`, `robot_state_publisher`, or `launch_ros`.

Check:

- `src/rm_localization_adapters/package.xml`
- `src/rm_chassis_interface/package.xml`
- `src/rm_description/package.xml`
- `src/rm_navigation_bringup/package.xml`

### CMake Install Directory Missing

If a launch/config/URDF file is not found after sourcing the workspace, check whether its package installs the directory in `CMakeLists.txt`.

### Python Launch Cannot Find Package

Make sure the workspace was rebuilt and sourced:

```bash
colcon build --symlink-install
source install/setup.bash
```

### xacro Or robot_state_publisher Fails

Check:

- `src/rm_description/urdf/rm_sentry_2027.urdf.xacro`
- `src/rm_description/launch/description.launch.py`
- `ros-humble-xacro`
- `ros-humble-robot-state-publisher`

### PGM/YAML Map Format Error

Check:

- `src/rm_nav_config/maps/phase1_empty.yaml`
- `src/rm_nav_config/maps/phase1_empty.pgm`

The Phase 1 PGM should remain a valid minimal PGM file.

### Duplicate TF

Use:

```bash
ros2 run tf2_tools view_frames
```

There must be only one `map -> odom` publisher and only one `odom -> base_link` publisher.
