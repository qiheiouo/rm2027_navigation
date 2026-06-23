# Docker Usage

This project provides a Phase 1 ROS2 Humble Docker environment for build and minimum launch validation.

The container is for development and validation only. It does not connect real MID360, serial, referee, or competition BT by default.

The image installs Livox-SDK2 during `docker compose build`, so `livox_ros_driver2` can be built inside the container after the driver submodule is initialized.

## Host Requirements

Recommended host:

- Ubuntu 22.04 LTS
- Docker Engine
- Docker Compose plugin
- At least 20 GB of free disk before rebuilding Gazebo and Livox dependencies

Check Docker:

```bash
docker --version
docker compose version
docker run --rm hello-world
```

If `docker run hello-world` requires `sudo`, either use `sudo docker ...` or add the user to the `docker` group according to the team's machine policy.

## Build Image

From the repository root:

```bash
cd rm2027_navigation
git submodule update --init --recursive
USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose.yml build
```

The UID/GID arguments help the container write `build/`, `install/`, and `log/` with the host user's ownership.

If the target network cannot access GitHub during image build, provide a reviewed Livox-SDK2 mirror:

```bash
USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose.yml build \
  --build-arg LIVOX_SDK2_REPO=<reviewed-mirror-url> \
  --build-arg LIVOX_SDK2_REF=<reviewed-branch-or-tag>
```

## Start Shell

```bash
USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose.yml run --rm rm2027_nav
```

Inside the container, ROS2 Humble is already sourced by `docker/entrypoint.sh`.

## Validate Packages

Inside the container:

```bash
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon list
```

The image already installs the current Livox driver build dependencies, including Livox-SDK2, `libaprutil1-dev`, and `ros-humble-pcl-conversions`. The `sudo apt-get update` step is still kept before `rosdep install` so a fresh container has valid apt package indexes if future dependencies are added.

Expected packages:

```text
livox_ros_driver2
rm_localization_adapters
rm_chassis_interface
rm_description
rm_nav_config
rm_navigation_bringup
rm_mid360_driver_bridge
rm_serial_driver
rm_simulation
```

## Build

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
```

Phase 1C protocol tests:

```bash
colcon test --packages-select rm_serial_driver --event-handlers console_direct+
colcon test-result --verbose
```

If the target machine is slow:

```bash
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Minimum Launch Checks

Use separate container shells when needed. In each shell:

```bash
USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose.yml run --rm rm2027_nav
source install/setup.bash
```

Then run:

```bash
ros2 launch rm_description description.launch.py
ros2 launch rm_localization_adapters localization_adapters.launch.py
ros2 launch rm_chassis_interface chassis_interface_stub.launch.py
ros2 launch rm_navigation_bringup phase1_bringup.launch.py
```

No-hardware Nav2 closed-loop validation:

```bash
ros2 launch rm_navigation_bringup phase1_bringup.launch.py use_fake_lio:=true use_nav2:=true use_chassis_stub:=true
```

On a host with GUI forwarding, RViz can be enabled:

```bash
ros2 launch rm_navigation_bringup phase1_bringup.launch.py use_fake_lio:=true use_nav2:=true use_chassis_stub:=true use_rviz:=true
```

Send a small goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

This should produce `/cmd_vel`; `chassis_interface_stub` should log mock packets. This is still no-hardware validation and does not launch FAST-LIO, real MID360, serial, referee, or BT.

Phase 1.5 Gazebo physics validation:

```bash
ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=true use_nav2:=true use_rviz:=false
```

For Gazebo GUI and RViz on an X11 desktop:

```bash
xhost +local:docker
ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=false use_nav2:=true use_rviz:=true
```

See `docs/phase1_5_gazebo_validation.md` for the ground-truth odometry,
holonomic motion, Nav2 goal, and duplicate-TF checks.

Phase 1.5B simulated obstacle validation is documented in
`docs/phase1_5b_obstacle_validation.md`. Its headless launch uses Ogre2 GPU
lidar with Gazebo headless rendering; it does not require a physical GPU, but
the host/container must provide a working EGL or Mesa software-rendering path.

Accepted Phase 1.5 MPPI baseline on the same world:

```bash
export LIBGL_ALWAYS_SOFTWARE=true
ros2 launch rm_simulation phase1_5_mppi.launch.py headless:=true use_rviz:=false
```

See `docs/phase1_5c_mppi_validation.md` for the recorded CPU,
footprint-clearance, and repeatability result. The Phase 1 DWB profile remains
available as a fallback.

Topic and TF checks:

```bash
ros2 topic list
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

`/odometry/lio` may have no data before raw LIO odometry is provided. That is expected in Phase 1 skeleton validation.

## Publish Test Command

With `chassis_interface_stub` running:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

The stub should log a mock chassis command. It must not publish localization TF, odometry, or navigation goals.

## Hardware Notes

This default Docker setup intentionally does not mount `/dev` and does not require privileged mode.

Future real-hardware validation may need:

```text
network_mode: host
privileged: true
/dev mounted into the container
Livox network interface and PTP/GPS time sync setup
serial device permissions
```

Those settings belong to Phase 2 hardware validation and should be added through a documented override file, not silently mixed into the default compile container.
