# Docker Usage

This project provides a Phase 1 ROS2 Humble Docker environment for build and minimum launch validation.

The container is for development and validation only. It does not connect real MID360, serial, referee, or competition BT by default.

The image installs Livox-SDK2 during `docker compose build`, so `livox_ros_driver2` can be built inside the container after the driver submodule is initialized.

## Host Requirements

Recommended host:

- Ubuntu 22.04 LTS
- Docker Engine
- Docker Compose plugin

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
```

## Build

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
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
