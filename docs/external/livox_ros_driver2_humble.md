# Livox ROS2 Humble Driver Intake Record

This record documents the external MID360 ROS driver added for Phase 1 hardware integration.

## Source

- URL: `https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble.git`
- Local path: `src/livox_ros_driver2_humble`
- Integration type: Git submodule
- Branch at intake: `master`
- Commit at intake: `2a2029a6e62a2196b280be6ec00bb2418065b8e0`
- ROS package name: `livox_ros_driver2`
- License: MIT
- Internal submodules: none observed; the driver repository has no `.gitmodules`.

## Why This Driver

The target runtime is ROS2 Humble and MID360. This fork was already reviewed during open-source research and matches the driver API expected by `rm_mid360_driver_bridge`:

- executable: `livox_ros_driver2_node`
- package: `livox_ros_driver2`
- config parameter: `user_config_path`
- point cloud frame parameter: `frame_id`
- output modes including `xfer_format=4`

## Required Linux SDK Install

Install Livox-SDK2 on Ubuntu 22.04 before building the driver:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libapr1-dev libpcl-dev

mkdir -p ~/third_party
cd ~/third_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

If GitHub is unavailable on the target network, use an explicitly reviewed Livox-SDK2 mirror and record its URL and commit before relying on it.

## Clone And Submodule Commands

Fresh clones should include submodules:

```bash
git clone --recurse-submodules https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

There is no second-level submodule initialization requirement inside `livox_ros_driver2_humble` at this intake commit.

## Build Notes

With Livox-SDK2 installed:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Without Livox-SDK2, non-hardware packages can still be checked by skipping the driver:

```bash
colcon build --symlink-install --packages-skip livox_ros_driver2
```

## Boundary Rules

The driver is a sensor input dependency only. It must not publish localization TF, canonical odometry, chassis commands, navigation goals, serial packets, referee data, or behavior-tree commands.

`rm_mid360_driver_bridge` owns the project-side topic remaps:

- `/livox/left/lidar`
- `/livox/right/lidar`
- `/livox/left/pointcloud`
- `/livox/right/pointcloud`
- `/livox/lio_imu_raw`

`rm_localization_adapters/imu_frame_adapter` remains responsible for converting the selected raw driver IMU into canonical `/livox/lio_imu` with `header.frame_id=lio_imu_link`.

## Known Frame Caveat

The inspected driver uses the `frame_id` parameter for point cloud messages, but its IMU message frame may not follow that parameter. The project therefore keeps the IMU adapter boundary instead of trusting driver IMU frame names directly.
