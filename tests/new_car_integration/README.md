# 新车软件基线复现

本精简基线从正式新车bd6cf689建立，选择性保留3911af9c和753131e6的完整有效代码与接口；最终冻结候选另行迁入AUD-003 dry-run守卫，真实硬件运动仍禁止。[运行合同](../../docs/contracts/new_car_software_20261012.md)规定两个独立环境，不将Humble与Jazzy混成一张运行图。

## Git/submodule恢复

当前精简分支保留64d0a2d4的实际生产代码、配置、测试与gitlink，历史日志/批量JSON/冷备不进入此分支。来源、远端恢复边界和回退见[软件保存摘要](../../docs/new_car_software_saved_20261012.md)。

最终冻结候选的FAST-LIO固定69f10703、ikd-Tree0438b0da、Livox2a2029a6；较早软件候选仍固定ae5bef29。当前.gitmodules仍指向上游；没有确认可写私有fork前，不能承诺任意异机都能恢复本地修复pin。离线恢复需从已验证完整bundle克隆FAST子模块、checkout固定commit，再按.gitmodules初始化依赖；请按摘要执行，不引用未迁入的历史validation文件。

## Humble目标构建/回归

镜像 `rm2027_navigation:humble` / `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。容器仅挂本工作区到/workspace：network none、ipc private、uid1000、2cpu、6GiB、pids256、cap-dropALL、no-new-privileges，无device/socket。

```bash
set -eo pipefail
ulimit -c 0
source /opt/ros/humble/setup.bash
mkdir -p build
bash tests/fastlio_deterministic/build_and_test.sh
source build/fastlio_validation/install_driver/setup.bash
export ROS_LOG_DIR=/workspace/build/humble_ros_logs
colcon --log-base build/humble_log build \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source build/humble_install/setup.bash
colcon --log-base build/humble_test_log test \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install --executor sequential
colcon test-result --test-result-base build/humble_build
```

Livox overlay必须先source，否则mid360_bridge确实缺依赖。不要对含可选Jazzy/Gazebo8包的整个src宣称Humble全仓构建。初始driver/backend构建与core独立；core不改源码，已有firmware/driver不用syntheticSDK替代。

复用原`tests/localization_validity`：设置`PROBE_INSTALL=/workspace/build/humble_install`，为每个独立case分配不同ROS_DOMAIN_ID，输出PROBE_OUTPUT到own build；依次跑probe_ros（amcl/gicp）、probe_backends（两backend）、probe_timing、probe_profiles。README保留原完整命令；禁止覆盖继承证据目录。时序probe只改fixtureclock，生产预算不变。

`source build/fastlio_validation/install_backend/setup.bash`后再sourcecore，运行：

```bash
export ROS_DOMAIN_ID=208 ENTRY_OUTPUT=/workspace/build/software_entry
python3 tests/new_car_integration/probe_software_entry.py
```

它验证inventory、opt-in、硬件mode拒绝、bundle无输入等待/零odom、未标定fusion拒绝，以及独立V2初始disabled、普通`/cmd_vel`不能绕过守卫和正常退出。正授权、限幅、超时、撤权与恢复由`tests/aud003_final_motion`在最终编码字节边界验证；两者都**不是硬件Nav2→serial授权链**。

完整ROS bundle matched/unmatched（同一Humble容器和Livox overlay）：

```bash
export ROS_DOMAIN_ID=209
python3 tests/fastlio_deterministic/ros_probe.py \
  --binary build/fastlio_validation/backend/fast_lio_multi/laserMapping_bundle \
  --output build/bundle_ros_debug
cmake -S src/fast_lio_multi -B build/fastlio_release
cmake --build build/fastlio_release --target laserMapping_bundle -j1
export ROS_DOMAIN_ID=210
python3 tests/fastlio_deterministic/ros_probe.py \
  --binary build/fastlio_release/laserMapping_bundle --output build/bundle_ros_release
```

upstream CMake实际强制Release；原debug fixture通过覆盖Release flags开启`-O0 -D_GLIBCXX_ASSERTIONS`，第二构建保留原`-O3 -DNDEBUG`。输入明确为900点平面/200Hz IMU，不能称真实sensor或精度。

## S2实际Goal/behavior与TF路由

复用已安装的Jazzy/Gazebo8环境，独立构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/jazzy_log build \
  --base-paths src/rm_nav_config src/rm_new_car_simulation src/rm_navigation_bringup \
  --build-base build/jazzy_build --install-base build/jazzy_install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
source build/jazzy_install/setup.bash
export ROS_LOG_DIR="$PWD/build/jazzy_ros_logs"
export ROS_DOMAIN_ID=194 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=rm2027_newcar_software_20261012
python3 build/jazzy_install/rm_new_car_simulation/lib/rm_new_car_simulation/smoke_baseline.py \
  --no-sensors --software-entry --output "$PWD/build/s2_goal_behavior"
```

工具的只读/禁socket沙箱不能有效运行本地DDS/Gazebo，需普通本地执行权限；不打开真实设备。S2只启动自身进程、full-pose oracle、RSP、Nav2/smoother，无LIO/global backend/serial/Mission。仿真Goal到达和Spin需分别status=4；实际raw publishers=controller1+behavior4、最终=唯一smoother；rclcpp test-only观察器记录map→odom/odom→base_link各唯一endpoint，不依靠Python版本缺失的publisher_gid。

所有soft失败保持证据；不以Goal成功替代odom、TF或路由检查。场地/模型继续为S2既有provisional软件假设，不调参验证真实动力学。原S2冻结记录留d5673e3a，不把它当本轮实车记录。
