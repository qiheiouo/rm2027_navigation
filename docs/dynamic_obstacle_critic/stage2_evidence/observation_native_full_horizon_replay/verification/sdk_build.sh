#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /out
colcon --log-base /out/build_logs build --base-paths /repo/src --packages-select rm_competition_interfaces rm_dynamic_obstacle_tracking rm_dynamic_obstacle_critic --build-base /out/build --install-base /out/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > /out/build.log 2>&1
