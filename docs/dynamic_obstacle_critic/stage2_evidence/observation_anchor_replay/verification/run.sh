#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /out
colcon --log-base /out/build_logs build \
  --base-paths /repo/src/rm_competition_interfaces /repo/src/rm_dynamic_obstacle_critic /repo/src/rm_dynamic_obstacle_tracking \
  --build-base /out/build --install-base /out/install \
  --packages-select rm_competition_interfaces rm_dynamic_obstacle_critic rm_dynamic_obstacle_tracking \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > /out/build.log 2>&1
source /out/install/setup.bash
python3 -B /repo/src/rm_dynamic_obstacle_critic/tools/replay_observation_anchor_preflight.py \
  /archive /out/replayed /out/build /out/install > /out/replay.log 2>&1
