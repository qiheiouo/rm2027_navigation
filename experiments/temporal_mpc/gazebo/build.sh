#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
repo=$(cd "$(dirname "$0")/../../.." && pwd)
# Build existing canonical simulation/localization packages from unchanged main,
# plus the experimental overlay. No hardware driver or original dirty tracker.
cd "$repo/build/temporal_mpc_ros2"
colcon build --executor sequential --base-paths \
  "$repo/experiments/temporal_mpc/ros2" \
  "$repo/src/rm_simulation" "$repo/src/rm_description" \
  "$repo/src/rm_localization_adapters" "$repo/src/rm_chassis_interface" \
  --build-base build --install-base install --cmake-args -DCMAKE_BUILD_TYPE=Release
