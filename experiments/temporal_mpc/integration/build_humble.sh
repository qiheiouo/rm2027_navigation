#!/usr/bin/env bash
set -eo pipefail
# Run INSIDE the pinned Humble project image, in this experimental worktree.
source /opt/ros/humble/setup.bash
set -u
repo=$(cd "$(dirname "$0")/../../.." && pwd)
output="$repo/build/temporal_mpc_ros2"
mkdir -p "$output"
cd "$output"
colcon build --base-paths "$repo/experiments/temporal_mpc/ros2" \
  --build-base build --install-base install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
