#!/usr/bin/env bash
set -euo pipefail
R4_COMP_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
R4_COMP_OUT=${1:-"$R4_COMP_ROOT/build/r4_finite_comparison_20261006"}
R4_COMP_MODE=${2:-build}
R4_COMP_PROFILE=${3:-common}
mkdir -p "$R4_COMP_OUT"
R4_COMP_OUT=$(cd "$R4_COMP_OUT" && pwd)
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
 --ulimit core=0 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$R4_COMP_OUT:/check:rw" \
 -w "$R4_COMP_ROOT" -e R4_COMP_MODE="$R4_COMP_MODE" -e R4_COMP_PROFILE="$R4_COMP_PROFILE" -e ROS_DOMAIN_ID=153 -e ROS_LOCALHOST_ONLY=1 \
 -e IGN_PARTITION=r4_finite_closed_loop_20261006 -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true \
 sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 bash -c '
set -e
source /opt/ros/humble/setup.bash
source build/r4_adapter_humble_check_20261004/install/setup.bash
source build/r4_source_bridge_humble_check_20261005/install/setup.bash
source build/r4_runtime_shadow_20261005/profile_install/setup.bash
export CMAKE_PREFIX_PATH="$PWD/build/r4_follow_endpoint_clock64_20261006/install:$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/build/r4_follow_endpoint_clock64_20261006/install/lib:$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
export ROS_LOG_DIR=/check/ros_logs
if [ "$R4_COMP_MODE" = build ]; then
 python3 experiments/r4_gazebo_comparison/prepare.py "$PWD" /check/assets
 cmake -S experiments/r4_gazebo_comparison -B /check/plugin_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/check/research_install
 cmake --build /check/plugin_build -j2
 cmake --install /check/plugin_build
else
 source /check/research_install/share/r4_gazebo_comparison/local_setup.bash
 export LD_LIBRARY_PATH="/check/research_install/lib:$LD_LIBRARY_PATH"
 python3 experiments/r4_gazebo_comparison/run_trial.py "$R4_COMP_MODE"
fi
'
