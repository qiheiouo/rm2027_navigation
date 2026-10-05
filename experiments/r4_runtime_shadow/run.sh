#!/usr/bin/env bash
set -euo pipefail
R4_TASK_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
R4_SHADOW_OUT="$R4_TASK_ROOT/build/r4_runtime_shadow_20261005"
R4_MODE=${1:-build}
mkdir -p "$R4_SHADOW_OUT"
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$R4_SHADOW_OUT:/check:rw" \
 -w "$R4_TASK_ROOT" -e R4_MODE="$R4_MODE" -e ROS_DOMAIN_ID=147 -e ROS_LOCALHOST_ONLY=1 \
 -e IGN_PARTITION=r4_runtime_shadow_20261005 -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true \
 sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 bash -c '
set -e
source /opt/ros/humble/setup.bash
source build/r4_adapter_humble_check_20261004/install/setup.bash
source build/r4_source_bridge_humble_check_20261005/install/setup.bash
source /check/profile_install/setup.bash
export CMAKE_PREFIX_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
export ROS_LOG_DIR=/check/ros_logs
if [ "$R4_MODE" = build ]; then
 python3 experiments/r4_runtime_shadow/prepare.py "$PWD" /check/assets
 cmake -S experiments/r4_runtime_shadow -B /check/caller_build -DCMAKE_BUILD_TYPE=Release >/check/caller_build.log 2>&1
 cmake --build /check/caller_build -j2 >>/check/caller_build.log 2>&1
 ldd /check/caller_build/r4_shadow >/check/caller_ldd.txt
else
 case "$R4_MODE" in S0|S1|S2) ;; *) exit 2;; esac
 python3 experiments/r4_runtime_shadow/run_scene.py "$R4_MODE"
 python3 experiments/r4_runtime_shadow/extract.py "/check/$R4_MODE"
fi
'
