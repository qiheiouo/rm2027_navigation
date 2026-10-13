#!/usr/bin/env bash
set -euo pipefail
STVL_STUDY_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
STVL_STUDY_MODE=${1:-build}
STVL_STUDY_OLD=/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption
mkdir -p "$STVL_STUDY_ROOT/build/stvl_cache" "$STVL_STUDY_ROOT/experiments/stvl_temporal/evidence"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 \
 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
 -v "$STVL_STUDY_ROOT/build/stvl_cache:/check:rw" -v "$STVL_STUDY_ROOT/experiments/stvl_temporal/evidence:/evidence:rw" \
 -w "$STVL_STUDY_ROOT" -e STUDY_MODE="$STVL_STUDY_MODE" -e ROS_DOMAIN_ID=174 -e ROS_LOCALHOST_ONLY=1 \
 -e IGN_PARTITION=stvl_temporal_consumption_20261013 -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true \
 -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs \
 rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash
if [ "$STUDY_MODE" = build ]; then
 python3 experiments/stvl_temporal/prepare.py "$PWD" /check/assets
 cmake -S experiments/stvl_temporal -B /check/plugin_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/check/research_install
 cmake --build /check/plugin_build -j2
 cmake --install /check/plugin_build
 dpkg-query -W ros-humble-nav2-mppi-controller ros-humble-spatio-temporal-voxel-layer > /evidence/dependencies.txt
else
 source /check/research_install/share/rm_stvl_temporal_research/local_setup.bash
 python3 experiments/stvl_temporal/run_trial.py "$STUDY_MODE"
fi
'
