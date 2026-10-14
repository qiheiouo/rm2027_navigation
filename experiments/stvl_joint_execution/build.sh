#!/usr/bin/env bash
set -euo pipefail
JOINT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$JOINT_ROOT/build/joint_cache"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$JOINT_ROOT/build/joint_cache:/check:rw" -w "$JOINT_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; cmake -S experiments/stvl_joint_execution -B /check/replay_build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/replay_build -j2'
