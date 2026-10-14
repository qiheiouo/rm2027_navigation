#!/usr/bin/env bash
set -euo pipefail
REALTIME_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --cpuset-cpus=0 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$REALTIME_ROOT/build/realtime_cache:/check:rw" rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
mkdir -p /check/views /check/execution_geometry
for input in /check/view_inputs/*.json; do /check/small_build/joint_bench "$input" 1 > /check/views/$(basename "$input"); done
for input in /check/execution_inputs/*.json; do /check/small_build/joint_bench "$input" 1 > /check/execution_geometry/$(basename "$input"); done
'
