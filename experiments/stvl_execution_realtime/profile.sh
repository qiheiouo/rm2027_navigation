#!/usr/bin/env bash
set -euo pipefail
REALTIME_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$REALTIME_ROOT/build/realtime_cache/profile"
REALTIME_PRIOR=/home/qihei/rm2027_navigation/build/stvl_joint_execution_20261014/build/joint_cache
# One logical CPU, one process per immutable snapshot; cold means first call,
# not flushed OS file cache. No concurrent Gazebo trial during profiling.
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --cpuset-cpus=0 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$REALTIME_ROOT/build/realtime_cache:/check:rw" -e REALTIME_PRIOR="$REALTIME_PRIOR" rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
/check/small_build/native_motion > /check/native_motion.json
for input in "$REALTIME_PRIOR"/inputs/*.json "$REALTIME_PRIOR"/fresh_inputs/*.json "$REALTIME_PRIOR"/unit_inputs/*.json; do
 /check/small_build/joint_bench "$input" 3 > "/check/profile/$(basename "$input")"
done
'
