#!/usr/bin/env bash
set -euo pipefail
JOINT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$JOINT_ROOT/build/joint_cache/results" "$JOINT_ROOT/build/joint_cache/unit_results" "$JOINT_ROOT/build/joint_cache/fresh_diagnosis"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$JOINT_ROOT/build/joint_cache:/check:rw" rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
for input in /check/unit_inputs/*.json; do /check/replay_build/joint_replay "$input" > /check/unit_results/$(basename "$input"); done
for input in /check/inputs/*.json; do /check/replay_build/joint_replay "$input" > /check/results/$(basename "$input"); done
for input in /check/fresh_inputs/*.json; do [ ! -f "$input" ] || /check/replay_build/joint_replay "$input" > /check/fresh_diagnosis/$(basename "$input"); done
'
