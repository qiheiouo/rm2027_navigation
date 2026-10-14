#!/usr/bin/env bash
set -euo pipefail
CONTRACT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$CONTRACT_ROOT/build/failure_contract_cache"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$CONTRACT_ROOT/build/failure_contract_cache:/check:rw" -w "$CONTRACT_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; cmake -S experiments/stvl_failure_contract -B /check/build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/build -j2; /check/build/failure_contract'
