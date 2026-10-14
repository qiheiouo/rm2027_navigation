#!/usr/bin/env bash
set -euo pipefail
C83_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
C83_COHORT=${1:-no_shadow_retry}
[[ "$C83_COHORT" =~ ^(no_shadow_retry|shadow|normal)$ ]] || exit 2
C83_CASE=${2:-narrow_C_83_aligned}
[[ "$C83_CASE" =~ ^(narrow_C_83_aligned|crossing_D_29_aligned)$ ]] || exit 2
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
 -v "$C83_ROOT/build/c83_cache:$C83_ROOT/build/c83_cache:rw" \
 -v "$C83_ROOT/experiments/stvl_c83_endtoend/evidence:$C83_ROOT/experiments/stvl_c83_endtoend/evidence:rw" \
 -e C83_REPLAY_DIRECT=1 -e C83_RUN_PATH="$C83_ROOT/build/c83_cache/$C83_COHORT/runs/$C83_CASE" -w "$C83_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; python3 experiments/stvl_c83_endtoend/causal_replay.py "$C83_RUN_PATH"'
