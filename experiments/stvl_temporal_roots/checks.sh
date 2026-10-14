#!/usr/bin/env bash
set -euo pipefail
ROOTS_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --tmpfs /tmp:rw --entrypoint bash \
 -e ROS_LOG_DIR=/tmp/roots_logs -e ROS_DOMAIN_ID=180 -e ROS_LOCALHOST_ONLY=1 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$ROOTS_ROOT/experiments/stvl_temporal_roots/evidence:/evidence:rw" \
 -e ROOTS_ROOT="$ROOTS_ROOT" rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source "$ROOTS_ROOT/build/roots_cache/research_install/share/rm_stvl_temporal_research/local_setup.bash"
"$ROOTS_ROOT/build/roots_cache/plugin_build/geometry_counterexample" > /evidence/geometry_check_final.json
"$ROOTS_ROOT/build/roots_cache/plugin_build/temporal_check" > /evidence/temporal_check_final.json
'
