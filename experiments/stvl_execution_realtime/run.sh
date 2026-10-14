#!/usr/bin/env bash
set -euo pipefail
REALTIME_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
REALTIME_SPEC=${1:?scene:mode:seed:aligned}
[[ "$REALTIME_SPEC" =~ ^(crossing|narrow|wait_release):(A|B|C|D):[0-9]+:aligned$ ]] || exit 2
REALTIME_LABEL=${2:-fresh}
[[ "$REALTIME_LABEL" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
REALTIME_CACHE="$REALTIME_ROOT/build/realtime_cache/$REALTIME_LABEL"
REALTIME_OLD=/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013
mkdir -p "$REALTIME_CACHE/assets"
if [ ! -f "$REALTIME_CACHE/assets/common_nav2.yaml" ]; then cp -a "$REALTIME_OLD/build/roots_cache/assets/." "$REALTIME_CACHE/assets/"; fi
python3 - "$REALTIME_ROOT" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);free=shutil.disk_usage(p).free
used=sum(f.stat().st_size for f in (p/'build/realtime_cache').rglob('*') if f.is_file() and not f.is_symlink())
assert free>=3.15*1024**3, 'disk reserve: no further trial'
assert used<170*1024**2, 'owned cache budget: no further trial'
PY
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 \
 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$REALTIME_CACHE:/check:rw" -w "$REALTIME_ROOT" \
 -e STUDY_MODE="$REALTIME_SPEC" -e REALTIME_PLUGIN="$REALTIME_OLD/build/roots_cache/research_install" \
 -e ROS_DOMAIN_ID=183 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION=stvl_execution_realtime_20261014 -e IGN_IP=127.0.0.1 \
 -e LIBGL_ALWAYS_SOFTWARE=true -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs \
 -e REALTIME_DIAGNOSTIC_CAPTURE="${REALTIME_DIAGNOSTIC_CAPTURE:-off}" \
 rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash
source "$REALTIME_PLUGIN/share/rm_stvl_temporal_research/local_setup.bash"
python3 experiments/stvl_execution_realtime/run_trial.py "$STUDY_MODE"
'
