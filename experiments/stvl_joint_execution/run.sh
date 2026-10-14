#!/usr/bin/env bash
set -euo pipefail
JOINT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
JOINT_SPEC=${1:?scene:mode:seed:aligned}
[[ "$JOINT_SPEC" =~ ^(crossing|narrow|wait_release):(A|B|C|D):[0-9]+:aligned$ ]] || exit 2
JOINT_LABEL=${2:-fresh}
[[ "$JOINT_LABEL" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
JOINT_CACHE="$JOINT_ROOT/build/joint_cache/$JOINT_LABEL"
JOINT_OLD=/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013
mkdir -p "$JOINT_CACHE/assets"
if [ ! -f "$JOINT_CACHE/assets/common_nav2.yaml" ]; then cp -a "$JOINT_OLD/build/roots_cache/assets/." "$JOINT_CACHE/assets/"; fi
python3 - "$JOINT_ROOT" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);free=shutil.disk_usage(p).free
used=sum(f.stat().st_size for f in (p/'build/joint_cache').rglob('*') if f.is_file() and not f.is_symlink())
assert free>=3.15*1024**3, 'disk reserve: no further trial'
assert used<240*1024**2, 'owned cache budget: no further trial'
PY
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 \
 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$JOINT_CACHE:/check:rw" -w "$JOINT_ROOT" \
 -e STUDY_MODE="$JOINT_SPEC" -e JOINT_PLUGIN="$JOINT_OLD/build/roots_cache/research_install" \
 -e ROS_DOMAIN_ID=181 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION=stvl_joint_execution_20261014 -e IGN_IP=127.0.0.1 \
 -e LIBGL_ALWAYS_SOFTWARE=true -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs \
 rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash
source "$JOINT_PLUGIN/share/rm_stvl_temporal_research/local_setup.bash"
python3 experiments/stvl_joint_execution/run_trial.py "$STUDY_MODE"
'
