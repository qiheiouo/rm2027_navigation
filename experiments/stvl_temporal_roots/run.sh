#!/usr/bin/env bash
set -euo pipefail
ROOTS_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
ROOTS_COHORT=${1:?cohort};ROOTS_SPEC=${2:?case spec}
case "$ROOTS_COHORT" in before|before_repeat|diag|repair|heldout_repair|crash_debug|crash_alignment|repair_abi|heldout_abi|heldout_abi_retry|repair_odom|heldout_odom) ;; *) echo 'unknown frozen cohort' >&2; exit 2;; esac
ROOTS_CACHE_NAME=$ROOTS_COHORT
if [ -n "${3:-}" ]; then
 if [[ ! "$3" =~ ^[a-zA-Z0-9_-]+$ ]]; then echo 'invalid separate replay label' >&2; exit 2; fi
 ROOTS_CACHE_NAME="${ROOTS_COHORT}__$3"
fi
ROOTS_OLD=/home/qihei/rm2027_navigation/build/stvl_temporal_consumption_20261013
mkdir -p "$ROOTS_ROOT/build/roots_cache/assets" "$ROOTS_ROOT/build/roots_cache/$ROOTS_CACHE_NAME" "$ROOTS_ROOT/experiments/stvl_temporal_roots/evidence"
if [ ! -f "$ROOTS_ROOT/build/roots_cache/assets/common_nav2.yaml" ]; then cp -a "$ROOTS_OLD/build/stvl_cache/assets/." "$ROOTS_ROOT/build/roots_cache/assets/"; fi
if [ ! -e "$ROOTS_ROOT/build/roots_cache/$ROOTS_CACHE_NAME/assets" ]; then ln -s "$ROOTS_ROOT/build/roots_cache/assets" "$ROOTS_ROOT/build/roots_cache/$ROOTS_CACHE_NAME/assets"; fi
ROOTS_PLUGIN=$ROOTS_OLD/build/stvl_cache/research_install
if [ "$ROOTS_COHORT" != before ] && [ "$ROOTS_COHORT" != before_repeat ]; then ROOTS_PLUGIN=$ROOTS_ROOT/build/roots_cache/research_install; fi
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 \
 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$ROOTS_ROOT/build/roots_cache/$ROOTS_CACHE_NAME:/check:rw" \
 -v "$ROOTS_ROOT/experiments/stvl_temporal_roots/evidence:/evidence:rw" -w "$ROOTS_ROOT" \
 -e STUDY_MODE="$ROOTS_SPEC" -e ROOTS_COHORT="$ROOTS_COHORT" -e ROOTS_PLUGIN="$ROOTS_PLUGIN" \
 -e ROS_DOMAIN_ID=179 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION=stvl_temporal_roots_20261013 -e IGN_IP=127.0.0.1 \
 -e LIBGL_ALWAYS_SOFTWARE=true -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs \
 rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash
source "$ROOTS_PLUGIN/share/rm_stvl_temporal_research/local_setup.bash"
if [ "$ROOTS_COHORT" = before ] || [ "$ROOTS_COHORT" = before_repeat ]; then python3 experiments/stvl_temporal/run_trial.py "$STUDY_MODE"; else python3 experiments/stvl_temporal_roots/run_trial.py "$STUDY_MODE"; fi
'
