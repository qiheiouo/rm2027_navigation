#!/usr/bin/env bash
set -euo pipefail
ACTIVE_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
ACTIVE_KIND=${1:?full or active};ACTIVE_SPEC=${2:?scene:C:seed:aligned}
[[ "$ACTIVE_KIND" == full || "$ACTIVE_KIND" == active ]] || exit 2
[[ "$ACTIVE_SPEC" == crossing:C:13:aligned || "$ACTIVE_SPEC" == narrow:C:83:aligned ]] || exit 2
ACTIVE_CACHE="$ACTIVE_ROOT/build/active_scope_cache"
ACTIVE_BASE=/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014
python3 - "$ACTIVE_CACHE" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);assert shutil.disk_usage(p).free>3.15*1024**3
assert sum(f.stat().st_size for f in p.rglob('*') if f.is_file())<200*1024**2
PY
ACTIVE_SCENE=${ACTIVE_SPEC%%:*}
ACTIVE_CONTAINER="rm_stvl_active_scope_${ACTIVE_KIND}_${ACTIVE_SCENE}"
docker run --name "$ACTIVE_CONTAINER" --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$ACTIVE_CACHE/$ACTIVE_KIND:/check:rw" -v "$ACTIVE_CACHE/prediction_adapter.py:$ACTIVE_BASE/experiments/stvl_temporal/prediction_adapter.py:ro" -v "$ACTIVE_ROOT/experiments/stvl_active_scope:/study:ro" -w "$ACTIVE_BASE" \
 -e STUDY_MODE="$ACTIVE_SPEC" -e PROJECTION_KIND="$ACTIVE_KIND" -e ROS_DOMAIN_ID=187 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="stvl_active_scope_$ACTIVE_KIND" -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true \
 -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs -e REALTIME_DIAGNOSTIC_CAPTURE=off -e C83_SHADOW=off \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash; source /home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014/build/c83_cache/research_install/share/rm_stvl_temporal_research/local_setup.bash; python3 -c "import warnings,runpy,os,sys; sys.argv=[\"experiments/stvl_c83_endtoend/run_trial.py\",os.environ[\"STUDY_MODE\"]]; runpy.run_path(sys.argv[0],run_name=\"__main__\")"' &
ACTIVE_PID=$!
while kill -0 "$ACTIVE_PID" 2>/dev/null; do
 sleep 2
 if ! python3 - "$ACTIVE_CACHE" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);free=shutil.disk_usage(p).free;used=sum(f.stat().st_size for f in p.rglob('*') if f.is_file())
sys.exit(0 if free>=3.03*1024**3 and used<200*1024**2 else 1)
PY
 then
  printf '%s\n' 'owned cache/disk guard: stopping current container, retain failed attempt' > "$ACTIVE_CACHE/${ACTIVE_KIND}_${ACTIVE_SCENE}_resource_abort.txt"
  docker stop --time 10 "$ACTIVE_CONTAINER"
  break
 fi
done
wait "$ACTIVE_PID"
