#!/usr/bin/env bash
# Isolated known-sweep planning and final-command filter; no deployment config.
set -eo pipefail
trial_dir=$1
profile=$2
case "$trial_dir" in /work/runs/*) ;; *) exit 2;; esac
source /opt/ros/humble/setup.bash
source /work/sim-install/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:$LD_LIBRARY_PATH"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$trial_dir/ros" XDG_RUNTIME_DIR="$trial_dir/xdg"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
launch_pid=
pose_pid=
guard_pid=
cloud_pid=
finish() {
  status=$?
  trap - EXIT INT TERM
  for pid in "$cloud_pid" "$guard_pid" "$pose_pid"; do
    if [[ -n "$pid" ]]; then
      kill -INT -- "-$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    sleep 0.2
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  printf '%s\n' "$status" > "$trial_dir/container_exit.txt"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid python3 /ws/docs/tdt_migration/evidence/dynamic_sweep_routing_v2_20260923/sweep_cloud.py \
  > "$trial_dir/sweep_cloud.log" 2>&1 &
cloud_pid=$!
setsid python3 /ws/docs/tdt_migration/evidence/dynamic_scaled_guard_20260924/scaled_guard_node.py \
  "$profile" "$trial_dir/guard_decisions.jsonl" > "$trial_dir/guard.log" 2>&1 &
guard_pid=$!
setsid ros2 launch /ws/docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/dynamic_guard.launch.py \
  params_file:="$profile" > "$trial_dir/launch.log" 2>&1 &
launch_pid=$!
setsid ign topic -e --json-output -t /world/phase1_omni/pose/info \
  > "$trial_dir/gazebo_poses.jsonl" 2> "$trial_dir/pose_stderr.log" &
pose_pid=$!
set +e
python3 /ws/docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/observe_guard.py \
  "$trial_dir/observation" --goal-x 5.6 --launch-log "$trial_dir/launch.log" \
  --verify-profile "$profile" > "$trial_dir/observer.log" 2>&1
observer_status=$?
set -e
printf '%s\n' "$observer_status" > "$trial_dir/observer_exit.txt"
set +e
python3 /ws/docs/tdt_migration/evidence/snapshot_revalidation_20260922/capture_runtime.py \
  "$profile" "$trial_dir/runtime_geometry.json" > "$trial_dir/geometry_capture.log" 2>&1
printf '%s\n' "$?" > "$trial_dir/geometry_capture_exit.txt"
set -e
exit "$observer_status"
