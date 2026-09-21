#!/usr/bin/env bash
# Container entry point. Only p2b_validation.sh starts this with an isolated network.
set -eo pipefail
trial_dir=${1:?Missing container trial directory}
profile=${2:?Missing complete simulation profile}
case "$trial_dir" in /work/runs/*) ;; *) exit 2;; esac
source /opt/ros/humble/setup.bash
source /work/sim-install/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$trial_dir/ros"
export XDG_RUNTIME_DIR="$trial_dir/xdg"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
launch_pid=
finish() {
  status=$?
  trap - EXIT INT TERM
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
setsid ros2 launch /ws/docs/tdt_migration/evidence/new_car_geometry_audit_20260917/reference.launch.py \
  params_file:="$profile" > "$trial_dir/launch.log" 2>&1 &
launch_pid=$!
observer_extra=()
if [[ "${TDT_HEADING_AB:-0}" == 1 ]]; then observer_extra=(--verify-profile "$profile"); fi
set +e
python3 /ws/experiments/tdt_planner/rm_tdt_planner/tools/observe_simulation.py \
  "$trial_dir/observation" --launch-log "$trial_dir/launch.log" "${observer_extra[@]}" \
  > "$trial_dir/observer.log" 2>&1
observer_status=$?
printf '%s\n' "$observer_status" > "$trial_dir/observer_exit.txt"
set -e
python3 /ws/docs/tdt_migration/evidence/new_car_geometry_audit_20260917/capture_runtime.py "$profile" "$trial_dir/runtime_geometry.json" > "$trial_dir/geometry_capture.log" 2>&1
exit "$observer_status"
