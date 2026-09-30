#!/usr/bin/env bash
# One isolated, pre-registered Navfn+V1 simulation capture.
set -eo pipefail
out=${1:?new trial directory}
profile=${2:?frozen profile}
source /opt/ros/humble/setup.bash
source /work/dynamic_prediction_runtime_head_20260929/install/setup.bash
source /work/dynamic_prediction_trace_head_20260929/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$out/ros" XDG_RUNTIME_DIR="$out/xdg"
export TDT_MPPI_TRACE_DIR="$out/mppi_cycles"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
pids=()
finish() {
  status=$?
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
  printf '%s\n' "$status" > "$out/container_exit.txt"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch /work/dynamic_prediction_near_x_south_crossing_20260930/dynamic_near_x.launch.py \
  params_file:="$profile" > "$out/launch.log" 2>&1 & pids+=($!)
setsid ign topic -e --json-output -t /world/phase1_omni/pose/info \
  > "$out/gazebo_poses.jsonl" 2> "$out/pose_stderr.log" & pids+=($!)
setsid python3 /ws/docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/static_map.py \
  --ros-args -p use_sim_time:=true > "$out/map.log" 2>&1 & pids+=($!)
setsid python3 /ws/docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/live_nodes.py record "$out" \
  --ros-args -p use_sim_time:=true > "$out/recorder.log" 2>&1 & pids+=($!)
setsid ros2 run rm_dynamic_obstacle_tracking dynamic_obstacle_tracker_node \
  --ros-args --params-file /ws/src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml \
  -p use_sim_time:=true -p map_topic:=/prediction_v1/static_map -p map_frame:=odom \
  -p scan_topic:=/scan -p prediction.velocity_decay_tau:=0.0 \
  > "$out/tracker.log" 2>&1 & pids+=($!)
set +e
python3 /ws/docs/tdt_migration/evidence/dynamic_reference_20260922/observe_dynamic.py \
  "$out/observation" --goal-x 5.25 --goal-y 1.10 --launch-log "$out/launch.log" \
  > "$out/observer.log" 2>&1
observer_status=$?
set -e
printf '%s\n' "$observer_status" > "$out/observer_exit.txt"
set +e
python3 /ws/experiments/dynamic_prediction_v1/frozen_cycle/record_physical_tail.py "$out/gazebo_poses.jsonl" 3.2 > "$out/physical_tail.log" 2>&1
tail_status=$?
set -e
printf '%s\n' "$tail_status" > "$out/physical_tail_exit.txt"
if [ "$tail_status" -ne 0 ]; then exit "$tail_status"; fi
exit "$observer_status"
