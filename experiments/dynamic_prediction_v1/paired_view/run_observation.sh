#!/usr/bin/env bash
# Static simulation-only robot, no navigation goal or command publisher.
set -eo pipefail
out=${1:?trial directory}
world=${2:?derived world file}
moving_model=${3:-}
source /opt/ros/humble/setup.bash
source /work/dynamic_prediction_runtime_head_20260929/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$out/ros" XDG_RUNTIME_DIR="$out/xdg"
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
launch_args=(world_file:="$world")
if [[ -n "$moving_model" ]]; then launch_args+=(moving_model_file:="$moving_model"); fi
setsid ros2 launch /ws/experiments/dynamic_prediction_v1/paired_view/observation.launch.py \
  "${launch_args[@]}" > "$out/launch.log" 2>&1 & pids+=($!)
setsid ign topic -e --json-output -t /world/phase1_omni/pose/info \
  > "$out/gazebo_poses.jsonl" 2> "$out/pose_stderr.log" & pids+=($!)
setsid python3 /ws/docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/static_map.py \
  --ros-args -p use_sim_time:=true > "$out/map.log" 2>&1 & pids+=($!)
setsid python3 /ws/docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/live_nodes.py record "$out" \
  --ros-args -p use_sim_time:=true > "$out/prediction_recorder.log" 2>&1 & pids+=($!)
setsid ros2 run rm_dynamic_obstacle_tracking dynamic_obstacle_tracker_node \
  --ros-args --params-file /ws/src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml \
  -p use_sim_time:=true -p map_topic:=/prediction_v1/static_map -p map_frame:=odom \
  -p scan_topic:=/scan -p prediction.velocity_decay_tau:=0.0 \
  > "$out/tracker.log" 2>&1 & pids+=($!)
python3 /ws/experiments/dynamic_prediction_v1/paired_view/record_until.py "$out" \
  --ros-args -p use_sim_time:=true > "$out/observer.log" 2>&1
