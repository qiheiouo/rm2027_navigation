#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
repo=$(cd "$(dirname "$0")/../../.." && pwd)
source "$repo/build/temporal_mpc_ros2/install/setup.bash"
set -u
output=${1:?absolute output directory required}
mode=${2:-shadow}
scenario=${3:-crossing}
if [[ "$mode" == mpc ]]; then
  gate=${4:?MPC requires an absolute prior audit file}
  python3 - "$gate" <<'PY'
import json,sys
with open(sys.argv[1]) as stream: audit=json.load(stream)
if audit.get("mpc_entry_gate") is not True:
    raise SystemExit("MPC entry gate failed; keep shadow mode")
PY
fi
if [[ -e "$output/events.jsonl" || -e "$output/scene/scene.json" ]]; then
  echo "Refusing to overwrite an existing evidence run: $output" >&2; exit 2
fi
mkdir -p "$output"
# Freeze all experiment source before spawning any process; runtime products excluded.
tar --exclude=__pycache__ --exclude=.pytest_cache -czf "$output/source_snapshot.tar.gz" -C "$repo" experiments/temporal_mpc
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen XDG_RUNTIME_DIR=/tmp/temporal_mpc_runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
export PYTHONPATH="$repo/build/temporal_mpc_ros2/python_deps:$repo/experiments/temporal_mpc:${PYTHONPATH:-}"
export TEMPORAL_MPC_SCENE="$output/scene" TEMPORAL_MPC_FRONTEND="$repo/build/temporal_mpc_ros2/frontend"
python3 "$repo/experiments/temporal_mpc/gazebo/prepare_scene.py" "$TEMPORAL_MPC_SCENE" --scenario "$scenario"
pids=()
finish() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep .5
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait || true
}
trap finish EXIT
config="$TEMPORAL_MPC_SCENE/nav2.yaml"
ros2 launch "$repo/experiments/temporal_mpc/gazebo/simulation.launch.py" > "$output/simulation.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/gazebo/scene_node.py" > "$output/frontend.log" 2>&1 & pids+=("$!")
if [[ "$scenario" == head_on ]]; then
  python3 "$repo/experiments/temporal_mpc/gazebo/actor_target.py" > "$output/actor.log" 2>&1 & pids+=("$!")
elif [[ "$scenario" != actuator ]]; then
  ros2 run rm_simulation moving_obstacle_controller --ros-args -p use_sim_time:=true > "$output/actor.log" 2>&1 & pids+=("$!")
fi
ros2 run rm_dynamic_obstacle_tracking dynamic_obstacle_tracker_node --ros-args --params-file "$TEMPORAL_MPC_SCENE/tracker.yaml" > "$output/tracker.log" 2>&1 & pids+=("$!")
if [[ "$mode" == calibration ]]; then
  python3 "$repo/experiments/temporal_mpc/gazebo/calibrate.py" --output "$output" > "$output/recorder.log" 2>&1
  cat "$output/recorder.log"
  exit 0
fi
ros2 run nav2_controller controller_server --ros-args --params-file "$config" -r cmd_vel:=/nav2/cmd_vel > "$output/controller.log" 2>&1 & pids+=("$!")
ros2 run nav2_velocity_smoother velocity_smoother --ros-args --params-file "$config" -r cmd_vel:=/nav2/cmd_vel -r cmd_vel_smoothed:=/cmd_vel > "$output/smoother.log" 2>&1 & pids+=("$!")
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "$config" \
  -p default_nav_to_pose_bt_xml:="$repo/experiments/temporal_mpc/integration/runtime_tree.xml" \
  -p default_nav_through_poses_bt_xml:="$repo/experiments/temporal_mpc/integration/runtime_tree.xml" > "$output/navigator.log" 2>&1 & pids+=("$!")
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true \
  -p node_names:="['controller_server','velocity_smoother','bt_navigator']" > "$output/lifecycle.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/integration/worker_node.py" --ros-args -p use_sim_time:=true \
  -p frontend:="$TEMPORAL_MPC_FRONTEND" > "$output/worker.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/integration/selector_node.py" > "$output/selector.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/gazebo/record_run.py" --output "$output" --mode "$mode" > "$output/recorder.log" 2>&1
cat "$output/recorder.log"
