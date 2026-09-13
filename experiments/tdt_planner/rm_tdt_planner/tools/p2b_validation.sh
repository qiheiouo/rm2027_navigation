#!/usr/bin/env bash
# Execute only when carrying out the handoff validation. All artifacts persist in /home.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
pkg_rel=experiments/tdt_planner/rm_tdt_planner
work="$repo/build/tdt_p2b"
series=${P2B_SERIES:-static_v2}
[[ "$repo" = /home/* && "$series" =~ ^[A-Za-z0-9_-]+$ ]] || exit 2
mkdir -p "$work" "$work/tmp"
docker_args=(run --rm --init --network none --user "$(id -u):$(id -g)" --entrypoint bash
  -v "$repo:/ws:ro" -v "$work:/work"
  -e ROS_DOMAIN_ID=174 -e ROS_LOCALHOST_ONLY=1 -e PYTHONDONTWRITEBYTECODE=1 -e TMPDIR=/work/tmp
  -e LIBGL_ALWAYS_SOFTWARE=true -e QT_QPA_PLATFORM=offscreen)
container() { docker "${docker_args[@]}" rm2027_navigation:humble "$@"; }
case "${1:-}" in
  deps)
    bash "$repo/$pkg_rel/tools/fetch_dependencies.sh" "$work/deps_source"
    container -c "bash /ws/$pkg_rel/tools/build_dependencies.sh /work/deps_source /work/deps /work/deps-build"
    ;;
  build)
    container -c '
set -e
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="/work/deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/work/deps/lib:${LD_LIBRARY_PATH:-}"
colcon --log-base /work/sim-log build --base-paths /ws/src --packages-up-to rm_simulation \
  --build-base /work/sim-build --install-base /work/sim-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
colcon --log-base /work/log build --base-paths /ws/experiments/tdt_planner/rm_tdt_planner \
  --build-base /work/build --install-base /work/install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DRM_TDT_BUILD_BENCHMARK=ON
'
    ;;
  check)
    container -c '
set -e
source /opt/ros/humble/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR=/work/check-ros
ctest --test-dir /work/build/rm_tdt_planner -V
'
    ;;
  profiles)
    extra=()
    [[ ! -e "$work/profiles_p2b" ]] || extra=(--verify)
    container -c 'python3 /ws/experiments/tdt_planner/rm_tdt_planner/tools/make_sim_profiles.py /ws/src/rm_nav_config/config/nav2_phase1_5_mppi.yaml /work/profiles_p2b "$@"' -- "${extra[@]}"
    ;;
  run)
    planner=${2:?Missing planner}; trial=${3:?Missing trial 1..5}
    [[ "$planner" =~ ^(navfn|smac2d|tdt_astar|tdt_qp)$ && "$trial" =~ ^[1-5]$ ]] || exit 2
    # Freeze relevant sources; reports elsewhere in docs may be edited between trials.
    source_paths=("$pkg_rel" src/rm_simulation src/rm_nav_config src/rm_description src/rm_localization_adapters src/rm_chassis_interface src/rm_competition_interfaces)
    [[ -z "$(git -C "$repo" status --porcelain -- "${source_paths[@]}")" ]] || {
      echo 'Relevant sources are not committed; establish the documented baseline before trials.' >&2; exit 2; }
    container -c 'python3 /ws/experiments/tdt_planner/rm_tdt_planner/tools/make_sim_profiles.py /ws/src/rm_nav_config/config/nav2_phase1_5_mppi.yaml /work/profiles_p2b --verify'
    target="$work/runs/$series/${planner}_$trial"
    [[ ! -e "$target" ]] || { echo "Existing trial preserved: $target" >&2; exit 2; }
    mkdir -p "$target"
    image_id=$(docker image inspect rm2027_navigation:humble --format '{{.Id}}')
    python3 - "$repo" "$target" "$planner" "$trial" "$image_id" <<'METADATA'
import hashlib,json,subprocess,sys
from pathlib import Path
repo,target=map(Path,sys.argv[1:3]); planner,trial,image=sys.argv[3:]
profile=repo/'build/tdt_p2b/profiles_p2b'/f'{planner}.yaml'
files=['src/rm_simulation/worlds/phase1_omni.sdf','src/rm_simulation/models/course_wall.sdf',
       'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml',
       'experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py']
metadata={'planner':planner,'trial':int(trial),'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
          'image_id':image,'profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),
          'fixture_sha256':{f:hashlib.sha256((repo/f).read_bytes()).hexdigest() for f in files},
          'scope':'static Phase 1.5D fixture, goal (4.3,0,0), no hardware',
          'performance_is_not_algorithm_rejection':True}
(target/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
(target/'profile.yaml').write_bytes(profile.read_bytes())
METADATA
    relative="/work/runs/$series/${planner}_$trial"
    set +e
    docker "${docker_args[@]}" --cidfile "$target/container_id" --label "rm.tdt.p2b.series=$series" -e "IGN_PARTITION=tdt_p2b_${series}_${planner}_${trial}" rm2027_navigation:humble \
      "/ws/$pkg_rel/tools/run_simulation_trial.sh" "$relative" "$relative/profile.yaml"
    status=$?
    set -e
    printf '%s\n' "$status" > "$target/docker_exit.txt"
    echo "Trial evidence: $target (exit=$status)"
    exit "$status"
    ;;
  summarize)
    filename=${2:-aggregate_$(date -u +%Y%m%dT%H%M%SZ).json}
    [[ "$filename" =~ ^[A-Za-z0-9_-]+\.json$ ]] || exit 2
    container -c 'python3 /ws/experiments/tdt_planner/rm_tdt_planner/tools/summarize_simulation.py "$1" --output "$2"' -- \
      "/work/runs/$series" "/work/runs/$series/$filename"
    ;;
  *) echo 'usage: bash p2b_validation.sh {deps|build|check|profiles|run PLANNER TRIAL|summarize [FILE.json]}' >&2; exit 2;;
esac
