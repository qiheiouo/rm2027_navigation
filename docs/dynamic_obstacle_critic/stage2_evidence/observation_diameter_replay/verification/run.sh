#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /out
colcon --log-base /out/build_logs build --base-paths /repo/src --packages-select rm_competition_interfaces rm_dynamic_obstacle_tracking rm_dynamic_obstacle_critic --build-base /out/build --install-base /out/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > /out/build.log 2>&1
source /out/install/setup.bash
python3 -B /repo/src/rm_dynamic_obstacle_critic/tools/run_observation_diameter_preflight.py /out/replayed --build /out/build --install /out/install --v1-archive /history/gazebo_native_cost_accumulator_evidence --anchor-archive /history/observation_anchor_preflight --expected-provenance /archive/provenance.json > /out/replay.log 2>&1
PYTHONPATH=/repo/src/rm_dynamic_obstacle_critic/tools python3 -B /comparison/auditors/compare_observation_diameter_preflight.py /archive /out/replayed /out/comparison.json > /out/comparison.log 2>&1
