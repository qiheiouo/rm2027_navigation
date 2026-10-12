#!/usr/bin/env bash
set -eo pipefail
ulimit -c 0
source /opt/ros/humble/setup.bash
cd /workspace
task_build=/workspace/build/fastlio_validation
mkdir -p "$task_build" /workspace/docs/validation/fastlio_deterministic_20261010
colcon --log-base "$task_build/log_driver" build --paths src/livox_ros_driver2_humble/src --build-base "$task_build/driver" --install-base "$task_build/install_driver" --executor sequential > "$task_build/driver.log" 2>&1
source "$task_build/install_driver/setup.bash"
colcon --log-base "$task_build/log_backend" build --paths src/fast_lio_multi --build-base "$task_build/backend" --install-base "$task_build/install_backend" --executor sequential --cmake-args '-DCMAKE_CXX_FLAGS_RELEASE=-O0 -g0 -D_GLIBCXX_ASSERTIONS' > "$task_build/backend.log" 2>&1
python3 tests/fastlio_deterministic/prepare_reference.py src/fast_lio_multi "$task_build/reference_measurement.inc"
cmake -S tests/fastlio_deterministic -B "$task_build/fixtures" -DFASTLIO_SOURCE=/workspace/src/fast_lio_multi -DFASTLIO_LIBRARY="$task_build/backend/fast_lio_multi/libfastlio_preprocess.a" -DFASTLIO_REFERENCE="$task_build/reference_measurement.inc" > "$task_build/fixtures_configure.log" 2>&1
cmake --build "$task_build/fixtures" -j1 > "$task_build/fixtures_build.log" 2>&1
"$task_build/fixtures/imu_fixture" > "$task_build/imu.stdout" 2> "$task_build/imu.stderr"
"$task_build/fixtures/measurement_fixture" > "$task_build/measurement.stdout" 2> "$task_build/measurement.stderr"
"$task_build/fixtures/measurement_fixture" unmatched > "$task_build/unmatched.stdout" 2> "$task_build/unmatched.stderr"
printf '%s\n' 'BUILD_AND_NATIVE_TESTS_PASS'
