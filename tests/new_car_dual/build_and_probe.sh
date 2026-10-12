#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /workspace
b=/workspace/build/dual_validation
colcon --log-base "$b/log_driver" build --paths src/livox_ros_driver2_humble/src --build-base "$b/driver" --install-base "$b/install_driver" --executor sequential > "$b/driver.log" 2>&1
source "$b/install_driver/setup.bash"
cmake -S src/fast_lio_multi -B "$b/backend" '-DCMAKE_CXX_FLAGS_RELEASE=-O0 -g0 -D_GLIBCXX_ASSERTIONS' > "$b/backend_configure.log" 2>&1
cmake --build "$b/backend" --target laserMapping_bundle -j1 > "$b/backend_build.log" 2>&1
python3 tests/fastlio_deterministic/prepare_reference.py src/fast_lio_multi "$b/reference_measurement.inc"
cmake -S tests/fastlio_deterministic -B "$b/fixtures" -DFASTLIO_SOURCE=/workspace/src/fast_lio_multi -DFASTLIO_LIBRARY="$b/backend/libfastlio_preprocess.a" -DFASTLIO_REFERENCE="$b/reference_measurement.inc" > "$b/fixtures_configure.log" 2>&1
cmake --build "$b/fixtures" -j1 > "$b/fixtures_build.log" 2>&1
"$b/fixtures/imu_fixture" > "$b/imu.stdout" 2> "$b/imu.stderr"
"$b/fixtures/measurement_fixture" > "$b/measurement.stdout" 2> "$b/measurement.stderr"
"$b/fixtures/measurement_fixture" unmatched > "$b/unmatched.stdout" 2> "$b/unmatched.stderr"
colcon --log-base "$b/log_packages" build --base-paths src --packages-ignore fast_lio_multi livox_ros_driver2 --packages-select rm_competition_interfaces rm_localization_adapters rm_mid360_driver_bridge rm_lio_bringup rm_description rm_relocalization_bridge rm_nav_config --build-base "$b/packages" --install-base "$b/install_packages" --executor sequential > "$b/packages.log" 2>&1
source "$b/install_packages/setup.bash"
colcon --log-base "$b/log_tests" test --base-paths src --build-base "$b/packages" --install-base "$b/install_packages" --packages-select rm_localization_adapters rm_mid360_driver_bridge --executor sequential > "$b/tests.log" 2>&1
colcon test-result --test-result-base "$b/packages" --verbose > "$b/test_results.txt"
python3 tests/new_car_dual/check_config.py > "$b/config_tests.log" 2>&1
ROS_DOMAIN_ID=171 python3 tests/new_car_dual/probe_bundle.py --binary "$b/backend/laserMapping_bundle" --output "$b/dual_ros" > "$b/dual_ros_probe.log" 2>&1
cmake -S tests/new_car_dual -B "$b/tf_probe" > "$b/tf_probe_configure.log" 2>&1
cmake --build "$b/tf_probe" -j1 > "$b/tf_probe_build.log" 2>&1
ROS_DOMAIN_ID=172 DUAL_TF_PROBE="$b/tf_probe/s2_tf_owner_probe" DUAL_OUTPUT="$b/dynamic_ros" DUAL_INSTALL="$b/install_packages" python3 tests/new_car_dual/probe_dynamic_adapter.py > "$b/dynamic_probe.log" 2>&1
printf '%s\n' TARGETED_PROBES_PASS
