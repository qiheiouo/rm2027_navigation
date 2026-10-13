#!/usr/bin/env bash
# Run inside the fixed, no-device Humble container from the repository root.
set -eo pipefail
source /opt/ros/humble/setup.bash
phase=${FINAL_DUAL_PHASE:-after}
case "$phase" in before|after) ;; *) exit 2 ;; esac
b=${FINAL_DUAL_BUILD:-$PWD/build/final_dual_reproduction}
fast=${FASTLIO_SOURCE:-$PWD/src/fast_lio_multi}
adapter=${ADAPTER_SOURCE:-$PWD/src/rm_localization_adapters}
mkdir -p "$b"
# Distinct phase outputs keep the baseline. Use a fresh root for another attempt.
test ! -e "$b/${phase}_started"
date -u > "$b/${phase}_started"
colcon --log-base "$b/log_driver" build --paths src/livox_ros_driver2_humble/src --build-base "$b/driver" --install-base "$b/install_driver" --executor sequential > "$b/driver.log" 2>&1
source "$b/install_driver/setup.bash"
cmake -S "$fast" -B "$b/backend_$phase" '-DCMAKE_CXX_FLAGS_RELEASE=-O0 -g0 -D_GLIBCXX_ASSERTIONS' > "$b/${phase}_backend_configure.log" 2>&1
cmake --build "$b/backend_$phase" --target laserMapping_bundle -j1 > "$b/${phase}_backend_build.log" 2>&1
python3 tests/fastlio_deterministic/prepare_reference.py "$fast" "$b/reference_$phase.inc"
python3 tests/final_dual_closure/prepare_visible_bundle.py "$fast/src/laserMapping_bundle.cpp" "$b/bundle_visible_$phase.inc"
cmake -S tests/fastlio_deterministic -B "$b/fixtures_$phase" -DFASTLIO_SOURCE="$fast" -DFASTLIO_LIBRARY="$b/backend_$phase/libfastlio_preprocess.a" -DFASTLIO_REFERENCE="$b/reference_$phase.inc" -DBUNDLE_VISIBLE="$b/bundle_visible_$phase.inc" > "$b/${phase}_fixture_configure.log" 2>&1
cmake --build "$b/fixtures_$phase" -j1 > "$b/${phase}_fixture_build.log" 2>&1
for fixture in imu measurement; do "$b/fixtures_$phase/${fixture}_fixture" > "$b/${phase}_${fixture}.stdout" 2> "$b/${phase}_${fixture}.stderr"; done
colcon --log-base "$b/log_packages_$phase" build --paths src/rm_competition_interfaces "$adapter" src/rm_lio_bringup src/rm_description --build-base "$b/packages_$phase" --install-base "$b/install_packages_$phase" --executor sequential > "$b/${phase}_packages.log" 2>&1
source "$b/install_packages_$phase/setup.bash"
colcon --log-base "$b/log_test_$phase" test --paths src/rm_competition_interfaces "$adapter" src/rm_lio_bringup src/rm_description --build-base "$b/packages_$phase" --install-base "$b/install_packages_$phase" --packages-select rm_localization_adapters --executor sequential > "$b/${phase}_adapter_test.log" 2>&1
colcon test-result --test-result-base "$b/packages_$phase" --verbose > "$b/${phase}_test_result.log"
cmake -S tests/new_car_dual -B "$b/tf_probe" > "$b/tf_probe_configure.log" 2>&1
cmake --build "$b/tf_probe" -j1 > "$b/tf_probe_build.log" 2>&1
ROS_DOMAIN_ID=181 python3 tests/new_car_dual/probe_bundle.py --binary "$b/backend_$phase/laserMapping_bundle" --output "$b/${phase}_bundle" > "$b/${phase}_bundle.log" 2>&1
modes='control backfill rewind gap new_epoch';options=()
if [ "$phase" = after ]; then modes="$modes imu_rewind lidar_restart duplicate"; options=(-p common.imu_max_gap_sec:=0.1); fi
for mode in $modes; do
    extra=();if [ "$phase" = after ]; then extra=(recovery); fi
    ROS_DOMAIN_ID=183 timeout 15 "$b/fixtures_$phase/bundle_state_trace" "$mode" "${extra[@]}" --ros-args --params-file "$b/${phase}_bundle/software_fixture.yaml" "${options[@]}" > "$b/${phase}_trace_$mode.log" 2>&1
done
ROS_DOMAIN_ID=184 DUAL_TF_PROBE="$b/tf_probe/s2_tf_owner_probe" DUAL_OUTPUT="$b/${phase}_legacy_dynamic" DUAL_INSTALL="$b/install_packages_$phase" python3 tests/new_car_dual/probe_dynamic_adapter.py > "$b/${phase}_legacy_dynamic.log" 2>&1
if [ "$phase" = after ]; then
    ROS_DOMAIN_ID=182 DUAL_TF_PROBE="$b/tf_probe/s2_tf_owner_probe" DUAL_OUTPUT="$b/after_reference" DUAL_INSTALL="$b/install_packages_$phase" python3 tests/final_dual_closure/probe_source_reference.py > "$b/after_reference.log" 2>&1
    ROS_DOMAIN_ID=183 python3 tests/final_dual_closure/probe_epoch_ros.py --binary "$b/backend_$phase/laserMapping_bundle" --output "$b/after_epoch_ros" > "$b/after_epoch_ros.log" 2>&1
    python3 tests/new_car_dual/check_config.py > "$b/after_config.log" 2>&1
fi
