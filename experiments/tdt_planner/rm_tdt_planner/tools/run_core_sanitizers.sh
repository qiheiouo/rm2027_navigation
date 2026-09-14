#!/usr/bin/env bash
# Called by p2b_validation.sh inside the isolated Humble container.
set -euo pipefail
source_root=${1:?Missing pinned solver sources}
run_root=${2:?Missing new evidence directory}
[[ "$run_root" = /work/sanitizer_runs/chain_* && -d "$run_root" ]] || exit 2
pkg=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
prefix="$run_root/deps"
# Do not inherit the unsanitized release prefix through CMake or the loader.
export CMAKE_PREFIX_PATH="$prefix"
export LD_LIBRARY_PATH="$prefix/lib"
export ASAN_OPTIONS=halt_on_error=1:detect_leaks=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
bash "$pkg/tools/build_dependencies.sh" "$source_root" "$prefix" "$run_root/deps-build" sanitizers
flags='-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all'
cmake -S "$pkg" -B "$run_root/core" -DBUILD_TESTING=ON \
  -DRM_TDT_BUILD_ROS2=OFF -DRM_TDT_BUILD_BENCHMARK=OFF -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++ "-DCMAKE_CXX_FLAGS=$flags" \
  "-DCMAKE_EXE_LINKER_FLAGS=$flags" "-DCMAKE_SHARED_LINKER_FLAGS=$flags" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "-DCMAKE_PREFIX_PATH=$prefix" \
  "-DOsqpEigen_DIR=$prefix/lib/cmake/OsqpEigen" "-Dosqp_DIR=$prefix/lib/cmake/osqp"
cmake --build "$run_root/core" --target test_planner -j2
python3 "$pkg/tools/audit_sanitizer_chain.py" "$run_root"
# A small QP isolates the same library allocation / caller destruction boundary.
"$run_root/core/test_planner" --gtest_filter=SolverBoundary.* 2>&1 | tee "$run_root/solver_boundary.log"
ctest --test-dir "$run_root/core" -R '^planner_safety$' -V 2>&1 | tee "$run_root/core_tests.log"
