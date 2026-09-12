#!/usr/bin/env bash
# Offline build. OSQP writes generated headers into its source directory.
set -euo pipefail
source_root=${1:?Usage: bash build_dependencies.sh SOURCE_DIRECTORY PREFIX BUILD_DIRECTORY}
prefix=${2:?Missing absolute install prefix}
build_root=${3:?Missing absolute build directory}
for path in "$source_root" "$prefix" "$build_root"; do
  [[ "$path" = /* && "$path" != / ]] || exit 2
done
[[ $(git -C "$source_root/osqp" rev-parse HEAD) = 0dd00a578cf1c2691c5c379965d504c75bf6cfad ]]
[[ $(git -C "$source_root/osqp-eigen" rev-parse HEAD) = 85c37623774c682db396505f0d4ea677040c2557 ]]
[[ $(git -C "$source_root/osqp/lin_sys/direct/qdldl/qdldl_sources" rev-parse HEAD) = 7d16b70a10a152682204d745d814b6eb63dc5cd2 ]]
for repo in osqp osqp-eigen osqp/lin_sys/direct/qdldl/qdldl_sources; do
  git -C "$source_root/$repo" diff --quiet HEAD
done
cmake -S "$source_root/osqp" -B "$build_root/osqp" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DUNITTESTS=OFF -DPRINTING=OFF -DENABLE_MKL_PARDISO=OFF
cmake --build "$build_root/osqp" --parallel 2
cmake --install "$build_root/osqp"
cmake -S "$source_root/osqp-eigen" -B "$build_root/osqp-eigen" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_TESTING=OFF
cmake --build "$build_root/osqp-eigen" --parallel 2
cmake --install "$build_root/osqp-eigen"
license_dir="$prefix/share/tdt_solver_licenses"
mkdir -p "$license_dir"
cp "$source_root/osqp/LICENSE" "$license_dir/OSQP_LICENSE"
cp "$source_root/osqp/NOTICE" "$license_dir/OSQP_NOTICE"
cp "$source_root/osqp/lin_sys/direct/qdldl/amd/LICENSE" "$license_dir/AMD_LICENSE"
cp "$source_root/osqp/lin_sys/direct/qdldl/qdldl_sources/LICENSE" "$license_dir/QDLDL_LICENSE"
cp "$source_root/osqp-eigen/LICENSE" "$license_dir/OSQP_EIGEN_LICENSE"
