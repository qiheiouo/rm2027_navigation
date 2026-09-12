#!/usr/bin/env bash
# Explicit network step. Does not install system packages or run upstream setup.sh.
set -euo pipefail
source_root=${1:?Usage: bash fetch_dependencies.sh ABSOLUTE_SOURCE_DIRECTORY}
[[ "$source_root" = /* && "$source_root" != / ]] || exit 2
mkdir -p "$source_root"
fetch() {
  local name=$1 url=$2 tag=$3 sha=$4
  if [[ ! -e "$source_root/$name" ]]; then
    git clone --depth 1 --branch "$tag" "$url" "$source_root/$name"
  fi
  [[ $(git -C "$source_root/$name" rev-parse HEAD) = "$sha" ]]
  git -C "$source_root/$name" diff --quiet HEAD
}
fetch osqp https://github.com/osqp/osqp.git v0.6.3 0dd00a578cf1c2691c5c379965d504c75bf6cfad
git -C "$source_root/osqp" submodule update --init --recursive
[[ $(git -C "$source_root/osqp/lin_sys/direct/qdldl/qdldl_sources" rev-parse HEAD) = 7d16b70a10a152682204d745d814b6eb63dc5cd2 ]]
fetch osqp-eigen https://github.com/robotology/osqp-eigen.git v0.8.1 85c37623774c682db396505f0d4ea677040c2557
