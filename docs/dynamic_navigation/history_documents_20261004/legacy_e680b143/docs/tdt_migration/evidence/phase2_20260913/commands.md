# P2A 复现命令

本轮以 `/tmp/rm2027_tdt_phase2` 作为代码 worktree，完成后移到持久目录。
以下使用最终目录，并采用新的 CSV/JSON 路径避免覆盖本次证据。
只读挂载现场地图，没有 ROS 主机网络或设备。依赖获取需要网络，构建与测试不需要。

```bash
tdt_code_ws=/home/wpie/worktrees/rm2027_tdt_phase2
tdt_work=/tmp/tdt_phase2
tdt_map_root=/home/wpie/rm2027_navigation/artifacts/maps
pkg="$tdt_code_ws/experiments/tdt_planner/rm_tdt_planner"

# 首次缺少依赖时执行；两份依赖脚本均校验固定 SHA。
bash "$pkg/tools/fetch_dependencies.sh" "$tdt_work/deps_source"
docker run --rm --network none --user 1000:1000 --entrypoint bash \
  -v "$tdt_code_ws:/ws:ro" -v "$tdt_work:/work" \
  rm2027_navigation:humble -c \
  'bash /ws/experiments/tdt_planner/rm_tdt_planner/tools/build_dependencies.sh /work/deps_source /work/deps /work/deps-build'

docker run --rm --network none --user 1000:1000 --entrypoint bash \
  -v "$tdt_code_ws:/ws:ro" -v "$tdt_work:/work" \
  -v "$tdt_map_root:/maps:ro" \
  -e ROS_DOMAIN_ID=174 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/work/runtime-log \
  rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/work/deps:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/work/deps/lib:$LD_LIBRARY_PATH
colcon --log-base /work/log build \
  --base-paths /ws/experiments/tdt_planner/rm_tdt_planner \
  --build-base /work/build --install-base /work/install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON -DRM_TDT_BUILD_BENCHMARK=ON
source /work/install/setup.bash
ctest --test-dir /work/build/rm_tdt_planner -V
/work/build/rm_tdt_planner/benchmark_planners /work/reproduction.csv 100 \
  /maps/old_car_field/20260912T070820Z/old_car_field.yaml \
  > /work/reproduction.log 2>&1
'
python3 "$pkg/tools/summarize_benchmark.py" "$tdt_work/reproduction.csv" \
  --output "$tdt_work/reproduction.json"
```

实际本轮输出名为 `benchmark_phase2.csv` / `benchmark_phase2.log`，汇总保存为本目录
`summary.json`，原始 CSV/log 改为较短文件名复制到本目录，内容未改。
恢复依赖、编译和测试均未写入日常工作区的 build/install/log。

`manifest.json` 包含地图输入、实验代码、证据文件 SHA 和镜像/二进制包版本。
复算 JSON 的命令不需要 ROS，且不会改动 CSV。重复实验的计时和极小 QP 浮点差异
不保证逐字节相同；检查输入一致、语义结果和统计范围，勿把旧数据重命名成新运行。
