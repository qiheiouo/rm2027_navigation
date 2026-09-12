# T-DT 规划迁移实验

状态：独立实验候选。常规 workspace 扫描由上层 `COLCON_IGNORE` 排除；所有现有
旧车/新车启动入口继续使用原有配置。此包无节点可执行文件，无传感器、TF、目标、
串口或速度发布器。只有显式加载插件才能参与 `ComputePathToPose`。

使用固定 T-DT YAstar、地图相关 Douglas–Peucker 化简、方形约束与 OSQP 后端。
五次多项式最小 jerk 输出经独立校验后转为标准 `nav_msgs/Path`。调用者仍由 Nav2
管理；MPPI、任务 action owner、最终速度门和定位 TF 的合同不变。

## 构建与检查

需要 GCC 11/C++20（算法副本）、C++17（适配层）、CMake、Eigen 3.4、GTest；
插件还需要 ROS2 Humble Nav2 与 pluginlib。求解器使用本包锁定版本，不能按上游
README 假定子模块已提供。不要在构建时下载依赖或运行上游 setup 脚本。

以下示例在已 source ROS2 Humble 的隔离开发环境中执行，`candidate_ws` 指向本
实验分支工作区。保持所有构建目录独立于日常导航的 `build/install/log`：

```bash
candidate_ws=/tmp/rm2027_tdt_planner
pkg="$candidate_ws/experiments/tdt_planner/rm_tdt_planner"
bash "$pkg/tools/fetch_dependencies.sh" /tmp/tdt-deps-source
bash "$pkg/tools/build_dependencies.sh" /tmp/tdt-deps-source /tmp/tdt-deps /tmp/tdt-deps-build
export CMAKE_PREFIX_PATH="/tmp/tdt-deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/tmp/tdt-deps/lib:${LD_LIBRARY_PATH:-}"
colcon --log-base /tmp/tdt-log build --base-paths "$pkg" \
  --build-base /tmp/tdt-build --install-base /tmp/tdt-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /tmp/tdt-install/setup.bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 TDT_BENCHMARK_CSV=/tmp/tdt-benchmark.csv \
  ctest --test-dir /tmp/tdt-build/rm_tdt_planner --output-on-failure
```

只有算法测试时，可直接 CMake，并使用 `-DRM_TDT_BUILD_ROS2=OFF`。
ROS 测试只配置空 costmap 和调用插件 API，不激活 costmap，不发布 TF 或速度。
测试镜像应使用 `--network none`、无设备挂载与独立临时目录。

生成完整候选配置（不覆盖源配置，不自动启动）：

```bash
python3 "$pkg/tools/make_profile.py" \
  "$candidate_ws/src/rm_nav_config/config/nav2_old_car_2026_dual_stvl.yaml" \
  /tmp/nav2_tdt_candidate.yaml
# 另一个新文件加 --frontend-only 可比较 A* 与 A*+QP。
```

片段 `config/planner_parameters.yaml` 不是完整 Nav2 profile。保留插件 ID
`GridBased` 是为了兼容现有 BT；当前旧车 BT 不调用 `SmoothPath`，故后端必须在
规划插件内部运行，仅注册 smoother 并不会生效。该生成器不复制主工作区未提交参数；
如需那些参数，显式以主工作区文件为 baseline，并在仿真中比较。

## 几何与失败语义

- 仅接受 axis-aligned Nav2 costmap：行主序、y 正向、保留 resolution/origin。
  不能直接传 PGM 顶行顺序或带旋转的 YAML origin；应先由 map_server/costmap 转换。
- 253/254/255 都阻塞，未知区不通行；其他 cost 转为自由区后重建距离势场。
  因而自定义软语义 cost 不保留。本候选不能据此替换语义路线/特殊通道执行。
- 使用 **padded footprint 外接圆 + clearance**，距离变换再预留一个栅格对角线。
  允许任意自转时仍保守；已经膨胀的 253 区域会再次保守膨胀，可能拒绝窄通道。
  狗洞、定向穿越和坡道不属于本轮适用场景，不得为了通过而缩小真实 footprint。
- 将上游角点结果转为单元中心，保留精确起终点；不把不可行 goal 移到别处。
- 开启上游严格碰撞模式，并独立对输出折线逐段做 supercover 检查。无解、非法坐标、
  footprint 内碰撞、超时返回空失败；QP 失败/不安全/控制点超预算，只回退到该次
  snapshot 已通过检查的 A* 路径，绝不复用旧路径。
- 搜索周期性检查时间预算；QP 仍只能在上游一次求解返回后判超时，`time_budget`
  是**拒收迟到结果的软预算**，不是可抢占的硬实时上限。未通过 target CPU p99
  与最坏情况测试前不能用于要求确定响应期限的运动链路。
- costmap 仅复制时加锁，求解期间允许原图更新；完成后若任意 cost、原点、分辨率、
  尺寸或 footprint 半径改变则拒收。保证对所检查 snapshot 一致；不能保证返回以后
  环境不再变化，局部感知/控制/安全门仍必须运行。
- 上游 `SolveOutput` 没有时间/速度/加速度/系数；`setDt()` 不控制当前 solve 路径
  所用的每段 11 点采样。重采样只细分折线，不恢复多项式。这不构成连续曲线碰撞
  证明，也不保证跟踪速度、加速度、jerk 或 MPC 可行性。

退出实验：使用原 Nav2 参数文件并移除实验 install 的环境 overlay。实验 package
不在常规构建/启动路径内，可整目录移除；无数据库、地图或 TF 状态迁移。

来源/补丁与后续阶段见 `docs/external/tdt_nav_kit.md`、
`docs/tdt_migration/assessment_and_plan.md`。
