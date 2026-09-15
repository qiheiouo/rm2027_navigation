# T-DT 规划迁移实验

状态：独立实验候选。常规 workspace 扫描由上层 `COLCON_IGNORE` 排除；所有现有
旧车/新车启动入口继续使用原有配置。算法 core、插件和 P2A 离线比较程序无传感器、
TF、目标、串口或速度发布器。P2B 的独立仿真记录器作为该隔离域的测试 action client，
仅经完整仿真入口发送一次规划预检查与一次导航目标；不连接任务层或实车。

使用固定 T-DT YAstar、地图相关 Douglas–Peucker 化简、方形约束与 OSQP 后端。
五次多项式最小 jerk 输出经独立校验后转为标准 `nav_msgs/Path`。调用者仍由 Nav2
管理；MPPI、任务 action owner、最终速度门和定位 TF 的合同不变。

## 构建与检查

需要 GCC 11/C++20（算法副本）、C++17（适配层）、CMake、Eigen 3.4、GTest；
插件还需要 ROS2 Humble Nav2 与 pluginlib。求解器使用本包锁定版本，不能按上游
README 假定子模块已提供。不要在构建时下载依赖或运行上游 setup 脚本。

当前开发与验证都使用 `/home` 下的持久工作区。按用户 2026-09-15 最新约定，
开发者同时负责执行测试、仿真和结果复核。static_v4 已通过统一 sanitizer 链和
46 项常规测试，但 T-DT 两组导航仍失败。端点连接修复及本轮实际结果见
[端点几何修复记录](../../../docs/tdt_migration/p2b_endpoint_connections.md)。

```bash
candidate_ws=/home/wpie/worktrees/rm2027_tdt_phase2
pkg="$candidate_ws/experiments/tdt_planner/rm_tdt_planner"
work="$candidate_ws/build/tdt_p2b"
# 仅依赖缺失时执行 deps（含显式网络取回）。
bash "$pkg/tools/p2b_validation.sh" deps
bash "$pkg/tools/p2b_validation.sh" build
bash "$pkg/tools/p2b_validation.sh" check
```

包装脚本使用现有 `rm2027_navigation:humble`，代码只读、`--network none`、无设备
挂载；所有依赖/构建/日志在本实验 worktree 的 `build/tdt_p2b/`，不写日常导航目录。
只有算法测试时仍可直接 CMake，并使用 `-DRM_TDT_BUILD_ROS2=OFF`。
完整的执行前提、预期结果与失败处置见
[`p2b_terra_handoff.md`](../../../docs/tdt_migration/p2b_terra_handoff.md)。

## P2A：同一车体与地图快照比较

显式 colcon 构建启用 `-DBUILD_TESTING=ON -DRM_TDT_BUILD_BENCHMARK=ON`
（上述包装脚本已启用）会构建 `benchmark_planners`。除基础依赖外，需要 Humble 的
`nav2_map_server`、`nav2_navfn_planner`、`nav2_smac_planner`；没有新增求解器依赖。
默认 OFF；测试程序不激活 costmap、发送 action 或连接机器人。

以下是已进入并 source 相应 install 的隔离容器内可用的命令（`/work` 映射至
上述持久目录，地图文件需另行只读挂载）。P2B 无需重跑历史 P2A 基准：

```bash
/work/build/rm_tdt_planner/benchmark_planners /work/new-p2a.csv 100 /maps/frozen/map.yaml
python3 /ws/experiments/tdt_planner/rm_tdt_planner/tools/summarize_benchmark.py \
  /work/new-p2a.csv --output /work/new-p2a-summary.json
```

省略最后一个 YAML 时运行 7 种合成快照；提供时增加现场地图。YAML 必须通过
map_server 加载，支持其图像/阈值/翻转语义，不支持旋转 origin。输出路径须是新的
普通文件路径。工具不会改变地图 bundle 的 candidate/approved 状态。

每种场景默认示例为 100 组确定性请求，四种方案为 Navfn Dijkstra、Smac2D（含该
Humble 版本自带平滑）、T-DT A*、T-DT A*+QP。共享旧车配置中的 0.64×0.54 m
矩形、0.02 m padding、外接圆和 0.02 m clearance，输入为相同二值膨胀地图。
`prepare_grid()` 产生不可变 snapshot；`plan_prepared()` 不重复膨胀，且拒绝与
snapshot 不同的 radius/clearance。运行插件始终调用原 `plan()` 完成自身膨胀，
没有可关闭 footprint 检查的 ROS 参数。

独立的线段到障碍方格距离检查整个折线扫掠圆。`physical_clearance_m` 是扣除
外接圆与 clearance 后的距离，上限截为 0.25 m；它不是实车测得间隙。
`common_geometry_ok` 记录更严格的膨胀栅格 supercover 检查，失败不能直接算作
物理碰撞。`accepted` 要求返回、扫掠圆无侵入、端点误差不超过一个栅格、建模耗时
不超过 250 ms。无解场景返回空是预期正确行为。

CSV 分列地图预处理与规划耗时，二者相加只用于比较模型；不是完整插件回调或
Nav2 action 延迟。`rss_max_kb` 是整个进程累计高水位，不能按行归给单个算法。
汇总器检查四方案请求配对、堵路/撤障配对、完整场景与试次；仅在双方都接受的请求
上比较长度、转折量和耗时，避免不同成功子集造成偏差。离线检查失败仍保存报告并
非零退出。结果与进一步限制见
[`validation_20260913.md`](../../../docs/tdt_migration/validation_20260913.md)。

按用户 2026-09-13 补充约束：当前设备资源或耗时限制只记录，不据此淘汰方案，
不开展针对当前设备的性能优化；后续在更高性能设备上验证完整导航负载。
运行时已有迟到拒收继续有效，不能把统计工具的性能项失败当成算法路线否决。

## P2B：静态仿真交接

`launch/simulation_comparison.launch.py` 复用现有 Gazebo base launch 和两条通道墙。
仿真模型/MPPI/costmap 不变，`make_sim_profiles.py` 仅替换 `GridBased`；使用该仿真
原有 0.60×0.50 m footprint、0.03 m padding，与 P2A 旧车模型不同。

`p2b_validation.sh` 提供 `deps/build/check/sanitizers/profiles/run/summarize`，每次 `run`
创建新隔离容器，自动结束自身 launch 进程组。原始轨迹、路径、命令、costmap、事件
和 action 结果写在 `build/tdt_p2b/runs/`；目录已存在时拒绝覆盖。日志留在 `/home`。
当前有 30 个 core、4 个 plugin、14 个纯 Python 工具和 4 个真实 ROS 消息测试，
共 52 项。`sanitizers` 在独立目录统一插桩构建固定求解器与 core，审计实际宏/库，
先运行求解器跨库生命周期用例，再运行包含它的 30 项核心测试。
默认新系列 `static_v5`；保留 static_v1/v2/v3/v4，单组失败后停止该组重复。
构建链命令仍见 [构建链交接](../../../docs/tdt_migration/p2b_sanitizer_chain_handoff.md)，
其中 static_v4 的次数与执行角色仅代表当时交接。当前结果以本轮修复记录为准。

几何记录器验证采样矩形及线性位姿插值下的间隙，无法独立证明采样间真实接触状态。
汇总回读原始事件和数据，不把 SUCCEEDED、CPU 告警或缺少日志直接当作方案验收。
没有观察到并发 snapshot 拒收时，该项保留未覆盖；移动障碍和实车仍是后续阶段。

逐步执行、失败分类、结果回传见
[`p2b_terra_handoff.md`](../../../docs/tdt_migration/p2b_terra_handoff.md)；当前状态见
[`p2b_work_status.md`](../../../docs/tdt_migration/p2b_work_status.md)。

## 生成候选配置

生成完整候选配置（不覆盖源配置，不自动启动）：

```bash
python3 "$pkg/tools/make_profile.py" \
  "$candidate_ws/src/rm_nav_config/config/nav2_old_car_2026_dual_stvl.yaml" \
  "$work/nav2_tdt_candidate.yaml"
# 另一个新文件加 --frontend-only 可比较 A* 与 A*+QP。
```

片段 `config/planner_parameters.yaml` 不是完整 Nav2 profile。保留插件 ID
`GridBased` 是为了兼容现有 BT；当前旧车 BT 不调用 `SmoothPath`，故后端必须在
规划插件内部运行，仅注册 smoother 并不会生效。该生成器不复制主工作区未提交参数；
如需那些参数，显式以主工作区文件为 baseline，并在仿真中比较。

## 几何与失败语义

- 仅接受 axis-aligned Nav2 costmap：行主序、y 正向、保留 resolution/origin。
  不能直接传 PGM 顶行顺序或带旋转的 YAML origin；应先由 map_server/costmap 转换。
- 默认离线 `Grid` 沿用 `ObstacleSeeds`：253/254/255 都作为障碍种子。Nav2 插件
  显式标记 `Nav2Master`：只从 254/255/边界种子膨胀，253 另保留中心禁区，避免
  内切区重复膨胀。没有关闭安全检查的 ROS 参数；未知区仍不通行。
  其他软 cost 仍不保留，本候选不能据此替换语义路线/特殊通道执行。
- 内部搜索/优化使用 **padded footprint 外接圆 + clearance**，距离变换再预留一个栅格对角线。
  允许任意自转时仍保守，修复不保证外接圆模型可通过真实矩形能通过的窄通道。
  狗洞、定向穿越和坡道不属于本轮适用场景，不得为了通过而缩小真实 footprint。
- Nav2 输入的精确端点按原始 254/255/边界闭方格检查整个圆，253 保留中心禁区。
  从端点连接到距离不超过 3 格的最近可连接自由格中心；每条短连接都检查连续扫掠圆。
  A*/QP 仅搜索/优化两锚点之间的保守内部路径，随后接回精确端点并验证全部输出段。
  没有连接时拒绝，不移动目标，不缩小半径，也不把碰撞目标投影到附近自由区。
  最近锚点策略不保证找到所有可行连接；离线 ObstacleSeeds 保留原整格拒收策略。
  PreparedGrid 同时保留不可变原始输入和膨胀掩码，raw/prepared 使用相同端点规则。
- 开启上游严格碰撞模式，内部折线逐段做 supercover 检查，Nav2 最终输出再做连续圆检查。无解、非法坐标、
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

### Exact endpoint rejection evidence

Nav2 endpoint failures include `endpoint_witness_v1` JSON from the same immutable
raw input and the original collision predicate. Each blocked endpoint supplies
one sufficient raw cell, distance and threshold; this is not a nearest-obstacle
query. `tools/audit_endpoint_witness.py EVENTS --output NEW_JSON` independently
checks the recorded geometry and plugin request. Published OccupancyGrid frames
are not substituted for the exact input. See
[diagnostic scope](../../../docs/tdt_migration/p2b_endpoint_witness.md).
