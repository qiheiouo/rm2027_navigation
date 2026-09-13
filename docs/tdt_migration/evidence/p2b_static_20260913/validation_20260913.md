# P2B 静态仿真验证回传

状态：**中断：共享记录器错误阻塞全部静态仿真；未形成任何规划器的有效静态验证结果。** 本文只记录实际执行结果，不将预期写为实际结果。

## 版本与环境

- 交接提交 / 实际 HEAD / 分支：`b1c40e6` / `b1c40e68f08771cec763d6bc30ff5fa7072365a8` / `experiment/tdt-planner-phase2`（相对 `origin/experiment/tdt-planner-phase2` 领先 1）。开发基线为 `ca608e5`。
- 验证日期、操作者/模型：2026-09-13（Asia/Shanghai）；Codex（GPT-5）。
- git status；是否存在源码偏离：执行前 `git status --short --branch` 仅输出分支状态，无工作树改动。试验后相关源码范围 `experiments/tdt_planner/rm_tdt_planner`、`src/rm_simulation`、`src/rm_nav_config`、`src/rm_description`、`src/rm_localization_adapters`、`src/rm_chassis_interface`、`src/rm_competition_interfaces` 的 porcelain 输出为空；本轮未修改源码、配置、测试断言或安全参数。
- 镜像 ID、CPU、可用内存：`rm2027_navigation:humble`，`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；Intel Core i7-10710U（6C/12T）；执行前 `free -h` 的可用内存为 11 GiB（总计 15 GiB，swap 2.0 GiB 且未用）。
- P2B_SERIES / 持久运行目录：`static_v1`；`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v1`。
- profile、场景及 aggregate 路径/SHA：四 profile 位于 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/profiles_p2b/`；SHA256 分别为 navfn `1ff9c0e9f9b92536a6228143a1c28c1f143f376821da945efb51d002f61f8a5e`、smac2d `6e6351f792b17307f383b8178aa42a785af831098ee4f51756a90b1cc0f0f868`、tdt_astar `593417c67ce5045188cf2635e244c213d99ea256d91a3a9be4e4daddc1dd2369`、tdt_qp `5718ec4045ba56634208a13d7f2bcee0b768ae40ab2c4110a4a01a66af4b4e67`。试次元数据记录的 fixture SHA256：`phase1_omni.sdf` `8c37e8003fd55f6418c8cb9a407a5c5198adfc5b561b50f0bfd88ac918dc97b3`、`course_wall.sdf` `8dd4d62b879626a206936f939e5212609da89a0f483ed0a394bb7df20f2c9c81`、基线 Nav2 配置 `0e2020c8eb5001e3f57c3626353a99fdad785cb6a716abd9d6fd89ae88628caf`、启动文件 `1e1d66725ce8951eb291cbcaf438113bb8c392a6a9630982f9a996ac25363acf`。部分汇总为 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v1/aggregate_20260913T063522Z.json`，SHA256 `6b6c082f9037d1c56960b18f0c89513ceefea71eb549b40031cda6df7b6ddf3e`。

## 实际执行记录

| 步骤 | 命令 / 开始结束时间 | 退出码 | 实际结果 / 日志绝对路径 |
| --- | --- | --- | --- |
| 依赖 | 未执行；执行前核对已有 `deps_source`、`deps`、`deps-build`，未发现需重新取回固定依赖的证据。 | — | 不重复下载；现有路径 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/{deps_source,deps,deps-build}`。 |
| 构建 | `bash "$pkg/tools/p2b_validation.sh" build`；14:33:56–14:34:03 +08:00 | 0 | 6 个仿真包与 `rm_tdt_planner` 成功构建。`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/build_20260913T063356Z.txt` |
| CTest：19 + 3 + 13 | `bash "$pkg/tools/p2b_validation.sh" check`；14:34:15–14:34:18 +08:00 | 0 | `planner_safety` 19/19、`nav2_plugin_contract` 3/3、`simulation_evidence_tools` 13/13；总计 35/35。`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/tests_20260913T063415Z.txt` |
| 四配置核对 | `bash "$pkg/tools/p2b_validation.sh" profiles`；14:34:27–14:34:28 +08:00 | 0 | 生成四 profile，后续首例再次输出 “Four frozen profiles match the baseline and generator”。`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/profiles_20260913T063427Z.txt` |
| Navfn 首例 | `P2B_SERIES=static_v1 bash "$pkg/tools/p2b_validation.sh" run navfn 1`；14:34:42–14:34:52 +08:00 | 2 | 在首个 costmap 回调时记录器崩溃；没有目标、轨迹、命令、costmap 或事件证据。`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/navfn_1_20260913T063442Z.txt`；试次目录见下。 |
| Smac2D / T-DT A* / T-DT A*+QP 首例及重复 | 未执行 | — | Navfn 暴露共享记录器错误；按交接的共用启动、依赖或记录器错误停止全部仿真条件，不启动其余组。 |
| 汇总 | `P2B_SERIES=static_v1 bash "$pkg/tools/p2b_validation.sh" summarize`；14:35:22 +08:00 | 1 | 写入部分汇总；工具明确输出 `complete=False, audited=False`，退出 1 反映不完整矩阵。`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/summarize_20260913T063522Z.txt` |

## 静态矩阵

| 方案 | 已运行编号 | Nav2 成功数 | 有效证据数 | 静态检查通过数 | 首个失败 / 未执行原因 |
| --- | --- | --- | --- | --- | --- |
| Navfn | 1（退出 2） | 0 | 0 | 0 | 记录器在目标发送前因 costmap 数据类型错误退出；不是 Nav2 或规划器结果。 |
| Smac2D | 无 | 0 | 0 | 0 | 共享记录器错误，按停止条件未启动。 |
| T-DT A* | 无 | 0 | 0 | 0 | 共享记录器错误，按停止条件未启动。 |
| T-DT A*+QP | 无 | 0 | 0 | 0 | 共享记录器错误，按停止条件未启动。 |

- 每次 metadata/summary/trajectory/plans/commands/events/costmap 路径见 aggregate：唯一试次为 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v1/navfn_1/`。其 `metadata.json`、`observation/summary.json` 和原始日志已保留；`trajectory.jsonl`、`plans.jsonl`、`commands.jsonl`、`events.jsonl`、`costmap.jsonl` 均为 0 字节，不能作为通过证据。
- fixture spawn 与实际 costmap 标记核对：`launch.log` 有 `spawn_course_wall_north exit=0` 和 `spawn_course_wall_south exit=0` 两条成功标记；记录器在首个 costmap 回调失败，未写出 costmap，故墙体实际感知标记**未覆盖**。
- canonical TF/恒等 map-odom 前提核对：`launch.log` 显示 `map_odom_stub` 已启动并声明动态 identity `map -> odom`；因记录器失败，未取得 `tf2_echo` 或完整运行时 TF 证据，故此前提**未完成验证**。
- 间隙、到点/yaw、停止命令、恢复结果：**未覆盖**；无 action、轨迹、命令或事件记录。
- 轨迹图与 global-plan revision；cross-track 的解释：**未覆盖**；无 trajectory/plans，未生成 SVG。
- snapshot 并发更新拒收：**未覆盖**；不得记为 0 次已验证。
- 完整预检查 action 延迟（含往返），非纯 core 时间：**未覆盖**；目标尚未发送。

## 问题与证据

1. 分类：代码（共享记录器）。最小复现命令：`cd /home/wpie/worktrees/rm2027_tdt_phase2 && export P2B_SERIES=static_v1 && bash experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh run navfn 1`。首次有效错误为 `'>=' not supported between instances of 'int' and 'bytes'`，写入 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v1/navfn_1/observer.log` 与 `observation/summary.json`；对应回调在 `experiments/tdt_planner/rm_tdt_planner/tools/observe_simulation.py:81`：`sum(c >= 99 for c in msg.data)`。试次 `docker_exit.txt` 为 `2`。已停止全部静态组：该错误发生在规划器无关的共享 observer、首个 costmap 到达时，且后续所有组都会使用同一入口和记录器；没有依据把它归因 Navfn 或任一算法。
2. 分类：输入或记录（受问题 1 影响，非独立根因）。原始启动日志 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v1/navfn_1/launch.log` 证明两个 fixture spawn 均 exit=0；但所有五个记录流为 0 字节。停止该组和全部后续组，原因是无原始测量数据时不能评估动作成功、几何、安全或恢复。没有仅以运行耗时或退出码推断原因。

## 验证结论与边界

- 已实现但未运行：Smac2D、T-DT A*、T-DT A*+QP 的首例和所有 2–5 次重复；Navfn 2–5；任何可用 action/轨迹/安全检查；人工 costmap、TF 与 SVG 审查。
- 实际通过的检查：构建通过；CTest 35/35；四 profile 生成及一致性核对通过；Navfn 首例的两个 fixture spawn 成功标记存在。上述都不等于静态导航验证通过。
- 失败、未知、缺失与待设计者复核：共享 observer 的 costmap 元素类型处理错误；全部规划器的 P2B 静态导航结论、几何/终点/停命令/恢复、TF 与 wall 感知、并发 snapshot 拒收和 action 延迟均为未知或未覆盖。修复必须由设计者审阅后进行；修复后需使用新 series，保留本次 `static_v1` 失败证据。
- 当前设备资源观测及待新设备验证：本次构建和测试在 12 逻辑 CPU、11 GiB 可用内存下完成；首例在记录器异常后很快终止，没有形成可归因的性能样本，也没有观察到 OOM 或设备资源耗尽。不得以此淘汰任何方案；完整静态和新设备验证仍待进行。
- 本轮未执行：移动障碍、LIO/双 STVL/MPPI 全负载、实车验收。
- 是否修改任何源码/配置/断言：没有。本报告和 `build/tdt_p2b/` 中的日志/证据是唯一新增验证产物；未 push/merge。
- 不宣布部署验收；等待设计者复核共享记录器修复与新的、不可覆盖的 P2B series 验证。
