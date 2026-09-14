# P2B static_v3 代价语义修复验证回传

状态：**中断：常规构建、45 项 CTest 和四 profile 核对通过；核心 sanitizer 在首个测试稳定触发 ASan heap-buffer-overflow，因此未启动任何 static_v3 仿真试次。** 本报告仅记录实测结果，不将 static_v2 试次混入 static_v3，也不宣布部署验收。

## 版本与环境

- 修复交接提交 / 实际 HEAD / 分支：`74f68e2` / `74f68e2d6e6957dfbdc64542165acbbb6f740f41` / `experiment/tdt-planner-phase2`（相对 origin 领先 3）。验证前相关源码工作树干净。
- 验证日期、操作者/模型：2026-09-14（Asia/Shanghai）；Codex（GPT-5）。
- 旧证据：按 `docs/tdt_migration/evidence/costmap_semantics_20260913/preserved_static_v2.json` 核对，`static_v2` 的 166 个文件 SHA256 全部保持；未覆盖或改写 `static_v1/static_v2`。
- 镜像/设备：`rm2027_navigation:humble`，`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；12 逻辑 CPU；执行后可用内存 11 GiB（总 15 GiB，swap 未用）。
- P2B_SERIES / 持久目录：`static_v3`；`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v3/`。
- profile SHA256 与 static_v2 一致：navfn `1ff9c0e9f9b92536a6228143a1c28c1f143f376821da945efb51d002f61f8a5e`；smac2d `6e6351f792b17307f383b8178aa42a785af831098ee4f51756a90b1cc0f0f868`；tdt_astar `593417c67ce5045188cf2635e244c213d99ea256d91a3a9be4e4daddc1dd2369`；tdt_qp `5718ec4045ba56634208a13d7f2bcee0b768ae40ab2c4110a4a01a66af4b4e67`。
- aggregate：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v3/aggregate_20260914T000005Z.json`；SHA256 `1101bb3bfdeea9ec86162973ec990962eee8f82b56ff3c1c30d8111605cb6b47`；无试次，`complete_20_trial_matrix=false`、`consistent_and_audited=false`、`accepted_for_deployment=false`。

## 实际执行记录

公共日志目录：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/costmap_semantics_20260914.r3RIlJ/`。

| 步骤 | 命令 / 时间 | 退出码 | 实际结果 / 日志 |
| --- | --- | --- | --- |
| 冻结证据核对 | 交接内嵌 Python SHA 核对 | 0 | 输出 `Original static_v2 evidence preserved: 166`。 |
| 增量构建 | `p2b_validation.sh build`；07:57:07–07:57:36 | 0 | 6 个仿真包与 `rm_tdt_planner` 成功。`build.txt` |
| CTest | `p2b_validation.sh check`；07:57:49–07:57:52 | 0 | 4 入口、45/45：core 24、plugin 4、工具 13、ROS 消息 4。`tests.txt` |
| sanitizer 初次 | `p2b_validation.sh sanitizers`；07:58:03–07:58:53 | 8 | `planner_safety` 的第一个 GTest 即触发 ASan；未达到 24 项。`sanitizers.txt` |
| sanitizer 重试 | 同一命令；07:59:17–07:59:18 | 8 | 对同一 `Planner.OpenMapActuallyRunsCorridorBackend`、同一地址与调用链稳定复现。`sanitizers_retry.txt` |
| 四配置核对 | `p2b_validation.sh profiles`；07:59:40 | 0 | 输出 `Four frozen profiles match the baseline and generator`；哈希见 `profile_sha256.txt`。 |
| 四组首例及重复 | 未执行 | — | sanitizer 安全失败阻止核心改动进入 static_v3 仿真；没有创建 planner trial 目录。 |
| 汇总 | `P2B_SERIES=static_v3 p2b_validation.sh summarize`；08:00:05 | 1 | 保存无试次的中断矩阵；退出 1 表示没有完成矩阵，不是规划器结果。`summarize.txt` |

## 静态矩阵

| 方案 | 已运行编号 | Nav2 成功数 | 有效证据数 | 静态检查通过数 | 首个失败 / 未执行原因 |
| --- | --- | --- | --- | --- | --- |
| Navfn | 无 | 0 | 0 | 0 | sanitizer 失败后未启动。 |
| Smac2D | 无 | 0 | 0 | 0 | sanitizer 失败后未启动。 |
| T-DT A* | 无 | 0 | 0 | 0 | sanitizer 失败后未启动。 |
| T-DT A*+QP | 无 | 0 | 0 | 0 | sanitizer 失败后未启动。 |

- 因未启动容器，fixture spawn、实际 costmap 标记、canonical TF/map-odom、预检查与导航 action、轨迹/计划/命令、间隙、停止命令、恢复、SVG、snapshot 并发更新和性能样本均**未覆盖**。
- static_v2 的 Navfn/Smac2D 5/5 与两组 T-DT 首例结果仅为受冻结清单保护的历史证据，不是本提交的 static_v3 结果。

## 问题与证据

1. 分类：代码或已安装依赖的 sanitizer 兼容性，待设计者定位。最小复现：`cd /home/wpie/worktrees/rm2027_tdt_phase2 && bash experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh sanitizers`。两次执行均在 `Planner.OpenMapActuallyRunsCorridorBackend` 的优化器路径中中止，ASan 首个有效错误为 `heap-buffer-overflow`（`Eigen::internal::handmade_aligned_free`，读取 8 字节）。销毁调用栈为 `OsqpEigen::Solver::~Solver`（`/work/deps/include/OsqpEigen/Solver.hpp:32`）→ `MinimumSnap::osqpExecute`（vendor `minimumSnap.cpp:903`）；分配调用栈为未重新插桩的 `/work/deps/lib/libOsqpEigen.so.0.8.1` 的 `OsqpEigen::Solver::getSolution()`。完整原始日志：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/costmap_semantics_20260914.r3RIlJ/sanitizers.txt`（首次 ASan 自第 51 行）及 `sanitizers_retry.txt`（自第 33 行）。
2. 此错误发生在 core/vendor 已插桩、求解器安装前缀未插桩的组合中；正常 Debug/RelWithDebInfo 的 24 core GTest 和总 45 CTest 通过不能证明 sanitizer 报告为误报。未凭该栈将根因确定为本次 Nav2 代价语义修改、vendor 或预编译 OsqpEigen 任一方；也没有修改链接、禁用 ASan、放宽 sanitizer、跳过首个测试或调整算法来取得通过。

## 验证结论与边界

- 实际通过：static_v2 冻结证据 166/166；增量构建；常规 CTest 45/45；profile 生成/一致性与四个 SHA256。
- 实际失败：sanitizer 入口退出 8，24 项核心安全测试未完成，且第二次稳定复现相同 ASan 报告。
- 未知/未执行：该 ASan 的根因归属；所有 static_v3 Nav2、T-DT、几何与安全运行时结论；移动障碍、LIO/双 STVL/MPPI 全负载与实车。
- 本轮未修改算法、业务代码、测试断言、目标、footprint、padding、clearance、超时、恢复或安全门；未 push/merge。本报告与 `build/tdt_p2b/` 内的日志、空 series aggregate 是新增验证产物。
- 不宣布部署验收；等待设计者处理或界定 sanitizer 失败后，使用新的不可覆盖 series 重新验证。
