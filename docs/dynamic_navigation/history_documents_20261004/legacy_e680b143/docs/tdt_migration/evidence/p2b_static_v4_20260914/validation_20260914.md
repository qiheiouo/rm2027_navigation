# P2B static_v4 静态仿真验证回传

状态：**部分完成，等待设计者复核；不构成部署或实车验收。** 本报告只记录
`static_v4` 的实际执行结果，不混入 static_v1/v2/v3 试次。

## 版本与环境

- 交接提交 / 实际 HEAD / 分支：`78648e2d2615c82730bdd4b0509991d9a92fa1d7` /
  同上 / `experiment/tdt-planner-phase2`。
- 验证日期、操作者/模型：2026-09-14（Asia/Shanghai）；Codex。
- git status；是否存在源码偏离：开始时相关源码无未提交偏离；本报告是本轮唯一新增
  工作树文件。未修改源码、配置、断言、目标、footprint、超时或安全门。
- 镜像 ID、CPU、可用内存：`rm2027_navigation:humble`，
  `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；
  12 CPU；汇总时主机内存 15 GiB、available 11 GiB（瞬时观测）。Navfn 首例容器
  资源快照为 712 MiB / 15.33 GiB、CPU 534.33%、318 PIDs，见其 `resources.jsonl`。
- P2B_SERIES / 持久运行目录：`static_v4` /
  `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v4`。
- 本轮日志：
  `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/sanitizer_chain_20260914T004818Z.7KKzlK`。
- 旧证据保护：开始及结束均校验
  `docs/tdt_migration/evidence/sanitizer_chain_20260914/preserved_inputs.json`，238 项
  保持匹配；未覆盖旧 series。
- profile SHA256：Navfn `1ff9c0e9f9b92536a6228143a1c28c1f143f376821da945efb51d002f61f8a5e`；
  Smac2D `6e6351f792b17307f383b8178aa42a785af831098ee4f51756a90b1cc0f0f868`；
  T-DT A* `593417c67ce5045188cf2635e244c213d99ea256d91a3a9be4e4daddc1dd2369`；
  T-DT A*+QP `5718ec4045ba56634208a13d7f2bcee0b768ae40ab2c4110a4a01a66af4b4e67`。
  `profiles` 实测与既有基线一致。

## 实际执行记录

| 步骤 | 命令 / 开始结束时间 | 退出码 | 实际结果 / 日志绝对路径 |
| --- | --- | --- | --- |
| 保护清单 | `python3` SHA256 核对；开始、结束各一次 | 0 / 0 | `Preserved inputs: 238`；本报告目录的父证据清单 |
| 构建 | `p2b_validation.sh build`；08:48:18–08:48:31 +08:00 | 0 | 6 个仿真包及实验包构建成功；`handoff_logs/.../build.txt` |
| CTest | `p2b_validation.sh check`；08:48:42–08:48:45 +08:00 | 0 | 4 入口、46/46：core 25、plugin 4、工具 13、ROS 消息 4；`tests.txt` |
| 独立 sanitizer 链 | `p2b_validation.sh sanitizers`；08:48:57–08:50:07 +08:00 | 0 | 新链目录见下；链审计、边界 1 项、core 25 项均通过，无 ASan/UBSan/LSan 报告；`sanitizers.txt` |
| 四配置核对 | `p2b_validation.sh profiles`；08:50:29–08:50:30 +08:00 | 0 | Frozen profiles match baseline；`profiles.txt` |
| Navfn 首例 | `p2b_validation.sh run navfn 1`；08:50:42–08:51:18 +08:00 | 1 | action 成功但发生 1 次恢复，静态门失败；`navfn_1.txt` |
| Smac2D 1–5 | 五次单独 `run smac2d N`；08:51:48–08:58:15 +08:00 | 0 × 5 | 5/5 有效且静态门通过；`smac2d_[1-5].txt` |
| T-DT A* 首例 | `run tdt_astar 1`；08:52:38–08:53:23 +08:00 | 1 | preflight 成功；导航 action status 6、16 次恢复；`tdt_astar_1.txt` |
| T-DT A*+QP 首例 | `run tdt_qp 1`；08:53:47–08:54:31 +08:00 | 1 | preflight 成功；导航 action status 6、16 次恢复；`tdt_qp_1.txt` |
| 汇总 | `p2b_validation.sh summarize`；21:30:46–21:30:47 +08:00 | 1 | 8 条记录可审计，20 次矩阵不完整；见下列 aggregate 与 `summarize.txt` |

`simulation_ros_messages` 实际显示 `Ran 4 tests`；普通 CTest 显示四入口全部通过。独立
sanitizer 链为
`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/sanitizer_runs/chain_20260914T004857Z.m4SWOu`：
`chain_audit.json` 的 `passed=true`、`eigen_allocation_consistent=true`；两份求解器均从
该新前缀加载并各有 ASan/UBSan 符号（`libOsqpEigen.so.0.8.1`
`3fdfaa8d551173a696aeb7606abcd626e065914444d944921db0abcdeded513d`，`libosqp.so`
`f730eb269029d57160387d96b144cc4d17fa40538b3d72fea5acf6025811be2a`）。
`solver_boundary.log` 的 1 项与 `core_tests.log` 的 25/25 均通过。这验证新独立
sanitizer 构建链，不等同于 ROS 全负载或实车验收。

## 静态矩阵

| 方案 | 已运行编号 | Nav2 成功数 | 有效证据数 | 静态检查通过数 | 首个失败 / 未执行原因 |
| --- | --- | ---: | ---: | ---: | --- |
| Navfn | 1 | 1 | 1 | 0 | action status 4，但 `no_recovery=false`（恢复 1 次）；按门不补 2–5 |
| Smac2D | 1–5 | 5 | 5 | 5 | 无 |
| T-DT A* | 1 | 0 | 1 | 0 | action status 6、恢复 16；按门不补 2–5 |
| T-DT A*+QP | 1 | 0 | 1 | 0 | action status 6、恢复 16；按门不补 2–5 |

- aggregate：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v4/aggregate_20260914T133046Z.json`，SHA256
  `b54d191aa68ab38c23035de1123344712b031a652b8598710f9353ae909fe072`。其
  `consistent_and_audited=true`，`complete_20_trial_matrix=false`，12 个缺失试次为
  Navfn、T-DT A*、T-DT A*+QP 各 2–5；`all_recorded_static_checks_pass=false`。
- 每次 metadata/summary/trajectory/plans/commands/events/costmap 的路径与 SHA 均在
  aggregate 的 `trials[].artifact_sha256`；8 个已记录试次都匹配原始 summary。
- fixture spawn 与 costmap：8 个 summary 都为 `fixture_spawn_verified=true`。Smac2D 首例
  末帧 costmap 为 `map`、240×240、约 0.05 m；中心块及南北墙在预期位置分别有
  154/154、200/200、200/200 个 lethal 标记。SVG/PNG 复核
  `smac2d_1/trajectory_review.svg` 与 `.png`：采样 padded footprint 未接触黑色 fixture。
- canonical TF：Navfn 首例保存 `node_list.txt` 和 `map_odom_tf.txt`；等待初始 frame
  出现后记录到恒等 `map -> odom`。其余试次沿同一 launch/profile，未额外采集 TF。
- 到点、yaw、间隙与停止：所有 action 成功的 Navfn/Smac2D 试次均满足位置 ≤0.15 m、
  yaw ≤0.20 rad、body clearance ≥0.05 m、padded footprint 无接触及 fresh stop。Navfn
  的实际 xy/yaw 误差为 0.07197 m / 0.16518 rad，body/padded 最小间隙为
  0.14731 m / 0.10727 m；但恢复次数为 1。Smac2D 五次无恢复，最小 body/padded
  间隙范围为 0.11025–0.14998 m / 0.06783–0.10763 m。
- 轨迹、global plan revision、cross-track：Smac2D 首例产生 386 个轨迹样本、18 条 plan，
  cross-track RMS 0.03917 m；五次通过样本均值 0.03657 m。该指标描述相对全局计划的
  跟踪偏差，不替代间隙或终点检查。Navfn 首例为 454 样本、23 条 plan、RMS 0.03406 m。
- snapshot 并发更新拒收：Navfn/Smac2D/T-DT A* 为未观察到；T-DT A*+QP 首例观察到 1 次
  `costmap or footprint changed during TDT planning`（事件 t=13.439 s）。端点拒收须单独
  统计，不能混入该数。
- 完整 preflight action 延迟（含往返，非纯 core）：Navfn 33.338 ms；Smac2D 五次均值
  18.498 ms；T-DT A* 32.709 ms；T-DT A*+QP 44.500 ms。两组 T-DT 的 preflight 均为
  status 4，因此后续失败发生在导航阶段。

## 问题与证据

1. **Navfn：运行行为/未知。** 最小复现：`p2b_validation.sh run navfn 1`。首次有效结果
   为 action status 4、所有几何/终点/停止项通过但 `recoveries=1`；因此退出码 1 是
   静态门失败，不能据此判 Navfn action 失败或推断恢复根因。原始证据：
   `runs/static_v4/navfn_1/observation/{summary.json,events.jsonl,trajectory.jsonl,commands.jsonl,costmap.jsonl}`
   和 `observer.log`。停止该组重复；原因是 `no_recovery` 门明确不通过。

2. **T-DT A*：功能/配置语义，原因仍待设计者复核。** 最小复现：
   `p2b_validation.sh run tdt_astar 1`。最早有效导航错误为 t=16.829 s：
   `start=free@cost=0, goal=blocked@cost=162 [input=nav2_master]`；随后共有 7 次端点拒收，
   action status 6、恢复 16。其 preflight status 4，故不能沿用 static_v3 的 preflight
   失败归因。原始路径同上替换 `tdt_astar_1`；端点分析为
   `handoff_logs/.../tdt_astar_endpoints.json`，SHA256
   `d08c6cf4697e572e15637f4500701d942c76d52add13ebc86f50e6b4d3ca954f`。停止该组重复；
   此分析使用发布 costmap 近似回调，不可断言精确 snapshot 时序或完整根因。

3. **T-DT A*+QP：功能/配置语义，原因仍待设计者复核。** 最小复现：
   `p2b_validation.sh run tdt_qp 1`。最早的 snapshot 拒收为 t=13.439 s；第一条端点
   拒收 t=15.404 s 为 `start=free@cost=120, goal=blocked@cost=175 [input=nav2_master]`，
   之后端点拒收 14 次，action status 6、恢复 16。两类事件分开计数。原始路径同上替换
   `tdt_qp_1`；端点分析为 `handoff_logs/.../tdt_qp_endpoints.json`，SHA256
   `c3b83164eff956d70b8f7ed4260de8fbe7019b2ab22039eb2ff0fa76aac39bf5`。停止该组重复；
   发布地图分析不证明精确锁内 snapshot 或根因。

当前 12 CPU/11 GiB available 观测未显示为本轮失败原因；没有据此淘汰任何方案，也未
调整性能参数。sanitizer 通过说明先前混合分配边界失败在新构建链下未复现，但不排除
其他运行期问题。

## 验证结论与边界

- 已实现但未运行：Navfn 2–5、T-DT A* 2–5、T-DT A*+QP 2–5 均因首例静态门失败停止；
  未执行移动障碍、LIO/双 STVL/MPPI 全负载或实车验收。
- 实际通过：构建；46/46 常规用例；独立 sanitizer 链审计、边界 1 项与 core 25/25；
  四 profile 核对；Smac2D static_v4 的 5/5 有效静态门。
- 实际失败/未知：Navfn 首例仅因恢复 1 次而未过有限静态门；两组 T-DT 的首例均在
  preflight 成功后导航失败。端点与 snapshot 事件已经归档，但根因、修复方案及四组
  完整 20 次矩阵仍待设计者复核。
- 当前设备资源观测仅作记录；需新设备验证的性能结论为未知。
- 没有修改任何源码/配置/断言；未 push、未 merge；不宣布部署验收。

