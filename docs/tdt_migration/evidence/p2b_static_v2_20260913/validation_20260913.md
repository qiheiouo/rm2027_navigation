# P2B static_v2 静态仿真验证回传

状态：**已完成 static_v2 的修复回归与静态矩阵验证；Navfn、Smac2D 各 5/5 通过有限静态门，T-DT A* 与 T-DT A*+QP 的首例均有有效失败证据。** 20 试次完整矩阵未完成：两个失败组按交接停止条件未运行第 2–5 次。本报告只列实际结果，不宣布部署验收。

## 版本与环境

- 修复交接提交 / 实际 HEAD / 分支：`e85b076` / `e85b076760c1367385b4f92a84ee24b14985c3c0` / `experiment/tdt-planner-phase2`（相对 origin 领先 2）。验证前相关源码工作树干净。
- 验证日期、操作者/模型：2026-09-13（Asia/Shanghai）；Codex（GPT-5）。
- 旧证据：`docs/tdt_migration/evidence/recorder_fix_20260913/preserved_static_v1.json` 的所有文件字节数与 SHA256 均核对通过；未覆盖或修改 `static_v1`。
- 镜像/设备：`rm2027_navigation:humble`，`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；Intel Core i7-10710U（6C/12T）；首次执行前可用内存 11 GiB（总 15 GiB，swap 未使用）。
- P2B_SERIES / 持久目录：`static_v2`；`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v2/`。
- 四 profile SHA256 与 static_v1 相同：navfn `1ff9c0e9f9b92536a6228143a1c28c1f143f376821da945efb51d002f61f8a5e`；smac2d `6e6351f792b17307f383b8178aa42a785af831098ee4f51756a90b1cc0f0f868`；tdt_astar `593417c67ce5045188cf2635e244c213d99ea256d91a3a9be4e4daddc1dd2369`；tdt_qp `5718ec4045ba56634208a13d7f2bcee0b768ae40ab2c4110a4a01a66af4b4e67`。
- aggregate：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v2/aggregate_20260913T112411Z.json`；SHA256 `953a7ee91bc454ddf839d8c44709b0a7b46c2b6745c5578ba82f5fd2432b16f5`；`consistent_and_audited=true`，`complete_20_trial_matrix=false`。

## 实际执行记录

公共验证日志目录：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/handoff_logs/recorder_fix_20260913.VQb2rj/`。

| 步骤 | 命令 / 时间 | 退出码 | 实际结果 / 日志 |
| --- | --- | --- | --- |
| 旧证据核对 | 交接内嵌 Python SHA 核对 | 0 | `static_v1` 与历史回传报告保留。 |
| 增量构建 | `p2b_validation.sh build`；14:56:28–14:56:34 | 0 | 6 个仿真包和 `rm_tdt_planner` 成功。`build.txt` |
| CTest | `p2b_validation.sh check`；14:56:45–14:56:47 | 0 | 4 入口、39/39：core 19、plugin 3、工具 13、`simulation_ros_messages` 4。`tests.txt` |
| 四配置核对 | `p2b_validation.sh profiles`；14:57:00 | 0 | 输出 “Four frozen profiles match the baseline and generator”。`profiles.txt`、`profile_sha256.txt` |
| Navfn 1–5 | `run navfn N`；14:57:14–15:05:29 | 全部 0 | 5/5 有效且有限静态门通过。`navfn_{1..5}.txt` |
| Smac2D 1–5 | `run smac2d N`；14:58:10–19:23:53 | 全部 0 | 5/5 有效且有限静态门通过。`smac2d_{1..5}.txt` |
| T-DT A* 1 | `run tdt_astar 1`；14:59:15–14:59:54 | 1 | 有效证据，但 preflight/action 失败；停止第 2–5 次。`tdt_astar_1.txt` |
| T-DT A*+QP 1 | `run tdt_qp 1`；15:00:08–15:00:43 | 1 | 有效证据，但导航 action 失败；停止第 2–5 次。`tdt_qp_1.txt` |
| 汇总 | `p2b_validation.sh summarize`；19:24:11 | 1 | 写出已审计的部分矩阵；退出 1 仅表示 20 试次不完整。`summarize.txt` |

## 静态矩阵

| 方案 | 已运行编号 | Nav2 成功数 | 有效证据数 | 静态检查通过数 | 首个失败 / 未执行原因 |
| --- | --- | --- | --- | --- | --- |
| Navfn | 1–5 | 5 | 5 | 5 | 无。 |
| Smac2D | 1–5 | 5 | 5 | 5 | 无。 |
| T-DT A* | 1 | 0 | 1 | 0 | preflight status 6、navigation status 6、恢复 16 次；第 2–5 次按功能失败停止。 |
| T-DT A*+QP | 1 | 0 | 1 | 0 | preflight status 4，但 navigation status 6、恢复 16 次；第 2–5 次按功能失败停止。 |

- Navfn 的 5 次平均 preflight 往返耗时为 20.621 ms、平均 cross-track RMS 为 0.0358 m；Smac2D 分别为 15.811 ms、0.0340 m。此 wall_ms 是完整 action 往返，不是纯规划 core 时间。
- 两个通过组每次均为 action status 4、无恢复、位置误差不大于 0.085 m、yaw 误差不大于 0.170 rad、原车体间隙不小于 0.108 m、padded footprint 间隙不小于 0.066 m，且有新鲜停止命令。
- Navfn 首例期间保存了 `/node_list.txt`、`/map_odom_tf.txt` 和 `/resources.jsonl`：Nav2 节点存在；TF 在最初等待后实际为恒等 `map -> odom`；一次资源快照为 539.4 MiB / 15.33 GiB、CPU 539.70%。
- 两个通过组的首例均人工核对 `trajectory_review.svg/png`：蓝色轨迹绕中心块和北侧通道墙抵达目标，绿色采样 padded footprint 未见与黑色 fixture 相交。末帧 costmap 均为 `map`、240×240、0.05 m；中心块、北墙、南墙的预期矩形抽样区分别全部为 lethal（154/154、200/200、200/200 单元）。两个 spawn 成功标记也存在。
- 通过组没有观察到 snapshot 并发更新拒收；T-DT A* 与 T-DT A*+QP 首例各观察到拒收，分别为 15/14 条 `start or goal outside` 警告及一次 `costmap or footprint changed`。这是观测结果，不等同于已覆盖所有并发时序。

## 问题与证据

1. 分类：规划器功能/配置语义，T-DT A*。最小复现：`P2B_SERIES=static_v2 bash experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh run tdt_astar 1`。有效 summary 为 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v2/tdt_astar_1/observation/summary.json`：preflight status 6（27.250 ms、无 path samples），navigation status 6，最终 xy/yaw 误差 3.617 m / 1.633 rad，16 次恢复；相关 launch 错误为 `bt_navigator: Goal failed`。保留该组第 2–5 次未执行；未通过改目标、footprint、clearance、超时或算法取得通过。
2. 分类：规划器功能/配置语义，T-DT A*+QP。最小复现：`P2B_SERIES=static_v2 bash experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh run tdt_qp 1`。summary 路径对应 `tdt_qp_1`：preflight status 4（43.144 ms、264 samples），随后 navigation status 6，最终 xy/yaw 误差 3.813 m / 2.550 rad，16 次恢复；相关 launch 错误为 `bt_navigator: Goal failed`。保留第 2–5 次未执行；不将当前设备或单次结果宣称为算法淘汰。
3. 分类：未知/运行时清理。仅 Smac2D 首例的 `/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/static_v2/smac2d_1/launch.log` 在整套导航成功、launch teardown 已向节点发 SIGINT 后记录 `lio_adapter` exit code `-11`。同次 action、几何和原始记录流都已完整通过，其他 11 个试次未见该行；因此仅记录为一次清理阶段异常，不归因 Smac2D 或 T-DT，且不是本轮改变 LIO/控制参数的授权。设计者应单独复核。

## 验证结论与边界

- 记录器修复实际通过：新 `simulation_ros_messages` 4/4 通过，运行时不再出现 int/bytes rosout 等级异常；启动 WARN 被写入 `events.jsonl`，两个通过组的完整五类原始流均存在。
- 通过：在本固定静态 fixture、原 MPPI 和目标 `(4.3,0,0)` 下，Navfn 与 Smac2D 各 5/5 满足本轮有限静态几何、终点、停止和无恢复门。
- 失败/待设计者复核：T-DT A*、T-DT A*+QP 各有一份有效的功能失败证据；根因不能只凭 action status 或耗时断言。Smac2D 首例的 LIO 清理阶段 `-11` 亦待独立复核。
- 未覆盖：失败组的 2–5 次；移动障碍、LIO/双 STVL/MPPI 全负载、实车；T-DT 失败根因设计审查；所有可能的 snapshot 并发时序。
- 本轮未修改算法、业务代码、测试断言、目标坐标、footprint、padding、clearance、安全门或超时；未 push/merge。报告与 `build/tdt_p2b/` 证据为新增验证产物。
- 不宣布部署验收，等待设计者复核。
