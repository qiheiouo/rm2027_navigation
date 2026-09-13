# P2B 静态仿真：Terra 验证交接

状态：**static_v2 记录器回归通过，两组 T-DT 首例失败；代价语义修复待重验**。
本轮优先读 [代价语义修复交接](p2b_costmap_semantics_handoff.md)，使用 `static_v3`。
本文是执行规范，不是当前修复版的验证报告。
开发基线为 `ca608e5`；本交接属于 `experiment/tdt-planner-phase2`，实际交接提交号
由用户提供并与下文 `git rev-parse HEAD` 核对。P2A 的历史结果仍属于其原提交，
不要拿 P2A 的实现哈希清单检查本轮新增代码后就宣布历史证据损坏。

## 分工与范围

设计者负责架构、算法和业务代码、记录器及验收标准；Terra 负责按本文执行、保留
原始证据、分类问题并回传。Terra 不自行改算法、测试断言、目标坐标、footprint、
padding、clearance、控制器、安全门或超时来获得通过。涉及这些问题时停下对应阶段，
报告最小失败证据，交回设计者。报告和证据整理可由 Terra 完成。

当前设备造成的 CPU、内存、渲染速度、实时率问题只记录，不做性能补丁，也不作为
放弃方案的理由。保留运行时已有迟到拒收/停止语义；不能把等待新设备验证写成算法
已经淘汰。退出码或日志本身不一定能证明原因，例如 137 不能单独证明 OOM。

本交接只完成 **P2B 静态场景**：现有 Phase 1.5D 场景、原 MPPI、四个全局规划器，
目标 `(4.3, 0, 0)`，每组最多 5 次全新容器。移动障碍、现场 LIO/STVL 负载、实车
均不在此轮执行范围。历史 Phase 1.5D 动态安全失败记录仍有效。

## 持久路径与恢复

| 内容 | 路径 |
| --- | --- |
| 代码 | `/home/wpie/worktrees/rm2027_tdt_phase2` |
| 依赖、构建、配置、日志 | 代码目录下 `build/tdt_p2b/` |
| 本次试验原始数据 | `build/tdt_p2b/runs/static_v3/<planner>_<trial>/` |
| 回传报告模板 | `docs/tdt_migration/p2b_result_template.md` |
| 当前进度 | `docs/tdt_migration/p2b_work_status.md` |

容器内 `/ws` 为只读代码，`/work` 为上述持久数据目录；容器没有主机网络或设备
挂载，ROS domain=174、localhost only。内部临时文件也定向到 `/work/tmp`。
`build/` 已由仓库规则忽略；不要删除它来清理 git status，也不要提交整个构建目录。
会话中断后先查看 git 状态、已有日志和各次 `summary.json`，不得覆盖或重跑已有编号。
缺少末尾 summary 的目录属于未完成证据，不算一次通过。

## 1. 核对交接版本

在主机执行以下命令；无需在主机安装 ROS：

```bash
cd /home/wpie/worktrees/rm2027_tdt_phase2
git branch --show-current
git log -1 --oneline
git status --short
pkg="$PWD/experiments/tdt_planner/rm_tdt_planner"
work="$PWD/build/tdt_p2b"
mkdir -p "$work/handoff_logs"
```

预期分支为 `experiment/tdt-planner-phase2`，提交号与用户提供的交接提交一致，
相关代码没有未提交改动。不要切换主实车工作区分支或操作其三个现场配置改动。
`run` 会拒绝未提交的相关源码；编译/日志产物不会导致此项失败。

读取 `p2b_work_status.md`。e85b076 构建、39 项测试和两组基线静态验证已通过，
T-DT 两组首例失败。本次核心输入解释已修改，需新版本回归；保留 static_v1/v2。

## 2. 依赖与构建

只有依赖缺失或其固定 SHA/安装前缀不完整时才执行 `deps`。它会在主机显式取回
固定版本，然后在离线容器构建；不运行上游 setup 或安装系统软件。

```bash
set -o pipefail
# 按需执行；已有 build/tdt_p2b/deps_source 与 deps 可先核对后复用。
bash "$pkg/tools/p2b_validation.sh" deps 2>&1 | tee "$work/handoff_logs/dependencies.txt"

bash "$pkg/tools/p2b_validation.sh" build 2>&1 | tee "$work/handoff_logs/build.txt"
```

预期：6 个既有仿真相关包可构建，随后 `rm_tdt_planner` 构建成功。
构建选项包含 `BUILD_TESTING=ON` 和 `RM_TDT_BUILD_BENCHMARK=ON`。
若依赖已验证，不要重复下载；如果源 SHA 不符、包缺失或链接失败，保留完整命令、
退出码和首个有效错误，停止后续仿真。不要临时升级求解器或改变构建配置。

## 3. 自动测试与配置核对

```bash
bash "$pkg/tools/p2b_validation.sh" check 2>&1 | tee "$work/handoff_logs/tests.txt"
bash "$pkg/tools/p2b_validation.sh" sanitizers 2>&1 | tee "$work/handoff_logs/sanitizers.txt"
bash "$pkg/tools/p2b_validation.sh" profiles 2>&1 | tee "$work/handoff_logs/profiles.txt"
```

修复版预期 CTest 有四个入口：

| 入口 | 期望测试数 | 检查内容 |
| --- | --- | --- |
| `planner_safety` | 24 个 GTest | 原 core 合同及 5 项 Nav2 代价语义回归 |
| `nav2_plugin_contract` | 4 个 GTest | 原插件合同及 master 内切区/原地图不变回归 |
| `simulation_evidence_tools` | 13 个 Python unittest | 多边形几何、间隙/到点分离、缺证据拒绝、性能约束、四配置一致性 |
| `simulation_ros_messages` | 4 个 Python unittest | 真实消息序列化与回调、日志等级、有符号 costmap、异常证据与清理 |

**共 45 个用例是本修复版预期值，尚未实测。** 原 e85b076 的 39 项通过仅属于旧版。
只有全部通过才进入下一步。Python fixture 不是 Gazebo，4 个 CTest 入口也不是 4 个场景。
本轮改动了 core 的碰撞输入解释，按新交接运行 `sanitizers`（预期 24 项核心）；
未修改 vendor 求解数学。默认离线语义保留，无需无理由重跑 P2A 的 3,200 次比较。

`profiles` 生成 `profiles_p2b/{navfn,smac2d,tdt_astar,tdt_qp}.yaml`。重复调用仅核对，
不会覆盖。唯一不同的配置块是 `planner_server.ros__parameters.GridBased`。
MPPI、BT、costmap、footprint、padding 保持原仿真值。配置被修改时脚本应拒绝。

这里车体模型是原仿真 0.60×0.50 m、padding=0.03 m，与 P2A 旧车 0.64×0.54 m
不同，不能直接拼接两轮统计。Smac2D 保留 Humble 自带平滑；T-DT 的后端仍输出
几何折线。所有方案保留自己的原生代价处理，没有声称完全相同的优化目标。

## 4. 每组第一次静态试验

使用上述统一入口，每次命令都会新建隔离容器、启动同一场景，自动预检查一次
`ComputePathToPose`，再发送一次 `NavigateToPose`。不用手动发布 `/cmd_vel`。
先逐条执行并查看每条结果，**不要先批量循环**：

```bash
export P2B_SERIES=static_v3
bash "$pkg/tools/p2b_validation.sh" run navfn 1
bash "$pkg/tools/p2b_validation.sh" run smac2d 1
bash "$pkg/tools/p2b_validation.sh" run tdt_astar 1
bash "$pkg/tools/p2b_validation.sh" run tdt_qp 1
```

每个命令结束后检查对应目录：

```bash
cat "$work/runs/$P2B_SERIES/navfn_1/observation/summary.json"
```

将路径中的方案名替换为实际方案。命令持续期间可通过另一个终端读取该次日志；
若记录资源，只查看该次 `container_id` 指向的容器，例如：

```bash
trial_dir="$work/runs/$P2B_SERIES/navfn_1"
docker stats --no-stream --format '{{json .}}' "$(cat "$trial_dir/container_id")" \
  >> "$trial_dir/resources.jsonl"
```

若试验仍在运行，可在另一个终端只读记录该容器的节点与 TF。下面的 timeout 退出码
124 表示观察窗口正常结束，不能单独算 TF 失败：

```bash
trial_id=$(cat "$trial_dir/container_id")
docker exec "$trial_id" bash -c 'source /opt/ros/humble/setup.bash; ros2 node list' \
  > "$trial_dir/node_list.txt" 2>&1
docker exec "$trial_id" bash -c 'source /opt/ros/humble/setup.bash; timeout 5s ros2 run tf2_ros tf2_echo map odom' \
  > "$trial_dir/map_odom_tf.txt" 2>&1
```

需要核对：

1. `launch.log` 有两条 `[TDT_P2B] spawn_course_wall_{north,south} exit=0` 标记，
   Nav2 lifecycle 激活、scan/odom/costmap 到达。记录器据此等待，缺输入不会发目标。
2. 读取静态 SDF 与本次 costmap/路径/轨迹，确认中心块和两条通道墙的感知符合场景。
   spawn 进程成功不单独证明所有墙面已被感知；这项需要复核数据。
3. 原模拟链保持 `map_odom_stub` 的恒等 map/odom、`lio_adapter` 的 odom/base、
   robot_state_publisher 静态 TF。场景假定 map 与 odom 相同；若 launch/TF 与该
   前提不符，应停止并报告，不能通过另建 TF 修正轨迹图。
4. `preflight.status`、`action_status`、`checks`、间隙和最终误差分别报告。
   action status=4 只说明 Nav2 返回成功，不能替代几何检查。
5. 查看 `events.jsonl` 中 costmap/footprint changed、deadline、起终点不可行、
   恢复等事件，并对照 `plans.jsonl`、`trajectory.jsonl`、`commands.jsonl`。
   未观察到并发 snapshot 拒收时，写“未覆盖”，不能把 0 次拒收写成这项已经通过。

试验结束后可以在主机导出 SVG，查看中心块、两条通道墙、全部已发布路径、实际轨迹
和抽样 padded footprint；无需安装绘图库：

```bash
python3 "$pkg/tools/plot_simulation.py" "$trial_dir/observation" \
  --output "$trial_dir/trajectory_review.svg"
```

打开 SVG 并对照原始 costmap 与统计检查。图是检查辅助，不能凭线条好看宣布通过。
没有 trajectory 的启动失败不能画导航轨迹，应保留原始启动日志。

记录器以消息时间保存约 25 Hz 的位姿、全局规划折线、最终 `/cmd_vel`、完整发布的
costmap 和警告。`costmap.jsonl` 是 Nav2 发布快照，不能据此精确重建两次发布间
每一版内部代价图。预检查的 wall_ms 包含 action 往返，不是纯 core 求解时间。

单次退出码：0 为有限静态几何/到点检查通过；1 为检查未通过；2 为证据或工具/启动
不完整。容器异常还可能产生其他码；先读日志分类，**均不是自动淘汰算法的信号**。
任一组首次有功能或安全失败时，停止该组剩余重复；其他独立组可继续首次检查。
如果是共用启动、依赖或记录器错误，则停止全部仿真，交回设计者处理。

## 5. 完成静态矩阵

只对首次已通过且人工核对无问题的方案执行剩余编号。例：

```bash
for trial in 2 3 4 5; do
  bash "$pkg/tools/p2b_validation.sh" run navfn "$trial" || break
done
```

逐组将 `navfn` 替换为其他已通过方案。每次都是新容器与新场景；首例计入 5 次。
命令拒绝覆盖已有编号。中断或新设计版本需要另一个 `P2B_SERIES`，旧目录保留并在
报告说明；不能删除失败记录后以同一矩阵补成 5/5。不同源码/镜像/场景哈希不得混为
一组。仿真噪声未冻结为相同序列，这是相同场景多次重启，不是严格同噪声回放。

静态检查采用：5/5 Nav2 成功、最终位置误差 <=0.15 m、yaw <=0.20 rad、无恢复、
原 footprint 间隙 >=0.05 m、padded footprint 无接触、末尾有新鲜停止命令。
轨迹采样缺口 >0.20 s 会标为证据不完整；此阈值用于记录质量，不用于淘汰算法。

几何量必须按以下含义解释：

- 对原静态方块/墙体计算旋转矩形的精确距离；接触/相交距离为 0。
- 另给相邻位姿线性平移与 yaw 插值的保守距离下界；它不是对采样间真实 Gazebo
  运动的无条件证明，更不是对所有机器人几何或真实接触传感器的替代。
- cross_track 是到最近收到的全局折线的距离，重规划会改变参照；不是 MPPI 内部
  局部参考的误差。用轨迹图和 plan revision 一起复核。
- command acceleration 是命令差分，不是测得的动力学加速度或 jerk。本轮不验收 MPC。
- CPU、wall time、控制周期告警与数据缺口单独记录。明确由当前设备资源限制导致
  的项目留待新设备，不改 MPPI 参数、渲染精度或数据频率来获得当前设备通过。

## 6. 汇总与回传

完整或中断的矩阵均运行汇总，输出可审查结果：

```bash
bash "$pkg/tools/p2b_validation.sh" summarize
```

输出保存在当前 series 的 `aggregate_<UTC>.json`，不会覆盖之前汇总。汇总会回读
原始 trajectory/commands/events 复算统计，核对 profile、源码/镜像/场景一致性并
生成原始文件 SHA。缺样本、缺试次或报告不一致不会得到通过；部分矩阵仍能出报告。

从 `p2b_result_template.md` 创建一个新的实际验证报告，建议保存在
`docs/tdt_migration/evidence/p2b_static_<日期>/`。放入报告、aggregate、构建/测试日志
和必要失败片段；较大的 costmap/trajectory/ros 日志留在持久 `build/tdt_p2b/runs/`，
在报告写绝对路径与 SHA。不要将整套 build、求解器源码或 Gazebo 缓存加入 git。
允许 Terra 写报告；源码修正、修改验收门、发布或合并代码由设计者复核后处理。

## 故障归类与停止条件

| 现象 | 本轮处理 |
| --- | --- |
| 构建/导入/脚本错误 | 保留首个有效错误及命令；停止依赖此工具的后续步骤，不临时改源码 |
| 缺 scan/clock/墙体/lifecycle | 记录 readiness 和 launch 日志；先标基础设施/输入问题，不归因规划算法 |
| T-DT 起终点在保守自由区外 | 保存同次 costmap、目标和 profile；不要移动目标或缩 footprint。需要设计者分析二次膨胀等语义 |
| 规划时 costmap 改变被拒收 | 记录次数、后续 action 和停止/重试；偶发拒收与持续无进展需分开 |
| Nav2 成功但间隙/终点/停止失败 | 静态检查失败，保留轨迹与相应命令，停止该组重复 |
| 明确的设备资源不足 | 记录观测与证据，标待新设备验证，不做性能优化、不淘汰方案 |
| 记录器证据矛盾/丢失 | 标 UNKNOWN/未覆盖，不能把缺数据填成 0 或通过 |

## 可直接交给 Terra 的任务说明

> 请在 `/home/wpie/worktrees/rm2027_tdt_phase2` 执行
> `docs/tdt_migration/p2b_costmap_semantics_handoff.md`，再按本文继续。
> 先核对用户提供的修复提交号，保留 static_v1/v2，使用新 static_v3。
> 你负责验证和报告，不改算法、业务代码、测试断言或安全参数。按顺序运行工具检查、
> 构建/CTest、四配置核对、各组首次静态试验，再按门完成最多 5 次矩阵。失败按文档
> 分类并保存原始证据，不为通过而修改配置。设备性能问题只记录，后续换设备验收。
> 所有目录放在 `/home`，不要覆盖已有运行或删除 build。用回传模板写报告，给出
> 提交号、执行命令/退出码、aggregate 与原始日志的绝对路径，明确未执行和未覆盖项。
> 不执行移动障碍、实车、push 或 merge。完成后将结果返回给设计者复核。
