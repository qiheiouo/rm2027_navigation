# A23 有限同条件 Gazebo 闭环对照

Research-only，主算法固定在 `ccd3eac4`。判决 **Modify**：当前配置动态安全收益重复出现，速度/circle差异未消融，生产化暂停。判定与限制见
[阶段记录](../../docs/dynamic_navigation/r4_finite_closed_loop_comparison.md)。

唯一新增运行实现为本目录的薄 Nav2 Controller wrapper。B0 委托原
native MPPI；R4 共用 native angular.z，只用原 WorldFollowAdapter 替换 XY。
原 controller_server/action/GoalChecker/smoother/chassis stub/Gazebo bridge 共用，
没有第二个最终输出 owner，没有 fallback、lease 或新 solver。本目录有
`COLCON_IGNORE`，不进入正式工作区构建和启动配置。

共同参数取 main 的 `nav2_old_car_2026_left_stvl.yaml`，使用已安装 STVL
LaserScan 接口。`prepare.py` 只派生场景、参数和真值观察器；既有 tracker /
public-v2 / observed sidecar / Sfc / OSQP 均复用。真值不进入控制器。

已构建的 A04/A13/A22 本地依赖和现有离线 Docker 镜像是运行前提，脚本
不会下载软件。仅挂载仓库只读与指定证据目录可写，网络关闭、无串口，
每个 trial 新建仿真实例，supervisor 只结束本次拥有的进程。

```bash
bash experiments/r4_gazebo_comparison/run.sh build/r4_finite_comparison_20261006 build
python3 experiments/r4_gazebo_comparison/batch.py build/r4_finite_comparison_20261006
python3 experiments/r4_gazebo_comparison/analyze.py build/r4_finite_comparison_20261006
```

有限序号从 101 开始；此前 pilot 启动/接线失败全部保留且单列。每组
S0 5 次、S1/S2 10 次，按序号交替组别先后；首次失效或真实 contact 结束
当前 trial 并保存环形原输入、事件、控制、预测、独立投影和日志。批次
恢复只跳过已记录结果，不重跑失败去替换原结果。30 ROS 秒上限。

原始文件位于 `build/r4_finite_comparison_20261006/runs/`；`summary.json`
和 `trials.csv` 是分析输出。无新大规模 bag、环境哈希或全仓回归。
独立物理包络函数只读复用冻结 R3 `audit_run.py/temporal_mpc.oracle`，
不启动 R3 链路；距离为 0 表示保守包络相交，真实 contact 另列。

本轮结果只适用于共同 20Hz 控制周期、平面仿真雷达和 phase1 机械模型。
Fortress CLI/MPPI 没有此次可用的 seed 配置，按实际轨迹核对公平条件，
不能宣称随机噪声逐样本相同。native MPPI angular 委托仍是 R4 的依赖；
它失败时单列原因，不能归因为 XY solver。真实底盘、新车三维感知及
生产级整合均不在本轮范围。

必要汇总、共同参数、配对轨迹指标、四次真实碰撞事件/控制及两张研究图在[证据目录](evidence/)；原输入/预测/真值/日志仍保留在build，未push。有效结果为S0两组5/5、S1 B0 9/10 vs R4 10/10、S2 B0 7/10 vs R4 10/10；不能用成功样本时间代表碰撞样本完成时间。

## A24 匹配速度与圆形支持

[A24协议和判决](../../docs/dynamic_navigation/r4_matched_closed_loop_comparison.md)对齐空场实测到达时间及circle支持，使用同一个A23插件二进制。`matched.py prepare`只复制已有A23 assets/install；不重新构建算法。两组共同32边形支持与local inflation=.50m；仅baseline `vx_max`在空场校准后冻结为.32，R4算法与原limits不变。完整小样本为S0各3、S1/S2各5，校准与启动失败单列。

```bash
python3 experiments/r4_gazebo_comparison/matched.py prepare build/r4_matched_comparison_20261006
python3 experiments/r4_gazebo_comparison/matched.py reference build/r4_matched_comparison_20261006
python3 experiments/r4_gazebo_comparison/matched.py calibrate build/r4_matched_comparison_20261006 1 .34
python3 experiments/r4_gazebo_comparison/matched.py calibrate build/r4_matched_comparison_20261006 2 .32
python3 experiments/r4_gazebo_comparison/matched.py freeze build/r4_matched_comparison_20261006 2
python3 experiments/r4_gazebo_comparison/matched.py batch build/r4_matched_comparison_20261006
python3 experiments/r4_gazebo_comparison/analyze.py build/r4_matched_comparison_20261006
python3 experiments/r4_gazebo_comparison/fairness.py build/r4_matched_comparison_20261006
python3 experiments/r4_gazebo_comparison/matched_audit.py build/r4_matched_comparison_20261006
```

`run.sh`第三参数仅用于选择experiment profile；默认`common`保留A23。原输入/真值/全部日志仍写入独立build目录，不录大bag。两轮的配置与证据分别保留，不能覆盖A23结果或把联合对齐当作三因素逐项消融。

A24结论 **Modify**：两组各13/13、零contact。S1净空几乎相同；S2 R4中位净空增加约12cm，耗时增加33%，WAIT中位3.64s且有小幅回退。原A23碰撞优势不再作为独立consumption收益证据，生产化仍暂停。[必要证据](evidence_matched/)；28次完整原始记录保留在build。

## A25 S2 等净空效率校准

[A25协议/修订/结果](../../docs/dynamic_navigation/r4_clearance_efficiency_pareto.md)。`pareto.py`复用A24 assets/install，R4文件逐字节保持，仅校准baseline既有local inflation；目标中位[.28,.32]m。6个参数候选及一次固定候选补样本，共20次均成功、零contact，但目标未达；**Modify，未冻结，未开始正式R4对照**。该结果不表示native无法达到目标，校准效率不能冒充独立Pareto判决。

复现顺序（每个输出目录只创建一次，全部结果保留）：

```bash
python3 experiments/r4_gazebo_comparison/pareto.py prepare build/r4_pareto_comparison_20261006
python3 experiments/r4_gazebo_comparison/pareto.py calibrate build/r4_pareto_comparison_20261006
# 初始4候选未达标后，已明确登记的一次半径修订：
python3 experiments/r4_gazebo_comparison/pareto.py calibrate_radius build/r4_pareto_comparison_20261006
# 最接近固定候选仅补2次信息，不改参数：
python3 experiments/r4_gazebo_comparison/pareto.py validate_calibration build/r4_pareto_comparison_20261006
python3 experiments/r4_gazebo_comparison/analyze.py build/r4_pareto_comparison_20261006
python3 experiments/r4_gazebo_comparison/pareto_plot.py build/r4_pareto_comparison_20261006
```

没有freeze时`batch`拒绝启动。若未来合法形成冻结点，`batch ... 5`、`pareto_audit.py`复用原分析器按预先登记规则判定；只允许有记录的判决不确定性才固定参数扩至10。本轮[紧凑证据](evidence_pareto/)与全部原始build数据保留，不push。
