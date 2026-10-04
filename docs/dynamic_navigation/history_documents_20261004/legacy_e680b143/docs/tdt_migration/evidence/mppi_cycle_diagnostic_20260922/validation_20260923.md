# P2B 移动障碍：MPPI 同周期诊断首例

状态（2026-09-23，Asia/Shanghai）：**A* 与 QP 的固定相位首例均取得完整逐周期证据，但动态几何门仍失败。** A* action 成功、恢复0次，却发生参考新车本体采样平面重叠；QP action 成功、恢复10次，本体最小采样间隙约0.0194 m、padded footprint 有采样重叠。两组都没有继续做重复试次，`accepted_for_deployment=false`。本轮不需要实车；新车尚未制造、老车雷达线更换中的硬件限制保持不变。

[逐周期独立复算](cycle_analysis.json)、[时间窗图](cycle_window.png) / [SVG](cycle_window.svg)和[审计命令结果](audit_check.txt)是本轮结论入口。此前[无插桩动态失败报告](../dynamic_reference_20260922/validation_20260922.md)与[离线分析](../dynamic_failure_analysis_20260922/validation_20260922.md)保持冻结；这两组插桩试次是**新 series**，不是原首例的逐帧复现，也不填充动态验收矩阵。

## 1. 输入、工具与失败保留

仍在隔离 `experiment/tdt-planner-phase2` 工作树中。试次准备时HEAD `764ee7c8c58e29399c419730cf2fa749205653bb`，T-DT及仿真运行时源码为 `a419654a8fed1fc8321c234fb212abd0a6cabe04`；`src/`、`experiments/`相对该运行时提交无差异。沿用原 v2 动态夹具、目标 `(5.6,0,0)`、固定相位0、13 m全局窗口、MPPI/BT/profile、参考八边形、footprint padding、0.05 m本体间隙门和其余原安全门。A*、QP profile 哈希分别为 `bd1aaba64d16dee9b1e1a93520fd2cf384980f499b65d7af0ade1bb192a035e9`、`624f6bd1c9f60ea6faff7c4a14a0800dfb2179ea9d15f3fdf3a8405936cac018`。[v2受控输入](../../../../build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2/inputs.json)逐项冻结工具、运行库和夹具哈希。

只为诊断在独立prefix从Nav2 1.1.20源码构建MPPI控制器与CostCritic，采集同周期锁内raw局部图、footprint、pose、rollout、各critic累计代价/碰撞标记、更新与滤波后的控制序列及最终返回命令。算法计算语句保持原样，未改T-DT、控制参数或部署默认。采集器从运行时进程实际加载的库映射确认两个插桩`.so`来自独立prefix；[构建输入与哈希](build_inputs.json)及[上游源码档案](upstream.json)可复核。29份上游头文件与已安装版本一致，但**没有逐字节重建apt二进制**，插桩会改变调度/时序。本轮只能用于定位，不能作为无插桩控制器的定量性能或因果A/B。

最初 `dynamic_cycle_diagnostic_v1/tdt_qp_1` 在仿真秒0.02因Python订阅回调错误退出：Humble只向回调传`msg`，采集器却要求`(msg, info)`。**该次未进入preflight或导航**，原始traceback、exit=2和空试次保留，不算规划器失败。修复为可选`info`并对不可用逐消息GID/接收时间明确存`null`，在ROS容器通过[回调测试2/2](observer_callback_tests.txt)后才创建新 `dynamic_cycle_diagnostic_v2`；没有覆盖v1。图谱快照只能说明哪些节点有发布端，不能为每条`/cmd_vel`消息确定publisher。

## 2. 构建和证据完整性

[软件检查](software_checks.json)：独立构建通过；Nav2原有12个测试入口共72/72、trace解码4/4、CostCritic专项1/1、回调2/2通过。专项测试在空图记录0/1000碰撞标记、致命图1000/1000并沿原代码路径报告规划失败；首次专项夹具漏设polygon的失败尝试保留。采集器单独ASan/UBSan生命周期与异常flush探针通过；这**不是**ROS/T-DT全栈sanitizer验证。两个仿真首例的运行前及导航后footprint查询均成功，运行前两张costmap参数与冻结profile完全一致。

| 采集项 | A* | QP |
| --- | ---: | ---: |
| MPPI周期JSON+二进制成对写入 | 198/198 | 172/172 |
| writer丢包/写错误/异常展开/缺输出 | 0/0/0/0 | 0/0/0/0 |
| 最终返回命令等于滤波后序列第1项 | 198/198 | 172/172 |
| 返回命令在`/cmd_vel_nav`找到同值消息 | 198/198 | 172/172 |
| observer复制开销中位/最大 ms | 0.232/1.397 | 0.211/1.277 |
| MPPI计算周期中位/最大 ms | 2.006/5.908 | 1.743/7.765 |

这些开销只度量本次插桩部分和本次周期；不构成未插桩实时性上界。`map_capture_sim_s`是持锁复制时的仿真时刻，**不是地图上次更新时刻**。`/cmd_vel*`的`t`是observer回调时最近`/clock`，Twist无header、逐消息GID与DDS到达时间未取得，不能由表中毫秒差求端到端延迟。

## 3. 实际动作与动态间隙

| 指标 | T-DT A* | T-DT QP |
| --- | ---: | ---: |
| preflight / navigation action | 4 / 4 | 4 / 4 |
| recovery | 0 | 10 |
| 终点XY / yaw误差 | 0.1207 m / 0.1771 rad | 0.0270 m / 0.1709 rad |
| cross-track RMS | 0.0415 m | 0.0942 m |
| 首次采样本体间隙 `<0.05 m` | 37.087 s | 28.337 s |
| 首次采样padded接触 | 37.132 s | 28.432 s |
| 首次采样本体重叠 | 37.211 s | 未采到 |
| 本体连续插值间隙下界 | -0.0102 m | 0.0122 m |
| padded连续插值间隙下界 | -0.0104 m | -0.0084 m |
| 动态几何与目标门 | **失败** | **失败** |

间隙由Gazebo实际机器人与移动连杆位姿、参考新车polygon、原有独立oracle复算。负插值下界只是采样间运动上界；A*本体与两组padded的**采样重叠**另有直接polygon结果。QP本体没有采样重叠，不把其负的padded插值下界说成本体撞击。此处仍是平面几何侵入，不是Gazebo接触力或真实车辆碰撞证明。QP有16条QP采纳、2条经验证A*回退；回退不改变动态失败事实。

## 4. 同周期窗口发现

首次本体间隙违例前的最近完整MPPI周期：

| 指标 | A*，37.025 s | QP，28.255 s |
| --- | ---: | ---: |
| 实际周期时刻 | 37.025 s（早0.062 s） | 28.255 s（早0.082 s） |
| 实际箱体范围内raw 254 / 253 cell中心 | 75 / 24 | 72 / 27 |
| CostCritic标记碰撞的rollout | 105/300 | 184/300 |
| 当前机器人参考本体到raw 254闭cell最小距离 | 0.0998 m | 0.1823 m |
| 同时刻本体到Gazebo真实箱体 | 0.0822 m | 0.0668 m |
| MPPI返回`(vx,vy,wz)` | `(0.637,-0.061,-0.029)` | `(-0.095,0.440,-0.460)` |

违例后首个周期A*为37.108 s：实际箱体内raw 254/253为83/16格，74/300条rollout被标记，机器人对raw 254距离0.0490 m、对实际箱体0.0383 m，仍返回`(0.648,-0.146,-0.052)`。QP为28.346 s：79/20格、244/300条标记，raw 254距离0.0757 m而实际箱体仅0.0452 m，返回`(0.003,0.443,-0.399)`。后续QP在28.436 s的raw 254距离已降至0.0393 m，真实padded footprint采样重叠。完整五周期数值在[cycle_analysis.json](cycle_analysis.json)。

两组在违例前的`/cmd_vel_nav`、共用`/cmd_vel`、底盘转发命令与实际odom均有非零平移速度，且原始MPPI返回命令能与平滑前topic逐值对应。**本次不能再把失败解释为CostCritic完全没看到障碍或MPPI实际加载了未插桩库。** 但标记的是采样rollout；本轮未对最终滤波后控制序列作独立动力学重放与连续碰撞认证，也没有取得raw地图上次更新时刻、控制队列内部状态、逐消息publisher身份或校准的DDS时延。因此不能单独定罪加权平均、滤波、velocity smoother、地图时延、底盘响应或T-DT；锁内raw 254与真实箱体间的距离差异也不能由单次观测推出唯一成因。

## 5. 结论与下一项

这两例把定位范围收敛到：**局部代价图已含移动障碍、CostCritic已标记一部分候选碰撞，实际执行链仍未在安全间隙耗尽前建立让行/停止。** A*本次无recovery仍发生本体重叠，说明action和零恢复不能代替动态几何门。固定相位首例不足以比较A*/QP成功率或证明插桩造成/消除了某种行为；不进行“选好相位”补跑。

下一步应在本series上做离线反事实重放：以冻结的同周期raw图、pose和滤波前后控制序列，按已安装MPPI运动模型重建最终控制轨迹，再分别核对代价/碰撞判据与连续polygon间隙。它应先给出一个可审阅的最小复现，再决定是否需要单一安全合同修复；不要同时改MPPI参数、终点选择、footprint或安全门。因raw图没有内部更新时刻，若重放仍无法区分地图时序，再单独增加相应时间戳采集。动态验收矩阵、完整传感器/TF审查、目标设备全负载与硬件验收仍未完成。

原始数据在`build/tdt_p2b/runs/dynamic_cycle_diagnostic_v1/`和`dynamic_cycle_diagnostic_v2/`，编译/测试日志在`build/tdt_p2b/mppi_cycle_diagnostic_v1/`。[本轮清单](manifest.json)冻结报告、运行输入、核心raw周期、命令/位姿流和实际检查日志；清单自身不自包含。此前662项旧清单条目在[开始](preservation_before.json)和[结束](preservation_after.json)两次核对均完整（清单间有重复）。没有修改部署默认，没有实车运行、push或merge。
