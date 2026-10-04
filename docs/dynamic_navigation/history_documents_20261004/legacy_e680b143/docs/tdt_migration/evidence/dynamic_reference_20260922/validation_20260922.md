# P2B 移动障碍首例验证：到点成功，动态安全未通过

状态（2026-09-22，Asia/Shanghai）：**无需等待实车即可继续软件与仿真工作。本轮动态首例中，
T-DT A*、QP均action成功，但分别恢复1、13次；参考新车本体均与移动障碍发生采样平面重叠。
两组停止重复，动态安全门均未通过，未作部署结论。**

[审计汇总](aggregate.json)、[轨迹与碰撞时刻图](dynamic_review.png) / [SVG](dynamic_review.svg)、
[独立重叠证据](overlap_corroboration.json)。此前[静态10次通过](../snapshot_revalidation_20260922/validation_20260922.md)
保持原有证据范围，没有填入动态矩阵。当前阶段完成的是首例诊断，未完成每组10次不同相位的动态验收。

## 1. 无实车期间的范围

用户已说明新车尚未造好、老车雷达线正在更换。本轮只使用隔离worktree与Docker仿真。
传感器实测、制动、真实轮组/附件包络、新车动力学与高速Spin仍需后续硬件验证；
没有把仿真成功当作实车安全证明。详见[无实车推进边界](../../p2b_without_hardware_plan.md)。

本地 `main-new-car` 仍是 `bd6cf68`，没有更新的已确认整车几何。沿用仓库382/126mm八边形参考顶点，
127mm保留为此前敏感性分析；base_link/几何/Spin中心共点已经用户确认。
本轮没有补写CAD确认项，没有改SDF碰撞体、删轮组或缩小footprint。
新车参考polygon、旧车0.64×0.54m矩形、仿真载体0.60×0.50m本体矩形分别计算。
后者不包括全部轮组，因此不声称完成仿真整车或真实整车的3D碰撞验收。

terminal selector继续暂缓，航向A/B保持冻结，没有调MPPI、BT、heading、clearance、安全阈值或部署默认。
没有开始新的定位、MPC、UWB、因子图或狗洞开发，没有实车执行、push或merge。

## 2. 版本、复用与受控输入

试验工作树HEAD `4237f617ec3e4fd3da896a7d2734eda3f5daaf27`；运行时源码仍为
`a419654a8fed1fc8321c234fb212abd0a6cabe04`，`experiments/`、`src/`与该提交无差异。
镜像 `rm2027_navigation:humble`，ID
`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。
新内容是隔离动态试验启动器、观察/审计工具与报告，没有修改规划器/控制器实现。

复用 `src/rm_simulation/models/moving_obstacle.sdf`、`moving_obstacle_controller.cpp`、
`course_dynamic_bridge.yaml` 与已有Phase1.5D动态规程：障碍模型x=4.9m，
箱体0.45×0.55×0.8m，Y关节目标振幅0.9m、周期8s，导航goal为 `(5.6,0,0)`。
这是与静态goal `(4.3,0,0)` 不同的独立场景，不能合并计算通过率。

先运行无导航目标的 `dynamic_fixture_probe_v1`，采集1730条实际连杆位姿，最大间隔0.019s。
实际关节Y约到±0.95m，超过±0.9m的目标振幅；独立oracle使用实际位姿。
没有将关节控制目标作为障碍运动真值。只读Gazebo transport的pose信息，不新增ROS TF发布者。

每个组预先固定相位0，在至少16仿真秒后按8s周期释放目标；没有挑选成功通行相位。
容器无网络、非root，源码只读挂载，使用独立ROS domain/IGN partition。

### v1：保留初始窗口不兼容的失败

`build/tdt_p2b/runs/dynamic_reference_pilot_v1/` 使用原12m全局滚动窗口。
A*首例preflight=6、action=6、恢复15次、无发布路径。
初始origin约(-5.95,-5.95)，goal到边界闭cell距离0.400000m，
小于派生padded半径+clearance要求0.400788908m。
15次端点拒收全部有可复算的goal边界witness；没有继续QP或重复本组。
原observer因没有可供cross-track比较的路径而 `evidence_valid=false`；该试次只用于窗口问题诊断。

首次后处理误用了要求goal4.3的静态审计入口，因goal5.6断言退出。
原始日志与工具保持，新增独立动态goal审计入口复算；没有改静态断言或原始summary。
运行失败与后处理错误分开记录。

### v2：仅修正动态场景所需地图覆盖

新目录 `build/tdt_p2b/runs/dynamic_reference_pilot_v2/` 只将实验profile的
`global_costmap.global_costmap.ros__parameters.width` 从12改为13m。
其它参数解析后完全相同，height仍12m、分辨率0.05m。

覆盖下界按 `2 × (5.6 + padded_radius + 0.02 + 1e-7 + 2×0.05)` 推导为12.201578012m；
两格覆盖边界闭cell与滚动取整，Nav2整数米宽度向上取整为13。
这是独立夹具的地图覆盖修正，不修改机器人大小或碰撞标准。两组发布地图均为260×240格。

v2 A* profile哈希 `bd1aaba64d16dee9b1e1a93520fd2cf384980f499b65d7af0ade1bb192a035e9`；
QP为 `624f6bd1c9f60ea6faff7c4a14a0800dfb2179ea9d15f3fdf3a8405936cac018`。
本体间隙门0.05m、padding0.03m、planner clearance0.02m、终点XY/yaw门0.15m/0.20rad均保持。
运行前planner/MPPI参数与各自profile逐项核对；heading与MPPI path orientation候选均关闭。

## 3. v2 两次实际结果

| 指标 | T-DT A* 首例 | T-DT QP 首例 |
| --- | ---: | ---: |
| preflight / action | 4 / 4 | 4 / 4 |
| recovery | 1 | 13 |
| terminal XY误差 m | 0.019465 | 0.006048 |
| terminal yaw误差 rad | 0.172266 | 0.109483 |
| cross-track RMS m | 0.036749 | 0.067393 |
| 行进yaw-to-path RMS rad | 0.803739 | 0.678445 |
| 实际wz RMS / max rad/s | 0.178470 / 0.492476 | 0.369264 / 1.054631 |
| 实际最大相邻wz变化 rad/s | 0.175758 | 0.447198 |
| 实际最大采样abs(dwz/dt) rad/s² | 4.393960 | 11.179956 |
| 新车本体对静态障碍间隙插值下界 m | 0.584095 | 0.165523 |
| 新车本体对移动障碍采样最小间隙 m | **0，重叠** | **0，重叠** |
| 本体对移动障碍间隙插值下界 m | -0.010008 | -0.011508 |
| padded footprint对移动障碍采样最小间隙 m | **0，重叠** | **0，重叠** |
| 最新快照复查准入 | 4 | 1 |
| endpoint拒收事件数 | 0 | 8 |
| 动态几何与目标门 | **失败** | **失败** |

QP成功输出中18条采用QP、1条经验证的A*回退。8次端点拒收全部含start blocked，
其中1次还含goal blocked；这些不是9个拒收事件。witness全部复算且无缺失。
A*无全局规划失败；两组没有观察到最新快照复查拒收。没有为了补满矩阵继续重复。

实际wz与命令wz分开记录；包括恢复与settle阶段。QP命令流最大相邻变化1rad/s、
按采样间隔计算最大变化率约500rad/s²，这是命令流观测值，不是实际机械角加速度。
本轮未新增角速度门或对这些异常调参。

## 4. 已定位的失败时间窗

| 事件（仿真秒） | A* | QP |
| --- | ---: | ---: |
| 首次采样本体间隙低于0.05m | 29.013 | 28.065 |
| 首次采样本体重叠 | 29.110 | 28.114 |
| controller_server报告MPPI无法计算路径 | 29.152 | 28.166 |
| 首次T-DT端点拒收 | 无 | 28.266 |

首个间隙违例之前的最近命令仍非零：A* `(vx,vy,wz)=(0.575,0.049,0.277)`，
QP `(0.547,0.147,0.067)`。这些是对应topic消息，不代表已审查完整制动时序。
MPPI的 `Optimizer fail to compute path` 是controller_server日志，不能当作T-DT QP solver失败。

重叠不是仅凭负插值下界推断：首次重叠样本中，障碍箱体右下角严格位于新车八边形内部，
独立半平面检验的8个叉积全部为正，见[重叠复核](overlap_corroboration.json)。
旧车矩形与仿真载体本体矩形也有采样重叠。上述结论是实际仿真位姿的**平面几何侵入**，
本轮未记录Gazebo contact传感器，不将它冒充已核实的3D接触力或真实车辆碰撞。

首次间隙违例前最近的全局/局部发布地图中，实际箱体范围内分别均有99个lethal cell中心。
A*两图时间戳分别早0.003/0.095s，QP早0.087/0.112s。
此比较沿用该夹具world/odom对齐及identity map→odom定义；Gazebo robot XY与ROS ground-truth
插值最大差异A*约0.000094m、QP约0.000529m。它说明该窗口存在障碍标记，
不证明MPPI当次内部代价图内容、处理延迟、完整TF所有权或marking/clearing全项通过。
发布OccupancyGrid不等同于规划器同次锁内raw snapshot。

目前支持的结论：**静态目标已通过，但当前移动障碍下的观测—局部避障—停止闭环未满足动态安全。**
现有快照复查检查“当前地图中的整条候选几何”，没有承诺未来障碍运动安全。
本轮没有取得违例时MPPI内部rollout或每次准入的完整同次raw图，不能据此把根因确定为
快照guard错误，也不能声称换terminal selector即可修复。下一步优先用此固定时间窗审查
scan/局部图时序、MPPI碰撞评分与首次减速/停令，而不是继续调终点或航向。

## 5. 证据质量、检查与局限

- 独立oracle组合Gazebo模型与连杆实际位姿，复用已有polygon距离算法；静态障碍与移动障碍分别计算。
  两物体线性位姿插值的运动上界都计入动态间隙下界。最大Gazebo采样间隔0.019s；
  scan/odom最大间隔分别0.067/0.020s。这个插值假设不覆盖任意未采样加速度。
- 新增oracle测试5/5通过：父子姿态组合、protobuf时间戳、障碍在样本间穿越、
  同速运动/padding和旋转重叠。[实际软件检查](software_checks.json)。
  上轮build、101/101常规、uniform sanitizer core70/70是同运行时源码的历史证据，本轮没有重跑或新增计数。
- 原observer summary只算静态矩形几何，原样保留并从raw流复算。
  `dynamic_summary.json`另行替换两项几何判据为移动polygon结果；action成功不覆盖动态几何失败。
- **QP导航后的footprint参数服务查询超时，未得到runtime_geometry.json或导航后完整raw地图。**
  原始 `geometry_capture.log` 保留。运行前planner/MPPI参数、profile哈希、发布地图尺寸与
  仿真实际轨迹可核查，但QP costmap footprint运行值的独立服务确认缺失。
  因此汇总 `full_evidence_complete=false`；已有轨迹与实际障碍足以确认参考polygon侵入，不能用于完整通过认证。
- 审计工具首次比较遇到Python tuple/JSON list表示差异，随后发现上述capture缺失；
  两份失败审计日志均保留，最终审计明确记录缺项。没有补跑QP、补造runtime图或覆盖原始记录。
- [图形](dynamic_review.png)已目视复核：两条路线均从静态障碍南侧通过，移动障碍附近出现侵入；
  右列呈现首次重叠位置，几何数值来自完整polygon而非图片像素。
- 两个先导样本不能比较算法动态成功率；未跑Navfn/Smac2D动态基线，也没有完成10次不同相位矩阵、
  完整marking/clearing、canonical TF、目标设备全负载、真实Spin或实车验收。
- 445份上轮静态证据全部保持；v1的14项工具/fixture及2份profile、v2的16项及2份profile
  哈希全部一致，见[preservation_after.json](preservation_after.json)。

## 6. 复现入口与冻结边界

原始数据位于 `build/tdt_p2b/runs/dynamic_fixture_probe_v1/`、
`dynamic_reference_pilot_v1/`、`dynamic_reference_pilot_v2/`；
执行与审计日志位于 `build/tdt_p2b/dynamic_reference_pilot_logs/`。
工具在本证据目录，实际调用为 `probe.sh`、`experiment.py prepare/run`、
`audit_dynamic.py`、`experiment_v2.py prepare/run`、`review_results.py`。
所有trial目录首次创建，工具拒绝覆盖。重新试验须建立新series；v1的原始错误后处理入口保留供审计。

[manifest.json](manifest.json)冻结本目录、原始series与日志；清单自身不自包含。
可变进度文档不纳入冻结。当前结论足以停止重复并进入失败时间窗分析，**不需要等待实车恢复**。
新车最终CAD/附件包络可在制造完成前审查；老车修好后可提供独立真实几何下的低速闭环证据，
但不能替代新车最终动力学与整机验收。
