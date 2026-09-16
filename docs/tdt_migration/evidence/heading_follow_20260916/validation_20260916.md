# P2B 航向跟随路径 A/B：2026-09-16 实际回传

**已完成并冻结 `heading_follow_v1` 最小 A/B。朝向跟踪改善，但没有消除终点 recovery；
两组导航均失败，静态门均未通过。保留为默认关闭的实验候选，不推广部署。**

本轮没有实现 terminal feasibility / terminal selection，也未将离线 SE(2) 几何库
接入规划器。未来“nominal goal + next action”机制只记为独立的后续设计输入。

## 版本、来源与验证

- 试验源码 `4ad918f`（完整 SHA 见 `tested_source.json`）；分支
  `experiment/tdt-planner-phase2`。先前离线几何检查点 `a7e9d42` 只加入测试库。
- 全部开发、依赖、构建、运行目录位于 `/home/wpie/worktrees/rm2027_tdt_phase2`。
- 镜像 `rm2027_navigation:humble`，ID
  `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。
  MPPI `1.1.20-1jammy.20260607.083249`，使用原 Omni 控制器。
- 已审计并复用 `828d4d898d61444ed2a5fb419e02add3421b4ced` 的
  chassis-heading `path_aligned` 策略；[原策略留档](reused_chassis_heading_policy.md)、
  [源文件哈希](reuse_audit.json)。没有重复实现速度控制器或导入该分支其他功能。
- 增量构建通过。5 个 CTest 入口、**87/87** 通过：core 56（含新增离线几何 19、
  路径 heading 4）、插件/实际 MPPI scorer 6、原工具 18、新角速度统计 2、ROS 消息 5。
  最终 [测试日志](tests_03.txt)。
- 独立 ASan/UBSan 全链审计通过，58 个编译单元受审计；OSQP/OsqpEigen 实际加载
  均来自本次统一插桩前缀。求解器生命周期首测通过，**56/56 core sanitizer** 通过。
  [链审计](chain_audit.json)、[核心日志](core_tests.log)、[完整日志](sanitizers.txt)。
  原始目录 `build/tdt_p2b/sanitizer_runs/chain_20260916T085509Z.JZntPD`。
- 原四份 profile 哈希未变。A profile 字节与原 QP 相同：
  `5718ec4045ba56634208a13d7f2bcee0b768ae40ab2c4110a4a01a66af4b4e67`；
  B `4a17fedbdaf9be9785adfcbf4ed2555f381a7c7d498e824998f01d774441373f`。
- 保留集分别 166、238、272、438、114 份以及主工作区三个未提交配置文件均核对
  不变（[运行后哈希复核](preservation_after.json)）（不同集合可能重叠，不相加当作独立文件数）。用户的 v4 未提交报告未修改。

## 实验改动与实际生效证据

T-DT A*/QP 共用一个缺省关闭的 `experimental_path_heading`。只计算 yaw、不写 XY；
弧长前后各 0.10 m 的弦估计切线，首 0.40 m 与末 1.00 m 用 smoothstep 过渡到起终
姿态，短路径各段至多半长。该固定实验设定不表示时间域角速度/加速度约束。
两个后端均实际通过开关前后路径 XY 逐点相等、精确请求端点和致命格拒收回归。

B 复用既有 PathAngle 正向偏好、Twirling 和 PreferForward 参数，增加
`PathAlignCritic.use_path_orientations=true`。完整差异、来源、命令和固定安全门见
[实验说明](../../p2b_heading_follow_experiment.md)。其他控制、规划、场景和安全参数未改。
这是**朝向策略组合** A/B，不能把差异单独归因于某一个 critic 或 yaw 生成器。

发送目标前，A/B 各自通过运行参数服务逐项核对：A 59 个 controller、13 个 planner
参数；B 66 个 controller、13 个 planner 参数，均见原始 `runtime_parameters` 事件。实际库 scorer 的测试证明：开启开关后，
同 XY 的不同 yaw 轨迹得到不同成本；接近当前路径末端时该 critic 仍按原逻辑退出。
这证明 MPPI 读取了 orientation，不代表硬性姿态锁定。

实际 15/19 版路径全部 yaw 有限、末端 yaw=0。最大相邻 yaw 变化 2.820 → 0.407 rad；
末端变得连续，但短路径上的 yaw 梯度仍可能较大，跨重规划的时间连续性未得到保证。
审查工具的“末端 XY 比特相等”字段为 false：两组最大差值均为 **8.88e-16 m**，
是已有世界/网格坐标换算舍入，不是目标容差或主动选点。原请求始终 `(4.3,0,0)`。
独立数值复核见 [端点与参数记录](plan_endpoint_numeric_review.json)。

## 同版两次首例的结果

| 指标 | A 原 T-DT QP | B QP + 航向跟随 |
| --- | ---: | ---: |
| 有效记录 / 原始重算一致 / 参数核对 | 是 / 是 / 是 | 是 / 是 / 是 |
| preflight status / 含往返延迟 ms | 4 / 34.13 | 4 / 54.02 |
| 导航 action status | 6（失败） | 6（失败） |
| recovery 次数 | 18 | 22 |
| 终点位置误差 m | 0.4240 | 0.1936 |
| 终点 yaw 误差 rad | 3.0730 | 1.7695 |
| 采样车体最小 clearance m | 0.09132 | 0.10422 |
| 车体线性插值间隙下界 m | 0.08351 | 0.10050 |
| padded 线性插值间隙下界 m | 0.04026 | 0.05763 |
| 行进 yaw-to-path RMS / median rad | 0.9770 / 0.7249 | 0.3213 / 0.0541 |
| 行进 yaw-to-motion RMS / median rad | 1.0036 / 0.8406 | 0.5062 / 0.1520 |
| 实际最大 / 时间加权 RMS wz rad/s | 1.000 / 0.2925 | 1.000 / 0.3660 |
| 实际最大绝对 Δwz/Δt rad/s² | 2.000 | 2.000 |
| 实际最大单次 Δwz / wz 总变差 rad/s | 0.0800 / 9.4942 | 0.0800 / 17.6415 |
| 命令最大 / 时间加权 RMS wz rad/s | 1.000 / 0.3588 | 1.000 / 0.3896 |
| 命令最大绝对 Δwz/Δt rad/s² | 1000.00 | 1317.50 |
| 命令最大单次 Δwz / wz 总变差 rad/s | 1.0000 / 22.7776 | 1.3175 / 39.3993 |
| cross-track RMS m | 0.06298 | 0.04460 |
| 导航仿真时间 / 墙钟 s | 36.025 / 38.860 | 39.245 / 42.853 |
| 新鲜停止命令门 | 未通过 | 通过 |
| 静态门总体 | **未通过** | **未通过** |

wz 的“实际”来自 Gazebo ground-truth odom，命令来自 `/cmd_vel`，不混为同一量。
命令为无 Header 的 Twist，按记录器 `/clock` 到达时刻差分；1 ms 级连续消息可产生很大
命令导数，不能解释为真实车体角加速度。两组同时间戳差分排除数均为 0。
两组实际角速度的采样间隔均为 0.04 s，命令频率/间隔及原值全部保留。
统计含 recovery 和末尾 0.5 s settle；A 命令有效区间比整段轨迹短，停止命令不新鲜，
未用补零或缩短分析窗口把该门改为通过。行进朝向指标只取位移速度 >0.10 m/s 的
343/422 个样本，并对当时的 plan revision 插值；两组参考路径不完全相同。

A/B 都通过 0.05 m 车体间隙及 padded 无接触检查；两者 action、位置、yaw、零恢复
均失败，A 另有停止命令门失败。两组首例均为有效功能失败，所以均不运行 2–5。
汇总可审计，**不是完整 20 试次或重复统计矩阵**，也没有向其中填入历史 baseline。

## 几何拒收与人工复核

A/B 分别 15/16 条端点拒收独立审计通过，missing witness=0：

- A：start blocked 6 条，goal blocked 10 条（1 条两端均阻塞）。
- B：start blocked 6 条，goal blocked 16 条（6 条两端均阻塞）。
- 所有 26 条 goal witness 的 raw 致命闭格距离均约 0.430116263 m，原圆模型要求
  0.452781708 m。加入 path yaw 不改变这个不等式；拒收逻辑仍有效。
- 两组另各有 1 条 snapshot 更新拒收。未把端点日志条数当作 recovery 数，未把
  所有恢复都归因于单一日志。raw snapshot 的 free claim 仍未被独立完整地图证明。

[原始拒收审计 A](baseline_endpoint_witness_audit.json)、
[原始拒收审计 B](path_heading_follow_endpoint_witness_audit.json)。

已实际查看 [轨迹/姿态/wz/间隙图](heading_review.png)：两组均从南侧绕障；B 行进时
车头更接近路径方向，终点附近仍出现旋转和恢复停滞。末版路径的 yaw 过渡改善可见。
实际 wall spawn 两条均 exit=0；发布 costmap 后续存在南北墙致命标记。首条 map 在
场景初始化时尚无墙体标记；两组发送目标前最后一帧发布地图也未标出墙体区域，
运动后才出现标记。该覆盖时序被保留，未改为等待全图标记，也不能据此宣称
preflight 使用了完整已知墙图。两组实际感知/重规划序列不同，限制单次因果归因。
发布地图仅用于感知覆盖检查，不冒充同次 planner raw snapshot。
本轮未重新做完整 TF 所有权审计，TF/launch 源码及 fixture 哈希一致属于配置证据。
日志未出现 LIO -11 或共享记录器异常。未进行 CPU/内存专项采样或性能优化。

## 结论与冻结边界

1. **已解决的部分**：默认关闭的 yaw 生成、末端过渡、实际 MPPI orientation 消费
   链路已实现并测到响应；本例行进朝向误差和路径相邻 yaw 跳变降低。
2. **未解决的部分**：整体角速度 RMS/总变差没有降低，峰值未下降，recovery 没有
   减少，action/终点静态门仍失败。圆模型的不可行终点没有被朝向策略改变。
3. **是否值得继续**：值得保留为后续独立跟踪候选；本轮没有证据支持将其作为终点
   recovery 修复或部署方案。停止对本系列调权重/补跑，不宣称稳定改善或算法淘汰。
   只有一对、未固定 MPPI 随机种子，不能估计总体成功率或显著性。
4. **下一项边界**：先以本报告和 manifest 冻结结果，后续才单独设计动作约束驱动的
   terminal feasibility/selection。普通导航与 Spin/狗洞/占领的终态语义不同；
   此次没有改变它们，也没有放松固定 goal yaw 测试或任何安全门。

未部署、未运行实车/动态障碍/完整 LIO+双 STVL 负载，未 push 或 merge。
旧诊断成功/失败仍保留，既不外推到本提交，也不覆盖。

## 可复现证据

- [最终审计汇总](aggregate_final.json)，原始
  `build/tdt_p2b/runs/heading_follow_v1/aggregate_final.json`。
- 两组目录分别为上述 series 下 `baseline/tdt_qp_1`、`path_heading_follow/tdt_qp_1`：
  metadata/profile、launch/observer 日志、trajectory/plans/commands/events/costmap 原始流，
  summary、container/docker exit 均保留。各原始文件 SHA 见汇总与本报告旁 manifest。
- 命令脚本 `tools/heading_ab.py`；图表重现脚本保存在
  `build/tdt_p2b/heading_ab_logs/review_heading.py`，被 manifest 绑定。
- 开发期间先发现两个问题：新参数客户端在 Humble 不可用；新增 XY 用例错误地假定
  栅格路径严格直线。改用原生 GetParameters 服务和开关前后逐点对照后，第二次
  发现新增 ROS yaw 用例未显式设置 quaternion.w=0（默认 1）。修正用例构造后
  87/87 通过。[第一次](tests_01.txt)、[第二次](tests_02.txt)失败均保留；没有改变
  已有安全断言或用多次运行挑选结果。
- 验证结果归档曾被自动审批服务因模型容量不足暂时拒绝，未执行写入；只读确认
  来源/目标后审批恢复。已有原始日志始终在 `/home`，没有绕过审批。
