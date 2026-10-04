# Temporal MPC 研究冻结记录

2026-10-04，Asia/Shanghai。依据用户本轮“这个方案也冻结”的明确请求，停止
当前动态障碍预测＋Temporal MPC路线的继续实现、调参、新试次及自动续跑。
状态为 **Research Frozen / Not Accepted for Deployment**。
这表示研究停在当前可恢复检查点，不表示候选已经通过部署冻结验收。

## 冻结身份和范围

- 独立分支：`experiment/temporal-mpc-main-20261004`。
- 直接main基点：`d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`；未从旧三轮分支派生。
- 最后实现及实验提交：`2d95b02648723eca52373a81aed70c4cf72077d7`。
- annotated冻结tag：`research/temporal-mpc-frozen-20261004`，指向包含本记录的冻结文档提交；不覆盖已有tag。
- 冻结`experiments/temporal_mpc/`、本路线文档、已提交无损证据、预启动源码/二进制身份与本地ignored实验产物。工作树保留，不删除原始数据或构建结果。
- 正式main、默认MPPI、T-DT/NEU已迁移规划前后端、旧研究tag以及原工作区dirty成员证据和未知core保留；无merge、push或部署。

本次只写冻结和历史归档文档，没有启动算法/物理实验，没有改变控制行为。
本轮及旧文档的“下一步”、示例启动命令、研究日历和聊天建议均为历史记录，
不能作为继续工作或自动执行的授权。恢复须用户另行明确要求并重新确认范围。

## 已完成的限定成果

静态map经T-DT搜索/路径简化/完整raw-cell走廊认证，压缩为reference/corridor；
MPC只负责下一1.5s全向平移、动态避障和有界控制。默认single QP保持，实验
portfolio最多左右/等待3个局部参考、共享390迭代/15ms预算。20Hz执行、15节点、
warm start、少量活跃障碍、全量重验及worker40ms/native10ms门均保留。

真实Humble Nav2同时加载`FollowPathMPPI`和`FollowPathTemporalMPC`，在同一上层
NavigateToPose任务中经controller selector双向切换；上一解新输入重验、有界减速
及MPPI回退链已做工程检查。实际Gazebo扫描、tracker v2、测量odom和源时刻TF
消费已核验，未来真值只供独立审计。共同保护不提供未经标定的连续安全保证。

最新主机/Humble各130项不同测试通过；真实DDS受控clock的8项检查与真实Nav2
14类故障门通过。测试、源码快照、binary副本、消息记录、失败和修正均已归档。

## 未通过的门和冻结原因

旧SLSQP与实时QP离线均有迎面接触或任务停滞；后续实际Gazebo首轮横穿双方
接触。引入共同保护后的多轮试次虽没有正Contact消息，却仍未形成稳定完成任务
和连续安全证据。目标排除、速度投影缺陷、未来净空veto、模糊时间合同拒绝及
接收连续性问题分别有记录，不能合成“solver不好”或“预测方向无效”的结论。

最后迎面对照双方40s取消；候选仅实际1个MPC通过周期，随后新测量重锚的
未来终态净空-14.953267mm而回退。1,998个请求全部current，未复现旧时间拒绝。
候选最终命令接收gap95.474ms、B0 Controller gap82.702ms超过75ms观察门，
B0另保留一次worker40ms截止拒绝。生产端约50ms节拍不能替代执行端证书。

可见表面锚点不是完整物体中心/隐藏形状；完整D与CV运动误差是两种不确定性。
当前没有留出数据校准的概率支持、短时实际响应误差包络、硬实时保证、比赛
部署净收益或公平多场景统计。dynamic_acceptance=false，部署候选冻结=false。
研究冻结不将这些字段改成true。

## 保留与阅读入口

- [最新结果和限制](temporal_mpc_timing_results_20261004.md)。
- [完整阶段进度](temporal_mpc_progress.md)、[架构](temporal_mpc_architecture.md)。
- [最新完整性复核](temporal_mpc_timing_integrity_20261004.json)、[冻结身份机器记录](temporal_mpc_freeze_20261004.json)。
- [全研究历史总文档](/home/qihei/rm2027_navigation/docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md)。

全部八个实现/验证提交与九组核心证据目录在此tag中保留；原提交中的接受撤回、
缺TF进入门失败、命令空档、启动/清理异常和原始日志空白均不覆盖。
原始大数据与源码以冻结Git路径为准；历史阅读副本不冒充重新执行的实验。
