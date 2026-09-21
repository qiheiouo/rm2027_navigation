# P2B 持久工作记录

状态（2026-09-21）：**新车八边形参考几何已消除当前 nominal endpoint blocker；
A*、QP 均到点但各一次恢复，静态零恢复门仍未通过。terminal selector 暂缓；航向 A/B 冻结。**
基线 `cb48ba4`。本轮仅采用已有 382/126 mm 参考顶点，127 mm 做敏感性复算，未确认最终 CAD。

完整审查与本轮回传见 [新车几何审查](p2b_new_car_geometry_review.md)：
Spin 本体半径0.339020 m；保留 padding/clearance 后要求0.400789 m，旧 witness距离0.430116 m。
v2目标平台轨迹/停车/静态Spin检查通过，唯一失败门为 no_recovery；2/10目标组试次后停止重复。
QP实际采用14次、安全A*回退2次；没有 endpoint拒收。全局T-DT仍为从新polygon自动推导的
保守圆，尚未完成纯polygon路径准入；占位轮组与最终整车包络也未闭环。

下一步先诊断 snapshot 更新拒收与恢复时序，保留所有安全门；不实施 selector、不调航向。
旧车真实运动仍须独立真实几何安全检查，狗洞专项保持隔离。

以下为此前阶段记录，不能替代本轮验证。

## 2026-09-16 航向实验历史证据

- 87/87 常规回归、统一 sanitizer 链审计和 56/56 core 通过。
- `heading_follow_v1`：原 QP / 航向跟随 QP 首例均 action=6、静态门 false，恢复
  18/22 次。两组均停止重复；没有将旧 series 结果填入本 A/B。
- 航向跟随候选复用已有 `828d4d8` path_aligned 策略，增加连续路径 yaw 和原生
  MPPI 对 orientation 的评分。实际运行参数已核对。
- 行进 yaw-to-path RMS 0.977 → 0.321 rad、yaw-to-motion RMS 1.004 → 0.506 rad；
  整段实际 wz RMS 0.293 → 0.366 rad/s，峰值均 1.0 rad/s，未消除终点旋转/恢复。
- 两组间隙门通过，终点位置/yaw/action 均未通过。候选 16 条端点拒收全部含
  同次快照的 goal blocked 证据，圆模型不等式未因路径 yaw 改变。
- 保留为默认关闭的跟踪候选；一对 A/B 不能证明稳定收益或总体成功率，不推广部署。
- [真实回传与冻结证据](evidence/heading_follow_20260916/validation_20260916.md)、
  [复用/差异/命令](p2b_heading_follow_experiment.md)。

## 工作顺序

由开发者完成实现、验证和复核；Terra 可做独立审查。所有文件在 `/home`。
此前朝向实验结束，不继续在同一系列调参；当时下一项独立讨论用户提出的
`nominal goal + next action + map + robot geometry` 自动推导与选择合法终端状态。
本轮没有实现该机制，也没有改变固定 goal yaw、安全门、控制权或部署默认。

此前 test-only 的离线 SE(2) 连续几何库已有 19 项回归，并在2026-09-16轮统一 sanitizer 中
通过；未接入 Nav2，不构成新的终点合同或闭环通过证据。
旧 v1–v5、endpoint_witness_v1 和用户后来以 cb48ba4 归档的 v4 报告保持不变。
当时规划迁移到可部署候选的阶段估计约55%；本轮未重新估计百分比，不能将它解释为测试通过率或工时。
未部署、push、merge，动态障碍、新设备全负载与实车仍待验证。

## 2026-09-16 终端选择设计检查点

已完成 [接口、复用与算法设计](p2b_terminal_selection_design.md) 及
[离线/交接验收计划](p2b_terminal_selection_validation_plan.md)。先区分普通停放与
完整 Spin 的可行集合；复用已有 mission/狗洞执行权和 test-only pose_geometry。
当时计划下一实现只做离线普通停放/Spin provider 与有限候选选择（现已暂缓），执行准入保持 NotEvaluated；
尚无 selector 测试、真实快照回放或新闭环通过证据。狗洞/堡垒先保留动作接口和缺失项，
不扩展执行器，不同时改控制器。此次只改文档，旧 heading 125 份证据保持不变。
