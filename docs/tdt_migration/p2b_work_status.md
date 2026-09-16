# P2B 持久工作记录

状态：**航向跟随最小 A/B 已完成并冻结；P2B 静态门仍未通过。**
试验源码 `4ad918f`，分支 `experiment/tdt-planner-phase2`，实际执行 2026-09-16。

## 当前证据

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
当前朝向实验结束，不继续在同一系列调参。下一项才独立讨论用户提出的
`nominal goal + next action + map + robot geometry` 自动推导与选择合法终端状态。
本轮没有实现该机制，也没有改变固定 goal yaw、安全门、控制权或部署默认。

此前 test-only 的离线 SE(2) 连续几何库已有 19 项回归，并在本轮统一 sanitizer 中
通过；未接入 Nav2，不构成新的终点合同或闭环通过证据。
旧 v1–v5、endpoint_witness_v1 和用户 v4 未提交报告保持不变。
规划迁移到可部署候选的阶段估计仍约 55%，只是阶段判断，不是测试通过率或工时。
未部署、push、merge，动态障碍、新设备全负载与实车仍待验证。
