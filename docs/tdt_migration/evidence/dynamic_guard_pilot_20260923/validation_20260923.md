# P2B 动态安全：已知扫掠区命令过滤首例

状态（2026-09-23，Asia/Shanghai）：**过滤器确实在底盘接收前截断了通往移动障碍扫掠区的命令，QP 首例的目标八边形连续移动障碍间隙下界为 0.0905 m；车辆未到点，90 秒超时并恢复 9 次，现有动态门失败。** 此项只能说明本固定夹具的一次仿真防侵入效果，不能当作通用动态安全节点或部署结论。

[独立审计](pilot_audit.json)与原始 series `build/tdt_p2b/runs/dynamic_guard_pilot_v1/` 是结果入口。此前[停车和扫掠影子检查](../sim_stop_probe_20260923/validation_20260923.md)保持冻结。本轮没有改运行时 `src/`、`experiments/`、部署默认、BT、MPPI、footprint 或安全门；没有实车、push 或 merge。

## 输入和实施范围

基线与候选均为 QP、目标 `(5.6,0,0)`、固定相位 0、13 m 全局窗口，逐字节复用上一轮已接通 controller odom 的 QP profile。候选仅使用隔离 launch copy 把 chassis stub 的订阅从 `/cmd_vel` 改为 `/cmd_vel_guarded`；独立节点在 velocity_smoother 之后、底盘之前读 `/cmd_vel` 和 ground-truth odom。四份 launch copy 的逐项差异见[启动链差异](launch_diff.json)；[输入哈希](../../../../build/tdt_p2b/runs/dynamic_guard_pilot_v1/inputs.json)在试次前冻结。

候选从已有 SDF 机械行程和箱体尺寸自动生成完整扫掠矩形 `x=[4.675,5.125]`、`y=[-1.225,1.225] m`。每条命令用当前参考八边形与 0.03 m padding、原 0.05 m 本体门检查下一响应期及假设制动轨迹的保守位移上界；无有效 odom 时发零。响应保持 0.15 s，线减速度 1 m/s²、角减速度 2 rad/s²是**仿真候选模型假设**，不是最坏时延或已验证新车物理能力。完整扫掠只对本机械约束夹具成立。

## 两份实际试次

| 指标 | 原命令基线 | 扫掠过滤候选 |
| --- | ---: | ---: |
| preflight / action | 4 / 4 | 4 / 5，90 秒超时 |
| recovery | 14 | 9 |
| 终点 XY / yaw 误差 | 0.0916 m / 0.1484 rad | 1.8962 m / 0.5671 rad |
| 目标八边形对实际移动箱体连续间隙下界 | 平面 oracle 拒绝认证 | 0.0905 m |
| padded 八边形对移动箱体连续间隙下界 | 平面 oracle 拒绝认证 | 0.0487 m |
| 现有动态门 | 失败 | 失败 |

基线原 observer 记录 action 成功和 14 次恢复；但独立 Gazebo oracle 在 36.438 秒首次检测到车体模型四元数水平分量范数 0.001962，超过既有 0.001 平面阈值，共 2 个样本超阈。[原审计异常](../../../../build/tdt_p2b/runs/dynamic_guard_pilot_v1/baseline/tdt_qp_1/audit_error.txt)保留。因此本轮**不声称基线的完整动态 polygon 间隙已通过复核**，也不放松非平面阈值来补 A/B 表。

候选的独立 oracle 证据完整：本体与 padded footprint 对实际移动箱体均无采样重叠，线性位姿插值下界为上述数值；但 action 未成功、终点误差和零恢复门均失败。不能把“停车安全”写成“导航成功”。`docker_exit=1` 是观察者按现有静态检查返回失败，不是容器启动失败。

过滤器记录 383 次准入、1753 次扫掠拒收。第一次拒收在 26.957 秒，当时上游约输出 `(vx,vy,wz)=(0.160,0.498,-0.177)`；记录的 `/cmd_vel_guarded` 与底盘转发在约 26.958 秒均为零。[命令链原始流](../../../../build/tdt_p2b/runs/dynamic_guard_pilot_v1/guard/tdt_qp_1/observation/command_chain.jsonl)具有 `/cmd_vel` 2136、`/cmd_vel_guarded` 2136、底盘转发 2137 条；额外的启动零命令与跨订阅回调顺序不允许逐条延迟归因。车辆最终停在约 `(4.085,-1.140)`，仍在目标外。

## 检查及后续判定

过滤核心的离线专项测试[4/4](tests.txt)通过；旧停车探针清单 29 项哈希核对一致，见[保护核对](preservation.json)。没有重新运行全链 sanitizer，也没有做多相位动态验收。当前失败说明仅把完整扫掠区放在最终命令拦截器中，会使原规划器持续请求穿越该区域，产生超时和恢复。下一项应在**新 series**让全局与局部规划地图也看到同一 SDF 派生扫掠区，同时保留本过滤器作为最后一道实验性检查；先跑预定 QP 首例，审计地图实际标记、命令链、完整实际间隙和 action。若仍失败，停止重复并定位对应环节。即使成功，也还需独立静态门和多相位试验；新车 CAD、真实制动与传感器时延仍需后验。`accepted_for_deployment=false`。

[本轮清单](manifest.json)冻结工具、输入、原始试次和实际结果；清单自身不包含自身哈希。
