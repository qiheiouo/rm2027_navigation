# P2B 精确端点拒收证据

本轮以 `e3b2754` 为基线，继续 `static_v5` 的端点问题。由开发者完成实现、构建、
测试与实际仿真。诊断系列使用 `endpoint_witness_v1`，不补写旧系列，不改变 MPPI、
BT、目标 `(4.3,0,0)`、footprint、padding、clearance 或静态验收门。

## 实现边界

`SweptCircle` 在原来的比较式 `distance <= required + 1e-7` 内记录命中的格子。
它来自本次 core 收到的不可变 raw snapshot，采用双精度原点、分辨率和局部端点。
254/255 或边界格的 required 为 padded 外接圆半径加 clearance；253 非边界格仅
禁止中心接触，required 为 0。只记录每个被拒端点遇到的首个阻塞格，因此它是
**足以解释拒收的证据，不一定是最近格**。不再次读取实时 costmap。

诊断作为 `endpoint_witness_v1={...}` JSON 附加到已有 PlannerException/rosout，
插件同时记录同一请求的 start/goal yaw。越界或非有限端点没有伪造碰撞格；
legacy offline 输入仍使用历史语义。正常候选的 A*、QP、采样、扫掠检查、预算和
更新后拒收流程不变。没有新增 ROS 控制接口或自动重试。

`audit_endpoint_witness.py` 独立用点到闭 AABB 的距离重算证据，并核对外层插件
请求与内层 core 元数据，拒绝错误 cost/边界、阈值、距离、非有限值和缺证据。
`free` 仅为 core 原始结论；工具不凭单格证据宣称全图自由或矩形/旋转可行。

## 本轮执行命令

在 `/home/wpie/worktrees/rm2027_tdt_phase2` 执行。每次命令输出保存到新的
`build/tdt_p2b/endpoint_witness_logs/`，sanitizer 使用自己的不可覆盖目录。

```bash
pkg=experiments/tdt_planner/rm_tdt_planner
bash "$pkg/tools/p2b_validation.sh" build
bash "$pkg/tools/p2b_validation.sh" check
bash "$pkg/tools/p2b_validation.sh" profiles
bash "$pkg/tools/p2b_validation.sh" sanitizers
# 以上通过且相关源码已提交后，串行运行两个诊断首例。
P2B_SERIES=endpoint_witness_v1 bash "$pkg/tools/p2b_validation.sh" run tdt_astar 1
P2B_SERIES=endpoint_witness_v1 bash "$pkg/tools/p2b_validation.sh" run tdt_qp 1
python3 "$pkg/tools/audit_endpoint_witness.py" \
  build/tdt_p2b/runs/endpoint_witness_v1/tdt_astar_1/observation/events.jsonl \
  --output build/tdt_p2b/runs/endpoint_witness_v1/tdt_astar_1/endpoint_witness_audit.json
python3 "$pkg/tools/audit_endpoint_witness.py" \
  build/tdt_p2b/runs/endpoint_witness_v1/tdt_qp_1/observation/events.jsonl \
  --output build/tdt_p2b/runs/endpoint_witness_v1/tdt_qp_1/endpoint_witness_audit.json
P2B_SERIES=endpoint_witness_v1 bash "$pkg/tools/p2b_validation.sh" summarize
```

这是针对拒收原因的诊断系列，计划仅运行两个 T-DT 首例。汇总仍使用原矩阵工具，
预期会列出未运行的基线和重复试次；实际结果必须另记，不能用预期填表。
若新首例失败，保留失败并停止该组重复。`static_v5` 的 Navfn/Smac2D 通过记录
只属于其历史提交；不填入本系列。

## 后续架构判断依据

若精确证据证明固定目标与 raw lethal 闭格在当前圆模型下相交，重新搜索或切换
QP 无法产生满足同一约束的路径。不得把 `goal=blocked` 当作端点整格问题再次
绕过，也不能忽略终点的独立扫掠检查。

须分别评估地图闭格近似、目标需要的姿态、允许的旋转与局部控制行为。只证明
目标 yaw 下矩形能放下，不能推出无约束旋转安全。姿态约束路线需要位置/yaw
联合的几何验证和控制器可执行约束；目标容差路线则改变精确端点合同，不能
隐式启用。本轮只提供事实诊断，尚未启用这些接口变更。

## 已执行结果（2026-09-15；2026-09-16 归档）

代码提交 `25a5a40`，59/59 常规测试、统一 sanitizer 审计和 33/33 核心测试通过。
两个 T-DT 首例均 action=4，恢复分别为 5/10 次，静态门仍失败。3/9 条端点拒收
全部获得同次输入证据且独立审计通过，目标到 raw 致命闭格距离为 0.430116 m，
小于圆模型要求 0.452782 m。未因本次到点而宣布算法改善。

完整[实际报告](evidence/endpoint_witness_20260915/validation_20260916.md)与
[下一阶段终点合同设计](p2b_terminal_contract_design.md)已保存。后者尚未实现。
上述 trial/output 路径已使用，不可直接重复执行覆盖；后续验证须新建 series。
