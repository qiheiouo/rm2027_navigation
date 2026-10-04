# Temporal MPC 离线第一轮预登记

2026-10-04；设计检查后、场景执行前记录。源码与参数哈希由每次运行manifest保存。

固定自身机械矩形0.65×0.60m，padding0.03m、动态margin0.02m；独立physical
clearance门0.05m。控制dt0.1s，碰撞网格0.05s，oracle网格≤0.01s且有连续
区间reserve。horizon主试次1.5s、5 acceleration blocks；限速/加速度见架构。
SLSQP最多45 iterations、最多2 seeds，离线配置预算0.5s；另外记录严格20Hz
50ms预算超期，离线预算放宽不代表实时门通过。

固定场景12项：10项核心矩阵，加near-face与analytic course sine诊断。
观测模式前10项与正弦提供完整当前polygon/当前速度用于控制可行性隔离；
near-face只提供近面。名义模式以近面锚点+完整D=1.6970562748477143m圆，
不按结果缩小D。场景时间窗12s、到点位置误差≤0.12m/航向≤0.1rad/速度≤0.05。
有接触则保留首接触后停止该离线试次，不删除失败。短遮挡丢帧区间2.0–2.6s，
源epoch保留，超过0.4s必须拒绝；不是oracle输入给控制器。

主矩阵24个试次，不调权重/足迹/时间窗让失败通过。wait_then_pass另比较
current_only/future_union机制消融；它们不叫STVL、R1或R2。主矩阵之后固定
wait_then_pass观测模式扫查0.5/1.0/1.5/2.0s。所有试次使用相同oracle。

正弦来自当前Gazebo场景的几何与target参数，但离线是解析运动、无joint PD、
无激光检测/KF误差/控制传输延迟。其结果不与旧Gazebo试次直接算净收益。
controller看当下snapshot，不看future truth；oracle独立读取完整真实box运动。

M1通过意味着核心契约/约束验证成立并且至少已知CV横穿可在模型内过物理任务门。
20Hz预算、真实感知或广泛场景失败保留为下一阶段风险；不宣布优于B0/R2。
若基础CV也不进展，则只作失败归因并停止扩展，不能改几何/放松oracle门。

## 执行后修正登记：制动保持周期

首轮独立回放发现小速度制动在collision_dt提前归零，实际执行却保持整个dt，
可能越过零点。`pre_fix/`保留原全部输出及拒绝记录；其接受判断作废。
修正只将制动在control_dt计算后重复到collision grid，保持原全部参数、
几何、权重、场景、时间窗和oracle。增加全周期命令一致性/禁止制动反向测试，
重新执行同一矩阵、消融和horizon扫描。deadline分支依赖主机时间，不能将
manifest原`deterministic_model_outputs=true`理解为跨主机确定性。


## 用户补充后的第4轮执行身份（补记，不冒称预注册）

职责改为既有T-DT静态前端压缩→局部reference/corridor→Temporal QP。
固定yaw/wz=0全向平移，20Hz执行、1.5s/15节点、solver 15ms、core 40ms、
400迭代、4相关track，全量最多64track重验。几何/速度/加速度/物理0.05m门
保留。支持1–2s配置；本轮固定1.5s，不继续以0.5s作为实时候选。
正式矩阵是12场景×2几何及1个静态map绕行，共25试次；run开始写出manifest
固定该参数和源码身份。完整结果与失败在单独realtime目录，不覆盖旧SLSQP。

实际T-DT path/corridor来自保守静态图，物理map包含边界occupied cell条带；
不是只替换solver的同输入配对实验。不准据此声称超过B0/R2，MPPI切换请求
只留shadow证据、不执行handoff。计时control_elapsed包括每周期window生成，
预测、QP组装/求解、输出重验；map规划另计。模拟计算暂停，真实wall执行门
另行取得。测试增加/parser加固后的来源差异如实登记，runtime QP未变化。
