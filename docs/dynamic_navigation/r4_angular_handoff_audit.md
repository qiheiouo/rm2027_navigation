# A20：既有 owner 的角命令过渡适用性

Research，只读源码与已完成的 A16/A19 记录。基线 `c744d4e7`。

假设：已有 smoother/输出所有权可以容纳 R4 的零角速度目标，但能否不改 A19
未来恒定朝向模型就直接切换，需要区分源测量、内部 last_cmd、实际发布/transport 与权限。
最小判定：固定 Humble1.1.20 源码、原 A11/A12 接口，以及 A19 同一 S1/S2 200 个窗口；
仅按原 20Hz、2rad/s²、OPEN_LOOP、scale_velocities=false 条件计算零目标过渡，
统计需要几拍、角度/支持差异与 terminal yaw 条件。不调用 Follow/solver/ROS，也不生成实际命令。

若源码提供真实历史与一致的过渡且 A19 rollout 条件成立，只提议最小适配；
若过渡产生未来 yaw 变化，Modify 最小消费模型，不能伪造零历史或交给新 owner 掩盖。
若必须重建导航链才能获得基本响应，停止工程化。

本轮不修改 A05/A08/A09–A12、main、public v2、runtime profile、controller、smoother 或串口。
角历史来自此前最近 actual_output receipt 的只读代理，不能作为 owner send/last-applied grant。
相位取 0/50ms 理想边界，持续零目标且每 tick 准时是条件假设，不是实测停止时间或 lease PASS。

## 现有接口能复用什么

| 边界 | 源码/接口 | 审阅结果与 R4 处理 |
| --- | --- | --- |
| 原 controller 选择与回退 | Humble `ControllerServer::computeAndPublishVelocity/updateGlobalPath`；冻结 R3 `integration/controller_subtree.example.xml` 与 `selector_node.py` | server 每次只调用所选 controller。异常在原 failure_tolerance 内发零，超时中止 action；不会自动调用 MPPI。R3 已有 ControllerSelector/FollowPath 选择模式和 health→ID 薄映射，但正式 profile 未启用。复用原切换责任，不把抛异常当正常 MPPI 回退或角过渡 |
| 最后平滑状态 | Humble `VelocitySmoother::smootherTimer/applyConstraints`、protected `last_cmd_` | OPEN_LOOP 以 `last_cmd_` 为当前值，20Hz 限幅/限速后保存，再执行 deadband 和 publish。内部值不能当物理 applied 或 transport write-success；没有外部 atomic history 接口。若后续确需观测，应在此原 seam 导出证据，不能创建另一个 history/output owner |
| 实际 ROS / stub 输出 | `src/rm_chassis_interface/src/chassis_interface_stub.cpp::handleCmdVel` | `/cmd_vel` receipt 后按原限值转发 `/simulation/chassis/cmd_vel`；mock feedback 是命令转发值，不是实测 odom。A16 实际运动仍须看原 odometry；receipt 只用于本轮观测代理 |
| 实际串口 | `src/rm_serial_driver/src/serial_transport_node.cpp::handleCmdVel/writeFrame` | 缓存并编码当前命令，steady timeout 归零，原 100Hz 写帧。`write_all` 返回成功/失败，但未公开带原来源的最后成功命令记录；ROS receipt 或 smoother last_cmd 不能代替写成功/物理执行。继续复用此唯一 transport，不新增串口输出 |
| A09/A10 当前几何、期限与权限 | `src/rm_navigation_execution_adapters/{current_geometry,lease_fence}.cpp` | 原纯值接口已有三轴旋转几何和最终候选接纳；ProposalReceiptGate 消费外部 fence，admit_for_send 返回值，没有 scheduler/send 或实际 history 记录。能复用能力，不代表 live host 已完成 grant、history 或来源接线；本轮不扩合同 |
| A11/A12 与 A19 的入口 | `src/rm_r4_nav2_controller/src/{controller,proposal_bridge}.cpp` | A11 仍创建固定 FollowSolver，并核对标准 Pose/Twist 与 raw source；A12 capture 仍用旧 fingerprint。A19 wrapper/新身份无法直接绑定，不能把内部 value 抽出并伪装旧结果；本轮保持冻结，先验证最小消费模型 |
| 终点朝向 | Humble `ControllerServer::isGoalReached/setPlannerPath`，原 SimpleGoalChecker yaw tolerance=0.20rad 与 native MPPI GoalAngleCritic | goal checker 使用 path 最终姿态和实测 pose，不使用 R4 free-s。恒定未来 yaw 的 Follow 不能生成任意终点转向。保留原目标/原 native 控制责任，不放宽 checker，不新增 goal manager/旋转控制器 |

源码依据：固定 Nav2 **1.1.20@a097086719c88f781aa59788eca29ac6ca5e56db**，
本机既有只读缓存 `/tmp/r4_nav2_owner_readonly_a097086`；未下载、导入或编译。
[原 smoother 225–323 行](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_velocity_smoother/src/velocity_smoother.cpp#L225)、
[原 controller 465–609 行](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_controller/src/controller_server.cpp#L465)。
原来源/license 仍见 [A07 已有 intake](r4_owner_adapter_design_sources.json)，本轮没有新外部模块。
冻结 R3 只读 `04291a41` 的 selector 示例，没有运行或迁入其 worker/guard。

正式老车 profile 仍为 controller/behavior 上游→velocity_smoother 唯一末级 ROS publisher→
所选 serial/stub。A16 仿真使用原 Humble Nav2、smoother 和 stub，R4 没有输出权。
protected last_cmd 是适配位置，不能把“有一个缓存”写成“已有完善实际输出仲裁证据”。

## 最小条件分析

A16 使用 `nav2_phase1_5_mppi.yaml`：OPEN_LOOP、scale_velocities=false、20Hz，
angular accel/decel=±2rad/s²、deadband=0。老车 STVL profile 的这些角参数相同，
平移限值不同，不把仿真平移数值泛化到老车。仅审阅这两个相关 profile。

在 target wz=0 持续生效且准时 tick 的条件下，每拍原实现使
`w_next = w + clamp(-w, -0.1, 0.1)`。需要的拍数是 `ceil(abs(w)/0.1)`；
首 tick 相位取 0 或 50ms，归零时刻为 `phase + (ticks-1)*50ms`。
目标抵达即减速是理想条件，不包含当前 solve/transport/jitter，实际可能更慢或被新目标改变。
原 smoother 是 wall timer，本条件计算假定 wall 与 ROS/model 时间按 1:1 推进，
没有根据 bag receipt 反推或证明真实 timer 相位/clock witness。

| 200 个原动态窗口的条件结果 | S1（40） | S2（160） |
| --- | ---: | ---: |
| 非零历史 receipt 代理 | 40 | 160 |
| 归零需要超过 1 tick | 34 | 142 |
| 即使首 tick 立即发生，归零仍超过 75ms | 30 | 95 |
| 首 tick 等 50ms，归零超过 75ms | 34 | 142 |
| 零目标→归零时刻 median，首 tick=0 / 50ms | 150 / 200ms | 100 / 150ms |
| 同上 max | 350 / 400ms | 350 / 400ms |
| 首 tick=50ms 的未来 yaw 变化 max | 0.15915rad | 0.14951rad |
| 同条件 swept support 单侧扩张 max | 0.03514m | 0.03146m |
| 固定 measured body velocity 1.5s 的端点差 max | 0.11293m | 0.10268m |

source→epoch 使用原 held-measured-twist 表达式；future 使用上述命令过渡假设，
再与 A19 future wz=0 比较。shape 支持 extrema 直接复用 A17 已有离线函数。
端点差采用固定 source measured body velocity；不是 A19 的全 controls、真实轨迹、
物理误差证书或碰撞结果。shape 数值是 mechanical support 条件变化，不是障碍净空。
即使下一 tick 即时令小 wz 归零，相位未知也不能默认整段未来 yaw 恒定。

这些时间说明单次旧来源不能一边等待任意长过渡、一边续成新的 normal proposal。
**它们不证明 75ms 本身不可行：** 每个新控制拍仍可重新 acquisition，并保持其原始期限；
过渡与 source lease 是不同责任，不能人为把上一提案完成/发送时刻当新起点。

终点朝向只作适用性观测：200 个 epoch 派生 yaw 中 112 个超过原目标 yaw=0 的
0.20rad tolerance（S1 16，S2 96）；条件归零后为 118 个（19/99）。
这些都不是 terminal pose，因此不统计为 R4 goal failure，也不证明 R4 从零命令启动时会发生同样偏航。
原 native goal result 附近的 odom yaw 为 -0.01390/-0.07850rad，原 native 导航仍成功；
不能把这些 native 成功转记到 R4，也不能用 free-s endpoint 替代原 goal checker。

## 判决与最小后续

**Modify：原输出所有权与零目标平滑可复用，但 A19 的恒定未来 yaw 无法表达从非零历史开始的过渡。**
没有必要新增 owner/arbiter/MPPI；也不能仅换一下 A11/A12 类型就取得真实接纳。
A19 的 value 结果保留，runtime behavior / closed-loop 仍未通过。

必须区分两种入口：

1. 从真实零历史启动 R4：A19 的受限命令条件可能成立，仍需真实 history 与行为证据；
   本轮 MPPI 轨迹的非零历史不是“R4 从静止启动一定失败”的证明。
2. 从 MPPI 非零角命令中途切入：可以研究原 action/selection 的先停止再重新 acquisition，
   也可以在同一 Follow 里消费已知的角过渡。不能靠 exception zero、虚拟零或另一个 publisher 获取入口。

优先下一个有限 Research 候选：**保留 45 个 `(vx_body,vy_body,s_dot)` 决策，
将已知角命令过渡作为条件输入，改变逐 stage 的 world 映射与 shape/static support，
不优化新的角速度决策，不修改 predictor/动态权重/OSQP 参数。**
已知 `theta_k` 下，位置关于 body velocity 仍是线性映射，可以复用原同一 QP。
source measured twist、命令 history、query pose、未来角 profile 必须保持不同身份。
段内旋转支持必须覆盖，不能只把 A19 的固定 support 换一个初始 yaw。

最小验证应先使用原 200 个输入的条件角 profile 与受控 hold→clear，比对零 profile
是否退化回 A19、动态响应是否保留、静态界与 solver 是否仍可用；相位假设和命令证据
不应冒称已被 runtime 验证。若没有消费收益就停止，不继续补 production contract。
通过这一步后才考虑有限 runtime shadow；A09–A12/current admission、grant、transport
仍是以后 Integration 的复用边界，当前不编码其新接线。

[复现与原始逐行条件结果](../../experiments/r4_angular_handoff_audit/README.md)，
[200 行结果摘要](../../experiments/r4_angular_handoff_audit/evidence/summary.json)，
[A19 值结果](r4_aligned_follow_adapter.md)。
