# R4 A10：原责任的 fence / acquisition lease 值适配

2026-10-05，Asia/Shanghai。承接 A07–A09，用户授权持续开发和有限实验。本阶段扩展既有被动 execution-adapter 包，不新增节点、action server、mission FSM、动作选择器、最终 publisher 或 MPPI。public v2 与正式运行配置保持。

## 复用依据与剩余缺口

main mission 的 cancelNavigation/cancelSpin 发出 async cancel 后立即清句柄；不能证明旧动作已停止。固定 Humble1.1.20 的 `SimpleActionServer::get_current_goal_id()` 在原 recursive mutex 下读取 action UUID，可以复用该原接口。`BtActionNode::halt()` 等待 cancel 和 result，但超时只记录错误后返回 IDLE；该默认路径也不是无条件成功屏障。A10 只读取已安装头文件，不导入或重写这些模块。

原 Twist 接口不能表达 acquisition、active execution、clock generation 或实际命令证据。所需最小值适配为：原 host 给出的唯一 active fence、源端本拍 atomic proposal、原输入责任的 replay/remaining-budget 校验，以及原最终变换后同一 candidate 的 current geometry 合成。fence 必须由原 profile 的动作切换责任显式提供；消息到达、action UUID 或 producer identity 本身不签发权限。active fence 来源和撤销确认仍待实际接线验收。

## 预登记合同

- `ExecutionFence`：明确的 authority instance/epoch、producer instance/ID、action execution、base frame、clock domain/generation、plan/body/limits revision 和 active 标记。只能由原 host 的注册/切换责任更新；同 epoch 内容改变、回退 epoch 均拒绝。收包不改变 fence。
- `ProposalPacket`：完整 fence 身份、cycle sequence、normal/revoke 类型、Twist、本拍 acquisition/send ROS epoch、源本地 steady 已耗时与剩余预算。最大 lease 固定75ms，`elapsed+remaining=75ms`；发送/acquisition 的 ROS delta 须符合已验证源 clock drift 上界。包是一个值，不靠到达顺序拼 Twist 与 metadata。
- `TransportWitness`：原接收点自己的 ROS/steady epoch、local clock/owner instance，以及由原 host 验证的 clock domain/generation、offset/drift/transport delay 上界。默认未验证，直接拒绝。接收 deadline=`receipt steady + source remaining - conservative transit delay`；绝不使用 `receipt+75ms`，不比较跨进程裸 steady。live clock/delay 验证尚未实现；合成 witness 的测试只是合同检查。
- 输入 adapter 只保留 fence 与 sequence high-water，不缓存或发送速度。拒绝 duplicate/reorder；同 fence 下 cancel/revoke 后不能靠更大 cycle 恢复 normal，必须由原 host 换 fence。owner restart 需显式重新注册，不能从陌生包自行恢复。
- 最终 admission 检查当前 active fence、原接收 deadline、local clock 身份、实际 candidate/identity、A09 Certified 的 map/body/epoch/完整两分支证据，以及几何处理预算。结果 valid-until 取 original lease 和允许 held interval 的较早终点。lease 不足50ms时只给较短区间；不会为凑整拍续期。

接收与最终发送由原 owner 的既有同步责任串行调用同一 gate。新合法提案、revoke、inactive 或不合法 fence/clock 观察会使旧 ReceivedProposal 不再获准发送；无第二 command cache。A09 最小补上实际 frame/base frame、limits revision 与 processing budget 回显，避免把另一配置的几何结果套到当前候选。版本仍是原 host 的可信配置合同，不冒称传感器或 actuator 认证。

原 owner 和 transport 仍负责执行/退化，adapter 不生成 zero/brake、不更新 smoother rate state、不消费 future prediction。normal lease 到期后退化未认证，不以零速度冒称物理制动证明。固定 current map/actual tracking tube 的 A09 限制继续适用；本阶段不能声明75ms真实串口反应、完整物理安全或无间断 native MPPI fallback。

## 预定有限验收

固定 Humble C++17 构建/安装与同范围 sanitizer；actual bounds/current geometry 组合；原 acquisition 75ms、40ms solve/50ms owner 相位反例；duplicate、reorder、old execution normal/zero/revoke、restart、fence change、clock unknown/reset/skew、delay、expiry、same-candidate、map/body/version、processing/held budget 负例。不得调宽15/40/75ms或几何合同。记录源码、失败与保留核对后再进入原 host 接线。

## 有限结果

13 个 lease/fence GTest、13 个当前几何 GTest 与19个原 provider GTest 在固定 Humble 下通过；三组 CTest 的同范围 ASan/UBSan 通过，最终安装后的 C++17 下游调用通过。clock-offset 扣减、旧 normal/zero/revoke 排队、owner restart、同 epoch 变化/回退、clock reset/rollback、撤销已持有提案、实际限幅后命令的同一证据及期限截断均有负例。

固定调度反例保留 acquisition0 / compute40 / receipt42 / owner50 / original deadline75，下一提案 acquisition50 / compute90 / receipt92 / owner100。第一期限没有改成 receipt117，与下次发送之间仍有25ms空档。这是确定性合同计算，没有新的独立执行进程或 timer 测量。

初次构建把 colcon 的全局日志选项放在 build 子命令之后，调用失败；修正位置后通过，日志保留。首次通过后补齐 frame/limits/processing-budget 绑定，最终重建同范围测试；没有更改时间/几何/物理门限。

实现见 [lease_fence.hpp](../../src/rm_navigation_execution_adapters/include/rm_navigation_execution_adapters/lease_fence.hpp)，来源/源码/日志摘要见 [A10 清单](r4_lease_fence_adapter_sources.json)。现有 action/BT 边界对照固定 [SimpleActionServer](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_util/include/nav2_util/simple_action_server.hpp) 和 [BtActionNode](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp)；验收只读安装头文件，未导入上游实现。

当前只证明在显式可信 witness 下的值合同，没有 live active-grant、clock synchronization、transport bounds、原 owner 接线、真实75ms反应或物理闭环证据。下一步把最小 R4 值求解映射到标准 Controller 的原调用/返回类型，以 typed cycle token 绑定 provenance；继续默认关闭与有限检查。host/owner 条件未满足时不给 normal 输出。
