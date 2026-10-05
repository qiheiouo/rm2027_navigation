# R4 A12：原 host 本拍结果到原子 proposal 的最小映射

2026-10-05，Asia/Shanghai。继续用户授权的隔离开发/有限实验，承接 A08–A11。扩展默认关闭的 Controller adapter，不新增 publisher、topic、线程、grant issuer、owner 或 MPPI。正式 host/owner接线仍关闭。

标准 TwistStamped会丢失 solver provenance；A11 typed result保留了本拍来源，但尚需在原host发布边界确认标准返回值、同token/result、原acquisition与当前fence完全一致，再生成 A10 的单个 ProposalPacket。mapper是原host可调用的值函数，不自行执行或发令。

实现前已登记：在compute前捕获 owned cycle的现有input fingerprint、原steady acquisition、ROS epoch和原host给出的fence；不从publish时刻创建新acquisition。Follow仅公开已有validate/fingerprint函数供共享调用，不新增数学/solver/frontend。mapper检查result/token/input digest、native Twist全部轴/frame/stamp、源deadline=acquisition+75ms、15/40ms结果预算和当前fence；源发送remaining由本地steady真实已耗时计算。源ROS clock/domain/generation/drift必须由原host显式验证；默认未知则拒绝。fence字段对应actual profile的证明仍是原host责任，capture不签发权限。

有限检查：标准plugin→typed result→mapper→既有ProposalReceiptGate→A09 actual candidate geometry→原send admission的值调用；old token/fence/input/native值、重复来源、延迟发布、deadline rebasing、未知/reset/ROS delta与solver failure负例。真实graph/active grant/clock监控、原smoother/serial原子发送与输出独占仍未验收；不把合成clock witness当作live证明，不延长75ms，不接实车或大规模实验。

## 接口与最小变更

- `SourceContext::capture(BoundCycle, ExecutionFence)`：原host acquire/bind边界调用；持有本拍原input、receipt、path、map、limits、body六项摘要及原acquisition身份。不是grant issuer，不能把inactive或其他host的fence转成有效权限。
- `map_source_proposal(context, typed_result, native_return, current_fence, SourcePublishStamp)`：标准compute与同token `take_result` 后调用。缺typed成功结果、native值不符、fence切换、超预算、未知/reset clock或expired acquisition均返回不可用原因。
- `ProposalPacket::provenance`：六项摘要与command/source/deadline同一个值对象。R4 mapper总是填入；原生来源可省略。A10 gate对存在的摘要作格式核对，ReceivedProposal与最终admission完整保留，不使用独立metadata topic事后拼接。
- Follow只导出已有输入validation/fingerprint；标准Controller算法、T-DT provider、OSQP kernel和public v2未变。没有增加wire消息、topic、publisher、线程、fallback或输出owner。

源发送时 `source_elapsed + remaining = 75ms`；receipt扣除保守传输上界，最终send deadline取原lease与本次current geometry期限的最小值。steady时间只在本地计算耗时；跨进程合同由显式时钟/传输witness表达，不能直接比较两个进程的裸steady epoch。这里没有实现或验证真实witness生产者。

## 有限结果

固定Humble/Nav2 1.1.20容器，无网络、设备或正式Nav2 graph。普通与ASan/UBSan检查构建均通过：

| 范围 | 最终GTest数 | CTest组 | 结果 |
|---|---:|---:|---|
| A05/A08消费与Follow值库 | 35 | 5 | PASS；最后provenance补充未改变该包源码，保留此前本轮日志 |
| A09/A10共用execution值适配与原provider | 46 | 3 | PASS；包含新增malformed provenance拒收且不消耗正常sequence负例 |
| A11/A12标准插件与源mapper | 20 | 1 | PASS；新增9个mapper/组合合同用例 |
| 安装后C++17下游、默认OFF且无插件注册 | — | — | PASS |

共101个不同GTest用例，每种构建各通过一次；不把历史各阶段重复执行累加为新场景数。最终测试XML failures/errors/disabled均为0，无skip。消费包另外两组为既有canonical CDR fixture和固定soft数学参考，仍只证明有限接口/数学合同。

合成完整值调用保留源原生返回值，同时对平滑后的actual candidate `.02 m/s` 作current admission，证明两个值可分别追溯且source deadline不刷新。延迟到acquisition后60ms的发布只剩最多15ms，75ms即拒收；typed失败或host零值不能伪装为正常R4 proposal。

首轮编译通过但有两个测试代码ROS消息隐式初始化警告，显式构造后重新构建/执行受影响范围；原日志另存，没有通过放宽时间、几何或数值门限处理。泄漏检测关闭、系统ROS库未instrument；这不构成完整系统sanitizer、WCET或物理安全证据。日志、XML、源码hash及保护核对见 [A12来源清单](r4_source_proposal_bridge_sources.json)。

## 本次停止点

用户返回后允许在有必要给结论的位置停止。A12完成后停止自动扩展运行接线；[返回验收结论与最新十二项复用矩阵](r4_return_checkpoint_20261005.md) 记录完整状态。

现有原host尚未实际capture/bind/compute/take/map，原owner尚未consume/admit/send。active grant/取消确认屏障、actual clock/transport witness、完整本拍state/TF/raw-map/last-applied与真实tracking误差界，以及平滑/限幅/编码后原子发送仍是下一阶段条件。非零speed limit仍unavailable；native MPPI选择不能抢占已阻塞compute，25ms正常提案间隙仍存在。不得从此次结果推出闭环、无间断fallback、75ms实物执行保证或B0收益。
