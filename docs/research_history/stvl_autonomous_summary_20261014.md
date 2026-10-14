# RM2027 持续研究阶段总报告（2026-10-14）

**整体裁决Modify。动态预测只值得有限投入简单CV的可靠消费和执行合同；复杂预测、全面替换MPPI或重启Temporal MPC仍没有新增必要性证据。** 原C83关键机制已解释，但接触未作为已修复关闭。用户授权离开期间自主逐阶段推进，所有工作仍在单一隔离Research分支，不迁入正式新车。

## 本次推进取得了什么

| 阶段 | 可核实结果 | 对决策的影响 |
|---|---|---|
| [无安全候选与延迟](stvl_lifecycle_hold_20261014.md) | 原16/16零safe仍新非零；4有效执行夹具、1工程失败；早制动可避免代理接触，延迟增加停车距离 | 主要问题是风险信息没有成为动作接受约束及迟停车，不是单纯预测不够准 |
| [原生重试/异常](stvl_failure_contract_20261014.md) | 8次真实固定库调用、6组源码语义回放；一处清零恢复第二评分，持续坏输入/soft NoSafe未改善 | 有确定性局部软件修正机会；不是碰撞修复证据 |
| [有限制动时刻](stvl_brake_window_20261014.md) | 两次新增物理运行；晚400ms净空0.04342m，晚800ms已停仍contact | “输出零并停车”不足以保证停车位置安全 |
| [active预测合同](stvl_prediction_contract_20261014.md) | 七输入真实validator复算；两次unused tentative使整包拒绝，active子集合法；stale仍拒绝 | 输入消费范围与tracker全包合同存在局部不一致，尚未在线适配 |

本次真实软件物理运行共7次启动、6有效、1起步前类型失败；没有在线修改MPPI后的导航闭环实验。额外原生评分及validator探针不是导航试验，不能把它们累加到闭环成功率。空场反馈203ms只延迟observer，不构成MPPI反馈延迟对照。

## 证据最强的结论

1. 原C83在2.514s的safe0是TemporalResearchCritic在优化之前记录的300候选离散净空计数；风险成本进入优化，但数量零不触发fail_flag，仍有300有限评分。它不是原生全invalid或全空间物理不可行。
2. 危险继续前进主要来自新控制，随后smoother/执行限制导致实际制动需要时间。原始持续明显减速3.618s、近停4.578s、物理接触5.308s；停止位置后来被扫掠。
3. 相近前缀的原代理早制动停车距离0.14815m、动态净空0.26558m；迟约100ms后停车距离0.21371m、净空0.19433m，仍无接触。不同启动和20Hz相位参与差异，不能把全部6.56cm归CPU延迟。
4. 再晚400/800ms分别出现门限失败/实际contact。800ms组3.918s停车、4.555s首次contact；没有target限加速度违例。有限点不能推导精确最晚安全时刻。
5. 100.590209ms来自另一次成功运行。它超过100ms仅0.590209ms；完整旧NAV保持120.232936ms。原C83缺同步入口计时，**没有证明超时造成原接触**。
6. native reset/fallback保留fail_flag会跳过重试评分；耗尽runtime_error绕过server的PlannerException容错。typed容错的源码语义是首次失败立即发零而暂保action，不是持续保持旧危险非零命令。
7. 两次后期预测锚点错误与未评分tentative有关；删除active时间错误或stale会危险地放宽输入合同，本轮没有这样做。没有active轨迹时safe300不是“预测证明所有候选物理安全”。

## 还没有得到什么证据

没有证明完整原C83确定性复现或修复；没有新车CAD尺寸/实车制动/MCU执行延迟；没有完整执行映射保证；没有C′、生产安全过滤器或新部署控制器。未证明侧向避让或全空间不可避免时刻。重试清零/typed异常/active范围候选都尚无修改后在线导航通行收益。

先前Prediction Value Test、140次真实STVL + MPPI研究及失败定位保存在[前序索引](../project_structure_and_documentation_index.md)。本次没有重新跑完整A/B/C信息价值实验，不将局部控制合同改善归因为预测精度提升。复杂HWS/KF预测的新投入优先级继续低于已有CV消费链的可靠性。

## 最小下一步

只保留三个明确局部选项：已有MPPI重试状态清零；已知优化器耗尽的专门异常合同；existing prediction adapter/consumer的active输入完整性合同。逐项隔离验证，不能把多个修正混合后称为某一个的信息价值。

NoSafe→动作接受仍需当前速度、完整制动和停车位置未来占用；旁路联合检查执行映射Unknown尚未解决，不能直接启用C′。如果下一步必须新增完整安全控制器、换MPC或重新做仿真框架，应停止扩大。真实车辆部分Wait。

## 文档、Git和资源

分支：`codex/stvl-lifecycle-hold-20261014`；工作区 `/home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014`。阶段1 `055f29beb74c3df7eaf7cba9e659e2c80e9371dc`、阶段2 `f10a521dd107af5486b32192a6357c709702bc0c`、阶段3 `af39a45bd536a225b7c5ca2904cbde9fb6659ba2`；当前完整HEAD使用 `git rev-parse HEAD` 核对，阶段4为本报告所属提交。[检查点](research_continuation_20261014.md)是续作权威入口，保存后续阶段身份。

新增内容限四个 `experiments/stvl_*` 独立入口、四报告、总报告、检查点和文档索引；全部实验COLCON_IGNORE。无正式配置或生产src改变。完整路径清单可用 `git diff --name-only a880c7dc01cb424ea865e1fa0f33732396078670 HEAD` 恢复。

121其他heads/remotes/tags在阶段1后保护核查不变；最终再次对比初始ref快照。FAST-LIO gitlink仍 `69f1070328c431f4c7b8bd586eca074141da3aae`。main `fcd9a05b03c9539b4b5ea44de0f09d30c4468455`、origin/main-new-car `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`、origin/main-old-car `820179457b70789b0f46697758dbe3d947ec1d4a`均未触及。主工作区原有9条无关dirty路径保留。

原始缓存约11.9MiB（按各目录实际收口复核），Git精选证据约3MiB量级。空场/reference完整1msnative有未纳入Git的唯一时序数据，必须保留；其余精选原始输入/物理观察/真值/参数/contacts有恢复归档。三个小构建目录及解包副本可重建。没有删除历史raw、bundle、冷备或其他worktree。

当前可用约3.30GiB，始终维持3GiB保护线。所有阶段本地中文提交，阶段提交后clean，无push/merge。回退只读使用阶段commit或a880，不reset正式工作区。按小时的本聊天续作已启用：额度未恢复等待普通额度，不用重置券/付费；有有价值的独立局部工作才继续，无新信息保持安静。
