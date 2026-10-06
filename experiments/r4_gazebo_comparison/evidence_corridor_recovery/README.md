# A26 final 原Nav2恢复行为：Stop并冻结路线

2026-10-06，用户明确授权的最后一轮。修正与协议在trial前提交`8f8d6c80`，只修改experiment wrapper已知native异常永久锁存/记录器终止，不新增速度、fallback、owner、publisher、lease，不改算法/双方参数/原S3场景。

**最终Stop**。baseline 0/5成功、5次contact；R4零contact、5/5通过gate，但仅2/5完整Nav2 goal success，3次目标附近原path输入契约抛degenerate path。安全WAIT→GO信号保留，完整任务稳定性未满足Go，不能把XY靠近goal重分类为成功。本轮结束，不补10、不S4、不修框架/终点/参数、不生产化。

[完整报告](../../../docs/dynamic_navigation/r4_final_corridor_research.md)给出限制、原始方法判定与最终判决。首异常终止的[原cohort](../evidence_corridor/)独立保留，不能合并为10次样本。

- `protocol.json` / `preflight_pass.json` / `schedule.json`：最终授权协议、复用原已通过的空通道可行性依据、5对固定顺序。没有新的S0或S4试次。
- `assets/`：实际同条件map/scene/scenario及两个原冻结配置。依赖仓库既有机械模型/安装依赖，不是可脱离仓库的单文件仿真。
- `failure_classifications.json` / `event_control_excerpts.json`：全部真实contact、R4自身失败；首次native事件/原因/时间保留，终点末plan及末控制拍保存。原输入ring不在此小摘录中，在raw目录。
- `summary.json` / `trials.csv`：10个新goal-started实例，无startup失败或替换，goal成功与gate通过分别计数。到达中位18.764s仅为R4两个成功样本；baseline无成功，不计算速度比。
- `fairness.json` / `corridor_audit.json`：冻结输入检查、共同存活时段内实际配对轨迹和输出图快照。10次count=1，8次owner名字可辨，2次DDS name unknown；不把未知名字伪称核实。退出cleanup -11另列。
- `freeze.json`：最终Stop和冻结边界，不以框架理由延续当前路线。
- `corridor.png`：所有完整保存的轨迹/净空/WAIT、成功样本到达、clear后gate通过；失败到达是缺测，不画成0。
- `wrapper_build.log` / `continue_recovery_classified.py` / `continuation.log`：本轮仅wrapper构建和已知失败后完成原固定schedule的过程。未知类别停止人工查看，不替换失败或调参。

R4五次均在首次native异常后继续消费并安全通过；其中2次Nav2 action success，3次仍按原R4/input错误规则终止。native事件共B0 4次/R4 5次，在原events中仍为controller_failure类型，fatal_to_trial=false且原原因不变；不能把action恢复说成异常从未发生。

R4动态净空中位/最坏.32581/.31493m，WAIT中位2.76s，clear后gate通过5.10s；clear时已前进，因此resume=0不是物理零延迟承诺。1/5两次前后切换，最大回退1.748cm。有效solver1858拍P95 1.305ms/最大3.444ms，native+R4合并P95 33.416ms/最大45.365ms；异常拍缺测不作为0ms。

完整原输入/预测/真值/所有日志/第一次native异常ring和最终失败ring在`build/r4_corridor_native_recovery_20261006`，约47MiB；原首异常终止35MiB仍保留。所有原始机械真值为采样包络，真实contact单列，不宣称连续物理证明或实车效果。代码及证据只在隔离experiment分支本地保留，main不改，不push。

复现入口见[实验说明](../README.md)。显式`R4_RESEARCH_NATIVE_RECOVERY=1`启用本实验修正，默认关闭；不进入生产构建/启动。
