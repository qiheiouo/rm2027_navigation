# Worker时序与提案老化诊断登记

2026-10-04，起点ca89b527，仍直接main派生的独立实验分支。
输出build/temporal_mpc_timing_20261004；原dirty/main/冻结refs和旧失败全部保留。

本轮只增加时序/原始请求诊断与采集后的模型检查，不改QP/有限候选、native v2
重锚/停止投影、几何/TTL/预算/selector回退或公共消息。StateRequest wire格式不变。
worker严格100ms且不接受未来请求的现有门不放宽；精确记录本地判定clock和有符号
age，将future/stale/frame/invalid分开。所有callback disposition（含duplicate、
no causal snapshot、silent）在独立worker_timing实验topic记录，避免覆盖有效solver
identity或将未求解回调算成solver输出。原预测回调与周期外frontend阻塞也记录。

原生health增加实际request publish前/后monotonic、publish时ROS clock及原始epoch/
generation，另记录proposal接收monotonic。worker记录callback start/check/solver/
publish/end与ROS clock、完整原始请求有限字段；DDS源/接收时间仅若可用则记录。
跨进程monotonic差仅限同一Linux主机/隔离容器；不能把差值唯一分为网络/DDS/
executor排队。DDS时间为单独system epoch，不与ROS/steady时钟混减。

提案老化只读工具将旧完整记录中的原worker轨迹、在同一旧公共模型推进到新的
评估epoch而保持原相对控制网格的轨迹、以及完整native移位/投影/停止尾分别检查。
后两者都是明确反事实，不能充当执行、传播plant或独立可加原因。保留原生已记录
实际状态/slack校验；不补过期输入、不重复coast，相对检查网格保持1.5s。

工程门：精确signed clock边界、duplicate/no causal路径诊断、真实DDS请求/消息info、
完整native构建与原14类Nav2故障门、分解实际已记录末端拒绝与负对照。
工程完成冻结后，只做新open_long迎面B0→portfolio各一次新禁网容器物理诊断；
两者都strategy=portfolio以匹配shadow计算负载，sim10s/40s保持，进入门依旧先审计。
登记名head_on_b0_01/head_on_portfolio_01（新目录，不覆盖上一阶段）。
运行时不并发重检查，不按结果调参/挑选重复。物理目的为新timing字段和拒绝身份，
不能将单次结果声称统计净收益。动态接受、部署冻结仍false，正式默认MPPI不变。
