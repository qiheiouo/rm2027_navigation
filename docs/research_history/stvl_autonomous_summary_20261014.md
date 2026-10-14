# RM2027 持续研究总报告（2026-10-14至15）

**总体Modify：动态预测值得有限投入可靠的时间消费与可执行安全输出；复杂HWS/KF、新完整控制器或Temporal MPC迁移没有新增必要性证据。此前把预测精度提升直接当成闭环收益的预期偏高。** 原C83接触没有作为已修复关闭；本阶段收口，停止追加仿真批次，等待评审和新输入，继续保留阶段资料。

用户已授权离开期间自主逐阶段推进和普通额度恢复后续作。本报告汇总该授权之后的八个局部阶段；全在同一Research分支，不自动迁入新车、push/merge、真实运动或启动R5。

## 历史证据支持什么

[140次真实STVL + 原生MPPI研究](stvl_temporal_20261013.md)实际运行旧车低速Humble配置；简单CV在横穿、迎面和可释放窄通道有可复现安全收益。原消费C为14/14到达、12/14通过四项冻结门，时间合同修正后21/21到达、19/21通过。Oracle仍有窄通道碰撞，效率未稳定改善，实际动力学还有额外限制。因此不能说“预测没有信息价值”，也不能把理想预测等价为安全部署。

这些是已有固定软件实验，不是当前新车默认STVL基线的验收。新车保存版默认VoxelLayer；新车CAD、MID360观测、实际制动与MCU控制响应未知，不能宣称正式新车STVL+MPPI已足够或不足。R1–R4、Temporal MPC/NEU及失败因果矩阵和PVT身份继续由[研究索引](../project_structure_and_documentation_index.md)链接，原冻结代码/负例保留，不全量迁入或重复全仓审计。

当前证据最强的瓶颈在消费和执行：预测风险进入了软成本，但没有成为最终动作接受条件；候选安全数量与优化/SG/shift后控制不一致；迟回退、限加速度和停车位置未来占用仍未闭合。这些问题不能单靠更准确预测解决。

## 离开期间完成的阶段

| 阶段 | 实际证据 | 裁决 |
|---|---|---|
| [NoSafe/旧命令/延迟](stvl_lifecycle_hold_20261014.md) | 5物理启动、4有效；原16/16 safe0仍新非零；早制动有物理机会，约100ms迟制动增加停车距离 | Modify，未证明超时造成原接触 |
| [原生重试和失败异常](stvl_failure_contract_20261014.md) | 8次真实固定库调用、6源码语义回放；清fail恢复第二评分，持续坏输入/soft NoSafe仍失败或危险 | 局部Go，在线收益未证 |
| [有限制动时刻](stvl_brake_window_20261014.md) | 两新增物理执行；迟400ms净空门限失败，迟800ms已停车后仍contact | Modify，停车不等于未来安全 |
| [active输入只读合同](stvl_prediction_contract_20261014.md) | 7输入真实validator复算；unused tentative可以误拒绝整包，stale仍拒绝 | 局部缺口，未放宽年龄/几何 |
| [active输入有限闭环](stvl_active_scope_20261015.md) | 5启动、4有效原生Nav2；均硬门通过，横穿慢2.479s，窄通道快0.670s但净空下降/失败仍4 | 没有稳定收益，停止单独过滤工程化 |
| [fresh-coasting关系](stvl_coasting_epoch_20261015.md) | 8对/16validator；一处比较接受合法observed==source，未来/过期/坏confirmed仍拒绝 | 确定性局部Go，未证明导航收益 |
| [诊断导出开关](stvl_observer_export_20261015.md) | 两冻结输入8次native weighted update，开关不改成本/flag/序列数值；无用raw读写可关闭；保存三工程失败 | 局部Go，只是研究可观测性/资源修正 |
| [等时有限闭环](stvl_coasting_live_20261015.md) | 2启动、2有效；均硬门通过却未消费等时；快0.118s、净空降0.115m、失败仍4、safe0仍3，出现101.600ms输出负例 | 没有在线归因依据，结束等待 |

本报告阶段累计12次有效软件物理运行/14次启动：6次执行夹具、6次真实Nav2导航；不把两类合并当某算法的导航成功率。额外native/validator调用及源码语义回放不是导航；observer另有一次编译失败、两次同根SIGSEGV，第四次修复夹具调用范围后通过，未藏入导航分母。原CSV/JSON/SDF/参数/时序和工程失败均保留。

## 已证实机制与未证推测

原C83在2.514s的safe0是300候选、30离散future步的TemporalCritic计数，300评分仍有限。风险成本参与优化；safe0不置fail_flag，也不说明全空间物理不可行。原16/16零safe均输出新非零控制，不能解释成单纯复制旧Twist。最终weighted U、约束、SG/shift与原raw安全身份也不能视为同一个轨迹。

原明显减速3.618s、近停4.578s、接触5.308s，停止位置后来被障碍扫掠。早制动执行夹具停车约0.14815m、动态净空0.26558m；迟约100ms后停车0.21371m、净空0.19433m，仍无接触。迟400ms净空0.04342m低于.047门，迟800ms3.918s已停、4.555s接触。这些有限代理点不确定精确最后安全时刻或侧向全空间物理不可避免性。

以前100.590209ms来自别的成功运行，仅超过100ms0.590209ms，不能证明原C83接触由CPU超时造成。本次新allow_equal c112实测native99.436ms、必要99.846ms、wrapper101.476ms、DDS发布101.600ms；前后NAV更新隔192.110ms，计算窗口采样机器人移动0.04805m。只有一个native点位于超过100ms的尾段，不把空区间距离0视为无运动。无contact，仍没有原碰撞超时因果证明。关闭raw导出不是整个控制线程的实时性保证。

固定库fallback/reset不清fail_flag，后续评分manager短路。只在准备重试后清一次可恢复第二评分；持续坏数据和soft NoSafe不会因此消失。耗尽抛runtime_error绕过server的PlannerException容错；typed源码回放首次失败立即零而保留action容错，不是持续旧危险非零。这些未在线迁入。

unused tentative误拒绝与合法fresh coasting误拒绝是两个不同根因。既有tracker可以在重新命中后保持coasting、miss0、observed==source，严格消费者错误要求observed<source。上一阶段16校验只改合法等时，其他年龄/完整性全部保留。

active闭环现场2→1 anchor都是fresh-coasting，不能归因于tentative过滤。本次等时pilot两组都没有消费fresh等时；strict唯一anchor反而来自observed比source早66ms的tentative。不能把strict1→allow0错误数差异归于未触发的coasting比较。修正的纯输入局部Go成立，在线收益仍未知；没有重新完成A/B/C信息价值实验，也不把日志减少称为预测收益。

## 未完成、停止与下一步

原C83确定性完整闭环修复、NoSafe动作接受、联合安全的已知执行映射、停车后未来占用和新车实物响应均未闭合。没有新C′、安全控制器、真实CAD数字孪生、侧向不可避免时刻或生产部署证据。未来真值没有给C，离线真值仅用于物理评价。

值得保留的最小软件项：合法coasting时间关系、原生重试状态/专门异常合同、显式诊断导出。它们是局部研究成果，没有证明单独处理能解决动态接触；不默认开发完整MPC或追求预测模型精度。

目前不再追加随机种子、重复纯validator或扩大控制器。下一项能改变在线裁决的证据是可重复触发重新关联时间边界的受控输入，以及可信的执行映射/真实制动数据。NoSafe方面若必须新增完整安全控制器或重写MPPI循环，应停止扩张。当前整体Modify，真实车辆Wait，本阶段结束等待评审/新输入；每小时续作在没有实质信息时保持安静。

## Git、资源与恢复

工作区 `/home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014`，分支 `codex/stvl-lifecycle-hold-20261014`。阶段提交：

- 生命周期 `055f29beb74c3df7eaf7cba9e659e2c80e9371dc`。
- 原生失败合同 `f10a521dd107af5486b32192a6357c709702bc0c`。
- 制动时刻 `af39a45bd536a225b7c5ca2904cbde9fb6659ba2`。
- active只读 `63faca0dbafc74ac8d4e44814f499a0b12d88027`。
- active闭环 `e0b1ae02e842cdc813213391a870633cd0a0bee1`。
- coasting关系 `8f367ea0bd53eb114f4dfe8ded22b35b84d225a2`。
- observer导出 `65caadeadde16dbd59e64ef837bffb5cc9499047`。
- 等时闭环：本报告所属提交，完整HEAD从`git rev-parse HEAD`及收口回复核对，避免循环写入同一提交身份。

121个其他heads/remotes/tags与阶段1保护快照对照不变；FAST-LIO gitlink仍69f1070328c431f4c7b8bd586eca074141da3aae。main仍fcd9a05b03c9539b4b5ea44de0f09d30c4468455，origin/main-new-car仍bd6cf689d3d5aca5c8edd430ff760ca5ceb27496，origin/main-old-car仍820179457b70789b0f46697758dbe3d947ec1d4a。主工作区9条无关tracker/surface-member改动保持。完整修改文件清单由`git diff --name-only a880c7dc01cb424ea865e1fa0f33732396078670 HEAD`恢复；只限八个COLCON_IGNORE实验入口、docs报告/索引/检查点，无生产src改变。所有阶段本地中文提交，提交后clean，无push/merge/部署。

八阶段owned cache总约199.5MiB；active_scope_cache约158.2MiB含四组完整raw candidate/control/native，是唯一未全入Git数据，不能删。新coasting_live_cache约25.8MiB含两组完整native/control亦必须保存，精选约1.41MiB，40成员SHA/六物理指标复算成功，未包含全部1ms native，不能冒称仅精选即可复算全加速度门。observer约3.44MiB，精选0.46MiB，生成构建/fixture输出可重建。其他阶段唯一native流同样保留。

当前可用约3.10GiB；最近两运行低于75MiB新cache上限，守卫持续≥3.03GiB；观察到磁盘最低约3.094GiB。先前active阶段因旧observer无条件导出将预算60→110→200MiB修订的负例完整保存。没有清理历史研究证据、Git冷备、原始录包或其他worktree。

回退只读查看上述阶段commit或a880，恢复各实验evidence归档到本轮可重建目录；不要reset正式或旧研究工作区。[持续检查点](research_continuation_20261014.md)是续作权威入口，各README提供运行/恢复命令。普通额度未恢复等下一次检查，不用重置券或购买额度；新输入前不制造重复实验。
