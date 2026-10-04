# 同运行晚期固定时刻三层诊断

## 背景与假设

goal+2s输入在01全体安全，不能检验排序；v4仅5条不安全。完整物理执行诊断已显示01取消后仍继续控制并破门，不能据此指控早期周期聚合。此阶段检查接近任务后半段时原采样、排序、输出分别是否失效，先定位层，再决定唯一算法改动。

假设：固定goal+6s的原300条rollout仍可能存在安全候选，但排序或原输出在完整3s预测上失败。如果0安全，结论限定Sampling/当前可达安全覆盖不足，Ranking不解释。不以完整物理结果或接近程度挑周期，不扩大batch、调temperature/权重/footprint。

## 固定方法和决策

仅使用原v4与01完整capture，按每运行第一个(sim_ns,cycle_id)≥goal_sent+6s选择一次。既有three-gate算法、7标准critic/rawV1、regularizer、原bounds/SG、两native时间口径、raw203、body≥.05/padded>0、正推进、首选安全+质量≥.5均保持。记录延迟与数据覆盖；无足够未来3s直接输入FAIL，不替换周期。

prepare最小变化只是将固定时间偏移作为显式参数（默认2s保持早期协议），不增加recorder/receipt/TF/身份门。准备/输入冻结后再读rollout安全标签；此前已读完整物理标签，且同运行强相关，**不称盲测或独立holdout，不补齐CA两试次协议**。

顺序：Gate1原候选数与安全正推进；有safe才Gate2原排序/Top-K/质量/各critic及总分；Gate3原online/single-safe/oracle-safe-only的weighted/bounded/filtered整段传播、静态及推进。层失败的优先诊断：0safe→A；有safe而原排序FAIL→B；排序PASS而online输出FAIL→C。多个失效同时保留，不通过优先规则删去后续事实。

只有B失败且已有因果四源条件满足，才将已冻结CA-full公式在同输入作一次诊断，不调公式/单位/支持/时间范围，不进入运行集成。若B不失败不追加CA评价。若A失败，下一实验必须先判当前状态/固定首列导致不可控失效，或同N分布缺少可行控制；不以0safe证明不存在安全控制。

## 数据、结果、限制、下一步

待源码及参数输入冻结。新算法源、输入/参数/镜像hash与结果保存到本文件/evidence。原mirror/native数值容差保持1e-5/1e-3/5e-5，float采用语义/数值/门一致，未新增跨设备byte门。未来truth只离线标签，main/部署配置/TF保持。

源码/prepare`1f67682`。仅按固定时间选择：v4 cycle40/28.106s，threshold28.102、延迟.004s；01 cycle219/28.227s，threshold28.224、延迟.003s。输入SHA256分别`e720611cf1eb9ec462e632d3bf0057c08e8fe6300cf1f560ecf0fd8dfcf7cc17`、`d6e32d1a32bec89f7927eb7c7111a9f653a85e74aa64b9c1051758b346751c43`；参数`22e0425b54e1499d6538bce52937ce9aae3824c1268154ab0fcd07fa95ac1bdf`、`7a15a6b93d8a26ed201bf10d2703c00ff5d800e6306c10d7fd37374bffcf2da5`。完整3s未来覆盖到31.106/31.227，原tail已覆盖；不选择尾段32.694s破门邻域。

如果两输入全部门通过，只说明这两个晚期周期未复现算法失效，停止在此数据调参；新的算法实验须另行假设。任何层失败都保留，选择最小针对该层的机制研究，不扩展工具。

## 原三层结果与条件输入

原native评价`7620f20`。v4 Gate1 FAIL：0/300动态及联合安全、static collision0、最好body gap0；不解释Ranking或输出。01 Gate1 PASS：58动态安全、57联合安全且57都推进、static collision5。Gate2 FAIL：最好安全第15、Top1/5/10/30安全0/0/0/1、质量.116506，top#79不安全。Gate3原online filtered两口径body/padded均0；原整段bounds越界.005405。原生pose/7标准score/weighted/before误差0，after/聚合误差≤5.96e-8，数值门通过。

01single-safe原最低成本候选经SG后两口径也失败；safe-only current-first body .085789通过动态几何，但offset计划body .034218失败，且bounds越界，不能把单条安全候选或oracle赋权当作输出保证。这是新固定晚期周期的三层诊断，旧“安全候选≠聚合安全”结论仍CLOSED，不重新计为机制创新。

按B失败条件已构造01最新四源27.919/.985/28.051/.117s，跨度.198s，全部因果几何构造条件通过。source input SHA256 `197b7bf9a1e28c0cd2e4343ff4dea0c11f65f3478026366f5f53f85799f4c932`，参数沿原`f2d97785237b59214b69ff0143e044dc8e603278547cbf640a887035a7eeef38`。后续H1真值误差仅作离线报告，CA是否构造由这些在线条件决定，不按H1标签选择另一个source/cycle。

source评价`5ab3bcd`，H1 PASS：最大位置误差.017131m、速度误差.114955m/s，estimate(-.582564,-.005559)、truth(-.467744,0)。同公式CA a=(1.173889,.406666)m/s²，risk范围.0129624–.0190362；host Python3.12.3/NumPy1.26.4准备。kinematic input SHA256 `19da12a30339c96044cbaf8638d44e3b5b4eb002e2bb07edc5aff6f261407a10`、parameter `a51fe7ee5b4a3746f7dffce3f822d65014b11ac0a5f53483c66c314f9f55a905`。CV文件随原prepare产生，仅保留不新增评价；本条件只评价冻结CA。CA scoring input `df076366548c8224a9d3806d43274379492720ae134ebfb1530e6d7520b845a5`、原参数`33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`，评价前冻结。

## CA算法结果及阶段决策

CA原生评价`327ac6a`。Gate2 PASS：top#280安全且推进.546175m、最好安全第1、Top1/5/10/30安全1/3/5/6，安全质量.786672（基率57/300=.19），总分/风险AUC .588333/.588694。单项风险只DIAGNOSTIC_ONLY，没有把均匀质量当实际MPPI赋权。

Gate3仍FAIL：filtered current-first body/padded .050251/.012182，推进.610867；returned-offset-plan body/padded .032983/.002983，推进.639865。static/bounds均通过；body .05门不可放宽，两口径都通过的要求不变。原生附加评分核对最大6.10352e-5<原1e-3容差，不新设byte门。

**新晚期输入已证明固定CA可以提供有效总排序信号，同时输出安全仍不足；不进入runtime。** 不能以current-first刚过门掩盖返回控制计划FAIL，也不能重调CA/单位/temperature救援。v4零安全属于A；01剩余主要问题为C输出与评分功能的一致性/聚合，而不是无安全候选。

[全部输入、原生结果及CSV](evidence/late_fixed_time_three_gate_20261001/manifest.json)、[v4 Sampling](evidence/late_fixed_time_three_gate_20261001/v4_summary.json)、[01 CA](evidence/late_fixed_time_three_gate_20261001/01_ca_summary.json)。

下一阶段先另行预登记C最小对照：固定当前失败周期及已有正对照，检验把同一CA风险评价在原bounds+SG+offset后的候选输出计划能否改善最终返回控制计划。保留原采样/硬V1/标准critic/正则/聚合/安全门；不改预测模型或引入新planner。该映射可直接复用原Nav2控制函数，没有新记录器；仍不能保证非凸风险下的平均控制安全。v4的A需要另行当前状态/可行见证实验，不与C同时改变。
