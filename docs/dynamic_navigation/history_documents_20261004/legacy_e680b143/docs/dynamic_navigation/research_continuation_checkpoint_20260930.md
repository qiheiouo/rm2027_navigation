# 动态避障研究持续工作检查点

本文件是追加阶段后更新的恢复索引；历史实验 result 和 evidence 不覆盖。**最新状态：用户要求先收尾并换思路，现有研究路线已暂停，自动续跑`rm2027`已PAUSED。** 此要求优先于此前持续至额度用完/恢复后继续的授权；不再自动启动研究，只有用户明确要求回到这条路线才恢复。历史记录中的“下一步”是当时计划，不是待执行队列。

## 必须保持的约束

最新用户方向纠正（2026-10-01）优先：见 `research_direction_reset_20261001.md`，基础设施默认冻结，禁止新增法证/recorder/identity/byte-exact等体系；仅明确阻塞三层算法判定或当前capture的最小工具修复允许。当前运行v4完成后直接按固定无安全标签选择的新近场holdout报告Sampling→Ranking→Aggregation，每阶段至少算法机制PASS/FAIL；数值采用既有冻结阈值/语义和最终门一致。后续工具问题统一notes，不扩展微小工具主章节。旧CLOSED结论不反复验证，三层机制不同时改，oracle只离线，main/部署/安全约束继续。

- 仓库 `/home/qihei/rm2027_navigation`，仅 `codex/dynamic-differential-risk`。
- 接手基线 `e067c3757e2272128a6af6f015526f893321e5f6`；main 保持 `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。禁止 merge、push、部署或 formal 配置改动。
- 遵守 `docs/development_workflow.md`、architecture、TF/topic 合同；保持单一 TF 发布者和低耦合。
- 不修改线上 critic，不改变 footprint/padding/body gap 0.05 m/padded gap > 0/raw CostCritic 安全门。未来真值仅离线标签。
- 每实验先预注册、固定输入，保存源码提交及输入/参数/镜像/二进制 hash；旧 evidence、失败结果、未知用途文件全部保留。
- 不重复以单纯 batch、temperature、窗口、中心、历史残差概率或权重调参代替研究。

## 已完成与当前断点

**最终暂停检查点（2026-10-01，用户要求收尾）**：最后实验`standard_output_cost_alignment_20261001.md`全部执行完成，执行commit `b3f706508f6422c44ef772c6886f01abc054d036`。H0/数值门两输入PASS，early Gate2 PASS、late Gate2 FAIL（top1原raw unsafe，safe mass0.493066）；两输入Gate3最终filtered两口径均PASS，整套协议FAIL。没有新算法运行/部署通过方案。完整输入/source/binary/log/result归档到`evidence/standard_output_cost_alignment_20261001/`，archive SHA256 `d5fa265be3d6f9187938aeff8052502cf73aa9963f930d44cd6d6ee60b943917`；全部输入文件hash和摘要归档核对通过，不覆盖旧证据。

收尾只更新文档和封存已有结果，**无待prepare、evaluate或补跑任务**。Docker无运行容器，相关实验进程无；持续研究goal保持PAUSED，heartbeat `rm2027`更新为PAUSED。磁盘约11GB可用，不清理未知文件或历史数据，`core`继续保留。分支`codex/dynamic-differential-risk`，main `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`不变；tracked研究改动仅docs/dynamic_navigation和experiments/dynamic_prediction_v1，正式源码/配置/TF/footprint/padding/安全门不变，无push/merge/部署。最终文档与证据本地提交是本次恢复点，可用`git log -1 -- docs/dynamic_navigation/research_continuation_checkpoint_20260930.md`定位。

若用户以后明确要求恢复，先读`research_stage_conclusion_20261001.md`和最后实验结果，再核验branch/HEAD/status/进程/磁盘；不要把下列历史“未执行”记录当当前状态，不重复生成冻结输入或覆盖结果。不自动重启原heartbeat，不沿已否定组合继续调参；新实验需新假设和预登记。

### 以下为历史阶段记录（最新暂停状态优先）

09:56UTC成本对象实验待评价：`standard_output_cost_alignment_20261001.md`。prereg7241d36，发现单候选改变整批path缓存，未执行单候选方案；缺失的每候选首列速度明确阻塞B，按用户最小字段例外只在offline probe加state velocity输入013a514，独立build保留e667旧binary。脚本/prepare c3d26f7，整批Freturned六非静态critic替代raw六项，原Cost/hardV1/R/CA risk/update保持。H0两个legacy默认入口全误差0、mappedpose≤4.768e-7 PASS。新binary85b0ecb1…951ba1；early inputa18612d0…5c154a、late ec1a94bd…e8d93f、param e560b7d7…51bdce；完整hash在stage doc。两个input目录已freeze，不重复prepare；待run standard_output_cost_alignment.py两case各新result目录，H1/H2尚无结果，无在跑进程。

09:30UTC B结果：`clearance_shape_risk_20261001.md`，execute8ed5c1f。构造H0 PASS、8arm/seed数值PASS；两种CA风险H1ranking/H2output均FAIL，全部TopKsafe0、online两口径body/pad0。clearance风险单项AUC.203–.256低于overlap.308–.398，safe风险更高；质量float32下溢0。固定CA未来中心最大误差x4.9447/y3.2671m、center-support16/30，仅离线标签；停止CA/support+两形状，不调unit/核/支持/温度或写方向投影救援。evidence新目录clearance_shape_risk archive bfa78e37…3638fb。下一项可回到有排序信号的旧late01/earlyv4：soft risk映射已FAIL且early返回退离，标准7critic仍raw评分；若推进须另行预登记只对齐标准critic评分对象与候选返回计划，A/risk/C更新不变，不调权重。不属于cycle40预测补救，目前未prereg/实现新项。所有进程已结束。

09:26UTC下一B实验已freeze：`clearance_shape_risk_20261001.md`，prereg e2c13be、源码/prepare dc50027；构造cycle40四源27.853–28.051 errors0/H0 PASS，source input79bd423b…fc93e2，risk input e01a2227…bc7148/parameter7e4b154a…dca5cf。四个旧mixture库不变，仅同CA支持下overlap对照与body.05 clearance-deficit，两风险单位/密度/其余cost/output不变。已有原expected-overlap公开reference、原source模型，未引入新架构。风险在标签前冻结；待原native评价，H1/H2未判定。source prepare与risk prepare均完成，无在跑进程；不得重复prepare同目录。

09UTC新算法结果：`zero_mean_proposal_20261001.md`，原生执行40a5a40。同N/std白协方差150current+150zero四seed，两口径完整可用候选46/29/36/49，white0，**主H1 PASS**；raw122/106/116/123safe（60/45/58/63推进）。保留原white目标mixture density修正后rank/online output全部FAIL，质量6.51e-38/1.99e-37/2.04e-38/6.75e-37，最好safe163/168/167/157、ESS20–40；8raw及2mapped数值PASS。safe-only/single-safe部分输出PASS但不是一般保证。修正前原成本首选也unsafe；停止mixture+原成本目标组合，不改比例/方差/temp/target。evidence新目录`zero_mean_proposal_20261001` archive e54de2d5…c0cf56，旧ρ/CA不覆盖。两个run均已完成，无在跑进程。下一项优先B，仅固定四库另立在线风险形状假设，不再扩采样；未预登记/实现下一项。main/正式配置/TF/footprint/门不变。

09UTC恢复：ordinary quota可用（5h used5%、weekly47%），未重置/购买。分支HEAD9182e48、main d735ee12，只有未知core未跟踪，磁盘11GB；Docker无运行实验。零均值mixture源码沿6263ae6，prepare9182e48，raw input9f5666e4…fc6fb0/parameter171b420d…6417c9，mapped inputf07b4a8c…dbdba9/parameterf2a7b0c1…01250a；完整hash在`zero_mean_proposal_20261001.md`。两个新输入目录已经freeze，**待原native评价，尚无算法结果**。不能重复prepare同目录；run用generic raw_temporal_proposal.py与raw_output_coverage.py，结果各新目录。H1主门为两口径完整usable coverage，不是generic raw H1。当前未启动进程。

2026-10-01 08UTC后收尾：用户要求先交付结论性内容，已汇总至`research_stage_conclusion_20261001.md`。本轮没有runtime/部署验收通过方案；CA晚期Ranking PASS但returned body.032983 FAIL，输出映射机制FAIL，rawρ覆盖PASS但完整可用输出0/0/0/1 FAIL，原white目标密度修正排名/输出FAIL。旧成功/否定及evidence完整保留。下一项零均值混合仅预登记+源码准备，提交`6263ae6047cd0e83015be8576ca54627d6362d0b`；**未生成/冻结输入、未执行、H1/H2未评价**，不得恢复成已有结果。generic raw消费者记录evaluation_script_sha256，旧输入仍用各自旧Git源码；输出覆盖按冻结arms/treated_arm读取。恢复先检查进程与目录，再prepare/hashfreeze后才evaluate，不重复运行已完成ρ/CA。普通额度已自然恢复，未用重置券或购买；无Docker实验运行，磁盘11GB，core保留。main保持d735ee12，tracked范围仅docs/dynamic_navigation与experiments/dynamic_prediction_v1。本次conclusion所在提交为最新恢复提交。

最新H3（05:20UTC附近）：rawρ八库逐候选原bounds+SG+offset已评价，2新库接入numeric PASS；rho both几何safe0/0/2/2、完整可用输出0/0/0/1，H3FAIL；white全0。几何及推进也丢失，不是只bounds。原rawH1PASS/H2FAIL保持，rho组合停止。下一项A须新预登记当前mean+零mean双Gaussian（N300/std/white covariance不变，150/150、四seed），完整mixture density修正对原white目标，先可用输出覆盖再Ranking/aggregation。不直接插入零atom后套density，不调ρ/target/权重，不runtime。原零见证PASS提供范围动机，未证明新mixture有效。H3evidence在raw_temporal_proposal目录新增usable_output_coverage子目录，旧archive/manifest保留。

最新A提案结果（05:03UTC附近）：`raw_temporal_proposal_20261001.md`。同N300/std/mean rawρ.9四seed联合safe31/19/22/22（正推进31/17/19/20），white全0；H1PASS。合法qwhite/qrho原固定周期目标修正数值PASS，但rank全部FAIL：安全质量2.8e-11/1.16e-12/1.30e-8/1.95e-12，ESS4.14/1.00/1.24/1.69；所有online/single-safe/safe-only filtered两口径body0，H2FAIL。停止组合，不调rho/target/std/temp，不runtime。下一项仅预登记八库逐候选原bounds+SG+offset的可用输出覆盖诊断（不算mapped密度，不替换H1/H2），区分A真实可行覆盖与B/C用不了；复用原函数/探针，不扩recorders。全部证据归档，磁盘11GB，无清理必要。

最新A见证（04:50UTC附近）：`late_zero_control_witness_20261001.md` H1/H2 PASS。latev4 cycle40原300/300第一步安全，首次破门16–22步；固定唯一零控制native body.158428/pad.124611、SG两口径body.125162/.174631、正推进且static/boundsPASS。支持原分布后期覆盖失败，不能说共同首列不可控/场景无解。下一项A另行预登记同N300/std/mean raw AR(1)ρ固定多seed，不预clip/filter，不扩batch；若走权重/输出需明确原white周期functional的qwhite/qproposal修正与ESS，不能套旧对角正则或mappedGaussian。B/C公式已停止，暂不改预测/critic/聚合，所有FAIL保留。main/source/config/TF不变。

最新C算法否定（04:42UTC附近）：`candidate_output_risk_20261001.md`。同CA风险只映射到候选bounds+SG+offset输出，8原生filter/pose核对PASS；late01原rankFAIL .455730，mapped rankPASS .813223但最终returned body.043611 FAIL；earlyv4原rankPASS但最终推进-.011013 FAIL。组合停止，不调参数、不runtime、不扩工具。原CA-top#280 one-hot mapped body.072145安全推进，说明可行候选存在仍不能保证平均。下一项A仅固定late v4 cycle40零控制原生可行见证+固定原首列失效诊断，先预登记/hashfreeze；若支持可行控制才研究同N时序采样，不重复batch/27原型。primary41%、weekly43%，磁盘约11GB，main/source/config保持。

最新晚期算法结果（04:30UTC附近）：`late_fixed_time_three_gate_20261001.md`。固定goal+6s选v4 cycle40/28.106、01 cycle219/28.227，非安全选择。v4 Gate1 0/300safe，不解释Ranking；01 Gate1 57safe全推进，原rank最好安全15/质量.116506，原outputFAIL。01因果source H1PASS后固定CA rankPASS #280安全推进/质量.786672/AUC.5883，但filtered current-first body.050251、返回offset计划.032983，Gate3FAIL。CA不runtime、不调参。下一项只针对C预登记同CA风险在原bounds+SG+offset候选输出计划的评价对照（原采样/critic/正则/aggregation不变），固定失败219+已有早期正对照；非凸均值仍可能FAIL。v4 A单独后续当前状态/零控制可行见证，不同时改三层。所有原证据已归档，main/source/config未改。

最新C结果（04:25UTC附近）：`physical_execution_consistency_20261001.md`。原v4/01完整8s动态门均PASS；01固定取消后tail观察到body.029603/padded0，32.694s首次body破门。已有日志取消后125个MPPI周期、命令持续33.275s、backup取消失败；observer无GID，不能确定节点责任。原cycle13的3s实际安全，破门不在该预测范围，不能指控聚合。v4tail采样PASS但线性body下界.041434无法保证。全部证据已归档，工具冻结/不修取消器，CA仍不runtime。下一项另行预登记已有v4/01 goal+6s晚期三层诊断（同run相关/已读物理标签，不充当CA独立试次），先Sampling，禁止安全挑周期。primary33%、weekly42%、磁盘11GB。

最新断点（方向纠正后）：v4窗口/tail完成、150周期完整，observer关闭超时仍FAIL；raw冻结，基础设施不再修补。fixed goal+2s选cycle23/27.719s（延迟3.617s）：Sampling PASS295/300联合安全、135正推进；Ranking FAIL最好安全第2/top1 unsafe/safe mass.717214；online/single-safe/safe-only filtered两口径均通过。固定旧连续风险迁移FAIL，288最低同分含全部5unsafe；停止组合。新四帧因果近面motion H1 PASS：位置最大.018612m、速度误差.064330m/s；接入相同短期CV软风险H2 FAIL全体同分，H3 PASS输出无实质变化。见 `near_field_surface_motion_20261001.md`。停止源中心/速度+短期CV组合，不调权重/窗口救援。下一项须另行预登记小运动模型/时间风险假设和固定消融，不是单纯扩window；不改sampling/aggregation。实际物体沿x运动，不沿用旧摘要y运动描述。额度最新5h14%、weekly39%，继续。

最新追加：固定CV-full/CA-full消融完成（`near_field_kinematic_risk_20261001.md`）。CV Ranking FAIL、CA Ranking PASS #67安全正推进/safe mass.939646；两输出PASS，但CA推进降低，risk/total AUC .214/.090低于.5，安全基率.983高于CA mass；仅已看周期诊断，不能部署。冻结四帧CA模型/单位/完整30风险，停止当前输入调参，下一项预登记新独立试次fixed goal+2s三层验证。复用现有capture，仅新输出目录/namespace绑定避免覆盖，不新增记录器或graph/TF/identity门；已知close-timeout保持FAIL、不为此修工具。

最新断点03:58UTC：两独立CA试次已结束，见 `ca_independent_holdout_20261001.md`。01全8s+tail、451周期，关闭timeout FAIL但输入可用；fixed+2s选cycle13/24.405s，原300/300safe/143positive，原及CA三层PASS，无unsafe故AUC不可定义；四帧source H1PASS速度误差.011047；CA输出body≥.43748、推进≥.02688。02原ready/phase timeout，goal未送、MPPI0，完整失败raw归档，不重试。协议独立验证未通过/证据不足，CA不进runtime、不补第三试次。下一项只用已完整v4/01冻结窗口研究actual物理执行与单周期nominal输出一致性（C），先预登记完整窗口/末尾固定区间，禁止按安全结果挑周期或修工具。额度03:51 primary26%、weekly41%，磁盘约11GB，无清理必要。

已确认环境和既有 freeze 文档；迁移 61 个数据项 hash 通过；Humble 容器单线程构建 27 包成功。新设备历史 capture replay 未通过：53 行标准评分变化，最大 0.47530985，控制相对旧 capture 亦变化。不能称完全历史 replay。

guarded PathAlign 的独立数值门通过：六个 C++ critic 加独立 Python PathAlign 与七 critic 差异 4.768e-7，静态 mask 与姿态一致，滤波前后控制与 Python 聚合差异低于 5e-5。该门仅支持反事实离线评分，guard 未修复 capture 差异。见 `device_replay_result_20260930.md` 与 `device_guarded_gate_result_20260930.md`。

周期 47 std4 的首次诊断停在 seed 0 标准噪声 map SHA 不复现；未进入 std4 排序。保留 `build/dynamic_research_device_20260930/fast_x_scale4/` 和日志。两种登记 CPU 设置均不能恢复四组历史 SHA，见 `device_sample_reproduction_result_20260930.md`。原协议停止。下一步执行独立的 `fast_x_device_frozen_control_preregistration_20260930.md`，先将默认设备四组控制字节冻结，再研究当前设备反事实覆盖/排序/输出。旧门仍失败，不放宽。

新设备输入已冻结为 `controls_frozen_device_20261001/inputs.json`，SHA `e35c5c4ece49f581f6548156685cb7e74226e6f1d5658f67594ed856f144f47e`。第一次新设备执行 seed 0 的聚合对旧 Python 参考门失败；已定位为正则 float32 累加顺序差，见 `device_aggregation_attribution_result_20261001.md`。原参考不改。`fast_x_scale4_native_reference_20261001/` 四组全部完成并通过数值门，但均无动态或联合安全候选，四组原生输出均失败。见 `fast_x_frozen_device_result_20261001.md`。停止扩大该分布。

27 条固定控制原型已完成：7 条严格几何安全，其中 6 条有独立原生传播核对；#18/#21 同时推进目标。见 `fast_x_control_prototype_result_20261001.md`。10 条单候选 raw scorer fail（含几何安全 #25）全部保存，不能混同 filtered 标签。

固定原型在线风险区分完成：硬 V1 raw/actual-filtered 九步全饱和，原总分首选不安全 #14，加原固定单位连续风险两口径均首选安全 #18。单项风险最低 #19 仍不安全。见 `fast_x_prototype_risk_result_20261001.md`，不作闭环或实际采样分布声明。

原型原生输出一致性已完成：18/18 新旧/零项字节门、4/4 非法输入拒绝通过；原基线动态失败，两个固定连续风险口径均产生 #18 的安全输出，body/padded gap 0.14761/0.11761 m，推进1.365 m。详见 `fast_x_native_risk_consistency_result_20261001.md`。两口径共享一条原型，不是独立闭环成功。新工具 `audited/install/.../frozen_cycle_replay` SHA `e667e19f75d7b5a937f7df78191359ff0d3e8c35fbdd017ae54d123ff1fa7d5c`。

固定四 seed 风险迁移已完成，24/24 基线字节门及八口径数值门通过，但动态输出0/8严格安全。详见 `fast_x_frozen_batch_risk_result_20261001.md`。确定性原型正对照不能外推到这些随机集合，停止在此数据上调单位/temperature/窗口等参数。

32组bounds审计完成：before全合规，after整段27组显著越界；实际返回只有seed0 raw vy越界。几何正对照#18整段也越界（真实返回合规），已追加限制，不覆盖标签。首次语法启动失败已保存并修正，见 `output_control_bounds_result_20261001.md`。

32组原bounds投影反事实已完成：8条原几何安全序列仍安全，投影严格越界0；std4基线0/4、风险0/8仍动态失败。见 `post_filter_projection_result_20261001.md`。该结果只保留同状态的nominal velocity可行见证，不是运行clamp或部署验收。

固定时间结构覆盖已完成，见 `temporal_coverage_result_20261001.md`。源码 `86e21b6`；输入SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`、参数SHA `7a9db00145f8d4c05a8c446510fafa5b189836aeb7bdfbad6c6f1e253dc1d9a2`。完整16000条clip/filter/clip后：原组四seed全0安全，rho=.9组联合安全154/149/189/166，兼正推进23/22/20/23；静态碰撞也增多。没有评分/权重/聚合或运行验收。输入在 `temporal_coverage_inputs_20261001/`，结果在 `temporal_coverage_result_20261001/`。

完整两组bounded-filtered七critic/固定连续风险排序已完成，见 `temporal_risk_rank_result_20261001.md`。输入SHA `4c112bf073b8f196ec50582b6e8a3e65fd73f641c87582bee864f98afc51891b`、参数SHA `8ca3596f716458a3b892373654130ca50b8d966c53d284bf20a66e1f7986cdc4`。八组数值门通过；相关组增强小计最好安全名次5/2/8/3、静态clear AUC约.884–.896，但四组首选全不安全；单项首选安全者退离目标。停止该组合在这些输入上的回调/推进，不套原对角正则做控制。输入/结果在 `temporal_risk_rank_inputs_20261001/` 与 `temporal_risk_rank_result_20261001/`。

周期43–47输入审计完成，见 `fast_x_online_input_audit_result_20261001.md`：五周期字段/源时间构造门全通过，共享6个不同source；scan/pose callback receipt及pose frame未记录，实际在线可用证明0/5验收，不是证明运行迟到或frame错误。输入SHA `f9e3a28a96ec1f74101de9f2114e003d9b768681c5a1a41fbf4cda7363a57501`、参数SHA `4c5de5bc710c052d91a2464f4814ff07fd179e2a6cad01df4d1efbf9795a36c9`，目录 `fast_x_online_input_audit_20261001/`。未读未来或计算几何。

条件化二维规则覆盖完成，见 `fast_x_face_coverage_result_20261001.md`。输入SHA `503807f99c94c3d7b0d64a0958844cce4e4b14211a98ed1b3d46563d85b05028`、参数SHA `207fadc745d9f8656308792fea91f817b5dc5bc2f1f5924ed97481dadfed1e3c`。五周期候选/原V1均45/45完整二维覆盖，共享6源；候选源x中点误差最大.11924 m，区间包含不等于中心精确。实际receipt/frame仍未验收。目录 `fast_x_face_coverage_inputs_20261001/` 与 `fast_x_face_coverage_result_20261001/`。

支持替换排序完成，见 `face_support_rank_result_20261001.md`：输入SHA `889cfa5629b2190eaab2e7fe1d935c838a126d5bfa243a995f523baf9b0ebaab`、参数SHA `7277b798813e23827e15ce771474efe31c489ec318b5f86b714188687c5fb43c`。旧16000风险逐字节一致；新增强静态clear AUC约.932–.941，但相关四组首选仍全不安全，最好安全名次7/2/7/4。停止回调/推进该组合，不做原对角正则或MPPI输出；receipt/frame限制保留。目录 `face_support_rank_inputs_20261001/` 与 `face_support_rank_result_20261001/`。

完整16000×30步失效归因已完成，见 `temporal_failure_timing_result_20261001.md`。源码 `fdc1ab4`、评价提交 `6d067c5`；输入SHA `a1a16a9247e7f06f733f3738c6cd2c494a39552ded046842aea1525f691ac512`、参数SHA `ec5e0234267f32eba92aeb925e100c8396685c5021fbfbb02f9f74a526020dce`。八组min-gap误差0、原标签逐条一致；相关组八个旧/新固定首选记录均九步安全、13–20步首次破门，三秒仍全失败；原组新首选也有3/9步早期失败。目录 `temporal_failure_timing_inputs_20261001/` 与 `temporal_failure_timing_result_20261001/`。停止该组合调参，不改horizon或安全门；下一项独立只读receipt/frame/源时刻TF记录器需先预注册，不能回填旧数据。

只读typed receipt记录器评价失败，见 `replay_receipt_typed_failure_result_20261001.md`。源码 `05aaf96`、评价 `08dbaf7`；inputs SHA `acdddaa3ad749e29b9d9fd2140b8b744ed00d1c97d6c0bf4b83affcb2ca88d86`，参数SHA `ad7b44bbfe48cf5dea45800c2b1fd7e2206cd5df6b068b70e87095f3b0f0dd22`。第3消息CDR字节门失败，只记录2事件、DDS未执行；完整归因native字段25/25一致，独立CDR游标变化全部padding，两个进程重序列化完整字节18/25及12/25一致。保留失败。下一项按 `replay_receipt_raw_cdr_preregistration_20261001.md`开发raw入口并复用旧25个bytes，新envelope冻结后才evaluate；observer receipt不能证明消费者receipt。

raw receipt新协议已评价通过隔离记录器门，见 `replay_receipt_raw_result_20261001.md`：源码 `ff8cc65`、评价 `ba3de94`，input SHA `b898d9b58249097238da83bd21fed9378bc44d72d523a40d652c4d27f1ee1796`、参数SHA `50b0a06a2cefe454b1b8b4ff2c82ec6a3ebe6ac415c741ee61291361dfa59f2b`。25/25旧CDR和固定事件行为门、3/3损坏拒绝、独立进程DDS6/6完整字节通过，TF/重置/零stamp等通过。目录 `replay_receipt_raw_inputs_20261001/`、`replay_receipt_raw_result_20261001/`；typed失败不变。observer receipt仍不证明运行消费者receipt，不称闭环安全；下一项需先预注册按事件顺序独立TF replay并核对原始capture。

独立因果TF replay已通过，见 `replay_event_tf_result_20261001.md`。源码 `c746f36`、评价 `8c8291f`；input SHA `9a478709c87ad91fab319ae58f196fda7f4e7b91c465c57a2a879c09a1f2414d`、参数SHA `a8683afa713a330883590f18711f7ee8333dca3ef09c7f42108bc853feaa25af`。31事件14个scan的status及全部返回字段位模式一致；预载未来TF无效对照将controlled序号9/19/22误变available，DDS正常对照无变化。原缺失不补；仍不是实际消费者receipt或完整MPPI历史replay。下一项需先预注册无clip/filter的相关Gaussian协方差双线性项代数审计，不做权重/聚合/新控制或风险组合救援。

相关提案协方差必要代数通过，见 `temporal_covariance_algebra_result_20261001.md`。源码 `c3bbf78`、评价 `1ab0f2c`；input SHA `a813be60135daddc45c1ee9907a75ae99f7aa05ac409f14943a172eb8e7c3865`、参数SHA `6d549edf98f8d64a204064dd4f97ba172725ec353828acd93617df4cda19dfdc`。矩阵/完整八组密集-白化总误差≤8.42e-17、rho0对角误差0、四组f32递推raw字节一致；相关-对角差RMS约.162–.168，mapped改变约98%元素。无truth/critic/权重/聚合，重要性目标与实际更新仍未验收，不救援停止组合。目录 `temporal_covariance_algebra_inputs_20261001/` 与 `temporal_covariance_algebra_result_20261001/`。下一项可先预注册无truth的raw→mapped非可逆反例，否定简单可逆线性换变量假设，不否定所有latent推导。

原bounds/filter非可逆反例已通过，见 `mapped_control_preimage_result_20261001.md`。源码 `d3e0818`、评价 `2f34221`；input SHA `f6ef6c33ccf2071944cc95afd89c9d94f8dc013fd85e9e01bdf4d3ee51fca783`、参数SHA `0a070fc673c5f124f492b0d38fc5b4a97b51be09af5d33d20dd307593741a206`。八组原mapped完整复现、mutant一元素改变在第一次clip消除，完整mapped各自逐字节相同；latent数学量不同，密集导数误差≤8.24e-18。不是八次独立成功，不推断所有mapped分布非Gaussian或所有latent理论无效，无安全/评分/权重输出结果。目录 `mapped_control_preimage_inputs_20261001/` 与 `mapped_control_preimage_result_20261001/`。下一项先只读新设备sim/launch/capture能力与TF所有权，新的独立周期协议需预注册；禁止拼接synthetic observer capture与历史周期冒充一次运行。

新设备采集前置只读inventory及actual路径补充已完成，见 `new_device_capture_inventory_result_20261001.md`。14项查询exit0，Gazebo6.18/Nav2 1.1.20/ROS-GZ0.244.25；workspace V1及guarded实际mppi库存在，原near-X五artifact迁移SHA完全一致。旧/work prefix缺失仍保持；候选库名/derived trial路径缺失不等于库或场景丢失。原评价c184a65、补充7a5bec1，summary SHA `6e7fffa4ad6e2dbaa44cd5dbf80e4c63d577312f87fe2aadc00658e43f425b99` / `a4a4c68134416b65fc020c625a8416c7d77048911f552a78aa67fb66f77e412f`。全部32结果文件在 `new_device_capture_inventory_result_20261001/` evidence。未启动或加载插件，TF只为源码预期所有者。下一项先预注册原sim关闭Nav2/RViz的限时被动raw/TF采集；保持world/bridge/adapters、无导航目标或算法配置修改。

被动sim前置capture已完成且门FAIL，见 `passive_sim_receipt_result_20261001.md`。执行3513f8c，113项资产input SHA `b670bdf7a89abf502cf1ce8d03cb1bc57f4a9199e2de46c4bcde9e2474be115b`；22394事件、279scan、918odom，数据流/frame门通过，源时刻TF71可用/208不可用（固定5s后68/205），严格门失败；graph canonical publisher、无导航节点或cmd_vel发布。监督等待TimeoutExpiredexit1，SIGINT后两进程return0、capturecomplete且逐文件verify；KeyboardInterrupt警告保留。capture manifest SHA `8d0d8da9dd80308eca288a6b9981ac024ae7e0e78133ca4d60623620660bd8a0`。完整tar/gzip evidence与build原件保留。停止该协议重试，下一项先预注册真实事件独立因果TF逐字段replay和缺边/后续首次可用到达归因；不回填失败、不改TF/等待策略，不称MPPI或安全验收。

真实TF到达归因完成，见 `passive_tf_arrival_result_20261001.md`；源码22fd569/评价bd8a014，input SHA `c5d6a0b76665d97585ac922bc76e33c0ca307da7367a9d73ca55f053efe05aaa`。279/279因果TF全字段一致，原失败208不变；缺odom58/map52/两者98，static叶边无缺。207后来可用steady中位9.020ms/p95 45.131/max58.491，1个sequence105源.002s始终不可用，5s后205失败都后来可用。summary SHA `cb0dbfff01263fedc4d0ba2f6db09d8b99308f54c92c617af076c6d444223dfe`，全行证据保留。不能推断actual消费者wait成功/失败；现有tracker .08s timeout和2线程未改。下一项先预注册隔离consumer trace及offline正/负对照，门通过后才考虑shadow-only采集，无Nav2或目标、禁止改TF/critic/配置/安全门。

consumer trace原clock门FAIL保留，修复版本七例全部转发门通过，见 `consumer_trace_fixture_result_20261001.md` 与原 `consumer_trace_fixture_clock_failure_20261001.md`。修复源码2d7cbc5、评价7026c05；input SHA `7b3c339b8b808888f809dd2347ee7a4c3b15236ca565e703c3547469b180b633`；旧21CDR逐字节复用，14项float32 wire native身份另冻结/严格验证。正常非空tracks、所有语义输出（仅latency_ms独立保留）、callback/lookup原参数/.08s/clocktype/返回对象和异常身份全pass；sentinel同对象、两损坏拒绝pass。summary SHA `14ec0bb318f1c00a0c4852606768c37db2735ab0348a93be1aea48e33c1625ea`，完整证据保留。不是异步wait/零开销/在线安全门；下一项独立预注册无Nav2实际shadow consumer+observer新capture，原YAML/TF/算法不改，新数据先冻结后分析。

原shadow实际consumer新capture已完成冻结，见 `shadow_consumer_capture_result_20261001.md`。执行ab2dd2f，input SHA `363be19416798e4e3c52aeedb396f082e1789d60513350b37ca2dd166d4e94fe`。observer8881事件/8731文件、manifest SHA `27dce5d9c7901e288c68355d926a1fde94b6b3acdf6c462defe0527616094112`；consumer754事件/538文件、manifest SHA `e8d3e3a1226b16a0328157cff9850022aaf53b6c784f5f047d732d565b482a01`。所有文件SHA验证、四进程return0、graph无control且canonical publisher门通过；early past TF警告和退出never-retrieved Destroyable警告完整保存，不称无异常退出。完整raw/typed/log tar evidence已冻结，未进行跨receiver成功率评价。下一项先预注册source/frame全native scan对齐、consumer lookup参数/时间/status、prediction身份，未匹配/歧义保留，不能以observer receipt代consumer。原YAML/TF/.08s/算法/正式配置不改，fixture map仅接口诊断。

实际receiver对齐10门通过，见 `shadow_receiver_alignment_result_20261001.md`；源码b40223a/评价32995fe，input SHA `a8009c4b64dd3635e2028746f52a3509a31759a549d3490bceb71f87dd82cfa4`。consumer108 scan、observer107，共同107全native一致，1consumer未匹配保留；实际query107available/1exception，cross零等待observer失败→consumer成功69（其余共同37成功/1异常）。固定5s后104都available。原.08s/ROS_TIME/frame/source/entry-exit/query/publish身份门全pass，lookup spanmax94.923ms不保证80ms wall上界；107 prediction源身份pass，106 raw observer全字段匹配、1未匹配，8mapbodyfixture一致。退出warning/原observer门FAIL仍保留，不称预测/闭环/MPPI安全。下一项先预注册新MPPI/V1实际动态链接身份和trace捕获接口核查，再考虑新near-X完整cycle采集；保持原scene/profile/TF/算法/安全门。

MPPI/V1动态链接门三库全pass，見 `mppi_module_link_result_20261001.md`；源码92dc8a1/评价ca1f875，input SHA `b5110d4c6cad5a4384d77c4306b9da24710e060c19a073a1272aba50ca4b9134`。100 loaded .so、controller/critics/V1唯一SHA及resolved路径正确，无/opt同名库混入；summary SHA `c5b855dcfb466ea177d2113f32198b44c1555ef0c7384b30b59ea5cea23eda23`。未instantiate/configure/score。capture合同核查明确缺V1 actual latest prediction snapshot/receipt及任意周期filter history[4]，不能凭旁路或成本反推同输入；旧runner不恢复。停止完整cycle运行推进，下一项先预注册隔离build原源码只读telemetry副本和严格offline原版/副本全字节/正负门；原critic/controller源码与库、TF/config/安全门不覆盖，未通过前不研究容器运行。

V1 actual snapshot telemetry隔离副本已生成并单线程构建exit0、未评价，见 `prediction_snapshot_trace_preregistration_20261001.md`。生成器2bafd81；原CPP SHA767ffc49...，副本c3471603...；仅CPP增加callback typedCDR/receipt与score snapshot/实际age，去除追加块还原原CPP全字节门通过，其他文件不变，完整source/diff evidence已冻结。worker构建exit0，12路径input SHA4bfd43e3.../parameter3ad8cfaf...已冻结完整evidence，下一项执行original/copy role并先冻结输出，再评价300×30score字节/身份门；不允许live/controller使用，原插件/controller安装源码/配置/TF/安全门未改。history[4]缺口仍待独立协议，不声称完整replay。

V1 actual snapshot隔离副本12路径离线门全部通过，见 `prediction_snapshot_fixture_result_20261001.md`。源码da1d700/评价c2720b9；input SHA4bfd43e3.../parameter3ad8cfaf.../summary27c7a673...。24进程return0，全部300score/fail逐字节一致、正负成本、writer完整性、实际加载V1/controller身份、full-native typedCDR/同cache receipt/source/frame/实际age均pass；legacy碰撞300全1666.6666、uniform1927.94–1947.75，明确不是安全或预测有效结论。原安装源码/config/TF不变、历史FAIL不变。下一项独立预注册control_history[4]只读telemetry副本及实际evalControl filter-only fixture/既有native replay语义门；完整cycle运行仍停止，未经另行协议不live使用。

filter history 16/16路径共298门全部通过，见 `filter_history_fixture_result_20261001.md`；input SHA c623e53f.../parameter8ea2d05c.../summarye073b72c...，执行7b11ab2/评价e4c299f。原版/副本32进程正常0/非法1；7种actual evalControl iteration0 filter-only路径before/after/history/returned/internal shift与logger_off全字节一致，连续第二次history继承正确；5 native cases×6输出×2 role共60 golden全字节相同、4非法×2拒绝通过，实际controller/critics身份正确。原算法/install/配置/TF及全部否定安全结论不变。两副本单独门已过，下一项先独立预注册组合加载下7critic+V1评分/快照/逐critic identity门（同冻结12消息/原300×30），通过后才考虑新的完整研究capture；不把两个单项门自动等同完整周期。

组合telemetry八critic门12/12共558门全pass，见 `combined_trace_fixture_result_20261001.md`。源码e57119a/执行7a96639/评价46ce2da；input SHA7884873f.../parameter65338333.../summaryee456fa3...。24进程return0，三库actual maps唯一身份、八累计300score/八fail/最后加法/whole raw map/full-native snapshot/receipt/age全门pass；没有新C++构建或控制运行。下一项先预注册可移植near-X七资产精确path-binding和只读launch依赖门，再独立完整capture协议。旧near-X evidence的reference.launch.py/comparison.launch.py也已实际找到且SHA与manifest一致（此前inventory仅列五项，不是证明其丢失）；绝不伪造旧/work安装或旧镜像身份。原main/source/config/TF/安全门与历史FAIL不变。

portable near-X首次preflight FAIL保存：input fbc2d8aa.../parameter370d98bf.../summary2545bdc6...，执行13876ae。四launch逆变换/原world-model-profile/196资产/15pkg/overlay通过；file检查误把ROS topic和日志字符串当filesystem路径，show-args在positionals中间插option使Humbleargparse拒绝params_file（return2）。未启动场景；首个parser探查误用ros2launch.verb import失败也保留，正确LaunchCommand源码已读取。完整失败evidence及否定doc先提交；下一项按新预注册改harness AST语义识别file contexts与CLI option顺序、全新v2输入/输出，不覆盖旧失败、不改scene/profile/TF/安全门。

portable near-X v2只读preflight全部13门pass（原FAIL保存），见 `portable_near_x_preflight_result_v2_20261001.md`；source d97fac7/executeac4b833，input SHA26afaa94.../parameter2cf65fd2.../summary23e0f33b...。196资产/15pkg/正确history+snapshotprefix、六真实filecontext完整集合及存在性、四launch逆变换、原world/model/profilebyte、show-args return0和source不变门全pass；其余topic/LogInfo单独分类保留。尚未启动场景；下一项先独立预注册有界new-device near-X full capture（same7scene/profile、原goalphase、实际consumer/V1/history/trace、canonicalTF和真实lib/参数身份），新capture先冻结后评价/first3eligiblecycles replay，禁止单次安全成功外推。保持旧/work runner不执行、main/source/config/安全门及全部旧FAIL不变。

## 环境和恢复路径

near-X v4工具纯UUID日志接入/source身份门和输入已冻结，尚未运行。协议9c0d386/source6486643；input SHA3565a62e.../parameter6f84d923...，392资产/15pkg/三库、actual两个helper与已通过14/19case源码SHA均一致，AST/prepare exit0。见 `new_device_near_x_capture_inputs_v4_20261001.md`。下一项一次v4完整capture（新output/partition/XDG），原8参数/7graph/9command门、scene/profile/phase/ready/窗口/tail/时限/安全门保持；全量数据先freeze/commit，再独立first3周期replay，不使用旧v3六不完整周期。main/src/config/TF不变。

UUID logging native fixture19/19通过，见 `near_x_goal_uuid_fixture_result_20261001.md`；source e38df28/eval76cc824，input5bb8e4e3.../parametere19f005e.../summarydf8baa7b...。12正例完整16byte/Pythonint/JSON值、原字段identity/type/content和CDR全字节一致，7非法拒绝且不改输入；旧list直接JSON在6个np/native容器TypeError、其余6本来成功。尚未接入goal/capture。下一项先独立预注册v4纯UUID记录接入、新output/partition/XDG及两helper与通过fixture实际sourceSHA验证，所有旧FAIL/六不完整周期、runtime/配置/TF/安全门不变。

near-X v3 capture FAIL，执行1993701；八actual参数、7graph/9command门全pass，ComputePath status4/path87，22.208s发送导航且handle accepted；新tool list(UUID numpy uint8)写JSON TypeError，窗口/tail未完成，少量周期全部保留且未评价。见 `new_device_near_x_capture_failure_v3_20261001.md`；capture envelope SHA1af8a6b4...，全raw/typed/MPPI/graph/参数/process/GZ/log在v3 failure evidence。v3停止，下一项先预注册实际ROS UUID/NumPy序列化fixture与仅记录转换修复，不修改UUID/handle/control/runtime，也不使用此不完整capture作first3实验。所有旧FAIL/main/config/TF/安全门保持。

near-X v3已预注册并完成最小tool接入/输入冻结，尚未执行。source d092420，input SHAa254e18e.../parameter5fee0832...，388资产/15pkg/三原研究库保持，实际helper源码SHA与14case通过记录一致。goal仅command gate换成精确九门AND、增加原behavior参数只读核对，另六图门及全部scene/phase/ready/时限/TF/config/安全门不变。见 `new_device_near_x_capture_inputs_v3_20261001.md`。下一项一次v3完整capture（新output/partition/XDG），全量数据先冻结提交；原两次前置FAIL保留，不按truth/安全/排名选周期。

精确原控制图helper offline14/14case通过（原图1接受、13负例拒绝），见 `near_x_command_graph_fixture_result_20261001.md`；source d5a2d9c/eval9792968，input6f1c0d2a.../parameter437e77e6.../summary3700ce19...。身份包括四single root control node、三command topic精确publisher Counter/type/namespace/unique非零GID、两exact chain subscription；host纯Python离线，81b325仅来源镜像。尚未接入goal/runtime。下一项先预注册v3 helper接入+actual behavior参数额外读验证+全新capture，原另六图门/TF/scene/profile/时间限/安全判据保持。

v2 graph只读归因完成：原7gate重算一致，6pass，仅nav_command_chain失败；5source门pass，summary511b735d...。见 `near_x_graph_attribution_result_20261001.md`，eval27033fb；input09302a41.../parameterbe3f50d1.../native source021c837e...。固定Humble1.1.20原launch中behavior没有cmd_vel→cmd_vel_nav remap，TimedBehavior::configure原header在cmd_vel建立publisher；actual四behavior+一smoother endpoint符合原四plugin来源。tool只允许smoother的预期错误，runtime/TF/config不改，原FAIL保留。下一项先预注册精确identity/namespace/type/count/source约束的graph工具纠错和offline正负fixture，通过才新capture；不能默认每条cmd_vel是MPPI输出，无rawpublisherGID限制保持。最初错误nested header路径探查也保留。

near-X v2实际capture再次FAIL，执行5fdfa54；属性修复成功，七actual参数匹配，graph原始文件已写但前置canonical/single/control-chain图门失败，未发送goal、没有可选MPPI周期。见 `new_device_near_x_capture_failure_v2_20261001.md`；capture envelope SHA84ed404f...，全raw/typed/graph/参数/process/GZ/log冻结于v2 failure evidence。v2停止，下一项先独立预注册只读graph/source归因，判断实际身份/remap和工具判据，不能放宽TF或安全门，也不能凭loaded_maps/writer_status声称有控制周期。

near-X v2工具最小属性修复及输入已冻结，尚未执行。协议c521afe/source68a65b4；native rclpy property四值与actualClock读取/三个AST检查通过。input SHA173f8369.../parameter880edc17...，380资产/15pkg/组合库/helper逆变换保持；见 `new_device_near_x_capture_inputs_v2_20261001.md`。下一项只执行一次v2完整capture（新output/partition/XDG），原scene/profile/phase/ready/窗口/tail/超时/所有门不变；完整输出先冻结提交再评价。原FAIL29564文件全保留，首三个eligible周期不能提前选。

完整near-X首次capture FAIL，执行2bd166a；七组actual参数均匹配，原ready/phase到22.174s，但新goal工具误调用rclpy nanoseconds属性，graph写入前TypeError、未发送任何goal、无MPPI周期。见 `new_device_near_x_capture_failure_20261001.md`；capture envelope SHA6d8b9497...，完整raw/typed/日志/参数/process/GZ transport在failure evidence，observer因无planned marker标interrupted。原协议停止，下一项独立预注册仅修复属性读法的新input/output绑定；不能把此FAIL当算法或TF图失败，不能用其做三个周期replay。原约束与全部旧FAIL不变。

完整near-X采集协议与工具已提交，准备输入完成exit0且未启动场景。见 `new_device_near_x_capture_preregistration_20261001.md` / `new_device_near_x_capture_inputs_20261001.md`；source ffb637e，input SHA f5f0cb93.../parameter b73762c5...，378源码/installed资产与helper两处import-only全字节逆变换门通过。下一项只运行单次有界capture（Domain174、same scene/profile/phase/goal、actual consumer+V1+history+MPPI trace），全量原始输出先冻结提交，之后独立预注册首三个eligible周期replay。raw observer marker仅固定窗口+tail完成才complete；失败不为有利结果重跑。原main/src/config/TF/安全门及历史FAIL保持。

主机 Ubuntu 24.04 / ROS Jazzy；主机无 Humble，不能使用旧 install 假定环境一致。固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，Humble/Nav2 1.1.20、NumPy 1.21.5。

所有新设备构建和原始输出位于 `build/dynamic_research_device_20260930/`：`restored/candidate_navfn_1`、`workspace/install`、`guarded/install`、`guarded_source/source_manifest.json`、`baseline_original`、`score_prefix_original`、`baseline_guarded/counterfactual_gate.json`。工作区整仓只读挂入容器 `/workspace/rm2027_navigation`，仅该 build root 另行可写挂载。Docker 运行 `--rm --network none --cpus 2 --memory 6g --cap-drop ALL`，无硬件与线上系统接触。

此前保存的每小时续跑 heartbeat，ID `rm2027`，已按最新收尾要求设为PAUSED，不会自动恢复旧研究。不使用付费重置或购买额度；恢复需用户明确授权。原配置保留供以后选择，未新增自动任务。

## 磁盘检查

已完成五个 2026-06 失败 SDK git clone 构建容器及五个无标签中间镜像清理，约恢复 15.6 GB，见 `device_disk_cleanup_result_20260930.md`。保留所有 tagged 镜像、旧 navigation 容器、core、地图、录包、实验与 evidence。未知文件不能作为清理目标。
