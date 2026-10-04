# 固定连续风险：新输入排序与完整输出

## 背景、假设与复杂度

cycle23三层诊断已显示Sampling PASS、原Ranking FAIL、online Aggregation PASS。本阶段只改变离线评分信号，采样、控制bounds、SG及传播保持原样。

假设：既有uniform-center expected overlap连续风险，在该新输入中能将联合安全候选置于总分首位，并保持safe weight mass≥0.5及完整输出安全/正推进。只做一次固定公式检验，失败不扫描单位、窗口、temperature或标准critic权重。

对应失败是原总分首选#293不安全而295条安全候选存在。硬相交几乎同罚使标准项主导，既有软重叠提供连续信号。Nav2原CostCritic已有空间软成本累加与trajectory length归一化（固定Humble源码cost_critic.cpp）；本项复用现有研究expected_overlap而非引入新依赖/新优化器。更小的验证是对既有原生replay输入固定additive分数文件，无运行源码修改。此前旧四seed组合的否定结果保持；此次仅一次新输入迁移诊断，不能重新回调旧失败组合。

## 冻结输入、参数与判据

- 唯一输入沿用fixed-time cycle23，input SHA256 `4fe24a21df02f8ceeaf2d76016c06ec776300118686c577a7bc5de29c81459ee`；原三层评价提交 `1848f85`，结果见 `near_field_three_gate_holdout_20261001.md`。
- 原300×30全部候选，无truth筛选或替换。风险仅来自actual prediction typed CDR、actual used_age、原轨迹及padded footprint。
- 直接调用既有 `occupancy_rank_probe.expected_overlap(order=5)`，原物理尺寸、acceleration、原1.0s prediction horizon不变。均匀支持不是校准碰撞概率。
- 附加单位保持既有 `(3.81/254)*1e6/9`，总分=原总分+该单位×risk；原正则不变、temperature不变。unit是历史固定诊断尺度，并非此次调参或安全阈值。
- Ranking门仍为首选联合安全且safe mass≥0.5；报告Top1/5/10/30、最好安全名次、风险单项与标准项/正则/总分。
- Aggregation由既有原生replay执行，记录weighted/bounds/filtered与两种完整3秒传播口径、raw静态门203、body≥.05/padded>0/整段bounds/正目标推进。先保存在线风险/分数，再读取已有标签进行判定。
- Oracle对照沿用原三层已保存single-safe和safe-only，不因新排名重选oracle，避免同时改变诊断条件。
- 参数/风险源码/输入/镜像hash先冻结再执行；数值误差沿用score1e-3、pose1e-5、aggregate5e-5，不增加byte-exact门。

## 结果、限制、下一步

评价前冻结：算法源码提交 `85d2f1d`；input SHA256 `ccd527523eac5da224c4e2348164d7cf24ee28a4c8cdbab2d2d807387441e265`、parameter SHA256 `33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`，固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

执行 `dc372ba755c9cc7a38d457d317d59975eba176d9`；[完整证据](evidence/near_field_fixed_risk_20261001/manifest.json)、[结果](evidence/near_field_fixed_risk_20261001/summary.json)。

**Ranking FAIL，停止此固定组合；Aggregation PASS不能替代排序门。** 300条原候选不变，首选仍#293不安全，最好安全名次2，Top1/5/10/30安全数0/1/5/25；safe mass由.717214变.717172。风险范围1.79356e-5–2.86274e-5，288条完全同最低风险，包含全部5条不安全者以及283条安全者。任意正附加权重都不能改变这288条内部排序，继续调单位没有合理依据。

风险单项safe-lower AUC .479661；单项首选#0虽安全，却是288条同分最低中的stable索引首位，不能当作动态信息成功。summary中的risk_only.verdict和safe_weight_mass使用均匀概率作展示，是诊断字段，不是实际MPPI风险权重或算法PASS；其“.983333”只反映295/300基率。完整总排序与输出判定不受该展示字段影响。

原生附加评分最大误差0。增强filtered输出body/padded gap在当前速度首列口径.143519/.113272m、offset口径.131062/.101039m，静态与bounds门通过，推进.153435/.159293m，和原在线输出几乎一致。两口径输出PASS只说明此次加项没有破坏原安全输出，没有安全改善证据。

当前实际tracker输入position(4.48803,.15548)、velocity(-.896889,-.133694)m/s、age .065s。这些字段本身不证明真实运动；下一阶段只研究在线运动信息如何给完整控制提供可分风险，而不是调权重救援该项。可以复用已有actual consumer/source字段评估一个最小运动估计假设，禁止新增采集层。

当前labels已经查看，因此此项是固定迁移诊断而不是新的盲holdout；均匀支持不是校准概率；只有一条周期，不能外推稳定性。旧四seed同组合的否定结论保持。本阶段没有运行算法修改。下一项必须先写新的运动/软风险假设，回答成熟实现与更小验证方式；若无法给出直接算法判据，则不实现。
