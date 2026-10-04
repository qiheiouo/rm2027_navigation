# 同N原始时间相关提案与原white目标

## 已复现问题与最小假设

late v4 cycle40原300/300全3s不安全，首列全部安全，唯一预登记零控制及原SG输出都安全推进。说明该分布后期覆盖不足；扩大batch/std已CLOSED。旧rho实验在N2000/std4且预clip/filter/clip上只证明候选覆盖，不具备mapped Gaussian密度；不把那些结果直接称MPPI。

H1：同N300、原std/mean，只改变raw噪声时间相关性，固定rho=.9可在四固定seed中每组提供至少一条联合安全且推进的完整3s候选。H2：对原white周期目标作合法proposal修正后，仍能满足首选safe/安全质量≥.5及原native输出两口径完整门。若H1成功但H2失败，记录采样覆盖与可用权重的区别，停止当前组合，不调rho/std/temp/batch或风险模型。

公开[IT-MPC论文](https://arxiv.org/html/1707.02342v1)给出基于分布/importance sampling的框架；[pytorch_mppi](https://github.com/UM-ARM-Lab/pytorch_mppi)包含SMPPI/KMPPI时间平滑实现，可作为时序结构参考。但本实验不导入它们或重设计控制器；低维插值不能直接当原全维white提案。最小选择是满秩AR(1)，复用既有相关代数和native optimizer，比新kernel/derivative controller小。以下固定周期functional与密度比是本实验推导，不冒称完整理论MPPI或已部署安全算法。

## 冻结目标、提案与方法

固定唯一cycle40和所有原状态/map/path/snapshot/history，原300×30×3与std(.2,.2,.4)。四seed0/1/2/3，共同标准Gaussian创新分别生成white及rho提案；raw Ui=原U0+noise，不预clip/filter，原bounds/SG只在加权更新后作用。未来truth仅离线标签。

令Cw(U)为原native7critic+hardV1+原对角regularizer的float32总成本，lambda为原temperature。固定目标为对原qwhite的有限周期函数积分：

`target(U) ∝ qwhite(U-U0) exp(-Cw(U)/lambda)`。

rho协方差为每轴sigma²*rho^|i-j|，初值stationary；轴独立。z=noise/sigma，

`log(qwhite/qrho)=.5*[Qrho-Qwhite+3*(T-1)*log(1-rho²)]`

`Qrho=sum_axes[z0²+sum_t>=1((zt-rho*z(t-1))²/(1-rho²))]`；`Qwhite=sum z²`。

rho有效成本为`Cw-lambda*logratio`。不改变原regularizer来假装另一个covariance目标；native仅加密度修正。为复用现有finite nonnegative additive接口，统一减去修正最小值与0的较小者，得到非负项；同批常量理论上不影响归一权重，报告float32数值差/最终门。原采样输入有float32舍入，密度对应连续Gaussian近似，不声称离散量化密度精确。

密度通过满秩raw Gaussian定义，不能在bounds/SG映射后套此比率。原目标与proposal差异可能使ESS崩溃；ESS仅报告，无新调参门，也不以覆盖PASS替代完整H2。

## 原三层门和停止条件

先freeze各raw controls/density及source/input/parameter/image hash，再用原native传播与7+V1计算。Gate1 body≥.05/pad>0/raw203、报告安全数和正推进；0safe直接Sampling FAIL，不解释Ranking。若有safe，Gate2按修正后native实际cost/prob报告最好safe rank/Top1/5/10/30/质量/标准及预测项/密度项；主门top1safe+mass≥.5。保存未修正成本作诊断，不能将rho未修正输出称为合法原目标更新。

Gate3 compare online修正权重、single-safe、oracle-safe-only（后两只离线），weighted/bounded/SG，原native当前首列及returned-offset-plan完整30步、body/pad/static/bounds≤5e-5/严格正推进；两口径filtered均PASS才主门PASS。对原white提案同样评价，用共同创新对照，不外推到闭环。数值沿pose1e-5/score1e-3/aggregate5e-5，原概率参考3e-5；采用语义和最终PASS/FAIL，不新增byte门/recorder。

H1要求四rho组均有safe且正推进，H2要求四rho组Gate2/3全部PASS。任一失败停止本组合，不继续改变密度/目标或加更多seed凑结果。四seed共用一个状态，不是独立场景；没有多场景/闭环/实时证明。原风险模型/critics/TF/正式配置/main保持，native/evidence复用冻结基础设施。

## 数据、结果、限制、下一步

待固定源与输入后评价。若H1/H2分离，后续只按失败层另行明确假设，不换理论定义救当前结果。若H1失败，不通过更大N或rho扫描证明它。即使全部门通过，仍须独立输入再研究，不能单周期多seed替代runtime验证。

源码/prepare`d91deff`。host Python3.12.3/NumPy1.26.4冻结8组raw controls及密度修正，原native评价固定Humble镜像`81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。input SHA256 `386bd836ac1a1b2b2e5057c861e6b94f03f8e71c6dc2903bbf6414b7292572eb`，parameter `c9141b999a93593102bfc4f4b0563c3c849e8d2635841bc708fc7665d5ec370d`；仅cycle40/std/N/mean原值，未读新候选标签。

## 算法结果与停止决策

native执行`9cebad0`。8组数值门PASS：pose最大4.768e-7，密度项原生score核对≤.000122071<.001，white目标理论prob与原生≤7.59e-7<3e-5，聚合在既有5e-5门内。

white四组全0联合安全，Ranking/输出不解释。rho **H1覆盖PASS、H2原目标排序/输出FAIL**：

| seed | 联合safe / 安全正推进 | raw static碰撞 | 最好safe名次 | 安全质量 | 最终ESS | 原native两filtered输出 |
|---|---|---|---|---|---|---|
| 0 | 31 / 31 | 49 | 55 | 2.80e-11 | 4.143863 | FAIL，body/pad均0 |
| 1 | 19 / 17 | 37 | 78 | 1.16e-12 | 1.001953 | FAIL，body/pad均0 |
| 2 | 22 / 19 | 42 | 38 | 1.30e-8 | 1.239750 | FAIL，body/pad均0 |
| 3 | 22 / 20 | 39 | 96 | 1.95e-12 | 1.691681 | FAIL，body/pad均0 |

四组Top1/5/10/30安全均0，首选均unsafe；原hardV1项在本输入0，仅复用事实，不重新验证CLOSED问题。修正前原成本排序最好safe14/174/32/105，也均首选unsafe，不能把失效完全归因于密度比。rho-only importance ESS1.152/1.001/1.062/1.001，safe logratio中位数比unsafe更低；原目标对这些时序提案的权重高度集中。未修改reference target/density/rho/temperature来救本实验。

所有single-safe/safe-only filtered两口径亦body/pad0，bounds还越界.003–.016m/s左右；不是通过放宽bounds即可变安全。表中的safe是原raw传播的动态+raw-map标签，它不自动证明原bounds/SG后的可行轨迹。固定零见证已有原门PASS，但此raw候选库是否覆盖同样可执行的输出尚需限定诊断。

**停止raw rho=.9+原white目标组合，不runtime集成。** 这是覆盖与权重/输出分离的算法结果，不是工具失败。[完整native/控制/density/逐候选/三类输出](evidence/raw_temporal_proposal_20261001/manifest.json)。四seed共用状态，不能称独立闭环。静态碰撞数量增加也是代价，保留。

下一项只补充本阶段必要算法归因：先预登记在这八固定库逐候选经过原bounds+SG+offset后的联合安全/推进数，区别raw几何覆盖与实际可用控制覆盖；原H1/H2结果不替换，不扩工具/变密度或未来risk。若没有可用输出候选，仍优先A；若有候选而排名/聚合不用，按B/C研究。旧clip/SG非Gaussian结论不重复计为成果，此诊断不计算mapped密度。

## 预登记的可用输出覆盖归因 H3

固定全部8库，不按安全挑raw候选，不变控制/risk/weight。对所有Ui单独one-hot原bounds+SG(history)映射，分别传播current-first与returned计划，原body/pad/raw203/整段bounds/严格推进全部保留。H3为四rho组每组至少一个候选同时通过两个完整输出口径；它不替换原H1。分开报告raw→mapped几何损失、static/推进及bounds排除数，不能把任意一个排除条件统称“没有可行控制”。

只复用已验证aggregate/Omni/mask函数，未计算mapped密度；源映射此前8候选已数值通过，本次只固定seed0白/相关各#0，用原nativeone-hot与returned plan核对新库接入（同5e-5/1e-5），不扩展numeric测试体系。prepare冻结所有映射控制/pose后再读新mapped标签。数据仍同一状态、标签已看，H3只限定下一实验层，不是新独立成功或controller修改。若H3失败，按几何/合法控制分别记录；不得修改SG/bounds门、反复试不同mapping或用单次见证代替统计覆盖。

H3源码/prepare`dbea66e`，input SHA256 `120cf6e0be272cfac6d142f26c58f42df09577d0b0265d32c516a3e2e54e3cb5`，parameter `4253852fcbbf0fafca52f8b3ed181098d9b90dfb304e1ea6c0a014ed154b4452`；原全部raw文件SHA沿冻结记录核对，不更换库/候选。映射host准备，原native/标签在固定Humble镜像评价。

H3评价`27574ca`：必要2个新库接入numeric PASS，filter≤1.1921e-7、pose≤4.768e-7。rho四库both-joint-geometry safe0/0/2/2，both完整输出数0/0/0/1，**H3 FAIL**；white两个口径几何皆0。rho returned几何safe1/1/3/4，其中原bounds合规的几何safe1/1/2/2；前两库都没有兼推进，seed2三条亦没有推进，seed3仅1条完整可用。

因此raw安全31/19/22/22主要不能保留为可用输出；不是纯粹由于SG bounds排除，几何和推进也丢失。零控制见证完整可用仍成立，rho机制没有稳定覆盖该可行区。停止rho及其原white权重组合的决定不变，H1原raw覆盖PASS亦保留。[H3完整结果](evidence/raw_temporal_proposal_20261001/usable_output_coverage/manifest.json)。

下一项A另行预登记当前均值+零控制均值的双Gaussian提案：同总N300/std/white covariance、固定150/150、四seed，不把确定性零原型当连续Gaussian密度。使用完整mixture密度与原white目标，先看可用控制覆盖，再看Ranking/输出；不是调rho或改变目标救当前组合。只改变提案均值结构，预测、cost、regularizer、aggregation全部冻结。仍可能因原目标对零均值区赋权不足而FAIL，需记录，不借此放宽门。
