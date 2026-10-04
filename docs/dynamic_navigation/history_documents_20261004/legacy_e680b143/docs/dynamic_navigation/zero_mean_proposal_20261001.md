# 当前与零均值双Gaussian提案

## 已复现问题、假设、成熟参考与最小选择

late v4 cycle40：原white四seed均0safe；raw rho增加31/19/22/22几何safe，但原bounds+SG后两个口径都完整通过的候选仅0/0/0/1。原单一零控制却完整通过。当前均值附近的时序相关噪声仍未稳定覆盖可用控制区，rho机制停止。

H1：固定当前均值与零均值双Gaussian，可在同N300/std下四seed每组提供至少一个原bounds+SG后两口径完整安全且推进的候选。H2：使用完整mixture密度修正保持原white周期目标时，原Ranking及online输出门亦能通过；若只H1通过H2失败，停止当前提案+成本组合，不删修正或换目标凑PASS。

[pytorch_mppi源代码](https://github.com/UM-ARM-Lab/pytorch_mppi/blob/master/src/pytorch_mppi/mppi.py)有sample_null_action及specific_action_sampler，可作为保留零动作候选的公开实践参考；不复制其实现/依赖或宣称它的安全保证。单个确定性零动作已只构成见证，不能直接当continuous Gaussian密度。更小且密度可算的实验是围绕零控制增加一个满秩Gaussian分量，保持已有sampler/cost/control函数；不引入新planner、行为监督或轨迹原型库。

## 固定方法、目标与门

唯一cycle40，原state/path/map/snapshot/history/7critic/V1/regularizer/temp/aggregation/TF及所有安全门不变。原N300/std/white时间协方差，四seed0/1/2/3；white对照直接复用原四个冻结white库。新mixture固定前150原均值U0、后150零均值，每分量同std、共同原seed创新；不预clip/filter，不加入确定性atom，不选择truth动作。比例.5不扫描。

`qmix=.5*N(U;U0,Sigma_white)+.5*N(U;0,Sigma_white)`。

固定原white目标`qwhite(U-U0)*exp(-Cw(U)/lambda)`，Cw仍原native总成本。用`logratio=log qwhite-log qmix`修正；两个相同cov常量抵消，用二次型与logaddexp计算，使用实际float32 Ui作连续密度近似。固定150/150分层抽样配同qmix分母，未归一积分的期望仍是原目标（两分量加权和=qmix）；不冒称有限样本无偏归一概率。same nonnegative additive common-shift接口复用旧数值门，无新density框架。

先prepare冻结raw controls/density，再原native三层评价；使用已有原参数输入读出两个arm名称的最小离线复用改动，不新增recorder/receipt/TF/identity。必要的可用输出coverage同H3映射函数，全候选原bounds+SG/current-first+offset/body≥.05/pad>0/raw203/整段bounds≤5e-5/严格正推进，不能降低门。H1要求四mixture组各≥1两口径完整可用；H2要求四组原raw安全候选存在、修正ranking top1safe+质量≥.5、online filtered两口径全门PASS。比较single-safe/safe-only/online，oracle只离线。

先保存raw覆盖/参考成本/risk/密度/ESS结果，随后另冻结全部mapped控制/pose再标签。原两个口径/数值容差保持，不以mapped label替换原Gate1/2。均值改变不一定克服原white目标对零区赋权不足；如果H1改善而H2失败，依据失败层另立成本假设，不继续改变mixture比例/方差/target或增加batch。

## 数据、结果、限制、下一步

**08UTC收尾时状态：预登记及离线源码准备完成，输入尚未生成或冻结，实验未执行，H1/H2均未评价。** 2026-10-01用户要求先给当前内容收尾；本方案当时不计入已完成成果。源码仅作Python语法和diff检查，不代表数值或算法验收。后续必须先记录prepare commit、输入/参数/source/binary/image hash，再评价。

四seed同状态、零模式动机来自已看可行见证，非独立blind holdout；无闭环/实时或一般性保证。host准备控制与密度，原native在固定Humble镜像81b325评价。任何结果保留，包括有效覆盖却权重拒绝。三层未全通过不能runtime集成，main/正式配置保持。

复用`raw_temporal_proposal.py run`时，`inputs.json`分别记录prepare脚本与evaluation脚本SHA；旧AR输入仍沿原`script_sha256`入口。`raw_output_coverage.py`仅从冻结参数读取white/mixture两臂及treated_arm，原阈值/映射不变。旧实验须使用各自记录的Git源码版本，当前源码变动不回填或重新解释旧结果。

## 09UTC恢复后的输入冻结

prepare commit `9182e48723fb444316e4a37b5fcc14b3b2ee9d41`；host Python3.12.3/NumPy1.26.4。原cycle40及原white库不变，无新标签参与生成。raw input SHA256 `9f5666e4b5584366c05f53ba2700ea1095be6d2a74f261e7be918607f5fc6fb0`、parameter `171b420d68dc25b32de88b7157db0127e9d9d35e05073693a3707043e46417c9`。

同时在标签评价前冻结全部one-hot原bounds+SG控制及两口径poses；mapped input `f07b4a8ce565fdbac762795a5078b14c0e23a9675764381e752382c6b3dbdba9`、parameter `f2a7b0c10c47bef30ce3a052c30eaaf75cbc6adf083de80970b1dd1a1501250a`。仍先报告raw三层，再判完整可用覆盖H1；generic脚本的`H1_coverage_verdict`只表示raw覆盖，主H1须用`H3_usable_coverage_verdict`，不能混同。

准备脚本SHA256 `b572fe94c57b34da8bc3d3c1ccb4b54209c07d354e0d7c7aec530312215b8c0f`；原三层consumer `72aaf0c66296b8d80c2d766e644d645a0ce656e24beeed43bae31dfa544343a1`；可用覆盖consumer `7b35c99b91196ca9f122011cc0c8cecbd59bf054efc96b4846228c12dcdcdafd`。replay binary `e667e19f75d7b5a937f7df78191359ff0d3e8c35fbdd017ae54d123ff1fa7d5c`；raw static binary `ce1c81c3f86bb5d87b188529e313e2694c10addafd9aedcc1807b1f0095d7faa`；原生镜像`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。各输入目录保存逐文件SHA；结果/evidence使用新目录，不覆盖ρ或white旧结果。

## 算法结果与停止决定

原生执行commit `40a5a4027d6aa19532aaf47328fb83737b39b6ff`。8库raw数值PASS：pose≤4.768e-7、附加score≤7.63e-6、原white目标概率≤5.12e-8；online before/after聚合满足原5e-5门。可用覆盖接入的2个数值核对PASS，filter5.96e-8、pose4.768e-7。

| seed | raw联合safe / safe且推进 | 两口径完整可用候选 | 修正后最好safe名次 | 实际安全权重质量 | ESS |
| --- | --- | --- | --- | --- | --- |
| 0 | 122 / 60 | 46 | 163 | 6.51e-38 | 26.4525 |
| 1 | 106 / 45 | 29 | 168 | 1.99e-37 | 33.8080 |
| 2 | 116 / 58 | 36 | 167 | 2.04e-38 | 20.4528 |
| 3 | 123 / 63 | 49 | 157 | 6.75e-37 | 40.5353 |

white四库raw及完整可用输出皆0，Ranking不解释。mixture主**H1完整可用覆盖PASS；H2排序和实际输出FAIL**。四组Top1/5/10/30安全皆0，online filtered两口径body/padded均0；三组另有bounds越界.000904–.006315，seed2没有bounds越界但仍动态FAIL，不能只靠clamp解释。静态碰撞0保留。

单安全候选离线oracle：seed0/2/3两filtered全门PASS，seed1 current-first body.024078 FAIL而returned .057220 PASS。safe-only离线oracle：seed0/2两口径PASS，seed1/3 current-first分别.025503/.043443 FAIL，returned均PASS。这些对照不能部署，但区分了有完整可用候选和原赋权不使用；也没有把单条raw safe或safe-only变成一般输出保证。

不含密度修正的原成本仅作诊断：四组最好safe第154/164/161/155，首选仍unsafe，安全质量约1.56e-7–4.07e-7。safe原weighted cost中位数7.05–7.29、unsafe2.45–2.61。mixture零均值部分log(qwhite/qmix)中位数约−83，当前均值部分约ln2，进一步压低原目标对零均值安全区的权重；不是ρ阶段那种ESS接近1的同类退化，也不能把全部失败归咎于密度实现错误。

**停止当前mixture+原成本/white目标组合，不改比例/方差/temperature/target来救结果，不runtime。** 采样机制已在同状态四seed稳定改善可用覆盖，当前失败优先属于B权重/成本；非凸C限制仍保留。下一项若推进，须固定这四个完整bank，仅另行预登记在线风险形状假设，保留原密度修正/成本/聚合/门；不得重新扩大batch或重试调参数。

[原输入、完整native、密度、所有控制/标签及两种覆盖结果](evidence/zero_mean_proposal_20261001/manifest.json)，archive SHA256 `e54de2d57bcb2f332fbad195ec04d62cef0b0321b892a40c81c1885fa6c0cf56`。四seed同一已看cycle，零模式动机来自旧见证；不是独立holdout、闭环或运行改进验收。09UTC恢复结果不覆盖08UTC收尾时的“未执行”历史状态。
