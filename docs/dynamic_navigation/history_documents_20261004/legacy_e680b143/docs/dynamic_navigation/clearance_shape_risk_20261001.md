# 冻结混合采样库上的间隙风险形状对照

## 背景、最小假设及公开参考

zero_mean提案在late v4 cycle40的四seed覆盖46/29/36/49条完整可用输出，但原white目标下实际安全质量仅1e-38–1e-37，排序和输出皆FAIL。采样在此诊断输入暂不再改变，下一项只研究B：在线风险能否抵消该状态原成本/密度对已存在可用安全区的拒绝。停止此前无风险mixture组合的决定保持。

原expected-overlap只度量原padded footprint接触面积，没有直接表达body≥.05m的裕量。此前CA改善排序却returned body.032983m也提示该缺口；该提示不是新实验通过证据。[Nav2 1.1.20 ObstaclesCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/obstacles_critic.cpp)已有按障碍间隙及collision_margin_distance产生近碰撞连续成本的公开实践。本实验不引入该critic、不复制其配置或代码；只复用已有polygon_distance，把同一预测支持下的风险核替换为原门规定的间隙不足。没有新planner、IMM、supervisor或geometry框架。

H0：原因果四源近面构造条件在cycle40可用；若不满足，保留输入FAIL并停止，不选择新源/周期。H1：固定CA支持下的间隙风险可使四seed原raw总排序均top1safe且safe mass≥.5。H2：原聚合/bounds/SG两口径的完整输出亦全部通过。任何主门失败记录否定，不调风险单位、margin、支持、temperature或密度；只排序PASS不等于控制PASS。

## 方法与冻结边界

固定zero_mean四个mixture raw bank、原state/path/map/硬V1/七标准critic/regularizer/history/bounds/SG/N300/std、white参考目标的mixture密度修正。对每bank新增两个预登记成本臂：同CA模型的expected-overlap正对照，以及新的clearance-deficit形状；原无风险结果沿上一阶段，不重跑white库。

CA采用既有四源近面center/六对斜率中位速度及[1,t,.5t²]拟合加速度规则。source只能从实际consumed snapshot约束下的最新四个producer-side源扫描及原TF构造；H0只按线上构造条件判定，未来truth不参与源选择、模型参数或风险。新cycle使用旧冻结公式，不调CA或支持；不是加入新的运动模型。四源实际是否都到达MPPI仍只有限证据，不扩receipt/TF工具。

两个成本臂共用原CA均值、原不确定性half-support、物体.45×.55m和5×5 Legendre中心积分、全部30步mean及原collision_unit=1666.6666259765625。overlap沿旧padded footprint面积归一公式。clearance使用**原本体footprint**对每预测物体求原polygon_distance d，核为`max(.05-d,0)/.05`，求同中心积分和30步均值。没有改变本体/足迹、padding或硬占用；margin直接来自原验收门，无自由可调值。核在接触处为1，不额外引入穿透深度/符号距离。

重要性目标仅在此新成本假设中明确为`qwhite(U-U0)*exp(-(Cw(U)+unit*risk(U))/lambda)`；qwhite/qmix密度比与原值逐项保留。native additive=原nonnegative density项+unit*risk。风险只从原raw rollout计算，评分对象/aggregation本阶段不再做候选输出映射；不得借此复活已停止的C映射组合。先冻结模型/全部风险/参数/source hash，再读取安全标签评价。

原Gate1/raw标签不替换成mapped标签；上一阶段可用覆盖只定位A。Gate2报告总排序/Top-K/actual质量及risk-only AUC，risk-only无实际MPPI权重。Gate3照旧online/single-safe/safe-only三种赋权、weighted/bounded/SG、native current-first及returned-offset-plan/full3s/body≥.05/pad>0/raw203/bounds≤5e-5/严格正推进。数值沿既有score1e-3/prob3e-5/pose1e-5/aggregate5e-5，无新增byte门或工具测试体系。

## 数据、结果、限制及下一步

本阶段尚未prepare或评价，源码和输入身份待冻结。唯一已看cycle40与四相关seed，近面模型及零模式均不是盲测；结果不补齐旧CA独立协议。任何阳性还须新的独立输入验证，三层未齐不得runtime。未来真值仅离线标签，TF/topic/正式配置及main保持。

若overlap与clearance都FAIL，停止当前CA支持+风险形状，不继续提高unit或试新的核。若clearance排序PASS而输出FAIL，仅定位剩余C；若两臂都PASS，不宣称clearance独有增益；若只有overlapPASS，也保留clearance否定。无论结果，下一机制须另行提出假设，不能在同输入连续调参。

H0构造执行`e2c13be061cf7b3c8c09d0fc9cca6b603cd94cc9`，原四源规则在cycle40最新源27.853/27.919/27.985/28.051s可用、errors为空，**构造H0 PASS**。source input SHA256 `79bd423bced4d6b44a81cadf8793ea7f55c077c2d33763fe2b09daec63fc93e2`，parameter仍`f2d97785237b59214b69ff0143e044dc8e603278547cbf640a887035a7eeef38`；未用源truth误差决定是否构造风险。这不是未来预测或排序PASS。

实现只新增离线实验脚本`clearance_shape_risk.py`；复用原polygon_distance、overlap_area、native和output_cases，无新依赖/recorders。距离包围盒下界≥.05时clearance核严格为0，可直接跳过精确距离，不更换几何门。新cost独立保存density与risk，理论概率包含二者，不把风险伪装成log density ratio。数值FAIL则算法未评价，不计机制FAIL或PASS。

风险prepare commit `dc50027ad794672797739e888e6b32d7de5fa95e`，host Python3.12.3/NumPy1.26.4；input SHA256 `e01a2227d86de248c8c16afde6c0ef2d9e0b5cc544782077fe55e8a66abc7148`、parameter `7e4b154abd2b31bf7c53d8d6faccab7d3d52517dad6d8184dbb50800b4dca5cf`。原raw bank input保持`9f5666e4b5584366c05f53ba2700ea1095be6d2a74f261e7be918607f5fc6fb0`，全部风险冻结后才读标签/native评价。overlap范围约.01209–.01848，clearance约.05714–.10085；有风险数值差异不等于安全辨别。

原生镜像保持`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，replay binary `e667e19f75d7b5a937f7df78191359ff0d3e8c35fbdd017ae54d123ff1fa7d5c`、raw static binary `ce1c81c3f86bb5d87b188529e313e2694c10addafd9aedcc1807b1f0095d7faa`。新脚本source SHA见冻结inputs.json，原output helper为`72aaf0c66296b8d80c2d766e644d645a0ce656e24beeed43bae31dfa544343a1`。输入目录保留模型/未来支持/每步和均值风险/逐文件hash，结果使用全新目录。

## 结果与方向停止

执行commit `8ed5c1f0473e697d129ab6d1357e2d8e7e58322b`；脚本SHA256 `5c701763fc4e5ed39ade7355c0055c1cb2937650557da5738b2fac1de84fe5b0`。8个arm/seed数值全部PASS，pose误差0、score≤3.052e-5、理论概率≤2.141e-6，聚合沿原5e-5门通过。不是工具失败。

| seed | overlap最好safe名次 / 质量 | clearance最好safe名次 / 质量 | risk-only AUC overlap / clearance | 两臂最终online filtered |
| --- | --- | --- | --- | --- |
| 0 | 156 / 9.99e-43 | 139 / 0 | .360564 / .220575 | 两口径body/pad均0，FAIL |
| 1 | 171 / 2.66e-44 | 155 / 0 | .384458 / .251118 | 两口径body/pad均0，FAIL |
| 2 | 167 / 2.61e-43 | 155 / 0 | .398004 / .256325 | 两口径body/pad均0，FAIL |
| 3 | 162 / 7.01e-45 | 154 / 0 | .308070 / .203252 | 两口径body/pad均0，FAIL |

两臂H1 Ranking、H2输出全部FAIL，Top1/5/10/30安全全0。表中clearance质量0为原native float32权重下溢结果，不宣称数学概率严格为0。四组safe风险中位数均高于unsafe：overlap约.0160对.0154–.0156，clearance约.0842–.0848对.0783–.0792；惩罚形状对齐真实裕量不能补偿当前预测排序方向错误。硬V1原值及原密度修正保留，不增unit救援。

冻结模型a=(1.826308,.714963)m/s²，source v=(-.601582,-.026060)m/s。在算法评价结束后，仅用真值作未来预测标签：30步中心误差逐轴最大4.944705/3.267111m，真中心落入原center-support仅16/30步，末步support-half为2.938711/3.141817m。center-support命中不是完整占用覆盖指标；此诊断也不证明在线信息对一切模型不足，只说明这个固定CA/support未表示该三秒运动。原近面source可构造不等于长时预测正确。

**停止当前CA支持+overlap/clearance形状组合，不试新的核、unit、margin、支持或温度；不runtime。** 仅去掉拟合的横向加速度也不足以解释并修复x方向大误差，暂不为此另写投影模型。此前旧CA其他周期的排序阳性保留，本cycle否定不回填其他实验。

[所有输入、模型、每步risk、原生及三类输出证据](evidence/clearance_shape_risk_20261001/manifest.json)，archive SHA256 `bfa78e3739b14e3a1da128700410820eee1e5712e5133e097b6d1c2a3d3638fb`。四seed同一已看状态，不是独立算法失败率。新的预测模型需要独立假设，不能靠不断改变此处CA公式追结果。

下一研究先处理已有有效排序的旧输入之输出一致性，而不立刻给cycle40增加预测架构：late01 cycle219的CA首选安全、加权输出FAIL；候选输出风险映射又使earlyv4返回推进为负。现成风险映射只对齐了soft risk，标准七critic仍评分raw rollout。更小的下一假设可固定该映射/risk/采样/聚合，仅把标准critic评分对象改为原实际候选输出计划；须另行预登记，不调整Goal/Path权重，不把MAP或single-safe当一般保证。当前尚未实现或评价该新假设。
