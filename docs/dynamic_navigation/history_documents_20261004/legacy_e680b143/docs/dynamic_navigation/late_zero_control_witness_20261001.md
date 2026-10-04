# 晚期零安全批次的固定零控制见证

## 背景与假设

late v4 cycle40/28.106s的原300×30全部动态不安全。此前std4/更大batch方向已CLOSED，不能再放大batch。单个批次0safe也不能证明当前状态下不存在安全控制。

H1：固定零控制在同测量speed/pose/map和未来完整3s上存在原生联合安全见证；若成立，原分布的覆盖不足得到明确反例。H2：原全部候选第一步若安全、后期才破门，0safe不能归因于共同首列已经不可控。若零见证失败，只否定这一个见证，不证明不可达或尝试更多原型凑PASS。

## 最小方法与门

仅已有late v4 cycle40。预先固定唯一控制Ui=[0,0,0]×30，不选方向/幅度/真值动作、不试27原型。保持原测量speed第一步、原Omni传播、30×.1s、八顶点body/pad.03、原rawmap203/静态probe/安全门。原CA模型与C输出风险失败组合不改，不runtime集成。

prepare保存原实际V1 snapshot/age、原control history[4]、map、零control和输入/参数/commit/镜像hash；不读未来标签。native不运行标准critic评分（本实验不是Ranking），只传播同状态的零见证；原V1消息保持，静态安全由原probe独立核对。报告原native零候选以及该one-hot零控制经原bounds+SG后两时间口径的动态/静态/bounds/推进；不重置history或用理想静止状态代替实测speed。

H1门仅原native零候选body≥.05/padded>0/raw203无碰撞；同时单独报告原Gate3完整标准（含bounds/正推进）。零控制只是Sampling可行空间见证，不能因仅安全而宣称可部署MPPI或替代导航。H2固定报告原300每步安全数、首次破门步分布及共同第一步gap；不改时间窗或以安全前缀验收。

可复用成熟部分是同Nav21.1.20 Omnidynamics/控制约束/SG和既有探针；新内容只是一个预定动作的算法可行性评价，无新架构/模型/工具。更小方案仅看第一步不足以排除后期覆盖缺陷，故保留完整3s，且只做一个见证。

## 数据、结果、限制、下一步

待冻结后评价。若H1+H2支持，下一项另行预登记同N/std的时间结构采样假设；不从一个零动作推断相关噪声一定有效。若FAIL，记录否定结果，不重试更多固定动作、降低安全门或声称无法避障。未来truth只离线标签；main/源码/config/TF保持。

源码/prepare`2f1412c`，镜像固定`81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。input SHA256 `5c752d51c593f3b2209d1a7568e31e5403afd0a5504f1ad96e6203d3ce423525`，parameter `8bb744dabae0ee906a0d2bdb608ddaee0ccafe0d50a372e1a5361c94c7bf98fc`；原实测speed(-.297777,0,0)，不是理想静止机器人。未读未来或增加动作。

## 算法结果与层定位

原native评价`7264e29`，H1 PASS。唯一零候选body/padded .158428/.124611m，原静态无碰撞、bounds0，推进.029467m。one-hot原SG输出current-first .125162/.091065、推进.061105；returned-plan .174631/.140710、推进.012681；两口径均通过原完整输出门。即使称为零控制，原实测首列和SG历史仍造成少量运动，不能误称理想静止。

H2 PASS：第一步300/300动态安全，body/padded最小.864056/.830612。首次破门步16/17/18/19/20/21/22分别18/59/121/76/18/7/1条；第22–24步所有候选破门。完整3s0safe保持，禁止用前15步安全替代验收。

因此此状态存在同原完整门通过的控制见证，原批次0safe属于实际分布覆盖失败，不能归因于共同第一步不可控，也不能证明场景不可避障。[全部冻结/native/每步结果](evidence/late_zero_control_witness_20261001/manifest.json)、[summary](evidence/late_zero_control_witness_20261001/summary.json)。这是单周期可行性结论，不代表零控制策略或原MPPI闭环部署通过。

下一项A先预登记原N=300/std/mean下的raw时间相关Gaussian提案，固定rho与多seed；禁止预clip/filter后套Gaussian密度，原bounds/SG仍只用于实际聚合之后。若要输出控制，必须对原white固定周期目标给出proposal density修正并报告ESS，不能直接沿用对角正则冒称相关MPPI正确。不扩batch、不改预测模型/critic/安全门。
