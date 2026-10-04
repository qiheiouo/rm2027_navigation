# 标准任务成本的候选输出对象一致性

## 背景、假设及最小选择

本实验回到已有排序信息的early v4 cycle23及late01 cycle219，不救援cycle40已否定的CA/support。仅风险评价候选returned plan的旧组合已FAIL：late returned body.043611m，early returned推进−.011013m；标准任务成本仍评价raw rollout。原CA early首选#67 one-hot returned本身退离目标，故单纯用MAP样本替代加权均值不是充分的最小修复，此处不为MAP重复写试验。

H0：既有native能在同参数下对原实际候选返回计划计算标准任务分数，单候选与批评分的必要核对通过。H1：仅替换标准任务成本的评分轨迹对象，保持既有mapped CA risk、采样与更新，两个固定输入的raw Gate2均PASS。H2：两个输入的online filtered两口径完整输出均PASS。任何门失败保留并停止，不调Goal/Path/Constraint权重、risk单位或聚合函数。

公开[Nav2 DWB轨迹生成](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)先模拟受运动学限制的速度与位置，[DWB评分](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)再评价生成的轨迹，可作成本对象与可执行轨迹一致的成熟实践参考。本实验不引入DWB、不改变规划器或模块接口，只用原native scorer改变离线函数的输入。

比新控制器/安全监督器更小的改动是评分对象替换；只移动soft risk已经失败，单独改Goal权重也已CLOSED。此处保持相关标准函数及权重，统一六个非静态critic的对象，避免继续逐项调参数。**原CostCritic仍按原raw rollout计分、原静态安全门不变。** 不声称七项全对齐：CostCritic含all-trajectories-collide批门，不能把单候选批fail改成自造成本。保留它也避免放松或重写原静态判定。

## 方法与冻结边界

复用旧candidate_output_{earlyv4,late01}_inputs，原300 raw Ui、实际4 history、bounds+SG、offset1重复末步补齐30得到的F-returned(Ui)、mapped CA risk全保持。原baseline和旧风险映射结果直接引用，不重跑其整个实验。只新测Constraint、Goal、GoalAngle、PathAlign、PathFollow、PathAngle六项在F-returned(Ui)上的分数。

原native六项raw批评分记S6raw；六项returned评分记S6out。新周期成本为`Cnew=Cwraw+(S6out-S6raw)+original_unit*mapped_CA_risk`，Cwraw保留原CostCritic、hardV1、regularizer。采样仍原raw控制和原目标定义，不在非线性mapped控制上套Gaussian密度。模型/风险/标准权重/temperature/更新函数/footprint/padding/TF均不变。

原native的当前速度首列每批共享，而F-returned首列随候选不同；最初预登记计划300个单候选测量。源码检查发现PathFollow/PathAlign使用整批furthest-path缓存，该方法无法保持原批上下文，**在生成新评分输入前弃用单候选方法**，不将它执行成等价结果。

缺失的具体能力是同批各候选首列速度不同，直接阻塞此B成本对象实验。按用户允许“具体字段缺失直接阻塞三层PASS/FAIL”的最小例外，仅在experiments的离线probe加入可选`offline_state_velocities_file`，读取原已冻结F-returned速度张量，替换用于传播/critic的state.vx/vy/wz；sampled控制、原正则/更新、runtime代码不变。其余入口/二进制身份/记录体系不扩展。保留原e667 binary，新binary独立build目录，禁止覆盖旧产物。

必要H0为新binary默认入口对两个旧原七critic frozen baseline数值核对（score≤1e-3、pose≤1e-5、概率≤3e-5、before/after≤5e-5），再整批override六项评分，原生pose对已有冻结mapped poses≤1e-5。源控制映射已有原native核对，不新增工具测试框架/recorder或byte门。任一测量缺失/核对FAIL则算法未评价，不删候选、手工填分或放宽判据。六项分数和新成本delta冻结后才读取未来安全标签。负delta以同批常量平移复用原nonnegative additive接口，概率/成本/聚合沿原门。

Gate1/2保留原raw labels，另报告已有mapped标签作诊断，不能替换主门。Gate3照旧single-safe/safe-only/online、weighted/bounded/SG、native current-first及returned/full3s/body≥.05/pad>0/raw203/整段bounds≤5e-5/严格推进。只early改善或只late改善不计完整机制PASS。

## 数据、结果、限制及下一步

预登记时状态：未prepare/evaluate。现已完成，最终结果见下节。两个已看同scene输入，不是独立验证，不补齐CA协议，不部署。每输入分别固定source、参数、native binary与image hash，新目录保存全部单测及最终结果，不覆盖旧风险映射证据。

若H0技术前提不满足，不修线上critic/工具门追PASS；若H1或H2失败，停止该对象对齐组合，不接着改权重、采样或聚合。即使两输入PASS，CostCritic/hardV1/regularizer对象仍未统一、非凸均值仍无保证，只有资格另立独立验证协议。cycle40预测失配尚未解决，不能把这个实验成功外推到它。

源码/prepare commit `c3d26f7bb9169452ce4b81fa793f7a7a93ab9c94`；新probe source commit `013a514863f36936a9b9bc3878590b5454ba3591`，独立单线程Release build完成。probe CPP SHA256 `ae94353cbf8b6beecc70142aa770e99ad8bf9fac6a5fd31a7355865275ed7865`，实验脚本`f9bc79e6cb9797a1d67f060331dbc0971cfe32d1c68f727831f33b2557518a35`。

earlyv4 input SHA256 `a18612d0329ea0f2591e5799b81f1bfed18a68096bec733244dc456fa95c154a`；late01 `ec1a94bd32b1370d542d987d72f6be9c709e2ce2c8da2edb36d93f271fe8d93f`；共同parameter `e560b7d7305dab191198353fee33bd43c502de5d3e41abf1e0e95f222751bdce`。原始raw、旧mapped风险及六项分数全部在新输入目录冻结，评分未使用安全标签。

**H0 PASS**：两个默认入口score/prediction/weighted/pose/before/after/prob对旧counterfactual baseline误差全0，原阈值未改变；两个mapped批传播pose误差4.768e-7，raw六项批pose误差0。这里只是当前原设备相同数据实际观测，不提出新的跨设备byte门，不扩工具测试体系。

新binary SHA256 `85b0ecb1ead6462ab9f180b64a882787d1afe29c44994d0430f1252d14951ba1`，镜像保持`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；原e667 binary未覆盖，raw static checker保持`ce1c81c3f86bb5d87b188529e313e2694c10addafd9aedcc1807b1f0095d7faa`。原正式source/config与TF合同无diff。以上为输入冻结时记录；H1/H2最终判定如下。

## 最终结果与停止决定

预登记提交`7241d36f88094d7fed5dfbed09518bb3c81b8df6`，冻结输入及算法执行提交`b3f706508f6422c44ef772c6886f01abc054d036`。两个新结果目录均正常完成；未重复prepare或执行旧组合。数值PASS与算法PASS分别报告。

| 冻结输入 | 数值门 | H1 / Gate2原raw排序 | H2 / Gate3最终filtered输出两口径 | 单输入算法判定 |
| --- | --- | --- | --- | --- |
| earlyv4 cycle23 | PASS；score误差0、概率2.4662e-5 | PASS；首选#67安全且推进0.088319m，安全质量0.639365 | PASS；current/returned本体0.132890/0.131466m，推进0.115443/0.086124m | PASS |
| late01 cycle219 | PASS；score误差6.1035e-5、概率2.0182e-6 | FAIL；首选#231原raw不安全，最好安全第2，安全质量0.493066 | PASS；current/returned本体0.068207/0.056962m，推进0.627075/0.650804m | FAIL |

最终filtered四条online输出的padded间隙依次为0.094915/0.093663/0.038207/0.026962m，原静态门均无碰撞、整段bounds违规均0、推进均严格为正。原生聚合before/after误差early为2.9802e-7/1.0431e-7、late为0/5.9605e-8，均保持原容差。Gate1原raw联合安全仍为295/57，安全且推进135/57，没有改标签或候选。

**协议整体FAIL：H0 PASS，H1要求两个输入均通过而未满足，H2两个输入通过。** late安全质量低于原0.5门且首名不安全，不能因接近阈值、最终输出通过或改用mapped标签而重判。保留完整输出的局部改善：旧仅映射风险early returned推进−0.011013m、late returned本体0.043611m均失败，本次分别转为0.086124m、0.056962m通过；这支持成本评分对象对C有影响，不能证明排序已稳定正确或风险预测可靠。

early/late Top1/5/10/30安全数分别为1/3/7/26及0/2/2/13；总成本安全低分AUC为0.070508/0.766298，ESS为3.109852/2.882778。early全局辨别仍很弱，不能把首名与质量门通过解释为全局排序良好。只报告两个既有相关输入，不能当独立验证。

single-safe oracle早期#67 returned本体0.180865m但推进−0.033865m，仍FAIL；其current口径及late两口径PASS。safe-only oracle两输入两口径最终均PASS，均仅用于离线归因。late未滤波weighted bounds违规0.049554、returned本体0.040972m，bounded returned本体0.041697m仍FAIL；最终SG通过不意味着各中间轨迹都安全，更不构成连续时间安全证明。

## 证据、限制与恢复条件

[evidence manifest](evidence/standard_output_cost_alignment_20261001/manifest.json)与[result.tar.gz](evidence/standard_output_cost_alignment_20261001/result.tar.gz)保存两份完整冻结输入、prepare中的三个native批评分及H0、最终native/静态/控制证据、全部构建与执行日志、实际新binary和冻结Python/CPP源码。归档2,711,728 bytes，SHA256 `d5fa265be3d6f9187938aeff8052502cf73aa9963f930d44cd6d6ee60b943917`。early/late摘要SHA256为`855e166a2a899c65f19c2fe9977a1b88677afbe4b7bcc3937a9e0e60a8902468`、`c378b89a2c256620923431ce0b61d7f28268e2d1106a20009161f672f77cb8f8`。原输入文件hash、新binary/source及归档内外摘要核对通过，旧evidence未覆盖。

停止此冻结对象对齐组合，不调整critic权重、风险单位、temperature、采样或聚合追求通过。仍只对齐六项非静态critic，CostCritic/hardV1/regularizer保留raw对象；cycle40 CA/support失配与一般非凸聚合安全均未解决。没有修改运行链，也没有闭环/部署验收。

用户最新要求先收尾并换思路：本实验已完成归档，无待执行项；整条现有研究路线暂停，自动续跑`rm2027`已PAUSED。下一步由用户的新思路决定，只有用户明确要求回到此路线才恢复；届时从[检查点](research_continuation_checkpoint_20260930.md)读取，不重跑已完成结果或删除否定证据。
