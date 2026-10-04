# 新设备近场holdout三层诊断

## 背景与假设（先于标签和评分评价）

现有基础设施冻结。本阶段只回答当前实际300条候选在哪一层失效，不调整batch、temperature、prediction窗口或critic权重。新设备v4是新的输入运行，但其关闭监督器FAIL保留；完整窗口/tail/150周期可用于离线诊断。一个周期不能证明稳定闭环效果。

H1 Sampling：固定选择的周期至少有一条完整3秒联合安全候选，并记录其中正目标推进者。若无联合安全候选，停止Ranking解释，结论限定为当前采样/可达控制空间未覆盖安全解。

H2 Ranking（仅H1存在安全候选时）：实际在线总排序首选联合安全，且联合安全weight mass≥0.5，视为本周期排序门PASS。0.5是预登记的高权重区诊断阈值，不是安全保证。报告最好安全名次、Top1/5/10/30、安全weight mass、原生动态项/七标准项/正则/总分，保持原始实际probability。

H3 Aggregation：原在线加权后的完整序列经原bounds/SG滤波和Omni传播，是否保持body≥0.05m、padded>0、原CostCritic raw-map门与正目标推进。比较single-safe、oracle-safe-only、online-weight。oracle只用于离线归因。

## 输入与固定方法

- 分支 `codex/dynamic-differential-risk`；raw冻结提交 `0f85833`；采集源码 `6486643839e78c5cb5f5944ef4d24a945c28a41a`、执行 `f420a39ce7946a9f6ef259b07ae69f6c8ceaec6a`。
- capture `build/dynamic_research_device_20260930/new_device_near_x_capture_v4_20261001`；原input SHA256 `3565a62ed44b7e8e8b90d426dae10eefbef68c19d60c0f2b8fb66e63663d6b38`，原参数 `6f84d9233c36c59d0059408e71a497bb017128bcf7fc9272e82037cd3367d595`；capture_files `d14e3cb590872da30f6ce9853c960722c45d60ee938b8735509da73f672b1238`。
- 固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
- 唯一选择：按(sim_ns,cycle_id)排序，选择sim_ns≥(goal_sent_sim_s+2.0)×1e9的第一个周期；不依据安全、评分、prediction状态选择。goal发送22.102s，阈值24.102s。必要字段缺失直接记录限制，不换周期。
- 保持原profile 300×30、dt≈0.1、std .2/.2/.4、temperature .3、gamma .015、iterations1、offset1；body8顶点/pad .03/raw阈值203不变。
- future truth仅由Gazebo transport插值构造30个离线polygon，时间锚点固定cycle.sim_ns+(j+1)×实际dt；覆盖不足直接停止，不移动锚点。
- 静态安全采用本周期原raw CostCritic collision mask；输出复用原生costmap_mask_probe，不调整门。
- 候选目标推进使用冻结局部path末点距离减少>0，另报告正式目标(5.25,-1.10)距离变化。
- 实际prediction来自本周期native typed CDR及used_age事件，旁路observer不能替代。分项为原累积分数相邻差；不再以硬V1饱和作为新发现。
- single-safe固定选联合安全者中实际weighted.costs最低者（index打破平分）；safe-only对联合安全者实际weighted.costs重新softmax，避免原概率下溢为零。两者分别保持原采样/原控制bounds/实际history和SG，不引入运行时真值。
- 输出分别报告raw weighted、bounds后、SG后。传播采用原生Omni的“当前速度首列、控制序列前29列”口径，并额外报告从实际返回offset1开始、末列保持补齐30步的开环执行计划。两口径不混同，不宣称后者等于闭环真实执行。
- 数值口径：姿态误差≤1e-5、分数≤1e-3、聚合/滤波≤5e-5，最终安全门一致；不新增默认byte-exact门。若数值验证失败则保留限制，原capture标签仍可独立评价。

## 复杂度与复用

本阶段没有新运行算法。直接复用当前Nav2/Humble原生rollout、已有聚合/SG公式、原生replay与静态checker。唯一局部适配是读取events数组（CDR按bytes），直接支持三层必要字段。若Sampling FAIL，下阶段仅研究时序结构；若Sampling PASS但Ranking FAIL，仅研究连续动态风险；若前两层PASS而输出FAIL，才研究最小aggregation机制。三层都PASS后停止本输入离线调参，预登记最小runtime A/B。

## 数据、结果、限制与下一步

评价前冻结：算法源码提交 `28d6dc0`；input SHA256 `4fe24a21df02f8ceeaf2d76016c06ec776300118686c577a7bc5de29c81459ee`，parameter SHA256 `6abc7e3e8d8426613f223efbfd63eadc1e30b70c795cd216389197ed141faeab`。规则选中cycle23、27.719s，比24.102s阈值延迟3.617s；阈值后此前无MPPI周期，保留该时间缺口，不换周期。输入目录 `near_field_three_gate_inputs_20261001`。

执行提交 `1848f85`。所有输入、逐条候选评分/标签、原生输出和日志保存在 [evidence](evidence/near_field_three_gate_20261001/manifest.json)，[summary](evidence/near_field_three_gate_20261001/summary.json)。关闭失败不改写为PASS。

### 三层结果与决定

**Sampling PASS，Ranking FAIL，在线Aggregation PASS。下一阶段只研究Ranking。**

Sampling：295/300联合安全，静态碰撞0；135条安全者正推进目标。最大body gap0.541584m。当前场景输入有大量安全解，无理由扩大batch或修改sampling。

Ranking：总分首选#293不安全，最好安全名次2（#236）；Top1/5/10/30安全数0/1/5/25，实际safe weight mass0.717214。质量虽超过0.5，但首选未过门。分项/正则/总分完整见candidates.csv。V1原硬项几乎恒定的既有结论不作为新成果。

Aggregation原在线完整输出通过两种传播口径，不能因为top1不安全就断言这次输出不安全。三种filtered输出如下；全部raw静态门通过、整段bounds合规，且正推进。

| filtered输出 | 当前速度首列body/padded gap m | offset执行计划body/padded gap m | 两口径推进 m |
|---|---:|---:|---:|
| online实际权重 | .143531 / .113284 | .131078 / .101055 | .153420 / .159275 |
| single-safe oracle #236 | .099844 / .064285 | .097173 / .062147 | .172466 / .146810 |
| safe-only oracle | .180014 / .147728 | .175090 / .142096 | .117409 / .122389 |

single-safe的未滤波offset计划仍FAIL（bounded body .028964m、padded0），SG后PASS。原raw该候选有.032929速度越界；原bounds消除。本阶段不能把原候选标签当作执行计划安全：时间索引口径和滤波都会改变完整路径。这里没有观察到safe-only均值破门，旧oracle否定结论仍保留，不能据此推翻。

### 数值与限制

捕获rollout与Omni镜像最大误差4.76837e-7；原生replay姿态/七标准项/weighted.costs/before_filter误差0，after_filter7.45e-9。实际probability重聚合before误差0、after1.49e-8、返回0；通过既有阈值，不要求跨设备byte-exact。

只有一个固定周期，且选择规则延迟3.617s；原采集10次recovery和关闭FAIL保留。30步polygon采样与Gazebo线性插值标签不是连续时间无碰撞证明；三秒开环执行计划不是闭环物理控制。风险尚未替换或扩展，原生输出安全只证明这个冻结输入。

下一步预登记一次固定连续风险在此新输入的排序与输出检验，不回调已有失败组合的单位/窗口/temperature；成功后必须在新的独立运行固定时间holdout验证，失败则停止该组合并归因信息/排序。main、默认profile、运行critic/TF/footprint/安全门均不改。
