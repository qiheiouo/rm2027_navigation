# 在候选输出计划上评价冻结CA风险

## 已复现问题、假设和最小选择

晚期01 cycle219：57/300原联合安全且推进，冻结CA排序PASS（首选#280/安全质量.786672），但原bounds+SG输出的返回offset计划body .032983m，完整输出FAIL。旧“安全候选≠安全聚合”已经CLOSED；本阶段只试一个针对评分功能与实际输出映射不一致的最小机制，不把旧事实算作新进展。

原CA风险评价原rollout；候选若单独获权，在原bounds、四条实际控制history、SG和offset1后会成为不同计划。假设：风险也评价这一候选输出计划，能维持安全候选排序并改善最终返回控制计划；若仍失败，该对齐机制不足，停止组合，不回调风险单位/窗口/temperature。

成熟部分直接复用[Nav2 1.1.20 optimizer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp)的加权控制更新、原约束、SG历史及返回offset次序，原生探针已具备；公开实现没有提供本实验的安全保证。新内容仅是同一风险函数的输入轨迹映射，无需IMM、intent、新planner、supervisor、预测新架构或记录器。更小选择是仅移动索引，但它不能包含bounds与实际SG history，所以一次固定映射同时复现原完整输出函数，不调时间范围。

## 固定数据与方法

- 两个已评价输入：晚期01 cycle219（失败）与v4早期cycle23（原CA输出正对照）。标签已读、同scene；不是盲测/独立验证，不补齐CA原两试次协议，不runtime集成。
- 原300×30 sampled controls、CA四源模型/参数/support/完整30风险/单位、原hardV1+7标准critic+regularizer、temp、原控制聚合、body/padded/raw203/推进门全部不变。只替换soft risk的输入poses。
- 对每原候选Ui，计算one-hot原聚合→原bounds→SG(history原4条)得到F(Ui)；取F(Ui)[1:]并重复末项补齐30步，用既有Omni积分得到返回offset计划。用冻结CA的原expected-overlap评价该计划，追加到原scores，再由原native更新/SG产生最终输出。不是聚合已滤波候选，也不新增运行clamp。
- 控制空间仍原raw white proposal，映射只用于成本函数；不把clip/SG后的样本冒称Gaussian，也不使用mapped likelihood。非凸风险和先均值再约束仍可能使均值危险，此机制没有一般安全保证。
- 先freeze在线可计算映射/risk，再离线truth标签。原Sampling/Ranking安全标签保持，不用新的mapped标签替换Gate1/2门；另报告mapped候选的联合安全/推进及原CA首选映射后是否安全，以判断评分对象变化。
- 主PASS要求两个固定输入原Gate1存在safe、新Gate2首选safe且mass≥.5、原native weighted/bounded/SG最终filtered两口径body≥.05/padded>0/static203/bounds≤5e-5/正推进全PASS。任一FAIL停止，不调参。报告单项risk AUC/原及mapped Top-K与安全质量，单项风险无MPPI质量。
- 必要数值核对只选0/149/299及原CA总成本首选（不按安全标签），用原native one-hot控制和返回plan分别核对F(Ui)与积分；filter≤5e-5、pose≤1e-5。复用同binary/镜像，无新增基础设施门；不要求跨设备字节相等。

## 数据、结果、限制、下一步

待源码/输入/参数/镜像hash冻结与评价。host Python3.12.3/NumPy1.26.4构造风险；native/final gate在原Humble镜像81b325，不冒称全程同runtime。实现只在离线实验目录，main/正式配置/源代码/TF不变。

源码/prepare`97e92b4`，两输入均保持原300候选。late01映射input SHA256 `bb9c6346de4bdd84c1cdb38135027a38dbafc916598e55046230e73d6541d246`、parameter `0c46e2d290123bd2b28d9a0af05d31d14a63aea32a442b9e1552d0cae4c1c845`，风险范围.0127272–.0169177，原生核对indices0/149/280/299。earlyv4映射input `828cb169fde609ecb531f6ec300405915eba765d177e18cbf698372525aef2b9`、parameter `cb170d5182ae539ee51fb272f330a9f10329fa2ee79f1b9eada27fbe2d6fa582`，风险.0076363–.0140644，indices0/67/149/299。未读取mapped几何标签构造风险。

原native评分/输出接口input分别`873256921c185e79259a2f68c25d730861bdfd62a81718c513176c91d2ecae25`、`6541b98bbbdfff93ab67cf66cb42db8b93eb35323cbc31ab345be4219e8bae21`，参数保持`33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`。映射风险冻结后才评价。

算法成功也仅支持这两个诊断输入；若输出仍失败或正对照退化，记录否定结果并停止该组合。更一般的非凸聚合问题须独立机制假设；不能通过继续对齐工具或增加identity门掩盖否定结果。v4晚期0safe的A单独处理，不同时改Sampling。

## 算法结果与否定决策

native/诊断执行`33d9208`。8/8候选数值核对PASS，filter最大5.96e-8、pose最大4.768e-7，均在旧冻结容差内；未加byte门。附加评分误差late6.1035e-5、early0。

| 输入 | 原raw标签Gate2 | 映射标签诊断 | 最终filtered returned-plan | 完整Gate3 |
|---|---|---|---|---|
| late01 | FAIL；top#231原unsafe、最好safe第2、质量.455730 | 32联合安全且推进；top#231映射safe，质量.813223 | body/pad .043611/.013611、推进.639872 | FAIL body门 |
| earlyv4 | PASS；top#67、质量.925020 | 294联合安全/136推进；top#67映射safe但退离目标.033865 | body/pad .171130/.130975、推进-.011013 | FAIL 推进门 |

late映射cost的原raw标签AUC .654177、mapped AUC .638410；不是全门PASS。原CA首选#280在one-hot映射后body .072145、padded .042145、推进.510181，联合安全。故这次明确映射后仍有可行候选；把风险评价对齐到one-hot输出也没有保证实际加权输出。原非凸平均/目标竞争限制仍存在，不能仅用mapped排名PASS覆盖原Gate2/最终FAIL。

latecurrent-first filtered body .052258通过、returned .043611失败，两个口径必须都通过；earlycurrent-first仍安全推进.060913，而returned退离目标，不能删去返回口径。新组合让early原CA正对照退化，不能称净改进。

**H1/H2组合机制FAIL，停止同CA风险+候选输出映射，不调单位/温度/窗口，不runtime集成。** 这是算法否定结果，不是数值/工具失败。控制映射与labels已够判定，不追加更细recorder、身份或byte门。[全部结果/evidence](evidence/candidate_output_risk_20261001/manifest.json)。

下一项只处理A：对late v4 cycle40的0/300安全，先预登记一个固定零控制的原生完整3s可行见证，区别原分布没覆盖与当前固定初始状态下是否已无法控制；不是增加batch、重新试27个原型或新planner。只有见证与原首列诊断支持可行控制，才另行设计同N的时间结构采样研究。
