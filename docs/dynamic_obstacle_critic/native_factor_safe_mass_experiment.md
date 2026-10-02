# 四因子原生权重与个体安全见证：离线实验登记

2026-10-02，从ced13e8建立`experiment/native-factor-safe-mass`，先
登记再运行。前序四因子的聚合在SG前后全部条件门均0/4；完整实测
速度与actual SG下的个体候选曾有39/300源pose条件safe-progress。
本轮核对权重的分布及个体/聚合路径区别，不开发新的控制选择器。

## 固定输入与关联

1. 原物理消费archive、source298完整native/cost/SDK链、原311
   SDK witness、processed CV、四因子评分、四native aggregate及
   SG阶段archive全部先验manifest与原位型核对。原300 raw风险
   评分对应300 sampler行；个体路径对应311 witness的11+raw行，
   是独立bounds/actual-history SG路径，不能当raw风险路径标签。
2. 复用冻结processed CV的两epoch动态标签（已全9330姿态独立
   标量核验），共用纯full-context函数补齐300个体的静态、原raw203、
   完整3.0000000447s原17ms网格、bounds原5e-5及进展。源pose全部
   几何/门需等于原311见证；主scoreclock与24ms敏感性分开。
3. 原四factor post-gamma300成本、native300权重及mean/SG全部
   使用冻结SDK输出。仅统计每类完整个体门的数量、质量和top权重，
   以及同300raw路径的risk/contact和其11+row个体SG几何之间的
   差异。质量用原native float32权重的float64 fsum描述，不重归一化
   为新策略，不把不同路径的标签叫成optimizer实际评分。
4. 独立raw单元标量核验的确定集合包含：全部原39个源pose安全个体、
   全部新主epoch被标为安全的个体、四factor各最高权重行以及row0。
   两epoch完整网格及原203门核验，报告被验证的原sampler索引。
   所有正安全标签都必须落在核验集合；其余失败标签仍为保守vector
   判定，不能宣称全部300的精确raw距离都已独立计算。
5. 负probe需拒绝raw/processed行混淆、权重篡改和不完整质量表；
   独立标量重算分组质量/计数与全集分割，原baseline39及其质量需
   复现前序。保留完整行级表、依赖闭包、失败/输出/图示与独立重跑，
   原14 runtime/SDK/offline ELF身份不变，不改历史manifest。

## 结论边界

个体控制具有条件安全正进展不推出其加权控制/积分路径安全；原
风险评分raw、bounds顺序、SG以及积分路径的区别必须明确。本轮
只描述当前固定周期的已有候选/权重和aggregate失败，不能从候选
质量直接提出custom ranking、恢复CA/sampler或执行候选。future
oracle仍不可上线，真实e/B/有效区间unknown；原task/raw203/age
FAILED、综合NOT ESTABLISHED及TF/公共契约/在线参数保持。

## 实现与结果

`audit_native_factor_safe_mass.py`是纯离线报告工具。先检查七archive
manifest、完整原348 native权重链和source298 SDK311见证，明确
sampler raw row i对应个体SG row 11+i。冻结processed CV两epoch
动态标签的完整9330姿态核验保留；共享full-context补齐全部311
路径的静态/raw203/bounds/进展，源pose每项几何/门/下界与原见证
完全相同。本轮统计仅使用其中300个体，不将前11注册提案混入。

主scoreclock48.004s、完整3.0000000447s、208点/最大17ms的个体
完整条件safe-progress为**38/300**；源pose47.98s、207点仍**39/300**。
差异仅raw row63（processed row74）的padded动态区间下界由
+0.003444444m变为−0.000390288m。两epoch密集采样的padded最小
距离都0.011234776m，没有由此证明接触；它是原保守连续区间门
不通过，不能放宽门或把源pose39直接宣称scoreclock39。

| 原风险因子 | 主scoreclock 38个体权重占比 | 源pose39个体权重占比 | 最高权重raw行 | 全部聚合门 |
| --- | --- | --- | --- | --- |
| actual baseline | 16.4561% | 17.3299% | 73 | before/after均失败 |
| source geometry | 11.4598% | 12.1703% | 73 | before/after均失败 |
| source bracket CV | 4.9892% | 5.3591% | 127 | before/after均失败 |
| future circle | 40.8640% | 40.8640% | 185 | before/after均失败 |

权重本身是原SDK float32 softmax输出，表中占比为其float64 fsum
质量除以原total的描述值。future-circle下59.135987%的未归一化
权重质量落在个体raw203失败行；其最高row185也未通过个体完整
门。不能由40.86%正见证质量推导aggregate安全，也不能据此进行
新排序、筛选或恢复sampler。风险路径是原raw，个体SG路径单独
评估，原成本与个体成本都逐行记录并分别标记path_kind/row。

独立原raw单元核验确定集合为43行：原39正见证、新主epoch38
正标签、各因子最高权重行及row0的并集；两epoch完整17ms网格
均与vector raw203门分类一致。所有正安全标签都被覆盖；余下
257行的失败标签为保守vector结果，不宣称全300精确raw距离
经过单元标量重算。固定地图来源仍unknown，标签不是执行碰撞。

`verify_native_factor_safe_mass.py`以精确有理数独立求和，检查300
raw/个体行映射、8 factor×epoch组、32互斥分组、所有门失败质量、
最高权重、源pose39质量及正标签raw子集；质量误差0。13项完整性/
CLI检查通过：raw/processed混淆、double一ULP权重/占比、缺factor、
路径成本替换、漏分组/正标签/行、伪造full门/标量映射和native权重
篡改均拒绝；已有报告在创建另一个产物前被保护。原14 ELF身份
逐一保持，无CMake/runtime/ROS/public/TF变更。

## 冻结与独立复算

[主archive](stage2_evidence/native_factor_safe_mass/manifest.json)67文件
约3.17MB，manifest SHA256
`21c8d9f0e647eaeac3c2514294f01b2dfb10402f5b466d98ee8d77717f756c69`。
保存23个Python依赖模块、300完整行表、43行密集raw标量见证、
有理核验、全部负probe、原14 ELF、日志与已视觉检查的PNG/SVG。
初轮11项probe与最终加入native输入/已有输出保护的13项结果均
保留；没有把重复验证解释为新的物理试次。

`replay_native_factor_safe_mass.py ARCHIVE NEW_OUTPUT`从冻结源码
重新计算全部300个体门和43行密集raw标量、重跑精确质量/13项
probe和图示；6报告/核验/PNG/SVG全部逐字节相同。
[独立verification](stage2_evidence/native_factor_safe_mass_replay/manifest.json)
8文件，manifest SHA256
`c63af9bda639a754cfcd741c6f12f991132049c68ad392d437b94c27ef8f7a71`。
八archive共2448文件身份通过，原67文件manifest保持。

下一轮将先登记原CostCritic与原raw203足迹门的地图语义对照；目前
只知道原profile明确`consider_footprint=true`，不能将raw216接触
直接解释为该critic没有考虑足迹或实现错误。e/B/有效区间仍unknown，
本轮新增证据没有满足上线支持条件，原FAILED/NOT ESTABLISHED保持。
