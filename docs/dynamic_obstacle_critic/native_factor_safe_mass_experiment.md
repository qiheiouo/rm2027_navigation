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
