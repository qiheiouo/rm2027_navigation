# 聚合/SG路径的纯CV目标：离线实验登记

2026-10-02，从1edb4ee建立`experiment/processed-cv-objective-audit`。
source298同raw路径已经发现真实动态接触而CV圆模型没有TTC；继续
固定同一周期，检验实际聚合/SG路径及原11提案/300条单条约束SG反事实
在**原实际消费CV输入**下的风险。真值仍只离线，不重排或执行提案。

## 运行前协议

1. 读取冻结1685文件物理archive和88文件SDK完整三秒条件见证，
   source298、实际scoreclock=48.004s、source pose=47.980s及原输入
   source_age/world transform/padded footprint/cost参数完全保留。
2. 单独导出实际消费RMDYNRP1子集和显式`RMDYNPX1`处理后路径。
   后者不伪装actual score证据；保留源ordinal、311×30原始SDK输出
   bytes、dt与各提案名字、原三项完整native数值验证标志/文件身份。
   实际aggregate行和反事实行必须区分，首速度不改、历史不清空。
3. 新BUILD_TESTING-only工具调用原`dyn::score`，采用runtime同一
   O3/无fast-math算式；不调用ROSnodes、TF、optimizer或command API。
   先核对该周期300条actual raw risk/after bits，再评估311条路径。
   原消费replay与新工具共用私有bounded reader，公共tracker类型保持
   原`rm_competition_interfaces`消息；不另建感知数据类型。
4. reader重构需原348周期104400 raw risk/after和既有4实际fixture输出
   逐字节回归；原one-ULP风险负probe仍需拒绝。新的处理后格式要求
   source ordinal与dt/网格匹配，错ordinal/dt/非finite/截断/尾字节拒绝。
   遇到错误不将部分输出认作PASS；保存完整失败/编译身份。
5. 保存311行risk/minimum_clearance/TTC；优先报告actual aggregate与
   zero在同一CV模型/scoreclock下的结果。对照原完整三秒physical标签
   要保留其pose源epoch，明确24ms差异；若再生成同scoreclock物理
   标签，使用前序独立几何核对方法，不替换原条件门或选择时刻。
6. 只编译/执行离线目标，记录5个运行ELF前后身份不变。参数/模型/
   sampler/optimizer/SG/guard/TF/public字段、main和research封存均不改。
   原task/raw203/全trace年龄FAILED和综合NOT ESTABLISHED保持。

本阶段不能通过成本数值推出安全、执行或coverage；若actual aggregate
本身在CV模型已有强风险，需要另检验目标加权/滤波的作用。若CV模型
继续漏真值动态接触，则优先追踪观测几何与长时预测支持条件，不凭
原39/300或新反事实数量恢复CA/sampler/ranking。
