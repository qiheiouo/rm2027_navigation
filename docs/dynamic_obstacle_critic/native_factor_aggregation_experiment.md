# 输入因子的原生聚合后果：离线实验登记

2026-10-02，从ca3a50d建立`experiment/native-factor-aggregation`。
前序b17b73a已固定source298的300 raw路径和四组离线风险；只有
future-circle真值诊断预报实际aggregate的未来冲突。下一步隔离风险
输入修复之后的原生优化/约束/SG环节，仍不供应可上线预测器。

## 预定操作和证明

1. 固定原source298、300×30 sampled controls、实测速度首列、实际
   warm control_sequence与四步SG历史。读冻结完整native/consume/SDK/
   weight身份链；baseline先核对原实际数值，不能用零历史或新的noise。
2. 原七critic成本、StaticStopping原增量（此周期需全300确认为0）、
   temperature/gamma/std/constraints/dt/Omni/offset1全部保留。只替换
   dynamic风险四组：actual、源几何、源有限局部CV、未来真值circle。
   risk double转float再加至七critic float的原顺序必须保持，baseline
   raw最终cost逐位对上实际native capture；每个hypothesis独立从原
   warm/history开始，不连成虚假的新闭环。
3. 新BUILD_TESTING-only对象调用现有Humble1.1.20原生
   `Optimizer::updateControlSequence`、约束和原SDK SG/状态传播/积分。
   不复制上游optimizer；源码/ELF/SDK FP flags记录。baseline需全
   post-gamma cost/weights、90 bounded mean/SG、returned command及
   raw模型/aggregate路径逐位等于原冻结结果，之后才分析反事实。
4. 保存4组native weights、完整控制/模型速度/姿态及300成本；source
   age和scoreclock相同。用已有纯CV/circle代码评估4个新aggregate
   路径，并独立检查全机械动态/静态、padded、原raw203、原5e-5 bounds
   和正进展。source pose与actual score epoch分开报告，不替代旧门。
5. 私有hypothesis输入/输出格式明确与实际capture分离。数值输入在
   SDK fast-math构建下也须按IEEE位型拒绝NaN/Inf；格式、网格、越预算、
   非finite、截断/尾字节/已有输出及actual baseline篡改负例保留。
6. 原纯factor评分代码可新增编译期4路径实例，默认311实例的行为/
   19项回归须保持。新数据不得伪装原311 SDK见证或actual score事件。
   保存依赖闭包、全部失败/输入/输出/编译身份及独立重执行；已有
   runtime、原SDK及离线ELF身份变化若发生必须解释和核验。

未来真值oracle仍inadmissible online；source oracle不是v1原输入。
重加权结果是固定周期条件反事实，没有执行、不认证deadline/goal/
新物理轨迹或未来地图。不能据此恢复CA、sampler、ranking或部署；
原task/raw203/年龄FAILED、综合NOT ESTABLISHED保持。main/research
封存、TF唯一所有者、公共字段/在线参数和guard路线不改。

该阶段回答“假设风险输入具备未来支持，原生聚合/SG在这个固定上下文
产生何种路径”，不回答真实感知如何获得这种支持。后者仍需可信e/B/
有效区间与公共供应契约，默认条件工具保持unknown。
