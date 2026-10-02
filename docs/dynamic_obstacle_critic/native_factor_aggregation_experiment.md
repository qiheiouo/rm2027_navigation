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
   七critic float先提升double，与risk double相加后整体转float的原顺序
   必须保持，baseline
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

准备阶段baseline成本核对拦截首次脚本的提前risk→float转换，运行源码
实际为`float(double(before)+risk)`；据此修正上述登记精度描述。
失败发生于创建输出之前，保留失败日志/首次源码；此前被冻结的实际
评分/after位型证据均使用原正确算式，不受此准备错误影响。

## 实现与结果

`prepare_native_factor_aggregation.py`检查原348周期完整native/consume/
weights身份链，输出独立私有RMCVAGH1。该周期StaticStopping全300
增量为0；只在七critic原float成本上按原精度顺序加四组风险。新的
BUILD_TESTING-only `evaluate_native_factor_aggregation`调用已安装
Humble1.1.20原优化器/约束/SG/Omni积分；没有复制上游优化器。

baseline的raw六矩阵、post-gamma300成本、300权重、90 bounded mean、
90 SG控制、return三轴与9×30路径逐位符合原capture；四组权重各自
逐位重聚合到bounded mean，首列实测速度全部一致。4路径私有
RMCVAGO1与原实际capture格式不同，baseline replica及三反事实显式
标记。NaN/Inf使用IEEE位型检查，SDK fast-math下的实际负例也拒绝。

共享纯CV代码新增编译期4路径实例`evaluate_cv_aggregate_factors`；
默认311实例ELF及19项全部输出/probe字节仍等于冻结前序。新实例
的1200 raw评分与原四因子逐位一致，baseline路径四评分也逐位一致。
本次matching score属于各因子**新生成的aggregate路径**，与前序在
固定实际路径上评分的数值不能混用。

| 离线风险因子 | 新aggregate matching risk / min(m) / TTC | scoreclock完整动态结果 | 原raw203 interval lower(m) |
| --- | --- | --- | --- |
| actual baseline | 0.549357 / 0.114527 / 无 | mechanical与padded接触 | +0.000030576 |
| source geometry oracle | 0.292334 / 0.164060 / 无 | mechanical与padded接触 | +0.000268636 |
| source bracket CV oracle | 0.001161 / 0.371287 / 无 | mechanical与padded接触 | −0.007597144 |
| future circle oracle | 0.406543 / 0.038635 / 无 | body/mechanical/padded下界0.218495/0.176158/0.130140m，通过 | −0.006935911 |

主epoch48.004s覆盖208采样点，source pose47.98s敏感性覆盖207点；
完整3.0000000447s、最大间隔17ms，均**0/4通过全部条件门**。
future circle的原bounds、静态物理门及正进展1.024586m通过，raw203
仍失败。独立120姿态×两epoch动态标量接触/距离核验误差0；原baseline
sourcepose完整几何与此前见证误差0。

原捕获100×100、0.05m地图的1483个raw≥203单元又经独立标量多边形
核验，四路径两epoch分类一致；正距离下vector保守下界可比精确单元
距离更小，不声明二者逐值相同。future路径实际scoreclock首次采样
接触在+2.982s：机器人姿态[4.144940,0.790873,−1.397423]，raw216
单元(78,38)，其范围x=[4.50,4.55]、y=[0.45,0.50]m，gap=0。
这是冻结地图原阈值下的单元接触，不是已执行物理碰撞；该单元的
动态膨胀/静态/unknown来源尚未建立。篡改为raw通过的负报告被拒绝。

28项原生可执行检查、17项4路径纯评分检查、19项默认回归、10项
私有输出格式检查及伪造raw通过负例全部通过。原五runtime、原SDK/
weights见证及三个纯评分共十个已有ELF身份保持。新原生ELF为
`0b8cb8ff2fe57bc91641f2bbd34d11cc0c274722f6b709a4696a4d877e706529`，
新4路径纯评分为
`c254dd8e0a7d68488bbce751b8b97d1647057afd4bd1719dfda4f3841829c79f`。
记录FP flags与实际加载`/opt/ros/humble/lib/libmppi_controller.so`；
仅补库路径metadata前后的数值JSON行和路径binary完全相同，初版本
输出/ELF身份与精度准备失败均保留。

## 冻结与独立复算

[原证据](stage2_evidence/native_factor_aggregation/manifest.json)179文件约
8.08MB，manifest SHA256
`01690b6230b5fb5c256b3fa41d3353b6576098cedda4aba1ad928ad60a180ece`。
保留27个Python模块依赖闭包、C++/头文件/CMake、原输入、新成本、
全输出、格式负probe、标量核验、失败源码/日志及修订前后图示。
`replay_native_factor_aggregation.py ARCHIVE NEW_OUTPUT`重建12项字节
精确结果；使用独立重执行native和纯评分三个产物参数后15项全同。

[独立verification](stage2_evidence/native_factor_aggregation_replay/manifest.json)
14文件，manifest SHA256
`1d6869009245a7a10e8040a493dc8298d6d07794a8fa9737ca114e0efbfaf326`。
四个引用archive共2064文件身份通过，原179文件manifest不改。
结论仍是固定上下文的条件反事实；future truth不能上线，完整物理
支持条件仍unknown，所有历史FAILED/NOT ESTABLISHED保持。

下一项独立登记同一四因子的SG前后路径比较，先测量raw216接触是否
在SG前已经存在。当前结果不能将失败归因SG、sampler或某一critic，
也不授权删除地图单元、改变203阈值、缩短三秒或恢复CA。
