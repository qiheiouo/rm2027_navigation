# 固定恒速/恒加速度消融：完整控制期的软风险

## 背景、假设与复杂度

四帧近面源运动H1通过，但短期恒速软风险全体同分、Ranking失败。停止继续修源中心和调风险权重。本项检验当前四帧是否含有可用减速信息，以及该信息能否改善完整控制期总排序。

H：共享同一源位置/速度/不确定支持/风险尺度/30步评价的CA（恒加速度）软风险，相对CV（恒速）固定消融，能将联合安全且推进候选排到高权重区，并保持原生完整输出安全/正推进。两模型均不通过则停止该运动/风险组合；若只有CV通过，不能把收益归于加速度，也不增加CA复杂度。

复现失败：准确源速度仍无法使原短期风险区分任何候选。现有CV名义运动没有减速项，原1s风险不评价完整控制期。公开成熟的加速度状态传播可见 [robot_localization EKF源码](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ekf.cpp) 中的速度×dt和加速度×dt²/2；这里只引用模型，不复制源码或导入EKF依赖。更小方式是现有四点二次最小二乘，仅取加速度项，位置/速度仍用已验证近面估计。不新增tracker、IMM、intent或optimizer。

这不是时间窗口扫描：唯一评价期固定为现有MPPI完整30步；CV-full作为必要消融，判断CA是否有独立增益。原硬V1运行horizon及YAML保持不变。旧“单纯改窗口不足”结论仍CLOSED，不回调30步长度寻优。

## 输入、参数与判据（评价前）

- 同一已看标签cycle23，输入 `4fe24a21df02f8ceeaf2d76016c06ec776300118686c577a7bc5de29c81459ee`；四帧motion输入 `680b10f98623c2a012ecaf8bc26d6212cf7e56a521bef1ae351c24df7d2c85ad`；镜像81b325。不是新盲holdout。
- 两臂共享motion最后source位置和六对斜率中位数速度；CA逐轴拟合 `[1,t,.5*t²]`，时间相对最后source，仅使用加速度系数，CV加速度0。四点不变，不扫描历史长度/滤波。
- 未来nominal center=p+v*h+.5*a*h²，h=actual age+(j+1)×actualdt。风险仍为原uniform-center expected overlap、5×5 quadrature、padded footprint面积归一化。支持半宽=原visible size/2+原physical size/2+.5×原reference_acceleration×h²；原加速度支持参数不改，不拟合历史残差概率。
- 每臂风险平均完整30步，原附加单位 `(3.81/254)*1e6/9` 保持；七标准项、硬V1实际项、原正则/temp/bounds/history/SG保持。原300候选全部参与。
- Ranking PASS仍首选联合安全且safe mass≥.5；报告最好安全名次、Top1/5/10/30及进度。Aggregation完整两口径body≥.05/padded>0、raw203、bounds/正推进。source样本/加速度不以truth挑选，truth只离线标签和模型误差描述。
- 若某臂通过，停止在本输入调参，下一步使用未查看周期或新的独立运行固定规则验证；不能称一次模型通过为稳定或部署安全。

## 结果、限制与下一步

评价前源码 `4085f2d`。风险输入 SHA256 `30e43ef4bf7c0294a9299ee72559eb28a11d86111a2247ced265964f4ef8d22f`、运动参数 `876aeedb0bef5f85cf9fcbd33d13af41613fbcf8bd5062189d4c11a0e8e199af`；CV evaluation input `b1dc54c7f55e5235c04f751904d2e7eabd207f612ed5dd1582e245b59118cd88`、CA evaluation input `d4b05096ec10cd74dc48efdf8020650d41d50b25bd857cd25455aae29cfe62d8`，共享原生score/output参数 `33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`。online风险在host Python3.12.3/NumPy1.26.4生成冻结，原生评价使用固定Humble镜像81b325；image字段指原生评价镜像，不冒称host风险在容器计算。

固定四点拟合加速度(1.489462,1.207854)m/s²；此数值未根据truth筛选或clip。CV风险全0，CA范围.008218–.014753。该阶段只冻结online风险，还未查看两臂排序/控制。四点加速度可能对range误差极敏感；恒加速度三秒外推不是换向意图预测，也不是预测保证。原body/padded/static门不改变；不以更精确预测替代排序和输出结果。

执行提交 `8366977`：[完整证据](evidence/near_field_kinematic_risk_20261001/manifest.json)、[CV](evidence/near_field_kinematic_risk_20261001/cv_full_summary.json)、[CA](evidence/near_field_kinematic_risk_20261001/ca_full_summary.json)。

| 臂 | Ranking | 首选 | 最好安全名次 | Top1/5/10/30安全 | safe mass | total safe-lower AUC |
|---|---|---|---:|---|---:|---:|
| CV-full | FAIL | #293不安全 | 2 | 0/1/5/25 | .717214 | .003390 |
| CA-full | PASS | #67安全，推进.088319m | 1 | 1/4/9/26 | .939646 | .090169 |

CA完整输出Aggregation PASS：filtered当前速度首列body/padded .146511/.111081m、offset计划.166195/.130806m，静态/bounds均通过，推进.086230/.050165m。原CV输出也PASS，推进.153420/.159275m；CA减少推进，不能只报告安全质量增加。原生附加评分误差0。

**决定：冻结CA作为离线候选，不继续在本周期调参；下一项新的独立试次固定时间三层验证。** 相同完整窗口CV风险仍0，CA产生区分并改变选择，说明此次变化来自模型项而非单纯加长窗口；该因果结论限本冻结输入。

CA风险单项AUC .214237、总分AUC .090169均低于.5；且.939646质量仍低于295/300=.983333的安全基率。通过预登记top1/mass门不代表全局风险有可靠安全辨别能力。四帧估计的y加速度1.207854不等于真实横向运动；nominal模型可能对range/近面误差过拟合。没有模型真实性或稳定闭环证据。只保留本周期三层门PASS，禁止据此运行集成或部署；独立失败则停止该组合，不修参数追PASS。
