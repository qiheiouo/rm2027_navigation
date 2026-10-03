# 原个体控制约束与SG阶段：离线实验登记

2026-10-03，从2a67835建立`experiment/native-individual-stage-contract`。
原raw185完整地图门通过，但前序不同的个体bounds/SG路径失败；
不能由此推断SG单一因果，更不能把个体标签回填原raw成本。

## 固定对象与实施边界

1. 保持source298原300×30控制、实际测量速度前缀、原生Omni模型、
   约束和独立使用同一实际SG历史。先复现原六块raw速度/位姿字节，
   再证明原311个体见证、aggregate及实际命令逐位保持。
2. 仅增加BUILD_TESTING文件工具实例，输出原300控制的三个有明确
   顺序的阶段：未处理原raw、同SDK约束后但SG前、同实际历史SG后。
   阶段私有格式不伪装为原采样器输出或在线CriticData；新格式预算、
   完整网格、行映射、非finite、已有输出和关联负例保留。
3. 调用未改安装SDK，复用已有离线见证源文件时默认实例ELF和原
   311输出必须保持。新实例单独记录宏/flags/ELF。输入、旧六块raw、
   311见证与真实消费来源绑定，不从cost总和猜各critic增量。
4. 固定原map bytes、padded足迹、raw203和完整三秒reserve，比较
   三阶段同路径地图门。原raw185及发现的边界变化用显式单元标量
   子集核验；全300只有保守vector标签，不自动扩大精确单元范围。
   若检查动态/完整输出门，保持原physical阈值/时域，单独标识未
   接受的控制假设，不能将raw或pre-SG直接作为发布命令。
5. 原参考归档不改，负例/失败/图示、依赖闭包和独立复现结果保留。
   不修改原七critic、StaticStopping、sampler/optimizer/SG参数、
   guard或地图，不建立排序器、选择器或未来静态墙。

## 结论边界

本轮分离固定个体控制的约束与SG效果；不能推广为全局原生优化器
缺陷、闭环成功或新采样策略依据。原生实际CostCritic内部状态、
各增量、地图单元来源和真实e/B仍unknown。原task/raw203/age
FAILED、综合NOT ESTABLISHED保持。所有运行profile、公共接口、
TF层级和唯一所有者不改，main与冻结research不改。
