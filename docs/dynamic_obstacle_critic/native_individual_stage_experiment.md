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

## 三阶段实现和结果

9626742后新增`native_individual_stages`宏实例，共享既有见证源码；
默认实例不启用新宏。其ELF重新编译仍为
`d798fbd5050a943a6e9a5d4fb5d2e4cc0d2883891723a5e94b72f3c808abec01`，
默认及新实例原311 binary/JSON全部逐字节等于冻结见证。新阶段实例
ELF为`ae89bcf377117db0beb71ba840dd23f61f72e3a33029c9fae661331e89d0bdb7`。
保留早期尚未增加登记参数严格匹配检查的实例源码/身份；最终实例
实际正例产物与早期输出完全相同。所有构建/执行只使用原已有镜像。

固定输入324940字节，原f32参数为`.1,-.5,.8,.5,1.2`，新实例只
接收该登记网格/约束。私有`RMCISTO1`输出995816字节：原raw300行、
约束后SG前311行、SG后311行，原raw对应offset分别0/11/11。
两个处理阶段保留原辅助11行布局以保持SDK算术；辅助行不是新增
sampler候选。原raw九块控制/速度/位姿字节直接证明相同，SG后
九块311张量也逐位保持。源pose原311完整几何/门及两epoch原300
个体SG全部标签精确复现。63项实际编译/格式/非finite/覆盖保护/
张量关联检查通过；fast-math实例的非finite输入用IEEE指数位拒绝。

| 主scoreclock，原raw185 | 原始raw | 约束后SG前 | 同实际历史SG后 |
| --- | --- | --- | --- |
| 精确raw203区间下界 | +20.782mm | +23.441mm | −5.873mm |
| 完整地图门 | 通过 | 通过 | 失败 |
| raw单元采样接触 | 无 | 无 | +2.914s，raw210，(76,37) |
| 控制数值约束门 | 失败 | 通过 | 通过 |
| 其余完整fixture几何/进展门 | 通过 | 通过 | 通过 |

约束只改raw185的4个cvy及1个cwz值；位姿仍变化，地图门继续通过。
此固定路径的地图失败出现在SG后；源pose敏感性同样如此，首次
接触+2.921s。精确单元距离只覆盖raw185三阶段两epoch，未将其他
路径的保守vector变化写成接触或精确单元标签。

整批主epoch地图vector通过数90→90→92，源敏感性92→90→92。
原raw到约束，主epoch1条通过变失败（52）且1条失败变通过（42）；
约束到SG，主epoch8条通过变失败、10条失败变通过，全部行号保留
在报告。因此不能把一个个体结果推广为SG整体恶化或禁用SG依据。
全部计算fixture门的主计数2→34→38、源2→33→39只是模型假设；
原raw/pre-SG没有被过滤并发布，原SDK实际aggregate结果不替换。
沿用的`SG_full_sequence_and_return_bounds`键检查各阶段数值约束，
不能从这个键名声称raw/pre-SG已经执行了SG。

## 冻结与复现

[归档](stage2_evidence/native_individual_stages/manifest.json)保留185个
manifest文件、24541881字节（不含manifest）、24个Python依赖模块，
manifest SHA256为
`c53cc4a6be2da3cdf1979e21dc97b784d66b09d4312f21e5db29458c80bf596c`。
包括所有完整负例输入/输出、首轮缺link.txt构建失败、首轮绘图bytes
reshape失败的源码/日志、初次成功图和图例修正后的已检查图示。
原参考archive不改，当前没有运行或控制修改。

```bash
python3 -B src/rm_dynamic_obstacle_critic/tools/replay_native_individual_stages.py \
  docs/dynamic_obstacle_critic/stage2_evidence/native_individual_stages \
  /tmp/NEW_REPLAY_DIRECTORY
```

默认6项逐字节复算使用冻结SDK产物。若提供全部
`--native-stages FILE --native-witness-binary FILE --native-witness-json FILE
--native-verification FILE`，另核验四个新实际SDK/63项检查产物，
共10项；部分提供会拒绝。新输出目录必须不存在。此复现不认证
新控制发布、真实运动支持或完整实际CostCritic内部状态。

冻结源码闭包重新生成输入后，独立实际SDK执行/63项检查及报告、
raw185标量、PNG/SVG共10项逐字节一致。另存
[独立verification](stage2_evidence/native_individual_stages_replay/manifest.json)：
10文件、13967字节（不含manifest），manifest
`ca82df8f6ef4f96803b05677248ef1726523458be30d97082b9844d84dbd1a03`。
五份archive共2236个manifest文件身份通过，原185文件不改。
