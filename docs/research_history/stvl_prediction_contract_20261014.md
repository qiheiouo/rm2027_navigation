# 预测输入的active轨迹合同复核（2026-10-14）

裁决 **Modify**：两次真实时间锚点拒绝可归因于动态评分本就跳过的tentative轨迹，而不是正在消费的confirmed/coasting预测。只读子集验证可通过，尚未更改在线adapter、tracker或消费者；不宣称闭环收益。过期active观测仍必须拒绝，原safe300也可能只是没有任何active轨迹。

## 固定输入与实际实现

基于有限制动阶段 `af39a45bd536a225b7c5ca2904cbde9fb6659ba2`，同一隔离分支。真实输入是durable c24/c98/c99/c129及原c0空包；另两项synthetic只检验bad active epoch和空active边界，清楚标注，非新导航数据。

未重新实现校验器：C++直接include上一阶段固定 `vendor/model.hpp` 的 `dyn::validate`，使用原max_age/max_observation_age0.4s、schema、尺寸和状态约束。原TemporalResearchCritic先校验完整包，随后风险循环显式跳过STATE_TENTATIVE。探针只读比较“全包校验”和“去掉不参与动态评分的tentative后校验”，保留源时刻、active观测时刻、状态、位置、速度、尺寸。没有抬高门限、改时间戳或拿真值替代观测。

首次准备将c0当成active正常样本，实际它是空包，synthetic准备失败；随后的无输入探针exit139、core关闭，无有效结果/运动。保留日志和[修改登记](../../experiments/stvl_prediction_contract/input_amendment.json)。在有效探针结果前明确将正常active样本改为最早合法confirmed的固定c24，并额外保留真实c0空包；未事后挑选成功导航。

## 七个输入结果

| 输入 | 原完整包校验 | active子集校验 | 证据 |
|---|---|---|---|
| c24正常confirmed | 通过 | 通过 | active id2、原字段不变 |
| c98原失败 | observation_anchor_epoch | 通过 | 排除未用于动态评分的tentative id3；coasting id2保留 |
| c99原失败 | observation_anchor_epoch | 通过 | 排除tentative id4；confirmed id2保留 |
| c129原失败 | stale_observation | stale_observation | active id2确实过期，不能放行 |
| synthetic坏active时间 | observation_anchor_epoch | 同样拒绝 | confirmed observed比source少1ns仍拒绝 |
| synthetic无active | validator通过 | 研究标记Unknown | 不是物理环境安全证书 |
| 原c0无任何track | validator通过 | 研究标记Unknown | 原safe300表示无已知动态占用，不能等同准确覆盖300安全候选 |

其中空子集的Unknown是**探针额外保守标注**，不是声称既有validator会拒绝空包。当前STVL仍处理当前占用；无track消息本身不证明有障碍或无障碍。若未来adapter丢掉全部tentative，它必须保留“尚未确认”信息，不能把它当成经证实自由空间。

[实际输出](../../experiments/stvl_prediction_contract/evidence/results.json)、[全部输入](../../experiments/stvl_prediction_contract/evidence/inputs.json)。七项运行完成，先登记的有效对照条件通过；2次启动尝试、1次无输入失败、1次有效，无新增导航运行。

## 为什么这值得局部处理，但还不能部署

active轨迹的时间和运动字段在两个失败快照上都合法；误拒绝源于“完整tracker包的逐track锚点合同”与“消费者只评分active”的范围不一致。原failure_flag和runtime_error把这个不一致放大为整个控制请求失败。它不是预测误差变小就自然解决的问题，也不是采样数量不足。

若继续适配，最小模块是**已有预测adapter与消费者的输入范围合同**：明确active subset的完整性和暂未确认轨迹，按track ID同步保留位置/未来points索引，保持所有active时间校验和source覆盖要求。不能只删array.tracks却保留未对应的predictions数组，也不能把tentative时间改成source伪装新观测。probe仅验证数据合法性，未计算删除后的动态成本或新Twist。

尚未证明tentative被忽略在全部场景都安全，未运行适配后的在线控制、传输时序、延迟或通行效率。该局部合同值得保留为后续候选，整体仍Modify。原C83首次safe0是成功评分，修正此拒绝也不会自动补上NoSafe→早制动决策空白。

## 资源与恢复

[复现入口](../../experiments/stvl_prediction_contract/README.md)。真实输入/结果/协议是小JSON，程序和固定依赖即可复算；构建 `build/prediction_contract_cache`约1.2MiB可重建，两个实际日志在本轮cache，初始失败不删除。磁盘仍约3.30GiB，无生产库/源码、正式配置、硬件或其他refs改变。阶段提交完整身份及总裁决见[检查点](research_continuation_20261014.md)。
