# Fresh-coasting观测等时边界（2026-10-15）

**局部Go：已有consumer的coasting时间关系值得修正；导航整体仍Modify。** 实际上游可在重新命中后保持coasting，同时miss_count=0、observed==source。原校验器误拒绝；将此关系由严格早于改为不晚于，在八组成对输入上只改变真实fresh-coasting反例，未来/过期/不完整/坏confirmed仍拒绝。没有运行修改后的导航，也不声称修复NoSafe或C83接触。

## 原始根因与最小修正

基线为active闭环阶段 `e0b1ae02e842cdc813213391a870633cd0a0bee1`。实际消息及controller trace来自该阶段active c100：coasting id2，118 observations、miss_count0、观测和source均13,861,000,000ns。固定core重新关联会更新last_observation、consecutive_hits并清misses；连续命中条件尚未满足时，原coasting state可以继续保留。这不是坏时间戳，也不应伪装成confirmed。

既有validator的coasting分支是 `observed_ns >= source_ns`即拒绝；与这个合法生产者行为矛盾。最小研究修正只将该处改为 `observed_ns > source_ns`；confirmed仍要求等时、全局future/0.4s stale约束全部保留，coasting可信度/预测不变。使用原时间数据，不回填新时间或重写state。

prepare在owned `build/coasting_epoch_cache`生成一份仅此单个关系运算变化的model头；原vendor/model.hpp和安装库不修改。分别构建before/after两个小程序，include真实原模型或该生成模型，调用同一inline validator。不是新tracker、控制器或生产消息接口。

## 实际16次校验结果

| 固定输入 | 原严格关系 | 允许等时 | 结果 |
|---|---|---|---|
| 真实fresh coasting c100 | observation_anchor_epoch | 通过 | 唯一接受变化 |
| 原durable98的合法旧coasting子集 | 通过 | 通过 | 原允许的coasting不变 |
| 原durable24 confirmed | 通过 | 通过 | confirmed不变 |
| 原durable129过期coasting | stale_observation | 同样拒绝 | 0.4s年龄门限不变 |
| synthetic coasting观测比source晚1ns | stale_observation | 同样拒绝 | 未来观测不放行 |
| synthetic confirmed观测比source早1ns | observation_anchor_epoch | 同样拒绝 | confirmed等时条件不放宽 |
| synthetic source比consume晚1ns | stale_or_future_source | 同样拒绝 | 源时间不能未来 |
| synthetic complete=false | incomplete_or_overflow | 同样拒绝 | 完整性不放宽 |

四真实输入和四明确synthetic控制，两个程序各运行八项；先登记的条件全部满足。[协议](../../experiments/stvl_coasting_epoch/protocol.json)、[全部输入](../../experiments/stvl_coasting_epoch/evidence/inputs.json)、[结果](../../experiments/stvl_coasting_epoch/evidence/results.json)、[唯一文本变化和身份](../../experiments/stvl_coasting_epoch/evidence/change.json)。没有工程失败或新增live导航。

## 收益边界与下一步

这个确定性缺口比单独过滤tentative更直接：它解释本轮full两次和active一次现场anchor拒绝。接受fresh-coasting意味着随后能进入原风险评分，**不等于该轨迹、控制或停车位置安全**；soft NoSafe仍可产生新非零指令，native重试/异常合同亦未改变。多个修正不能捆绑后声称一个的信息收益。

下一阶段若检验闭环作用，应只替换这一个validator关系，保持原full输入、native算法、CV、所有约束；不要同时应用tentative过滤、typed异常或NoSafe控制干预。当前完整候选导出即使shadow off仍占大量磁盘，因此先给已有研究observer增加明确的诊断导出开关，并确保成对组同期开关/输出行为；不能拿关闭日志带来的时延改善归因于预测。资源或最小合同不满足则停在本结论，不扩大批次。

本轮原头只读、两个增量小程序，cache约1.7MiB可重建；真实输入和小结果已入Git，原active消息raw仍在完整缓存。当前磁盘约3.14GiB，不继续大型写入。无生产src/参数/FAST-LIO/定位/MCU/V2/AUD-003、其他ref或硬件运动改变，无push/merge。完整HEAD及clean由Git和检查点核对。
