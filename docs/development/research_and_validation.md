# 研究与验证

采用 **Research → Integration → Deployment**，执行 **Minimum Sufficient Validation**。
规则整理自 `main` 的分级验证原则，删去重复例子和统一测试矩阵；不改变既有研究 Stop/冻结结论。
阶段由任务目标和风险决定；优先用户声明，不以分支名或“能测更多”为升级理由。

## Research：最低成本判断价值

先明确 hypothesis、稳定 baseline、Minimum Decisive Experiment，以及少量会改变决策的指标。
只建设回答核心问题所需的代码和观测。动态导航可选成功率、碰撞/净空、到达时间、控制/求解延迟；
指标和场景按假设选择，没有统一次数或必填模板。

实验主体必须能构建或运行，排除明显实现错误，保存复核结论所需的关键条件。
实验中的临时接口、mock、独立 harness 或简化模型要说明限制，不污染正式默认配置。
Git 隔离不免除真实串口、运动授权和既有安全合同。

默认不做与假设无关的工作：

- 全仓回归、exhaustive test、大量异常输入、全套 ASan/UBSan。
- 所有 launch profile smoke、所有历史兼容检查、Docker 镜像重复确认。
- 逐文件 checksum、大量 SHA256、构建产物 hash、第三方库重复 hash。
- 同一 commit SHA 在多处重复记录、大规模 evidence archive。
- production fallback、部署文档或 prototype 的 production-size test suite。

结果必须允许 **Go / Modify / Stop**：

- Go：关键证据支持价值，再做所需集成；实验 Go 不等于部署 Go。
- Modify：只补会改变决策的信息，优先消融、关键场景或失败原因。
- Stop：核心假设失败，先排除明显实现/配置/环境错误并最小复核；仍失败则停止工程化。

失败是有效结论；不为已停止路线继续扩充回归、fallback 或兼容层。
重要阶段更新已有研究结论，保留关键代码与最小关键证据，不要求每轮任务写报告。

## Integration：证明正确接入

方案已有价值后，按本次影响面验证：

- 接口语义、TF/topic ownership、frame、timestamp/clock domain。
- 配置、启动组合、生命周期及主要上下游交互。
- 核心回归和真实失败边界，必要的 timeout/watchdog/fallback。

这些机制按实际故障和边界选择，不是每个模块都要补齐的清单。
禁止自动将 Integration 扩大成全仓 Deployment 验收。

## Deployment：准备正式使用

真正准备进入 main 的正式功能、实车、比赛或发布时，依据实际风险完成所需的
系统回归、边界、失败恢复、安全/硬件、资源实时性、可复现性与回滚验证。
必要时固定 dependency、artifact/hash 和少量里程碑；不要求无关模块重复验证。
文档整理或局部低影响修复仍按影响范围验证，不自动跑完整系统矩阵。

Simulation Go ≠ Hardware Go；offline replay Go ≠ field Go；build success ≠ functional acceptance。
报告区分软件、仿真和实物证据，不能用某一类通过代替另一类接受。

## 测试必须发现有意义的错误

新增测试前回答：它保护什么行为？真的可能坏吗？已有测试是否覆盖？失败会改变什么决策？
这些是取舍问题，不要求逐项提交 checklist。

优先选择真实 bug 回归、边界、核心数学/算法、接口 contract 和少量 end-to-end 测试。
不为数量写 trivial getter/setter 测试，不锁内部实现细节，不对同一边界写多套近重复测试。
已有测试存在不代表每次都要全跑；每次代码修改至少做一次直接相关有效验证，无法完成就说明限制。
纯文档修改检查 diff、路径、链接和 Git 状态即可。

## SHA、hash 和证据链是工具

**SHA、SHA256、哈希和证据链是工具，不是工作成果。** 不因“可验证”就默认验证 hash。
只有需要确认大文件数据集身份、地图 bundle/artifact 完整性、外部下载文件完整性、
Deployment 冻结，或严格 A/B 必须证明输入完全一致时，才值得使用。
普通 Research 记录必要代码版本、关键配置和环境信息即可；同一身份集中记录一次。
本规则不取消既有地图 bundle 等运行合同中的完整性检查。
