# 研发方向纠正：三层算法判定

## 决策与背景

用户在2026-10-01明确要求已有研究基础设施视为足够使用并默认冻结。此前工具验证消耗过多时间；后续成果必须回答Sampling、Prediction/Ranking、Aggregation中的至少一个机制问题，不继续完善实验法证体系。

当前已启动的v4近场capture完成并保存，只有直接阻塞这次capture或已明确算法PASS/FAIL的最小工具缺口可以修复。不新增recorder、receipt/CDR层、graph/library/launch gate、byte-exact/TF replay或验证工具的验证工具。旧实验、工具失败及证据保留，不删除已写章节。后续工具错误统一记 `experiment_infrastructure_notes_20261001.md`，主研究结果以算法决策为中心。

## 下一实验决策

新capture完成后先预登记独立近场holdout，按goal相对固定时间选择周期，不读取未来安全标签或排名作选择。固定原batch/profile/footprint/原0.05m/padded/rawCostCritic/full3s判据。记录source commit、输入/参数/image SHA，浮点验证采用语义一致、既有冻结数值阈值和最终判据一致，不要求跨设备逐字节一致。

Gate1报告联合安全候选数及目标推进；没有安全候选则只判Sampling/reachable control space未覆盖安全解，不解释ranking。Gate2在安全候选存在时报告best safe rank、Top-K安全数、safe weight mass及risk/标准critic/总分。Gate3报告weighted/filtered控制、native dynamics、dynamic/static/progress，并比较single-safe、oracle-safe-only、online-weight aggregation；oracle仅离线诊断，不是可部署算法。

优先分清raw candidate→aggregation→filter→integration在哪一步失去安全，再仅研究该层。不同时改三层，不回到已CLOSED的batch/temperature/时间前缀/硬V1饱和/中点/residual/PathAlign/CostCritic反复验证。新机制必须说明对应复现失败、现有方案不足、成熟公开实现及能否更小改动；没有答案不实现。

## 限制与下一步

尚无本次holdout算法结果。连续60–90分钟没有算法机制判断时，主动停止非阻塞工具工作。三层全PASS才考虑最小runtime candidate与预登记闭环A/B；单个holdout不宣称稳定安全。main/比赛profile/真实footprint、TF契约与低耦合规范保持，仅当前研究分支继续，不merge或部署。
