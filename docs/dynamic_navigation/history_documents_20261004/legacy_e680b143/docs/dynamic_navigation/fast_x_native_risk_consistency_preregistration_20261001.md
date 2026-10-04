# 固定原型风险与原生最终输出一致性预注册

## 背景与假设

固定 27 原型上，原总分首选 #14 不安全，加已固定在线风险后首选 #18 安全。但评分安全不代表实际输出安全。本项检验固定风险是否在实际原生 MPPI 加权/限幅/滤波之后仍形成严格安全轨迹，仅作有限原型反事实。

## 最小工具扩展

仅扩展 `frozen_cycle_replay.cpp` 离线工具：可选 metadata 指向逐候选 float32 additive costs，读严格长度和有限非负值，加入原七 critic/V1 之后、原生正则之前；默认不读取任何附加项。导出原生 update 后 weighted costs，以及用相同原生表达式从该 costs 重建的概率，明确重建来源。原 Nav2 实现、运行 V1、正式配置、TF、footprint/padding/判据均不改。

新二进制单线程构建独立目录，不覆盖旧二进制。先用冻结原 300 条和 27 原型基线与旧二进制逐项比较：scores/V1/full/poses/before/after 要求字节完全相同；可选全零 additive 对 27 基线也要求完全相同。若失败则停止风险输出声明并保留证据。

## 固定输入与方法

27 fixture SHA `760c37648394d418b8be7055c1c1d8e703f12b78670d425cbcd8f77192eaaba5`；风险来自 `prototype_risk_20261001` 已冻结 raw/actual-filtered 文件，不重新计算或用真值选择。附加项 `float32(original collision unit × risk)`，第三项为原基线；原 std、temperature、gamma、history 和积分/滤波不变。每条 additive bytes 和参数文件先冻结、hash 再执行。镜像仍为 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

评分数值对既有七项/硬 V1/固定风险加法 <=1e-3；原生 weighted costs 对 native-order 参考 <=1e-3；前后控制 <=5e-5，姿态 <=1e-5。概率标注从原生 weighted costs 按同表达式重建，验证其和为 1 及无不安全真值筛选。未过数值门停止对应输出安全结论。

完整原生 after-filter 序列以当前速度首步传播30步，独立核对物理 body>=0.05 m、padded>0、原 raw map 静态门；记录三口径目标推进、最小间隙、首个破门、实际返回控制和单项/加权/滤波差异。真值只在最后评价，不能进入风险文件或权重。

## 限制与下一步

prepare 执行 `f94c9c2`，冻结输入 SHA `01e18215c4e3803d181670e94c0611f1dd058270e3c84bbeadf9c7320bebba08`、参数 SHA `0da9bf38e0668b74d5d959ccecaeee5dbbf6f8f3b33343a23669ce54dbd8af54`。新工具构建执行 `1b4c1c2`，独立 `audited` build/install 单线程 Release，39.2 s 完成，旧工具保留。评价还检查负值、NaN、短/长 additive 文件必须 exit 1 且不导出 scores；这是输入门，不改变安全判据。

确定性原型不是 Gaussian 样本，此项不形成上线策略或闭环成功率；即使两风险口径输出通过，也只表明此有限输入的一致性反事实通过。旧八组 oracle 中安全聚合失效的反例保持成立；下一步必须另行验证真实采样覆盖与在线可算风险，不直接修改线上 critic。
