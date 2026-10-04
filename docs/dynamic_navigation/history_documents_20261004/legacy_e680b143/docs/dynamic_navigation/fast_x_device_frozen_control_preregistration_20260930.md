# 周期 47 新设备控制矩阵冻结离线诊断预注册

## 背景

历史 capture 数值门与标准噪声姿态字节门均未通过，结果保留。不能重新标记为通过，也不继续原协议。当前有界七 critic/原生 MPPI 与独立参考数值门通过，仅允许新设备反事实离线研究。

## 假设

沿用迁移前登记的 cycle 47、四 seeds 与 std4，当前设备字节冻结的控制矩阵可能包含严格联合安全候选；即使存在候选，原 V1 排序及最终聚合仍可能不能利用。此诊断先回答分布覆盖、排序、输出三个层次，不据此扩大运行 batch 或调整运行 std。

## 方法

1. 从已保存的默认执行四个 2000×30×3 float32 控制矩阵读取，验证其 SHA；不在评分时再次调用 RNG。保持原初始控制、原 settings/history/profile 和四项 EXPECTED 输入 hash。
2. 依照已有公式 `initial + float32(4) * (base - initial)` 形成 std4 控制。先写出各 seed base/scale4 与原标准噪声 map，保存输入 manifest、参数 SHA、源码提交、镜像和控制文件 hash。输入冻结阶段不评分、不读取未来真值标签。
3. 评分阶段只读取并验证输入 manifest 和控制字节；标准噪声 map 须与新冻结 map 完全相同。原历史 map SHA 差异仍报告，不使用容差放宽旧门。独立 schema 标注 new-device frozen-control counterfactual。
4. 四 seeds 各自运行原生有界七 critic、V1、正则、softmax、限幅、滤波；七项与六 C++ 加独立 Python PathAlign 误差 <=1e-3；姿态 <=1e-5；聚合/滤波 <=5e-5。原门不变，每 seed 未过门则停，不解释排序。
5. 真值仅评分后离线标注：单条控制先限幅/原滤波，再按当前速度第一步传播；最终原生控制另行按同一口径传播。联合安全仍 body gap >=0.05 m、padded gap >0、raw CostCritic 无碰撞。记录安全条数/最佳总排序/概率质量/输出安全和进展，记录首次 V1 占用是否饱和。

## 固定输入与参数

沿用 `near_x_fast_coverage.EXPECTED` 四 hash 和 cycle 43–46 历史 SHA。profile 参数 SHA `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`；输入 manifest 另存 std4 扩展参数规范 JSON 的 SHA。镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。PathAlign 有界头 SHA `56b45373d9a4a094286a684baf7990365d4d1b769400820024c3ceb161da70a2`。

## 判据、限制与下一步

四组全部完成才报告 seed 间结果；单次成功不接受为部署证据。无安全候选记录分布否定结果；有候选但权重/输出不安全记录排序或聚合限制。旧结果不删不改；本项不闭环、无线上修改、无 TF 改动，不声称 std4 参数有效或历史精确 replay。完成后按观察决定最小在线可计算风险或输出一致性实验，仍先离线预注册。
