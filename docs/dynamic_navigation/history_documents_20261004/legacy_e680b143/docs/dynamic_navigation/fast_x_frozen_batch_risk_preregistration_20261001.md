# 周期 47 固定随机控制集合风险迁移预注册

## 背景与假设

在线固定连续风险在 27 条确定性原型上改变排序且原生输出通过，但四组固定 std4 随机控制各无联合安全候选。下一假设：同一风险项在这些固定集合中可能改善原生聚合输出；即使候选全不安全，非线性聚合也未必与某条候选相同。也可能无改善，反映覆盖或预测/输出限制。本项不改变采样。

## 固定输入与准备

沿用 `controls_frozen_device_20261001/inputs.json` SHA `e35c5c4ece49f581f6548156685cb7e74226e6f1d5658f67594ed856f144f47e`，原四 seeds/每组 2000×30、std4、profile、原 limits/temperature/gamma/history/footprint/padding、安全门均不变。全部控制与 baseline 已冻结，不基于真值选子集或增补原型。

每 seed 风险的 raw 输入采用此前实际原生评分导出的 `native_poses.bin`；actual-filtered 输入采用此前冻结 `filtered_map.bin` 中的姿态，不重新调用 RNG或改变滤波数值。轨迹文件 SHA 与此前 result/manifest 逐项核对。在线风险仍是相同既有 uniform-center overlap、order=5、原 tracker/age/支持/0.9 s horizon，附加项固定 `float32((3.81/254)*1e6/9 × risk)`。风险输入/参数 JSON 与二进制在评分前冻结、登记 hash。

## 执行与判据

prepare 提交 `6645f55`，完整冻结输入 SHA `1d99e0c063c26194c1589bbba5a47e2f5ea382154c89737a36584b4b86d78541`、参数 SHA `f98d8e5dde4dc46a184a81971377d1a6ac427156159dabae3df9e46305be68ab`。[风险输入证据](evidence/fast_x_frozen_batch_risk_inputs_20261001/manifest.json)在执行输出评价前保存；四组 raw/filtered 风险均有 span，但不据此声称安全区分。

镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。使用已通过 300/27/零项一致门的 audited 离线 replay。每 seed 新工具无附加项对此前 baseline 六类文件先要求 byte exact；未过门停止该组结论并保存。

然后每组 raw 和 actual-filtered 两个固定风险口径，原生七项/V1保持，附加项在正则前加入。score/weighted 对原生顺序参考 <=1e-3；control 前后 <=5e-5；姿态 <=1e-5；概率和误差 <=1e-5（数值门，非物理门）。所有风险/成本/控制记录后，真值只检验完整 after-filter 序列当前速度首步的 body>=0.05 m、padded>0、原 raw map 静态门及目标推进。

完整报告四 seeds 两风险口径与各自原 baseline 的间隙/静态/推进/返回控制/有效样本数/最大概率索引/风险 span。四组共享一个冻结周期，不当独立闭环率；候选零安全时不计算 AUC或最佳安全名次。保存所有中间结果与失败，不用个别成功接受算法。

## 限制与下一步

历史 capture/样本门依旧失败，此研究仅当前设备反事实。风险 uniform 支持不校准为概率，0.1 s 点安全不证明连续安全。若此固定迁移无稳定收益，停止该方向的参数尝试，记录为何有限原型正对照不能传到固定随机集合；再据数据预注册控制时间结构或在线预测输入实验，不能直接改线上 critic。
