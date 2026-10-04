# 周期 47 std4 离线聚合参考差异定位结果

## 背景与假设

新设备控制冻结诊断首次 seed 0 在聚合门失败，评分与姿态门已通过，尚未解释排序。按 `device_aggregation_attribution_preregistration_20261001.md` 定位既有 Python 参考与 Nav2 float32 运算的差异，不改变运行算法或阈值。

## 方法与冻结数据

只读首次失败 seed 0 的标准/V1 full scores、原生 before/after、initial/history 和字节冻结 controls；不使用真值。冻结 manifest SHA `e35c5c4ece49f581f6548156685cb7e74226e6f1d5658f67594ed856f144f47e`，参数 SHA `5684cd51c74676838a35599ecbd5b7f0875eea188020903e33594e03cb3feb85`。固定 Humble 镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

初步执行 `08d09b8`，指数式隔离执行 `50a9b6c`；脚本与每项输出 SHA 归档。Nav2 源码直接向 costs 按 vx/wz/vy 分别累加正则；旧参考按 vx/vy/wz 先合并正则、再一次加入 costs。

## 结果

| 参考 | 滤波前对 native 最大差 | 滤波后最大差 |
| --- | ---: | ---: |
| 原参考 | 0.00023406744 | 0.00012421608 |
| 仅替换累加顺序，保留原指数式 | 1.1920929e-7 | 5.9604645e-8 |
| 再采用 native 指数式 | 1.1920929e-7 | 5.9604645e-8 |
| 再采用 float32 系数 | 1.1920929e-7 | 5.9604645e-8 |
| 再采用时间顺序 reduction | 1.1920929e-7 | 5.9604645e-8 |

因此已查明旧参考的正则累加口径足以解释本 seed 0 的控制门差异。保留旧失败；新设备协议改用独立 `device_aggregation_arithmetic.native_reference` helper，历史参考不改。门仍为 `5e-5`，不从单 seed 外推其余三个 seed。

## 限制

这是离线参考实现修正，不能据此宣称修改了 MPPI、改善安全或解决历史 capture 差异。大公共 V1 成本下，实数等价的正则表达式在 float32 中可能不等价，冻结 pipeline 应保留实际运算顺序。尚未直接导出原生内部 weighted costs/概率；目前依据原生完整控制和源码核对参考，概率报告仍须明确来自该独立参考。

## 下一步

同一冻结控制在新目录运行四 seeds，每组核对七项评分、姿态与完整聚合/滤波；全部完成后另写覆盖、排序和输出结果。原捕获/历史样本门继续失败。

[证据 manifest](evidence/device_aggregation_attribution_20261001/manifest.json)包含冻结控制、参数、首次失败全部输出与两次只读归因结果。
