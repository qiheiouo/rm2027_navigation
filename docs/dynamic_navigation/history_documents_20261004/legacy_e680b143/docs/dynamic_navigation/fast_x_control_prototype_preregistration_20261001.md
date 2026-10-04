# 周期 47 固定允许控制原型可行性对照预注册

## 背景与假设

四 seeds 原标准噪声和新设备 std4 冻结控制均没有联合安全候选，不能再靠风险排序改善这些集合。假设这只是固定高斯分布覆盖不足，有限的预定义允许控制中仍可能存在严格安全动作；也可能状态/历史滤波已使这些简单动作均失败。

## 固定方法

保持 cycle 47 EXPECTED 四输入、history 43–46、原 profile/footprint/padding/raw map、3 s horizon、dt、当前速度首步。冻结 **27 条恒定 body-frame 控制**：vx 从 `{vx_min, 0, vx_max}` 取值，vy 从 `{-vy_max, 0, vy_max}`，wz 从 `{-wz_max, 0, wz_max}`，按 vx/vy/wz 字典积固定顺序，不按未来轨迹挑控制。不改变 velocity constraints，不增加 Gaussian batch 或噪声。

准备阶段将控制矩阵和参数规范 JSON 写出并 hash，提交关联后才评价。所有原型先原限幅/滤波，再当前速度首步重积分 30 步；用原 raw map 原生检查器和未来完整物理箱体独立标签，body>=0.05 m、padded>0、无静态碰撞原门保持。保存每步 gap、最小值、首次破门、静态结果、目标推进、控制输入/过滤输出/姿态 SHA。真值只评价，不生成或修改动作。

## 数值与安全判据

这项只检验几何可行性，不运行 MPPI 排序或聚合，不宣称这些控制来自高斯分布。原型 filtered trajectory 与既有 Python 原滤波/传播口径相同；再用独立原生单候选 replay（只在其 raw scoring 无 fail flag 时）复核限幅/滤波/传播。单候选评分失败不否定动作几何，只保存失败并停止对应数值声明。完整 body/padded/raw map 门不放宽。

存在至少一条联合安全原型，只能否定“本有限动作对照也没有安全”；全部失败只形成 27 条有限集合的否定结果，不能证明所有动作不可行。只检查采样点，无连续安全或闭环结论。镜像仍固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，源输入 profile SHA `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`。

## 结果、限制与下一步

准备阶段执行 `b7f2dea`，控制冻结 inputs SHA `760c37648394d418b8be7055c1c1d8e703f12b78670d425cbcd8f77192eaaba5`，参数 SHA `6ca3d7000d2470b61b8e0e2c50b61ad9592b9ec93c428cb10ea1ac9147534db0`。已在评价前归档 [固定输入](evidence/fast_x_prototype_inputs_20261001/manifest.json)。

先记录控制冻结 manifest 与执行 commit，再执行评价，结果保存新目录且同步 docs/evidence。若安全原型存在，后续风险区分实验另行预注册；不得将 oracle 标签用作线上控制选择。
