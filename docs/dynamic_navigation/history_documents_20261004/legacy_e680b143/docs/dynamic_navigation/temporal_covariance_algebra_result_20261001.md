# 相关提案协方差代数通过，mapped控制与latent提案需另证

## 背景与方法

按[预注册](temporal_covariance_algebra_preregistration_20261001.md)完整使用八组冻结提案，不读truth、coverage labels或critic。只定义无舍入线性Gaussian模型的数学量 `gamma*u^T*Sigma^-1*epsilon`，以密集solve和独立局部白化两种计算核对。原f32 rho/q/std不调，不偷换为理想Toeplitz协方差。没有候选排名、权重、聚合、新控制或运行更新。

## 数据身份

源码/prepare `c3bbf782b66501790c82b80c5c23065a646ef15b`，评价 `1ab0f2c5d9382c96aeb6bcb6d66b4e6338f59a6f`。input SHA `a813be60135daddc45c1ee9907a75ae99f7aa05ac409f14943a172eb8e7c3865`、参数SHA `6d549edf98f8d64a204064dd4f97ba172725ec353828acd93617df4cda19dfdc`；原提案manifest SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`。结果summary SHA `a1195078a1be262fe092531890d856fa501e2b8765a082ecacb15393a5b46821`，镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

## 数值结果

AB/BA恒等误差分别1.1102e-16/2.2204e-16；密集逆R与B^T B误差5.3174e-14，均通过。R条件数256.5696，对角相对1偏差最大7.2124e-8，来自实际f32 q与rho的组合，不能静默用理想stationary R替代。

完整八组密集/白化每轴最大误差5.6379e-17、总项最大8.4134e-17；rho0/q1对白f64对角式误差均0。四组相关f32 noise递推及raw控制再构造字节全部一致；白化stored eta恢复xi最大误差7.8894e-7，低于预先固定3e-6。

| seed | 相关项减原对角项范围 | 差值RMS | f64恢复xi最大误差 |
| --- | --- | ---: | ---: |
| 0 | -.535892 至 .537896 | .163729 | 7.2223e-7 |
| 1 | -.551423 至 .633240 | .162223 | 5.9024e-7 |
| 2 | -.501781 至 .606044 | .168228 | 7.8894e-7 |
| 3 | -.578989 至 .565078 | .167742 | 7.1203e-7 |

independent组使用rho0/q1，相关/对角差值为0。Python按native源码形式的f32对角量与f64对角量最大差9.7854e-8；它使用NumPy sum，**不是xtensor原生二进制或实际控制门**，本实验没有重新执行native更新。

## bounds/filter映射诊断

每组每轴60000个元素，完整八组raw→bounded-filtered改变58837–59228个（98.06%–98.71%）。raw提案尚未经过bounds，越界计数不能外推实际返回命令。四seed范围如下，完整逐组计数仍保留：

| 输入组 | raw越bounds vx / vy / wz | 最终mapped恰在bounds vx / vy / wz |
| --- | --- | --- |
| independent | 27999–28179 / 34849–35033 / 27630–27884 | 1857–1883 / 2767–2943 / 1296–1343 |
| correlated | 28000–28602 / 34596–34937 / 27559–27909 | 10986–11274 / 14004–14233 / 10673–10901 |

将同一形式代入mapped-initial与raw-initial的总项差RMS：independent .10477–.10679、correlated .01653–.01703。这只说明参数不同，不能解释为“mappedGaussian的正确代价”。bounds/filter是原实验映射，没有新投影或运行参数。

## 决定、限制与下一步

**接受无舍入线性模型的必要代数核对，实际MPPI proposal/权重/更新仍未验收。** 原对角式与相关式不同，不能仅改变噪声后把对角std表达式宣称为协方差一致；clip/filter后控制也不能未经证明当同一latentGaussian扰动。此次没有证明完整重要性采样目标测度，不能直接把公式作为上线改动，已失败风险组合继续停止。

下一项可用纯控制、无truth的固定反例审计原bounds/filter是否可逆：先按预注册从raw越界元素确定扰动位置，再将其向bounds外进一步改变，核对两个不同raw提案是否映射成同一完整控制序列。用于否定简单可逆线性换变量假设；即使存在反例，也不能推断所有latent空间推导无效，更不能作为安全候选或控制输出成功。

[输入与参数](evidence/temporal_covariance_algebra_inputs_20261001/manifest.json)、[完整矩阵、16000条每轴数学量、映射诊断及日志](evidence/temporal_covariance_algebra_result_20261001/manifest.json)保留。main、TF、运行optimizer/critic、正式配置和原安全门均不变。
