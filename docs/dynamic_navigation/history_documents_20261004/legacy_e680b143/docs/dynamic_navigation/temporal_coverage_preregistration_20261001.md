# 周期 47 固定扰动时间结构覆盖预注册

## 背景与假设

已冻结四 seed 各 2000 条 std4 随机候选均无严格安全覆盖；原 bounds 投影后的恒定原型 #18/#21 却仍安全且推进目标。假设缺失的时间持续性可能使固定随机集合难以覆盖这些可行动作。该项只比较离线候选覆盖，不接受 MPPI 算法或运行配置更改。

## 冻结输入与单一变换

保留周期 47 与其四帧历史，源控制 manifest SHA `e35c5c4ece49f581f6548156685cb7e74226e6f1d5658f67594ed856f144f47e`。四 seed 各 2000、30 步、原 std4、initial、pose/speed、dt、原 footprint/padding、raw map 与安全门固定。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。不产生新随机数、不改 seed/batch/std、不按未来真值生成候选或选择参数。

对每轴保存的扰动恢复 `xi = (controls - initial) / std4`。预先固定 rho=0.9，float32 递推 `eta[0]=xi[0]`，`eta[t]=rho*eta[t-1]+sqrt(1-rho^2)*xi[t]`，新控制为 `initial + std4*eta`。原组直接使用保存字节，不经恢复重建。理想独立标准高斯下该递推保留单时刻方差、改变时间协方差；不声称有限冻结样本的经验方差相等，须报告逐轴经验幅度和 lag1。

两组统一执行原 bounds clip → 原历史 Savitzky–Golay 滤波 → 原 bounds clip；最后一次 clip 是已明确登记的离线投影。先保存原扰动/相关扰动、未限幅控制、bounded-filtered 控制、参数和全部 hash 并提交，再读取未来标签。所有输入/源码/执行 commit 与镜像登记，不覆盖旧未投影标签。

## 方法与报告

评价完整 8 组共 16000 条 bounded-filtered 三秒轨迹，当前速度首步，body ≥ 0.05 m、padded > 0、原生 raw map 静态门。记录每 seed 的动态/联合安全数、联合安全且正目标推进数、静态碰撞数、body/padded 最佳与中位间隙、逐条标签和全部矩阵 hash。不只报告最优案例，不增补恒定原型。

使用现有已核对的滤波 helper 与 raw map 检查器。冻结初始/历史与原 footprint，不缩小 footprint、不放宽静态或动态门。原组此次新增投影可能与旧未投影标签不同，必须完整重算，不能沿用旧 0/2000。

若任一组联合安全为零，明确报告该固定变换在相应 seed 失败；不追加 rho/std/batch 搜索。若多个 seed 有安全覆盖，只证明这个已查看周期中时间结构能产生几何支持，不证明真实预测可排序、聚合安全或闭环有效。

## 限制与下一步

四 seed 共享同一状态，非四个独立场景；只验 0.1 s 采样点，未验加速度/轮速/连续时间。相关高斯改变协方差及采样测度，原对角 MPPI 正则不能直接移植；本项禁止套用原 gamma 聚合声称运行正确。覆盖阳性之后仍须另行预注册协方差一致性、在线风险排名和最终输出核对，覆盖阴性则停止本变换并记录。

只新增实验脚本/记录；main、正式配置、线上 critic、TF/topic 和既有架构不动。

## 准备阶段身份登记（评价前）

准备提交 `86e21b6`，输入 manifest SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`，参数 SHA `7a9db00145f8d4c05a8c446510fafa5b189836aeb7bdfbad6c6f1e253dc1d9a2`。四组原控制字节 SHA 全部与源相同，两组全部 bounded-filtered 矩阵严格符合原 bounds；准备只核对未来文件 hash，未计算未来标签。完整噪声、raw/bounded 控制、历史、参数和源码身份见[输入清单](evidence/temporal_coverage_inputs_20261001/manifest.json)。
