# 周期 47 固定原型在线风险区分预注册

## 背景与假设

27 条已预定义原型中有 7 条通过严格几何联合门，其中 6 条有单候选原生滤波传播核对。固定高斯集合没有安全动作，故先用此正对照分离预测区分能力与覆盖问题。本集合已查看，不能当独立留出集或证明算法泛化。

## 冻结假设与方法

只使用原已接受 tracker/V1 消息、source age、footprint、完整原型控制和固定原 profile。检验现有硬 V1 是否仍饱和、既有连续 uniform-center expected overlap 是否对这些动作有信息，以及该信号在总分中是否保留。复用现有 `occupancy_rank_probe.expected_overlap` 五阶每轴 Gauss-Legendre 规则及原保守支持，不拟合概率、不改中心/速度/窗口/uncertainty，不调权重。

固定比较四项：原生七 critic + 原 legacy V1 + 原正则；纯连续风险 raw；纯连续风险 actual-filtered；总分加 `原 legacy collision unit × 连续风险`，风险分别采用 raw / actual-filtered 两个口径。单位沿用 `(3.81/254)*1e6/9`，硬 V1 原项保持。raw 七项以六 C++ 加独立有界 Python PathAlign 逐行核对，<=1e-3；不改 score 阈值。

原型不由实际 Gaussian noise 采出，因此仅将总分排序和诊断 softmax 当作有限原型比较，不运行或宣称新的 MPPI 输出算法。记录纯风险 AUC（安全风险小于不安全的 pairwise 比率，tie=0.5）、span、top1/top3 安全数、总排序最佳安全名次与诊断安全权重。风险评分先写出及 hash，再读取已冻结安全标签。

## 输入与参数

cycle EXPECTED 四 hash 和 history 不变，原型 inputs SHA `760c37648394d418b8be7055c1c1d8e703f12b78670d425cbcd8f77192eaaba5`。完整 27 条，不基于标签排除任何一条。参数规范 JSON 单独 hash，镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；guarded scorer、原 checker、源 manifest 维持既有 SHA 核对。

## 判据与限制

数值门失败则保存并停止排序结论。风险有 span 不等于有安全区分；纯风险有效也不等于总分有效。即使安全 top1/权重通过，也不接受为预测校准、闭环安全、候选覆盖或部署证据。原 capture/历史样本门仍失败，原型 #25 的单候选 raw fail 保留。后续根据有限结果另行预注册最小实验，不调参寻找单次成功。
