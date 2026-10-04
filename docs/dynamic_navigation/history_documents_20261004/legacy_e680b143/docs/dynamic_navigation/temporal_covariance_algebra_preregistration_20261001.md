# 时间相关提案协方差双线性项：仅代数必要条件审计

## 背景、假设与边界

时间结构实验已有安全候选，但固定风险组合首选仍失败且已停止推进。不能将相关噪声套用原对角std正则后宣称MPPI一致。此次不救援该组合，只审计Gaussian线性提案模型的必要代数：密集逆协方差与独立白化能否给出相同双线性项，rho=0是否回到原对角形式，以及clip/filter映射改变了哪些控制参数。

这里定义数学量 `gamma * u^T Sigma^-1 epsilon`，不宣称已证明Nav2 MPPI的全部proposal/目标测度或重要性权重推导。即使代数通过，有限精度噪声、clip/filter映射、latent均值更新与实际输出一致性仍未验收。无真值、七critic/risk、候选排序、softmax、权重、聚合、新控制或线上修改。

## 固定输入与数学方法

只读取已冻结完整8×2000×30×3 raw/noise/bounded-filtered及initial控制，source manifest SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`，参数SHA `7a9db00145f8d4c05a8c446510fafa5b189836aeb7bdfbad6c6f1e253dc1d9a2`。不读取coverage labels、物理truth或排序。rho与q用原f32已冻结值转换成f64，std=.8/.8/1.6原值不调。

无舍入线性模型 `eta0=xi0; eta_t=r*eta_(t-1)+q*xi_t`：`A[i,0]=r^i`，`A[i,j]=q*r^(i-j)` (1≤j≤i)，`R=A*A^T`，每轴 `Sigma=std_axis^2*R`。B为下双对角，B00=1、后续Btt=1/q、B[t,t-1]=-r/q。密集计算用solve(R, epsilon)，独立白化用 `(B*u)^T*(B*epsilon)`；固定f32 q不是精确sqrt(1-r²)，不偷换为理想Toeplitz R。

先核对AB/BA与I最大误差≤1e-12、密集逆R与B^T B≤1e-10；完整8组逐条每轴与总双线性项误差≤1e-10。rho0/q1白化与对角f64量≤1e-12。复现原f32相关noise递推和raw控制须四seed字节一致；B白化stored eta与原stored xi的舍入误差固定≤3e-6，失败停止代数解释。

报告四seed相关提案原对角与相关双线性项差异完整分布、原native f32累加式与rho0 f64差异（仅数值诊断，不伪称原生二进制控制门），R对角偏离1及条件数。报告原raw超bounds数、raw→bounded-filtered改变数与最终饱和数；用同一形式代入不同mapped控制只能标为参数对照，不能把映射后变量当相同Gaussian扰动。所有8组和逐条项保存，不选有利seed。

## 冻结、结果与限制

源码/参数/输入envelope在evaluate前提交；镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`不变。必要条件失败保存并停止；通过也不修改任何运行critic/optimizer/正式配置/TF/main或安全门。否定结果及全部数值写入docs。下一步只能依据审计识别剩余证明义务，不能直接把该双线性量作为可部署更新。

## 评价前冻结登记

源码/prepare `c3bbf782b66501790c82b80c5c23065a646ef15b`；input SHA `a813be60135daddc45c1ee9907a75ae99f7aa05ac409f14943a172eb8e7c3865`，参数SHA `6d549edf98f8d64a204064dd4f97ba172725ec353828acd93617df4cda19dfdc`。原八组输入文件hash已核对，没有打开truth/coverage label或critic score。[输入及日志](evidence/temporal_covariance_algebra_inputs_20261001/manifest.json)先提交，尚未evaluate。Python按native源码形式的f32诊断使用NumPy sum，不称与xtensor原生二进制完全一致。
