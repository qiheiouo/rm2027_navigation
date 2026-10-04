# 原bounds/filter映射的非可逆反例预注册

## 背景与假设

必要协方差代数已通过，但raw→bounded-filtered大部分元素改变；未证明mapped变量可以沿用latent Gaussian表达式。此次假设两个不同raw提案可以经原clip/filter/clip映射成同一完整控制序列，用冻结数据否定简单可逆线性换变量假设。不证明所有latent推导无效，也不研究安全/排名。

## 输入与固定选择

完整八组原提案manifest SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`、参数SHA `7a9db00145f8d4c05a8c446510fafa5b189836aeb7bdfbad6c6f1e253dc1d9a2`；读取raw、已冻结mapped、initial及原四输出history，无truth/geometry/critic/label。每组按(candidate,time,axis vx/vy/wz)字典序选首个raw越界元素，不用安全或代价挑选。

准备阶段只将该元素向原bounds外再加/减f32 .125，其余元素完整保留；每组mutant raw全矩阵及唯一改动位置/前后值/source/参数/镜像hash先冻结提交，之后evaluate映射。没有新RNG、batch/std/temperature或控制参数。

## 固定检验

两个raw须恰好一个元素不同且仍在同一bounds外；第一次clip后完整字节须相同。独立重新执行已有 `filtered_all_critic_probe.filtered_controls` 及原bounds末端clip：全部八组原mapped必须与已冻结文件逐字节相同，mutant mapped也须完整逐字节与原相同；任何失败停止反例接受。原history/滤波系数/顺序/bounds均不改。

另仅报告Gaussian线性模型数学量 `gamma*u^T*Sigma^-1*epsilon` 两raw间差，和固定精度矩阵线性导数乘实际f32 delta的对照，误差≤1e-12。它不是critic/重要性权重或安全判据；若数学量相同也报告，不据此换位置。无softmax、聚合、轨迹传播或新运行命令。

## 判定与限制

反例只能否定这条具体映射的可逆线性解释；非单射本身不证明所有映射后分布都非Gaussian，也不否定可在latent空间另行推导重要性权重。mapped变量的密度、边界质量及latent均值→输出更新仍须独立证明。即使通过，不接受动态安全、MPPI一致性或已失败风险组合。

镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`不变；结果和否定失败全部留docs/evidence，不改线上optimizer/critic/TF、正式配置/main或安全门。待有完整目标测度与映射处理依据才考虑新的离线更新协议，不能把本反例作为部署修改。

## 评价前冻结登记

源码/prepare `d3e0818207368e3e88e6c95fd203028213b1f0a4`；input SHA `f6ef6c33ccf2071944cc95afd89c9d94f8dc013fd85e9e01bdf4d3ee51fca783`，参数SHA `0a070fc673c5f124f492b0d38fc5b4a97b51be09af5d33d20dd307593741a206`。八组mutant全矩阵先保存，seed0首个位置(0,0,vy)，其余(0,0,vx)；原/相关首步共享xi0，所以每seed位置相同，不是八个独立成功实验。[完整输入](evidence/mapped_control_preimage_inputs_20261001/manifest.json)先提交，尚未evaluate。
