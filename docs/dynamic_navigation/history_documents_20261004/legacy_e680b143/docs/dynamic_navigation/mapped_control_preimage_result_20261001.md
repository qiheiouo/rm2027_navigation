# 原bounds/filter映射存在不同raw的相同完整像

## 背景与方法

按[预注册](mapped_control_preimage_preregistration_20261001.md)完整八组输入，按固定字典序选择首个raw越界元素并向同一bounds外再改动f32 .125。选择没有truth/安全/critic参与。两个raw恰好一个元素不同，第一次原bounds clip后完整字节相同，再按原四history、原in-place SG滤波、原末端bounds clip核对完整控制矩阵。没有轨迹传播、权重或运行输出更新。

## 数据身份

源码/prepare `d3e0818207368e3e88e6c95fd203028213b1f0a4`，评价 `2f34221734ef99212d21e353e958cd2d19ed3bc6`。input SHA `f6ef6c33ccf2071944cc95afd89c9d94f8dc013fd85e9e01bdf4d3ee51fca783`、参数SHA `0a070fc673c5f124f492b0d38fc5b4a97b51be09af5d33d20dd307593741a206`；原提案manifest SHA `bc8d5245fe69fd654c9acede1b2cf5da4db88a92a09dcd0cf489b6553148a699`。结果summary SHA `f43eb6ad91387ca3edae5a84ea66e6930d2fb206af40f51e3c02c13191052037`，镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

## 结果

八组原mapped全矩阵（每组2000×30×3）逐字节复现冻结文件；八组mutant经映射后也与各自原mapped完整字节相同。每组raw只改变一个元素；第一次clip已完整消除该差别，后续滤波/history/bounds无变化。所有反例通过固定门。

位置记(candidate,time,axis)，从0开始。原/相关共享首步xi0，所以每seed两输入组位置和raw改动相同，不能视为八个独立成功实验。

| seed | 原/相关位置 | raw前→后 | 原独立模型数学量差 | 相关模型数学量差 |
| --- | --- | --- | ---: | ---: |
| 0 | (0,0,vy) | -.688496→-.813496 | +.001002238 | +.000479250 |
| 1 | (0,0,vx) | 1.029190→1.154190 | -.001037382 | -.000659686 |
| 2 | (0,0,vx) | 1.009536→1.134536 | -.001037382 | -.000659686 |
| 3 | (0,0,vx) | 1.579627→1.704627 | -.001037382 | -.000659686 |

两raw的 `gamma*u^T*Sigma^-1*epsilon` 数学量不同，虽各自完整mapped控制相同；独立密集精度线性导数乘实际delta核对最大误差8.24e-18，低于预注册1e-12。该量只属于无舍入线性latent模型，未当作critic或重要性权重。

## 结论、限制与下一步

这条具体原bounds/filter/clip映射非单射，不能仅凭线性Gaussian逆协方差公式，就把mapped变量解释为同一个可逆线性换变量；实际更新仍未验收。非单射本身不证明所有映射后分布非Gaussian，也不否定可以另行在latent空间推导目标测度和重要性权重。反例没有安全、排名或控制性能结果，不能救援已停止风险组合。

矩阵保持原bounds、history及滤波；修改只在冻结raw提案的实验副本，没有发送运行命令。现有必要代数已给出边界，后续不能跳过mapped密度/边界质量、latent均值→实际输出一致性和原三秒安全验证。

下一项回到独立输入采集：先只读确认新设备已有Gazebo/launch/diagnostic capture能力与已冻结镜像相容性，记录版本、场景/配置hash及TF所有权，不运行旧镜像门已经失配的holdout、不改旧协议或正式配置。新的完整周期数据协议须另行预注册，不能拼接synthetic observer capture与历史周期来冒充同一次运行。

[八组固定mutant输入](evidence/mapped_control_preimage_inputs_20261001/manifest.json)、[两组完整mapped文件、数学量与日志](evidence/mapped_control_preimage_result_20261001/manifest.json)保留。main、运行optimizer/critic、TF、正式配置和原安全门不变。
