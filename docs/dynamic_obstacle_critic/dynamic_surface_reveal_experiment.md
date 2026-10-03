# 可见动态表面与 reveal / stopping 证据实验

2026-10-03，从配对失败证据节点 `e061a245` 建立
`experiment/dynamic-surface-reveal`。用户提供路线评估并明确意见仅供参考、
由开发者根据证据自主选择。本阶段选择先审计可观测性与停止/逃逸条件，
之后决定是否实现 C；保留 visible-radius A、nominal-diameter B。正式
参数、main、冻结 research/tag、TF、Nav2 optimizer/sampler 均保持。

## 判断依据与边界

完整物体重建不是架构必需条件，但安全需要对未观测区域有明确语义。
有效有限激光返回支持该射线前方在测量时刻/误差/高度条件下的自由，
不支持射线之间、被遮挡区域、无效/缺失返回或未来区域自动成为 free。
未知不是已确认 occupied，也不是 free。当前圆模式是带条件的模型代理，
不能把 B 在小 box 的一次无接触写成“非常安全”。3 秒 MPPI horizon 与
观测预测可信期限可不同，但缩短可信期限不能删除原全三秒接受门。

高频重规划不能独立保证揭露后的停止可行性。`v*tau+v²/(2a)+margin`
只在对应速度/延迟/实际减速度及目标运动条件下成立；转向/轮廓旋转、
动态目标在制动期间继续靠近和自车已停后的目标侵入需另计。已停后被
撞不能靠增加自车制动解决，需要可行逃逸/通行或场景约束。配置中的
1m/s²仅为停止代理参数，不是测得的实际制动能力。

现有资料中“38/300”和 raw185/SG 数值只属于既有选定周期/时钟/条件，
不是当前任意周期覆盖率；raw、约束、aggregation、SG、实际输出分开。
保留 confirmed projected contact、margin violation、interval uncertainty
区别，SDK边界查询不替代 raw203 完整单元/区间门。

## 第一阶段：冻结轨迹 reveal 审计（先登记、后运行）

输入为 A/B 已冻结 manifest，源 scan、canonical odom、Gazebo独立标签、
实际消息/guard/command和消费文件，不重跑或改参数。固定模拟单层
LaserScan、SDF box、source-time外参/<=0.04s插值及 odom/world核验；
不能代表双 MID360、真实不规则目标/任意旋转或全部地面机器人。

1. 原始有限有效射线按最近固定SDF交点离线标注实际box面，实际range
   残差必须<=3*已记录sigma；保留全部失败、遮挡、无返回与姿态缺失。
   该残差门是标签选择，不是高斯安全保证。角点距<=3sigma标记为
   ambiguous，不强制归面。面上固定5cm material bin，仅表示采样分区。
2. 同一面>=3个有效返回、相邻两个source scan间隔<=0.15s，且同一bin
   两次至少1返回，定义**离线可靠揭露代理**。首帧记录为left-censored；
   未揭露bin/整面均列出，不以从未揭露当作“无危险”。不声称该代理
   就是实际tracker/critic使用了该表面：现有公共消息未保留members。
3. 每次首次/间歇重新出现，记录source/确认/observer receipt时间、物理
   机器人pose/measured速度/actor运动、到该material segment的base、
   完整机械与padded距离，旁列最新guard和command的epoch/age。
   区分传感器可见与下游实际消费；无trace关联不填因果latency。
4. 计算停止模型**敏感性**：tau=0,0.05,0.10,0.20,0.40s；a=0.5,1,2m/s²，
   margin保持0.05m。旁列所需a/最大tau与目标继续靠近的影响，不把这些
   假定当实际制动能力，也不根据敏感性为实车给PASS。日志observer
   receive-source、tracker wall计算和消费source-age分别统计，不混加
   不同钟域，不用latest匹配制造总反应延迟。
5. 审计现有第一次margin/contact前后：何时已经收到足够表面、guard
   何时制动、物理速度是否已零、未揭露区域是否仍存在。结论只能为
   “现有轨迹标签支持/不支持某解释”，不能等价于 C 的闭环运行。

## 第二阶段：证据驱动的 C 设计与隔离仿真

首阶段和独立复现后再登记实现选择。有限线段不能跨点簇断口把无返回
空间补成表面；双面/L形不能强制拟合单墙；端点未知、噪声、稀疏/混簇、
旋转和时戳都需明确。只描述实际观测，不恢复假想车中心或全车轮廓。
公共契约显式版本化，producer与critic/guard低耦合，不传私有tracker类。
若缺少真实停止/观测保证，C只能是隔离实验，未知风险不能静默清零。
0/0.5/1/2/3s覆盖与预测有效期限分别记录，不用certainty降权作安全界。

如进入A/B/C，新C与原场景独立相同35s+3.5s、goal、足迹、地图、
sampling/SG/原critic/guard门。记录goal/time、物理body/full机械/padded、
raw203、guard介入/紧停、实际速度/进展、风险贡献、reveal证据。原严格
三秒实际控制与候选见证、150ms值门、5e-5 bounds和 Cases A–E保留。
单次随机闭环不作因果归因；需全部类别/端点遮挡/旋转等独立场景。

当前 Q1/Q2需此审计；真实tau/a未知，不能下可靠停止结论。Q3/Q4尚无
C闭环结果。Q5/Q7需反例与最小可认证假设，不先选最大圆或零隐藏风险。
Q6需按实现判断：仅刷新当前占用没有逐future-time几何就是reactive；
逐rollout时刻传播观测面并查询相对间隙才有time-aware动态风险。STVL
具体包版本功能另以固定源码/官方来源核验，不靠名称推定。

所有原始失败、源码/配置/ELF、审计参数、独立复现与git节点保留；
本阶段无硬件/串口/正式参数推广/合并/推送。A/B原档案不改。

## STVL 边界核验

查阅日期2026-10-03，官方[Nav2插件目录](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)
将STVL描述为带传感器模型衰减的稀疏三维体素占用；[ROS2项目说明](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/ros2/README.md)
解释时间衰减和视锥清理。这里的time主要是观测占用随时间更新，并不
因此自动提供目标速度/转向与rollout未来时刻的表面位置关联。由此推断：
C若只刷新当前表面占用，会保留reactive占用特征；若另做逐未来时刻
预测/相对几何查询，则增加time-aware风险。尚未引入STVL依赖，也未
把当前在线文档当本仓库固定SDK的已安装功能证明。

### 刹停响应补充登记

原配置减速度及SDF限值不当实际能力。仅从已冻结A/B提取最终命令
连续零段（最大observer接收间隔0.10s、上一条非零），起点前<=0.04s
canonical odom值、初始平移速度>=0.05m/s；零段内最多观察2s，要求
平移<=0.005m/s且|wz|<=0.01rad/s持续>=0.20s才能标记已测停止。
不足/重新非零/缺样本均保留未测得；位移取物理truth累积路径，角度
旁列，不能用命令值当物理速度。按所选真实窗口列时间/距离等效
减速度，只是该模拟试次的观察值，不是实车/所有状态制动下界。
记录没有command源stamp、bridge/底盘执行epoch，端到端响应仍unknown。

## 第一阶段结果（源码/审计冻结前检查点）

18项针对角点、遮挡、稀疏、连续确认、时钟/缺样、旋转停止、静止自车
被动态目标逼近及manifest篡改的检查通过。两组reveal/empirical-brake
报告和PNG/SVG在全新目录重执行，8项逐字节比较全部同。一次相对路径
manifest判断错误的初始失败已保留，统一resolve路径后补相对路径回归。

| 离线证据 | visible A | nominal B |
|---|---:|---:|
| 有效源时间scan | 581 | 577 |
| 曾可靠揭露/固定material bins | 20/40 | 20/40 |
| 始终未可靠揭露 | 20 | 20 |
| 首揭露机械gap范围 | 3.803–4.226m | 3.607–4.366m |
| 间歇重新出现事件 | 132 | 289 |
| 实测模拟停止段/零命令段 | 3/15 | 4/18 |
| 等效时间减速度最小观察值 | 1.0931m/s² | 0.53655m/s² |
| 最长已观察source停止时间 | 0.400s | 0.320s |

两组只可靠看到x_minus与y_plus；另外两面40分区中20个始终unknown。
A首次机械净空违反39.898s涉及x_minus:10与y_plus:0，首可靠观测分别
24.157/29.635s，最近源观测距事件33ms；39.966s首次投影接触的相关
分区距最近可靠源观测35ms。此事件不支持“首次揭露太晚”解释，但
实际tracker/critic没有members证据，不把sensor可见直接写成控制链
有效利用。已有零命令/固定物理位置后仍接触的证据继续成立。

Q1/Q2尚不能得可靠停止保证：真实/保证tau、a、旋转和目标继续逼近
的界未知，首次距离较远和少量模拟减速度观察都不足。Q3/Q4没有C
运行。现阶段不选最终Conclusion A–D，也不证明完整prior必需/不必需。
先开发纯观测表面提取的离线质量/端点/缺失回波/双面测试，再决定
新消息/critic/guard配对和隔离C闭环；不因名字改变放宽原门。

## 原生三秒输出补充

固定SDK从两份manifest重新生成原risk/optimizer输入逐字节同，实际
risk、四组SDK输出（正例、zero-history、weights、uniform负例）全部同；
14项SDK比较/完整性门通过。349/350周期完整原生raw状态与pose重现
mask63、aggregate SG和实际MPPI返回control全部位型精确。源姿态时间
完整3s输出及10个独立固定proposal已作条件几何审计，非guard实际执行。

| 实际aggregate路径的完整3s条件门通过数 | visible | nominal |
|---|---:|---:|
| body | 221/349 | 342/350 |
| full mechanical | 214/349 | 339/350 |
| padded | 212/349 | 340/350 |
| 冻结native raw203+区间reserve | 81/349 | 131/350 |
| 完整SG序列与返回bounds | 199/349 | 246/350 |
| 所有上述条件同时通过 | 31/349 | 75/350 |
| 同时通过且有正向推进 | 31/349 | 59/350 |

这不是实际闭环接触率/执行三秒动作，也不能单独怪SG：模型序列、
物理插值、当前冻结map和未执行反事实均有条件。十个proposal的zero
也经过实际history的SG，不能当“瞬间静止”。全300 raw候选虽SDK重现
精确，其全量未来物理几何仍待完成；未来map来源和e/B仍unknown。
本checkpoint A/B整体FAILED、C尚未运行、main与TF保持。

揭露/停止审计已保存在 `stage2_evidence/surface_reveal_preflight`：
225个文件、34,845,477 bytes（不含manifest），manifest SHA256
`d1ad403692a075c2092492f10f9a61053b83c0ded83f2c98dbcf3ba390d26501`。
冻结0d7dd31c审计源、初次与独立执行报告/图、18项单元检查及失败日志。
2026-10-03继续开发时临时目录已清空；此仓库归档和原A/B试次保留。
从归档源再执行的4份报告与4张图已逐字节同。此前仅在临时目录的
SDK/三秒几何文件正在从原始manifest恢复，在完整新归档完成前，文档
中的三秒计数仅作为既有checkpoint摘要，不单独构成可复查的接受证据。

恢复已完成并另存为 `stage2_evidence/observation_native_full_horizon_replay`：
57个文件、367,320,808 bytes（不含manifest），manifest SHA256
`3d4e28af58d7c2e175d1ef239fc09f052ed5aa981c4d39574310432cd49dbcdc`。
新构建三helper ELF与旧冻结记录一致，14项实际SDK比较/完整性PASS，
349/350周期全部3s几何文件完整且两组gate计数与durable checkpoint
摘要一致。旧临时完整几何文件已丢失，因此不声称与旧文件逐字节同；
原risk与四类SDK输出则与原始A/B归档逐字节同。实际返回控制和完整
原生路径保持位型精确，原闭环总体FAILED。初次恢复driver对归档文件名
的错误假设已保留，修复后复核已有新SDK输出，未替换失败记录。

## 下一步离线表面原型登记

先用原source beam身份保留局部观测链，不做box/中心/完整轮廓补全。
输入是已选cluster的真实endpoint、原scan index/range和整数source epoch；
纯模块无ROS/TF、tracker状态或hidden truth。只在原index相邻且range
差及端点欧氏距离均<=0.20m时形成名义短弦，不跨缺失成员、scan接缝、
显著range跃变；孤立返回保留为退化point。预算4096 points/1024 elements，
超预算/非法输入整体拒绝，不能以丢弃表面当complete。点间弦仍是一项
局部连续性近似，只有端点是实际观测，不认证弦内部/隐藏端点后方。

保留全部真实观测端点，不拟合延长线、不连接首尾、不找隐藏中心；
单面/双面/L形/稀疏/缺失/混簇/噪声/平移旋转坐标变换分别测试。名义
CV仅做可配置0.5s age+future期限标记，超期保留UNKNOWN/TR趋势而不
静默成为free；此0.5s只是实验选择，不是可信物理保证。max nominal
translation speed=3m/s用于输入预算，非真实目标运动界。保留旋转
未建模标记，无risk函数/正式配置改变。完成固定源scan质量离线审计后
才登记实际members关联证据与配对消息/critic/guard C隔离实现。

## 表面链原型质量检查（2026-10-03）

18项端点/拓扑/时间/budget反例与3项距离标签检查通过。两组原始试次
均按全部point cluster提取后才作独立truth标签，未用actor位置筛选输入。
旧检测值与顺序逐scan精确一致；原端点没有丢弃。

| 原source-time离线输入 | visible | nominal |
|---|---:|---:|
| 有canonical source姿态并通过旧检测一致性检查 | 583 | 583 |
| 点簇 | 494 | 490 |
| 完整保留端点 | 30,548 | 13,262 |
| 名义短弦 / 孤立point | 30,002 / 4 | 12,766 / 0 |
| 原beam成员断裂 | 52 | 4 |
| 与最近actor交点range残差<=3sigma的端点 | 30,209 | 12,931 |
| range不吻合 / 射线无预期box标签 | 225 / 96 | 204 / 72 |

range标签只对truth bracket与world/odom吻合的scan计算，visible两帧
alignment缺失，nominal一帧truth缺失与五帧alignment缺失。nominal另有
一帧canonical source bracket缺失未提取；因此此表583不是reveal的
581/577，后者先要求truth对齐。不同分母没有改写旧报告。

truth residual门为已记录sensor sigma=0.01m的3倍，是离线一致性分类，
不是误差保证。230/211个actor标签cluster还含其他/未知标签，431/352
个短弦的端点标签不一致；不得把这些全部称为物理多目标混簇。实际
运行tracker TF、关联和内部成员仍未捕获，不能用本回放冒充实际消费。
短弦未提供FREE射线区间、完整occupancy或认证endpoint extension。
本阶段没有surface critic/guard C闭环试次，部署结论仍未建立。

原型源、两轮完整source-index cluster记录、4项冻结源逐字节比较、
21项检查、原tuple索引失败及初次粗标签均保留于
`stage2_evidence/observed_surface_geometry_preflight`：72个文件、
21,075,084 bytes（不含manifest），manifest SHA256
`db65236783dc189979bc848e13eda10cdf28c396c791499e76b34485b4a1c41e`。
依赖的审计源由surface_reveal_preflight manifest绑定；另一次冻结
reveal源执行的4份报告和4张图全部逐字节同，输出一并归档。
