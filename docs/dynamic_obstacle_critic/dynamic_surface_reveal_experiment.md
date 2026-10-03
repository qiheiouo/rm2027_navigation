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
