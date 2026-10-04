# 动态障碍公共输入：几何语义与未知边界

2026-10-02，根据实际tracker生成端、公共v1消息和critic/guard消费代码
核对。本文件明确既有字段，不新增公共结构、话题或TF所有者；尚未
建立完整物体/未来运动安全认证。适用于独立实验路线，不能作为实车
或2027合规证明。

## 既有字段的实际含义

表中描述默认 v1。显式 `prediction.anchor_mode=last_observation_cv` 采用
`rm_dynamic_obstacle_predictions/v2_observation_anchor`，只改变 position 的
锚点语义：最近真实关联检测质心 q_o，由本次滤波速度 v_hat 外推至
scan 源时间 s，`position=q_o+v_hat*(s-o)`。匹配/滤波/可见尺寸算法
保持；显示严格使用未裁剪、未衰减 CV，整数 ROS 观测 stamp 原样保留。
critic/guard 静态 input_schema 必须同步选择，schema 转换拒绝并替换
旧快照，非 coasting 的 o 必须等于 s、coasting 的 o 必须早于 s。
公开字段仍无真实误差界或完整轮廓；错关联的质心也不属于真实物体。
详见[实验登记](../dynamic_obstacle_critic/observation_anchor_cv_experiment.md)。

| 字段/数据 | 生成与时间 | 可以表达 | 尚不能表达 |
|---|---|---|---|
| array.header.stamp / frame_id | 输入LaserScan源时间，map世界frame | 该次更新的滤波状态epoch；扫描通过源时间TF投影 | 发布到达时间、完整物体瞬时几何或逐束同步误差界 |
| position.x/y | 可见端点算术质心观测，CV Kalman滤波；coasting时预测 | 滤波可见簇锚点 | 几何中心、质心在完整物体凸包内的可信界 |
| velocity.x/y | 同一滤波器的速度状态；实际公开值未用显示速度裁剪 | 锚点的估计平面速度 | 材料点/车体中心的真实速度界或三秒误差界 |
| size.x/y | map轴可见点max-min，至少cluster_tolerance；匹配时0.7旧+0.3新 | 平滑可见范围，可能随视角变化 | 完整机械长宽、保守上界、带朝向的矩形或尺寸误差认证 |
| last_observation_stamp | 最近真实匹配观测源时间 | 区分观测与coasting年龄 | 全部物体一直可见或跟踪ID关联正确 |
| prediction[] | 显示配置的速度裁剪/衰减或CV外推，步数/步长在array | 显示锚点序列 | 自然与critic实际CV一致；消费者从xy/vxy重算CV |
| complete / total_track_count | 输出轨迹条数预算，截断时complete=false | 消息列表没有被本预算截断 | 全场障碍均检出、无遮挡/漏检、没有未知物体 |
| 内部Kalman方差 | tracker私有滤波状态 | 当前算法内部估计参数 | v1公开covariance、完整物体置信度或严格确定性误差界 |

`processing_stamp`是诊断时间，不能替代源时间。v1没有目标车型分类、
物体朝向、完整轮廓、geometry reference point、可认证的anchor error
或未来motion bound。消费者不能从markers/diagnostics或感知私有类
补读这些内容；不能把`confirmed`等同几何/动力学认证。

## 当前和未来支持是两个条件

观测锚点模式的条件支持从 o 起算，不能从更新后的 s 重新开始：
`h=(s-o)+(n-s)+t`，`R=D(o)+e(o)+(B+|v_hat|)*h`。其中 e 必须覆盖
测量、source-time TF、关联及表示/运算误差，B 约束完整投影而非仅
车体平移。`conditional_observation_support.py` 只做条件数学，显式
last_observation_epoch 与 observation-origin motion semantics；未知或
到期仍输出 UNKNOWN/null。旧 source-epoch 数学接口保持原样。该公式
未接入在线半径，默认可见尺寸圆依旧无法证明三秒真实支持。

后续显式 `obstacle_radius_mode=nominal_diameter` 在 v2 下选择
`max(minimum_radius, nominal_obstacle_diameter)`，完整D作半径，避免给
未知物体中心使用半直径。配置限定0<D≤3m及非空来源；默认visible模式
要求D=0/来源空，并保持旧公式。固定D只是名义代理，未加真实e或
coasting/未来运动项，仍不证明未来支持。新的私有v2消费文件完整
记录所选模式/D/来源，不能把metadata来源标识当物理认证。范围与
原门见[登记](../dynamic_obstacle_critic/observation_diameter_consumption_experiment.md)。

对物体某时刻完整机械投影集合S，其直径有可信上界D；若锚点q到
conv(S)距离另有可信上界e，则任意p∈S满足`|p-q|≤D+e`。q不必
在规则矩形中心，任意不规则形状也适用。半对角线只在额外已知矩形
几何中心时成立，不能直接放到可见面质心上。

用户要求覆盖全部地面机器人。2026 V2.0.0手册条件下四类目标的
未知类别最大D≈1.697056m；来源、工程两种条款和2027适用性限制见
[尺寸参考](../dynamic_obstacle_critic/robot_extent_manual_review.md)。该D
要求所有目标确属所列四类、符合任意机械状态的完整伸展包络；未核实
载物/附件不能暗中并入保证。原v1消息不提供这些认证，尺寸YAML只是
离线参考数据，当前不加载到在线critic/guard。

源时刻s、评价时刻n、未来t，实际CV锚点为
`q_CV(n+t)=p(s)+v(s)*(n-s+t)`。当前界e(s)不能自动用作n或n+t
的界；预测锚点到该时刻完整投影凸包的e(n+t)必须另外建立，覆盖
观测/滤波/关联/定位误差、源年龄、遮挡和物体运动。超时只限制外推
时长，不证明误差大小。`validation.max_speed`是异常输入拒绝阈值，
显示`prediction.max_speed`是显示裁剪，两者都不是物理最大速度认证。

冻结消费试次中D且e=0假设当前支持278/278，但3s仅89/278；46个
外推至scoreclock的锚点在实际投影外。见
[离线包络实验](../dynamic_obstacle_critic/ground_robot_extent_prior_experiment.md)。
这些真值标签只能诊断条件失败，不能选择在线e、半径或物体类别。

固定source298同路径因子对照进一步表明：补足源时刻物理中心/尺寸后，
实际aggregate的CV风险下降但仍无TTC；换成有限局部真值斜率继续CV
时风险为0，只有不可在线获得的未来真值圆报TTC2.6s。该周期需要未来
运动支持，不能只修当前几何。见
[离线因子审计](../dynamic_obstacle_critic/cv_support_factor_experiment.md)。
它没有建立真实机器人运动上界、观测误差界或在线prediction认证。

后续固定native上下文重加权显示：future-circle真值可使新aggregate
完整动态mechanical/padded下界达到0.176/0.130m，但在+2.982s接触
冻结raw216单元，四因子均未通过全部条件门。模型支持、原生输出及
当前地图门必须分别核验，不能把未来真值诊断当可消费契约或输出
安全认证。见[聚合因子审计](../dynamic_obstacle_critic/native_factor_aggregation_experiment.md)。

同权重的bounded mean与actual-history SG完整路径对照确认该周期
future-circle raw216采样接触在SG前已经存在；两stage原全部门
仍0/4，不能用滤波修正推导输出安全或宣布采样器失败。见
[SG阶段审计](../dynamic_obstacle_critic/native_sg_stage_experiment.md)。

个体SDK见证在源pose有39/300完整条件正进展，实际scoreclock为
38/300，差异row63仅padded连续区间保守下界未通过，不能据此声称
接触。原raw风险成本与11+row个体SG标签属于不同路径；即使future
oracle给正见证40.86%权重，aggregate仍失败。见
[权重质量审计](../dynamic_obstacle_critic/native_factor_safe_mass_experiment.md)。

原SDK地图足迹helper进一步确认：中心raw、边界栅格返回值与完整
多边形/连续reserve门属于不同检查。固定局部CV路径边界最高202，
整足迹仍接触raw216；future路径主epoch边界216与原接触见证一致。
不得由任一sample/helper低成本推导整足迹clear，也不得从独立查询
猜测实际CostCritic择支或增量。真实完整内部状态与map单元来源均
unknown，见[地图契约审计](../dynamic_obstacle_critic/native_cost_map_contract_experiment.md)。

原300 raw同路径查询进一步分开原30点网格、密集SDK边界、整多边形
及连续reserve。主7条SDK低于203而保守门失败的路径中，显式十行
单元标量复核证实3条采样接触、4条仅负区间下界；原raw185精确
地图下界为正，而不同的个体bounds/SG路径失败。不能把处理后的
标签接到原风险路径，也不能据此推导SG单一因果或已接受控制。
全300只有保守vector标签，精确单元范围仍是十行。见
[原raw地图审计](../dynamic_obstacle_critic/native_raw_map_contract_experiment.md)。

固定个体三阶段复核将raw185的约束和SG效果分开：原raw与约束后
精确地图下界均为正，SG后出现raw210采样接触。整批主epoch地图
vector通过数90→90→92，约束到SG既有8条通过变失败也有10条失败
变通过。这个个体的阶段效果不认证聚合/闭环，不替换原raw成本，
也不支持禁用SG的全局结论；精确单元范围仅raw185三阶段。见
[个体阶段实验](../dynamic_obstacle_critic/native_individual_stage_experiment.md)。

离线纯函数已将一个可复算的充分条件具体化：若完整未来投影到源
集合的有向Hausdorff位移≤B*h，源集合直径≤D、锚点到凸包≤e，
则原CV锚点可用`D+e+(B+|v_hat|)*h`支持，h包含source age。
这里B涵盖所有机械点/展开/附件，底盘平移速度上界不能单独替代。
bound/来源及源有效区间显式给出；缺失或过期返回unknown/null，
默认真实查询全部unknown。严格源时刻h=0只需D/e。该充分条件不
修改v1字段或在线模型，不把来源字符串当认证。见
[条件支持界实验](../dynamic_obstacle_critic/cv_support_bound_experiment.md)。

## Unknown和退化边界

当前实现对消息不完整/非法/stale/跳变/错误frame失败：critic fail_flag，
guard零输出；完整新鲜空列表仍依赖当前costmap。该行为已有实测，
零输出不代表瞬时物理停车或完整未来支持。

新的原生CostCritic观察试次再次给出实际消费支持0/235（0/1/2/3s）。
首次机械余量失败前0.2s最终命令为零、物理pose不变；接近中的actor
随后在48.108s触及机械投影。这里只是固定试次的物理/诊断关系，不
扩大成全局brake因果证明；已执行停车不是未来完整支持或安全证书。
349组实际CostCritic两侧float变化也不提供目标完整外形/未来运动界。
见[直接成本证据](../dynamic_obstacle_critic/native_cost_accumulator_evidence_experiment.md)。

当前缺失的物理支持条件不会被v1现有validator识别，属于明确的未解决
模型假设。正式消费若需要物理安全保证，应当经公共契约提供或在明确
受限场景配置中建立上述条件，并对unknown停止宣称“clear/safe”；保留
当前地图及最终完整刹停检查，不能把未来支持缺失当成无障碍。是否对
unknown一律制动、缩小授权运行域或由上层拒绝运行，需在后续显式
实验策略中验证任务可行性；本文件不暗中改变当前行为。

若将来需要扩展消息，必须在原`rm_competition_interfaces`版本边界
定义参考点、时间/有效范围、保守误差/支持含义和unknown值，兼容性
与失败策略一同登记。不复制tracker类，不让critic读取感知内部数据，
不靠真值/SDF在线填字段，也不制造未经验证的概率模型。

TF维持[唯一所有者契约](tf_contract_2027.md)：tracker仅在源时刻消费
sensor→map；critic/guard仅消费有年龄限制的同一map→odom世界修正。
尺寸规范或显示需求均不改变TF方向、frame层级、yaw或发布者。
