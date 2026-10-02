# 动态障碍公共输入：几何语义与未知边界

2026-10-02，根据实际tracker生成端、公共v1消息和critic/guard消费代码
核对。本文件明确既有字段，不新增公共结构、话题或TF所有者；尚未
建立完整物体/未来运动安全认证。适用于独立实验路线，不能作为实车
或2027合规证明。

## 既有字段的实际含义

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

## Unknown和退化边界

当前实现对消息不完整/非法/stale/跳变/错误frame失败：critic fail_flag，
guard零输出；完整新鲜空列表仍依赖当前costmap。该行为已有实测，
零输出不代表瞬时物理停车或完整未来支持。

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
