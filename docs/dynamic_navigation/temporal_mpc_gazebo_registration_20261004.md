# Temporal MPC 实际仿真输入与配对实验预登记

2026-10-04，继续授权后，运行前登记。main派生worktree，已有M1/M3失败证据保留；
不修改正式src/默认，不接硬件。Humble镜像81b325bebf2f…，Nav2 1.1.20。

## 次序与进入门

1. 同一SDF底盘、实际15Hz激光和joint PD动态actor，在单障碍开放地图采集完整v2
预测、processing/source/receipt时间、TF、canonical测量odom、实际cmd_vel和Gazebo物理pose。
tracker从项目冻结b5645eca选择性引入，无算法修改；显式启用last_observation_cv。
Gazebo pose只给观察/物理oracle，不进入预测或MPC。仿真canonical odom按项目
已有GT->lio_adapter边界产生；这是仿真定位，不冒充实车传感器。
2. MPC先shadow，不控制底盘。确认schema/authority、同源TF、整数epoch、源延迟、
confirmed输入、状态/执行子域；保留所有拒绝。控制输出仍由MPPI。
3. 输入门成立才运行同配置B0=STVL+MPPI与MPC/MPPI故障策略配对；同一静态地图、
T-DT规划路径层、动态actor、目标、物理oracle、速度/加速度与sim phase。
不将实际观测extent代理当完整物体；默认名义D圆。若为归因使用代理，单列实验不
作为部署接受，不随结果缩小它。失败不删除，超时/接触保留。
4. 主横穿后验证迎面及完整course静态窄通道；当前外接圆走廊比0.8m通道保守，
不降低足迹/unknown门。若路线或实际输入/动态可靠性明显失败，先归因再有限修正。

## 固定基准

单横穿：从main phase1_omni SDF保留整套底盘/轮子/激光/物理系统；仅移除center_block，
不加入course墙，以隔离动态作用。actor保留main moving_obstacle.sdf，x=4.9、A=.9、T=8s；
goal=(5.6,0)，robot=(0,0)，物理自身外包矩形0.65×0.60m、actor0.45×0.55m。
需要另建head_on时使用同actor机械形状、沿x的prismatic joint和注册固定目标运动，
真实pose独立记录，不拿解析目标充当真实轨迹。

MPC保持20Hz、1.5s/15节点、400iterations、solver15ms/core40ms、4相关/64全量track。
MPPI同时加载，20Hz，Omni，vx[-.5,.8]/vy±.5，角控制0（单切片固定yaw），
非零wz采样方差避免数值除零；VelocitySmoother公共限ax/ay=1、alpha=2。
source-age .4s、原生10ms提案重验、影子状态/时间/plan/map校验保持。
使用实测canonical twist，不用指令代替测量。MPC若因实测子域不成立全部退化，
如实记录，不作为MPC有效闭环成功。

两控制器使用相同STVL当前点云与静态层，不加入未来占用。实际LaserScan转PointCloud2
只保留source header与端点，不生成真值云。T-DT YAstar/SfcSquare承担路径/走廊；
控制核不搜索静态拓扑。上层NavigateToPose与BT接口不变；实验可用固定的T-DT
ComputePathToPose动作前端。此匹配B0是受控profile，不等于旧车正式10Hz基线。

## 判据与完整记录

分别记录action终态、接触、独立真实扫掠净空下界、到达/等待/距离/速度、输出间隔、
source->receipt/solve/command延迟、实际响应偏差、fallback与MPC参与比例。
物理任务接受：成功到达、无接触、完整机械净空>=.05m、实际控制门成立。
物理oracle以真实SDF物体pose和完整矩形独立计算，时间缺口使用相对位移reserve；
收集全shape pose不可用或间隔过大则接受=false，不能拿tracker框替代。
没有同条件B0成功见证则false_block仍null；不要将工程输入通过等同部署接受。
每轮至少保留manifest/固定源码与配置哈希/完整输入事件/action与系统日志/指标；
首次配对完成再决定有限重复，失败先定位输入、可行性、响应或回退。

启动修正登记（正式运动试验前）：v2显式设置display decay_tau/max_speed=0，符合冻结
README要求；Python动作前端使用4线程避免规划阻塞激光与action acknowledgement；
BT acknowledge上限1s，仅路径动作等待，MPC solver/core/native限时不变。目标固定
在sim=10s（8s周期phase=2），输入须9.5s前ready，否则保留启动失败，不随控制器改phase。
世界SceneBroadcaster桥接丢失外层stamp，物理oracle只使用带stamp且启用nested model
PosePublisher的model+link组合；world桥接仅额外原始观察，不能用于净空接受。

后续归因登记（shadow04失败后，未改MPC模型门）：固定已知仿真地图原点用唯一
static map->odom替代main动态identity占位stub，仍由lio_adapter发布odom->base，
不桥Gazebo TF。目的是消除固定原点TF的插值等待，不能修复实际旋转速度越界。
增加actor物理Contact sensor +系统，机器人接触单列（排除地面）；仍保留独立完整
3D机械外包体投影净空，只能称诊断采样下界，未辨识全程速度界则不称物理安全证书。
另建无actor actuator场景，保留同底盘/物理/激光，20Hz ax/ay<=1斜坡脉冲：
[2,5)s vx=.4；[8,11)s vy=.3；[14,17)s vx=.4/vy=.3；[20,23)s vx=-.3，
其余指令0，全部wz=0，sim26s结束，不加载导航控制器，不用于动态性能比较。
迎面真实PD target固定sim10s开始沿x-.5m/s，从x4.9到1.4（17s后保持）；
与解析目标误差按实际pose计算。MPC输入/测量门失败时只做MPPI+MPC shadow归因，
配对候选闭环不进入，不借回退MPPI将其判成MPC成功。

生成器修正登记：shadow01..04/calibration01/head_on01将XML ignition扩展前缀重写为
ns0；摩擦fdir1 expressed_in可能因此未被Gazebo解析，横移实测0。其运动结论不能
作为原main底盘见证，也不能作为有效B0；只保留调试失败。后续固定保留literal
ignition namespace并增加回归断言，再重新辨识/采集。源激光/v2契约记录依然真实，
但不借其宣称动态导航性能。

配对进入门明确化（修正namespace、calibration02与shadow05完整回放后，候选闭环前）：
calibration02四个轴向/组合脉冲稳定误差<1e-4、全程|wz|<=1e-8；shadow05
623/623公共预测按修正消费者接受、623/623源TF由canonical CDR回放可用；
目标前1s至少30条实际测量状态满足原模型门。此联合门允许开始独立候选实验。
完整MPPI影子轮接触/速度越界/非平面拒绝仍记false，不能消除或算物理接受；
它不应把另一个从无接触初始条件开始的MPC试验判为无法启动。运行中仍按原
source/observation/model/足迹/时间/solver全部门检查，任意退化立即原生制动+BT回退。
不改模型容差、足迹或名义D，不用当前捕获来选取proxy或缩小障碍。
固定最终两对：crossing B0(shadow06)/MPC(mpc01)，head_on B0(head_on02)/MPC(head_on_mpc01)。
每场景同SDF/config静态地图和goal phase=sim10s；最多各1轮，失败不寻找有利种子。
Contact topic按镜像内官方示例置于sensor/contact/topic并always_on；旧head_on01
Contact消息0为采集缺失，不判无碰撞。完整source快照在启动前生成。
