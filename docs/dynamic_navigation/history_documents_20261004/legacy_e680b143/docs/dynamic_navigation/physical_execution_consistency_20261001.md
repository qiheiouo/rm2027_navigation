# 原V1完整物理执行与单周期输出一致性

## 背景、假设和决策

既有v4与独立01的固定goal+2s周期原native filtered输出通过完整3s几何/原bounds/raw203/推进门；这只能证明同状态开环计划，不代表完整目标窗口的实际安全。CA独立验证证据不足，继续留在离线。此阶段只研究C，不修改Sampling、Ranking、线上代码或采集器。

H1：两个已有完整运行中，单周期计划通过时，完整8s物理轨迹仍可能违反body≥.05m或原padded>0门。H2：若同一周期未来3s物理轨迹也破门，应先报告实际轨迹与两种原native计划的偏离及后续控制更新；不能由偏离直接证明MPPI加权/SG错误。否定结果同样保留。

## 固定方法

- 固定数据为`new_device_near_x_capture_v4_20261001`和`ca_independent_01_capture_20261001`，均实际运行原V1；不新采集、不追加试次，也不作为失败CA两试次协议的替补。
- 每运行评价闭区间[goal_sent,window_end]和[goal_sent,原tail.final_sim_s]；另评价此前固定goal+2s周期的[cycle_sim_s,cycle_sim_s+3s]。不按接近/碰撞/排序选择时刻。插值只用于两个窗口端点及30个原计划比较时刻，源pose覆盖不足直接保留失败。
- 复用已审计`rows_from_transport`、convex polygon_distance、placed、radius、travel：实际Gazebo model/link组合pose，原profile八顶点body、原axis-sign padding .03m、原actor box .45×.55m；不改TF/footprint/安全判据。
- 主判定为实际动态body最小间距≥.05m且padded最小间距>0。分别报告采样最小值和既有线性插值相对运动下界；下界不过只称无法保证，采样值不过才称观察到破门；零polygon distance只证明投影接触/重叠，不推断物理碰撞力。
- 报告完整窗口全局goal(5.25,-1.10)距离减少、时间跨度/最大采样间隔、首次采样破门、全部间距CSV。固定周期另报告同一local path末点推进，以及30时刻实际robot对两种原native filtered计划的平移/角度偏差；偏差是诊断，不新增任意PASS阈值。
- 本实验只评价物理动态几何。单周期原raw203结果已冻结；不会把一张局部costmap外推为全窗口静态安全证明。原运行goal取消/recoveries与capture关闭FAIL保持。
- 只有观察到实际破门且其控制链相关时，才读取已有command/odom日志进行有限归因；不添加receipt/GID/TF/身份框架。信息不够则报告限制，停止工具扩展。

## 数据与可复现性

prepare记录当前源码commit、输入/参数SHA、原镜像81b325、pose源和原native结果hash，再评价。新分析在host Python3.12.3/NumPy1.26.4运行；原capture/native结果来自冻结Humble镜像，不冒称host代码在容器执行。单一决策脚本`physical_execution_consistency.py`；无recorder、运行配置或正式模块修改。结果/evidence不覆盖。

## 结果、限制、下一步

源码/prepare提交`3a28eeb`；输入SHA256 `08f6e8921264ef48915b6e8df4f27352d323280eb345d8cb3e0626c2fb634b74`，参数SHA256 `6bb215cc7b3904e83390cc07a60e1f6395b9a8d382396b9636795143f976a709`。v4固定22.102–30.102s、tail到33.496s、原cycle23的27.719–30.719s；01固定22.224–30.224s、tail到33.643s、原cycle13的24.405–27.405s。未按标签选时段。

待评价。若两个范围均通过，不证明一般闭环安全，只否定这两个运行的H1；转回明确尚未判定的算法层。若破门，用固定周期与完整窗口的时间关系限定原因，最多做已有数据支持的C归因，不直接上线CA或新安全监督器。

## 算法结果与执行归因

评价提交`981ed1d`。原8s窗口两条动态几何均PASS，H1在这两个目标窗口未被支持；固定尾段01观察到FAIL，不能把目标窗口PASS外推至完整取消/停止过程。

| 运行/范围 | body采样最小 / 线性下界 m | padded采样最小 / 线性下界 m | 全局目标推进 m | 判定 |
|---|---|---|---|---|
| v4 原8s | .072321 / .061825 | .038509 / .028013 | .448967 | PASS |
| v4 +固定tail | .053237 / .041434 | .020259 / .008214 | .472266 | 采样PASS，下界FAIL |
| 01 原8s | .193852 / .186823 | .151425 / .144397 | .355430 | PASS |
| 01 +固定tail | .029603 / .021331 | 0 / -.010009 | .967773 | 观察到FAIL |

v4的下界FAIL只说明现有采样不能保证≥.05m，不声称观察到破门，也不通过加密插值追PASS。01首次采样body破门32.694s：body .042195、padded .007256；之后padded降到0，只能断言二维投影接触/重叠。两pose流最大间隔.018/.019s，原tail及body/padded保持。

原固定周期的实际3s：v4 body/padded采样 .053237/.020259、body下界 .041434；01 .296045/.259548、下界 .285987，两者没有观察到破门。实际对两种原native filtered计划的最大平移差v4 .127193/.111756m，01 .249810/.252561m。01尾段破门不在cycle13的24.405–27.405s内，不能构成该周期聚合失败。后续重规划意味着这些偏差亦不单独证明模型或SG错误。

按已预登记条件只读现有observer CDR/control日志，固定Humble镜像用rclpy解码Twist，没有新记录器。v4取消后8个周期、最后30.202s，末次非零chassis命令30.307s；01取消后125个周期、最后33.147s，非零chassis命令直到33.275s。原日志30.231s记录`Failed to cancel action server for backup`。32.682/32.701/32.703s的cmd_vel_nav/cmd_vel/chassis值相同(-.394194,.235148,-.061851)，邻近首次破门。

因此保留**C执行停止过程FAIL**，责任范围是取消后仍执行控制的观测，不是已证明的MPPI加权/SG失效。观察者receipt不是消费者时间，无GID不能确定具体发布者；不继续扩展工具或修改取消程序。原navigation status5和capture关闭FAIL保持，不把尾段结果作为CA闭环验证。

[结果与全部CSV](evidence/physical_execution_consistency_20261001/manifest.json)、[主结果](evidence/physical_execution_consistency_20261001/summary.json)、[既有控制诊断](evidence/physical_execution_consistency_20261001/command_summary.json)。

![原V1动态间距与取消时刻](evidence/physical_execution_consistency_20261001/physical_dynamic_gap.png)

下一项回到三层算法：仅在已有两个完整capture按另行预登记的goal+6s固定时刻做未评价晚期周期诊断。它们是同运行相关输入、物理标签已读，不称独立验证，不补齐CA失败协议；先看Sampling，只有存在安全候选才解释Ranking/输出。禁止选择破门前的有利周期、重新采集或修改工具。
