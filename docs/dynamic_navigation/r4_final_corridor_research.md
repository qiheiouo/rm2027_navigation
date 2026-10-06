# A26 最后一轮 Research：短时阻塞的单通道

2026-10-06，基于`89f9035b`，保持main不变、不push。用户明确授权最后一轮新的核心假设；A25的Stop结论保留。这轮不是继续调优，而是判断冻结A24 R4的时间相关消费，能否在不可绕行、即将开放的通道中，产生STVL+native MPPI难以简单替代的WAIT/GO收益。没有新solver/cost/prediction/free-s或输出架构。

## 核心假设与公平场景

冻结预测是观测速度CV外推，无法获知静止障碍未来突然启动。因此采用**已在匀速移动的障碍**穿过通道，开放时间可由现有观测预测；环境运动脚本的未来安排不传给controller或tracker，机械真值只用于观察。

初始通道沿x，起点(0,0,0)、goal(4,0,0)，两侧壁内边界y=±.70m，宽1.40m，端点封闭。x=2m设1m宽的垂直通道，y=±2m封闭，形成两个盲支路，没有连接起点/目标的绕行路径。原动态障碍0.45×0.55m沿y运动；原机器人yaw-invariant圆支持半径.43869m、native同一32边形支持不变。初始有效几何被冻结前端拒绝，正式场景在下文明确修订，原记录保留。

S3运动：goal接受后t<1s障碍在y=−1.40m，之后以.30m/s匀速行进至+1.40m（t≈10.333s），不停车。地图与Gazebo物理墙由同一几何生成；环境扩展旧障碍slider范围到±1.6m，不修改感知/预测模型或PID。墙和动态障碍均有contact观察，机械真值不进入控制器。

两组地图、起终点、footprint、速度限制、传感器/预测输入及真值定义相同。R4全部A24参数及算法/已安装binary不变，内部cruise .4、free-s .5、1.5s horizon保持。baseline采用A25已验证的local inflation .60m/factor6，只将vx_max从.32对齐到R4原有.50；其余native设置不动，不按本轮结果调参。两组vy_max=.5、vx_min=−.3、wz_max=1.2，原smoother/unique chassis stub输出链共用。native内部参数与R4固定cruise不等同，空通道实际时间作为描述性参照，不做速度校准。

正式trial之前各跑1次无动态障碍的同一通道S0，验证两组能够通过。原规则只允许修复fixture错误；本轮出现冻结前端对有效窄通道的拒绝后，在任何动态trial之前显式修订一次场景可行性规则，见下文。有效空通道检查之后才开始正式比较；正式结果之后不改几何、算法或参数。

## 最小样本、指标与判据

- 主场景各5次，新实例、101–105交替先后，不用preflight数据充当正式样本。首次contact/controller failure保留原输入并停止自动批次；分类后完成已计划剩余样本，不替换任务失败。启动失败与任务失败区分。
- 沿用成功/contact、到达、sampled动态最小机械净空、WAIT<.02m/s、最长停滞、world-forward反向切换及最大物理回退、solver/call latency。新增纯观察：过gate时间（完整圆支持中心x>2+.225+.43869）、相对实际clear的通过/恢复、静态墙机械净空、实走路径/横向及盲支路动作、plan发布数量。plan发布是正常BT刷新，不能全部称为无效重规划。
- 效率Go的必要条件：R4全部成功/零contact、成功率/contact不劣，动态净空中位和最坏均不比baseline低>.05m；到达中位快≥10%、至少4/5配对同向（n10为8/10）；现有输入确实预测开放且动作有时间相关收益，不能只比较不同保守度。
- 安全Go的必要条件：R4全部成功/零contact，baseline出现至少2/5（n10为4/10）重复任务失败/contact，而R4成功时间无>10%效率损失；确认fixture公平及预测相关证据，不把single偶然失败或感知/solver接线错误称为预测优势。
- 仅在结果接近（到达中位差<10%）或配对/安全信号不一致且增加样本能改变Go/Stop时，原参数补至各10次。若5次明显baseline不劣，则直接Stop，不用额外trial拖延结论。
- 如主场景出现候选Go，再以**预选已有A24 baseline .50m/factor6**、相同速度限制做5次reactive challenge，无搜索。若这个简单既有调整能消除优势，不判Go。需要验证候选Go对阻塞时长的稳健性时，最多附S4（同一几何，速度.45m/s）各5次，其他不变；否则不运行变体。

没有明确、可重复、简单reactive设置难以替代的独立安全或效率收益，则**Stop并冻结当前R4/HWS-style low-level prediction-consumption研究路线**。保留关键结论与复现，不继续owner/lease/fallback/ROS生产化；研究重点回到主线STVL+MPPI及其他有价值的问题。Go只表示值得讨论Integration，不自动实施。

## 复现与结果

入口[实验说明](../../experiments/r4_gazebo_comparison/README.md)、[场景与有限host编排](../../experiments/r4_gazebo_comparison/corridor.py)。原A23/A24 controller二进制复用，只扩展实验场景及观察器。证据输出`build/r4_corridor_comparison_20261006`；原始新数据上限约100MiB，无bag/core，不清理既有失败或无关改动。使用现有离线Docker，无网络或硬件。

协议及实现先本地提交，再做preflight和正式trial。初始协议提交`02a2b6f0`。

### 动态trial之前的一次场景修订

初始S0 baseline 9.382s到达、零contact；R4在goal+49ms被`Sfc rectangle lacks raw-static support`拒绝，尚未调用solver。地图与物理墙一致。只读调用原未改T-DT `SfcSquare::getBound`复现：起点矩形y边界约±.725m，包含y=±.725m处的占用格；原wrapper的严格校验正确拒绝它。这是冻结静态前端/适配对窄通道的限制，不能称为prediction失败，也不通过放松校验救结果。

为让最后一轮真正进入时间相关对照，在任何动态trial前显式修订为**宽2.10m的单通道**（内边界y=±1.05）、动态机器人0.45×1.00×0.80m，其他运动/速度/控制/预测不变。这是一次场景设计修订，超出原只准fixture错误修复的自设规则；不把旧场景错误化或隐藏失败，也不根据正式性能挑场景。直通道中心两侧空隙各.55m，小于ego机械横宽.60m和圆直径.877m，动态机器人居中时仍实际堵塞；盲支路不存在绕行出口。按固定圆支持估算中心附近不可通过的区间约2.18s，随后开放；障碍完整离开中心线支持带名义时刻约8.796s，prediction horizon仍1.5s。

只读起点probe在修订地图返回的矩形内没有占用格。两组必须重新各做一次S0；不继续按算法失败改场景。`fixture_amendment.json`、原`fixture_initial/`及两次原preflight完整保存。R3 oracle对旧障碍extents写死，因此只在观察器用原world pose/rotation和原hull函数按新SDF尺寸重新投影；ego机械包络、时间/坐标/距离定义不变，两组使用同一真值适配，不修改R3源码或tracker。每个trial保存其实际scene/map/scenario，避免修订后重分析读错原输入。

### 已完成的冻结运行行为样本

场景修订提交`8d8aadc5`后，重新S0：baseline 8.733s、R4 12.582s，均到达且零contact。限速相同，空通道实际到达时间不同；R4原cruise/算法保持，不用空场结果重新校准。正式S3两组各5次，101–105交替先后。`S3_B0_102`在发goal前地图lifecycle响应超时，单列启动失败，保留并以同参数`S3_B0_202`替换；其余任务失败均未替换。首次出现的contact、native异常先保存和分类，再完成已登记的剩余样本。

| 指标 | STVL + native MPPI | 冻结A24 R4 |
|---|---:|---:|
| 目标启动后的成功率 | 0/5 | 0/5 |
| 真实动态障碍contact | 3/5 | 0/5 |
| native MPPI异常导致终止 | 2/5 | 5/5 |
| sampled动态最小机械净空，中位/最坏 | 0 / 0m | .33799 / .31671m |
| 终止前WAIT中位 | .08s | .32s |
| 终止前最长停滞中位 | .08s | .24s |
| 最大横向偏离，中位/最大 | .51810 / .54298m | .00938 / .01330m |
| 前后向切换 | 0 | 0 |
| 最大物理回退 | 0m | .000935m |
| 到达、完整gate通过、clear后恢复 | 未观测 | 未观测 |

各对结果：101为baseline contact/R4 native异常；102为replacement baseline contact/R4 native异常；103、104均native异常；105为baseline contact/R4 native异常。baseline contact为动态机器人与ego轮端接触，静态墙未contact。所有正式任务在goal后4.355–4.837s终止，早于名义8.796s完整中心线开放时刻。失败经过时间不是到达时间；终止前WAIT不是完整等待时长；未观测项保持null，不以0补齐。

baseline的横向动作是**侧向绕行尝试**，不能称为已经进入盲支路：中心|y|最大.543m仍小于主通道半宽1.05m。两组每次均收到5次plan发布，规划路径没有进入侧支路；这是正常BT刷新数量，不等于无效重规划次数。R4接近x≈1m时减速/WAIT，侧向偏离小、无明显前后振荡。有效R4 XY消费475拍，solver合并P95 1.196ms、最大1.921ms；native+R4合并有效调用P95 32.902ms、最大35.364ms。5个native失败拍尚未调用XY solver，计时字段0代表缺测，已从有效latency汇总排除，不能宣称异常调用耗时为0。

参数检查确认R4 YAML与A24逐字相同，两组正式配置除local inflation .60/.50外相同，限速一致，最终Gazebo输出仍是原唯一chassis stub。实际配对障碍轨迹在**共同存活的观测区间内**位置差最大.411mm、速度差.002232m/s，起点位置差0；不能据此声称已核查未运行的后半段开放轨迹。收到gate附近非零CV预测的次数为baseline 33–37、R4 39–40；这些观测支持移动障碍已进入prediction，不证明其未来完整无遮挡形状或时间相关收益因果。

退出清理时沿用的Nav2 cleanup −11仍出现：所有15个实例中13次，正式goal-started样本中8/10次，均在试次已记录终止后出现，日志与`corridor_audit.json`单列保留。未把它替换成任务内contact/native异常，也不据此宣称运行链生产稳定；本轮不开展无关cleanup工程修复。

### 方法限制与当前判决

原A23实验wrapper在任意异常后把`failed_`永久锁存；记录器收到首次`/research/failure`就终止试次。native MPPI先计算原angular/command，再调用R4 world XY消费，因此native的`Optimizer fail to compute path`会先于R4求解发生。共同配置原本允许`failure_tolerance=.3`及Nav2原BT恢复，但此次记录器结束和锁存使其无法运行完整重试/恢复。它不是底盘wz非零eligibility问题，不应恢复angular handoff，也不是R4 solver失败。

对**冻结的首异常终止运行行为**按原全部成功必要条件计算是Stop；它没有形成可用完整WAIT/GO执行。但针对本轮用户要求的**时间—拓扑价值判定**，当前判为**Modify：实验在开放前被首异常终止，核心比较未完成**。R4零contact是部分安全信号，不能冒充成功避让或完整效率优势；baseline的3次真实contact仍作为失败保留，不以方法限制删除。A25已给出的Stop当前配置生产化结论继续有效，当前没有Go或进入Integration的依据。

不补到10、不运行S4或reactive challenge，不修改R4参数/算法。唯一可能改变本轮判决的信息，是保留首次native异常后让**现有Nav2原恢复行为**运行至goal/contact/timeout，再完成相同场景各5次新的独立样本；不能用更多相同提前终止样本解决问题。

### 未应用的最小实验修正，等待用户确认

已准备[可审阅patch](../../experiments/r4_gazebo_comparison/native_recovery_proposal.patch)，仅限本实验：默认保持旧行为；显式实验开关下，仅取消已知native MPPI异常的永久锁存，原异常仍上抛给原controller_server处理，首次异常立即留证，记录器等待既有Nav2 action结果。contact和R4/input/solver异常仍终止。不合成速度、不新增fallback/owner/publisher/lease；速度、场景、R4配置及同一算法不变。

patch未应用、未编译、未运行。已验证patch可应用及Python拟议文本语法，不能称运行验证通过。需要用户确认，是因为其“保持冻结配置，不修改输出架构”约束下，此举虽然复用原恢复机制，仍改变实验异常传播/终止行为。等待确认期间只整理已获授权的证据；没有自动批准或启动新样本。如不允许，则停止本轮并冻结当前路线，同时保留“完整WAIT/GO未被本实验检验”的限制。

### 证据与复现

[紧凑证据](../../experiments/r4_gazebo_comparison/evidence_corridor/)保存协议、两版场景、配置、固定schedule、启动替换表、失败分类、全部summary/trials、公平性核查、图和未改前端probe。完整输入/预测/真值/日志约35MiB继续保留于`build/r4_corridor_comparison_20261006`；无新bag/core，磁盘剩余约2.2GiB，不删除无关dirty或已有大core。原main保持`2849cbe4`，全部留本地，不push。

原运行和观察器的复现入口见实验README。当前分析仅重新读取已保存证据：

```bash
python3 experiments/r4_gazebo_comparison/corridor_analyze.py build/r4_corridor_comparison_20261006
python3 experiments/r4_gazebo_comparison/corridor_plot.py build/r4_corridor_comparison_20261006
```

重现已保留失败时，不覆盖原目录、不替换任务失败。原始运行行为和未来恢复行为样本如获批，必须用不同输出目录并分开报告。
