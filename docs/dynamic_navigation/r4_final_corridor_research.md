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

正式结果待填。
