# A26 最后一轮 Research：短时阻塞的单通道

2026-10-06，基于`89f9035b`，保持main不变、不push。用户明确授权最后一轮新的核心假设；A25的Stop结论保留。这轮不是继续调优，而是判断冻结A24 R4的时间相关消费，能否在不可绕行、即将开放的通道中，产生STVL+native MPPI难以简单替代的WAIT/GO收益。没有新solver/cost/prediction/free-s或输出架构。

## 核心假设与公平场景

冻结预测是观测速度CV外推，无法获知静止障碍未来突然启动。因此采用**已在匀速移动的障碍**穿过通道，开放时间可由现有观测预测；环境运动脚本的未来安排不传给controller或tracker，机械真值只用于观察。

主要通道沿x，起点(0,0,0)、goal(4,0,0)，两侧壁内边界y=±.70m，宽1.40m，端点封闭。x=2m设1m宽的垂直通道，y=±2m封闭，形成两个盲支路，没有连接起点/目标的绕行路径。原动态障碍0.45×0.55m沿y运动；原机器人yaw-invariant圆支持半径.43869m、native同一32边形支持不变。机器人通过主通道有约.26m单侧中心余量；障碍位于中心时，剩余左右配置空间不能连通，不是用贴近footprint的墙强迫baseline失败。

S3主场景：goal接受后t<1s障碍在y=−1.40m，之后以.30m/s匀速行进至+1.40m（t≈10.333s），不停车。中心对齐时主通道暂时堵塞，随后可重新通过；运动中距中心线支持带完全clear的名义时刻约8.046s。地图与Gazebo物理墙由同一几何生成；环境扩展旧障碍slider范围到±1.6m，不修改感知/预测模型或PID。墙和动态障碍均有contact观察，沿用独立机械真值。

两组地图、起终点、footprint、速度限制、传感器/预测输入及真值定义相同。R4全部A24参数及算法/已安装binary不变，内部cruise .4、free-s .5、1.5s horizon保持。baseline采用A25已验证的local inflation .60m/factor6，只将vx_max从.32对齐到R4原有.50；其余native设置不动，不按本轮结果调参。两组vy_max=.5、vx_min=−.3、wz_max=1.2，原smoother/unique chassis stub输出链共用。native内部参数与R4固定cruise不等同，空通道实际时间作为描述性参照，不做速度校准。

正式trial之前各跑1次无动态障碍的同一通道S0，验证两组能够通过。若fixture错误导致失败，先保存证据、分类，只允许修复真实地图/物理或记录器错误；不根据算法表现改变算法、cost、速度、阻塞时间或通道使某组得利。有效空通道检查之后才开始正式比较。

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

协议及实现先本地提交，再做preflight和正式trial。结果待填。
