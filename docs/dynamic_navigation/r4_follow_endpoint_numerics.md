# A22：Follow 末端数值修复与有效时序复验

2026-10-06，Asia/Shanghai。Research，基线 `0374f4b3`，原 main 派生的 `experiment/r4-hws-prediction-consumption` 隔离分支。留本地、不push。

**结论：Go 保留 world-XY 入口的等价约束消冗余和原 OSQP 单次严格精修；Modify 保持 Research，下一步做与既有 STVL+MPPI 的有限效果对照。ROS/Nav2实际输出接线仍暂停。**

三类理想反馈全部到达，原S1/S2动态窗口覆盖恢复为40/40、160/160。没有缩小footprint/净空、扩张corridor、增加ADMM迭代、放宽容差或接纳inaccurate。原fixed/body和A19入口仍使用原168行及求解流程；四个fixed probe控制和stage差为0。

## 假设、baseline与最小判定

原值库在无动态/clear后x≈0.68m返回400迭代solved inaccurate，与wz无关。先定位同一QP，禁止通过新owner/fallback或angular transition隐藏失败。

使用同一observed fixture、canonical Sfc、45变量Follow源码、30ms source→epoch条件模型和50ms理想world ZOH。原15/40/75ms、400 ADMM迭代、eps_abs/eps_rel=1e−6及全部权重不变。baseline源码由Git `0374f4b3` 生成到ignored build，诊断入口只拦截原OSQP调用；不维护第二份手写QP/solver。trace矩阵复制不用于正式延迟判定。

首失败cycle42：P条件数113.26，primal残差1.5656e−6、dual4.3781e−7；rho=0.1、auto interval=20、updates=0。原末端X28/29/30同时活跃且rank仅2。原box包括不变reserve后X上界0.9627267116m；它不是新的终点或被放宽的corridor。

## 同一优化问题的最小改动

- 未来world XY在100ms ZOH节点之间是直线，同一个圆支持及静态凸rectangle保持不变；50ms中点XY约束是相邻节点约束的平均。只把独立XY节点行交给OSQP。
- 所有进度rate已≥0且输入s0∈[0,L]，terminal progress界蕴含中间界。保留原75个速度/率界、16×2个XY节点界、terminal progress界，共108行。
- 原168行仍完整装配，并在解后原限幅投影后逐项重验。P/q、bounds数值、50ms soft cost采样、terminal cost和静态reserve不变。这是等价可行域，不是删除安全边界。
- OSQP0.6.3默认仅对strict SOLVED调用已启用的polishing。World入口对iter==400且SOLVED_INACCURATE/MAX_ITER_REACHED、原预算尚有余量的结果，最多再调用一次**同一个OSQP的原polish**；随后update_info、check_termination(work,0)，只有原strict SOLVED才store_solution/提案。不碰已超时/infeasible/nonconvex，不重装QP、不改初值、不新增ADMM或新lease。精修越过原wall deadline仍丢弃。

此修改仅适用于当前直线ZOH、同一本地凸box、非负progress模型；未来负progress、时变静态box或曲线段需重新证明。精修使用固定0.6.3的内部API，是Research可维护性限制，升级依赖前须重审。

原失败cycle42的P/q/A/l/u和warm逐项差为0；中点行等价误差≤1.39e−17。精修解在原168行上的最大primal违反5.55e−17、dual stationarity残差3.89e−16。不是对新QP或更宽容差求解的因果对照。

## 修正时序后的诊断结果

| 条件 | clear | hold30 | hold120 | 决定 |
| --- | --- | --- | --- | --- |
| 原baseline | 42失败 | 81失败 | 171失败 | 共享末端数值问题重现 |
| 仅失败拍zero primal | 45失败 | — | — | 首拍成功不构成通用修复 |
| 仅rho interval=10 | 42失败 | — | — | 反证更新间隔过迟 |
| 仅rho更新比值门5→1.5 | 62拍到达 | 81失败 | 174失败 | 不采用rho救场 |
| 仅等价108行 | 42失败 | 81失败 | 171失败 | 单独不足 |
| 仅inaccurate触发原polish | 45失败 | 84失败 | 174失败 | 单独不足 |
| 等价108行+仅inaccurate polish | 43失败 | 102拍到达 | 191拍到达 | max-iter同样需要严格精修 |
| 等价108行+400迭代出口单次strict refine | 62拍到达 | 102拍到达 | 191拍到达 | 保留最小world数值修改 |

最后诊断355拍中348拍原strict solved，6拍原inaccurate、1拍原max-iter，经一次polish后均满足原strict条件。另有rho_at42单拍对照，只证明原更新比值门造成首拍差异，不用于最终方案。

## 无trace值库的理想反馈结果

| 指标 | clear | hold30（1.5s持障） | hold120（6s持障） |
| --- | --- | --- | --- |
| 原5cm XY标准到达 / 时长 | 是 / 3.10s | 是 / 5.10s | 是 / 9.55s |
| 有效提案 | 62/62 | 102/102 | 191/191 |
| 到达后XY误差 | 0.048061m | 0.041726m | 0.049563m |
| 持障WAIT（speed<0.02） | 不适用 | 1/30 | 87/120 |
| 持障oracle最小机械净空 | 不适用 | 0.206535m | 0.206535m |
| 持障XY方向反转（±0.02死区） | 不适用 | 1/0 | 1/0 |
| clear后三拍持续forward>0.05的首拍 | 不适用 | 50ms | 50ms |
| solver最高 | 1.838ms | 1.767ms | 1.747ms |
| acquire→result最高 | 2.174ms | 2.114ms | 2.088ms |

全部原epoch步长严格50ms。长持障末20拍speed最高1.331e−5m/s，先退后稳，无持续反转。原有前8拍future soft支持重叠保留，没有未来整段hard veto；虚拟plant当下机械oracle重叠0。复验原A21无动态消费消融在同一有障oracle的前30拍有12个重叠样本、最小−0.195384m，表明消费在这个fixture改变了响应；这是消融而非MPPI baseline。

5cm是离线XY终止准则，执行最后一拍后检查，不宣称Nav2 GoalChecker/角目标已完成或末端速度已为零。plant是理想world速度跟踪，源yaw .6rad/s独立转动；无Gazebo/实际R4命令/物理跟踪误差认证。

## 原S1/S2有限回放

原453拍native导航source，同一public/private预测和path；分别删除副本tracks作消融，native MPPI仍是原实际controller。

| 指标（observed / 无动态消费） | S1 | S2 |
| --- | --- | --- |
| 原导航窗口coverage | 225/230 / 225/230 | 218/223 / 218/223 |
| 原动态窗口coverage | 40/40 / 40/40 | 160/160 / 160/160 |
| 当前净空正时预测引起forward下降≥0.05 | 0拍（max0.0452） | 36拍，首拍goal+2.644s |
| 动态窗口WAIT / forward<0.05 | 0/0 / 0/0 | 0/11 / 0/7 |
| clear后首个连续三拍forward>0.05 | 0.374s / 0.374s | 0.044s / 0.044s |
| 动态窗口vx/vy/forward反转 | 0/2/0 / 0/2/0 | 4/3/2 / 2/5/4 |
| observed solver中位/p95/max | 0.706/0.876/0.929ms | 0.706/0.905/1.446ms |
| observed acquire→result最高 | 4.425ms | 2.591ms |

每场前5拍缺原输出receipt代理，仍NA，不造零命令/WAIT；全部443个可构成原输入的observed拍严格解出。原S2 cycle144现在230迭代strict solved，future观测支持最小净空0.059207m、plateau0，没有采用A21危险冷线性化重试。

S2提前响应计数由38变36，首时刻不变。S1 observed和全部无动态消费命令最大差≤2.05e−6m/s；S2 observed从原失败后的145拍起改变（max0.09435m/s），因为现在继续保留warm历史，而baseline失败清warm。原观测支持当前/未来重叠均0。coverage改善不等于动态指标全部提高；native-source counterfactual反转也不是实际R4振荡。

63个yaw/wz等价probe全有效，最大配对stage差6.52e−15，world积分误差≤2.78e−16；原4个fixed probe差0。没有取消原raw源yaw/wz、source stamp或frame边界。

## 实验时序错误与证据范围

第一次值复测发现旧A21 probe超过固定循环上限。`source+i*50000000` 的乘法两端都是int，第43拍起signed overflow造成UB，Release可消除终止条件。A21/A22旧after42理想反馈/长hold-clear不作为有效时序证据；首次“62/102/191拍到达”先撤回。本次仅保留修成 `source+int64_t{i}*50000000` 后的完整复验值，重新重现原失败及最后修复；原S1/S2真实int64时间戳不受此错影响。

失控容器已停止，文件保留ignored build，未拿无限稳态样本计入统计。历史A18/A19若采用相同表达式，长循环数据亦须修正后复验，不能直接推广。报告生成在实验脚本运行期间编辑后发生shell读偏移错误；容器实验已完成，分析单独重跑成功，没有重跑场景或修改结果。

必要证据及复现见[实验入口](../../experiments/r4_follow_numerics/README.md)。本轮没有新ROS/Nav2节点、tracker/producer/frontend、第二MPPI、watchdog/lease、angular transition或最终输出owner；main、public v2、A09–A12及原无关dirty不改。

下一步只增加能判断消费价值的信息：与既有STVL+MPPI在相同场景/反馈条件下做有限效果对照，评估目标完成、净空、响应和振荡。未建立同条件闭环优势，暂不推进运行输出接线或把harness升级为导航链。
