# 显式动态预测：一个月退出计划

登记日期：2026-10-03，Asia/Shanghai。

> **2026-10-04 用户暂停更新：本方案暂时停止，计划中的新实验未启动。** 下列窗口、预算和阶段安排保留为原计划记录，不构成自动继续工作的指令。恢复需用户明确要求，再核对任务范围和时间安排。三轮方案及全部文档入口见[停止记录与文档总目录](all_dynamic_avoidance_documents_20261004.md)。

## 1. 问题、当前结论与硬截止

只回答：**相比 STVL + MPPI 基线，显式动态障碍预测能否在 RoboMaster 代表性工况下提供稳定、值得复杂度代价的净收益？** 不以“必须实现预测避障”为目标。

**当前部署决策：C — RESEARCH FROZEN。** 正式导航继续 STVL + MPPI，无显式未来位置预测。历史 V1 存在夹具净空改善，但恢复/绕行未稳定满足原门；最新 visible/nominal 两试次综合失败，且没有同条件 STVL 对照。当前材料不足以选 A，也不足以定义已验证的 B 条件触发场景。

有限评估的日历窗口为 **2026-10-03 至 2026-11-03 18:00（Asia/Shanghai，UTC+08:00）**。最终文档必须在该时刻前落地并选择A/B/C；证据不足、配对条件未建立或被更高优先级问题挤占，默认 **C**。暂停不延长截止，不追加“下个月再研究”作为本任务结论。

本次任务仅交付[迁移审计](hws_migration_audit_20261003.md)、[逐模块对照](hws_vs_current_prediction_analysis.md)和本计划，**未启动计划中的新实验**。原持续开发目标停止扩展；已有未提交成员证据开发保留，不自动继续surface接口、critic C或新分支链。

## 2. 冻结范围与候选数量

Baseline **A：STVL + MPPI，不使用显式未来位置预测**。版本/配置不能用“main默认”四个字代替：实际full-entry默认是高风险spin测试profile；最新native试次又是ObstacleLayer。研究先在隔离环境选定并记录一个STVL基线，不能启动实车full-entry来凑基线。

只保留 **一个活跃研究候选 R1**：既有原生 `DynamicObstacleCritic`，CV、source-age补偿、时间评分，短有效预测段0.5s或1.0s，配套停止保护作为明确整体。普通STVL/current CostCritic一直保留。模型/geometry/保护语义在pilot后一次锁定，不继续边跑边调。

候选上限2，但本计划**不启动第二候选**：

- HWS temporal renderer/FDDP不在本地运行链，重建不属于最小复用。
- 本地没有已接线的predicted occupancy layer；本月不新写一套。
- 旧 `PredictionV1Critic` 只作为历史机制/净空收益参考，不重开CA、概率overlap、proposal或sampler搜索。若R1失败，优先选C，不自动转新候选。

允许的有限消融仅为[对照矩阵A–G](hws_vs_current_prediction_analysis.md#9-最小差异实验矩阵计划未执行)：current-only、0.5/1s、单一decay开关、既有visible/nominal代理。observed raster/new surface若需新公共契约则标NOT RUN并退出该arm，不追加研发。**不扩展CV/CA/IMM、复杂概率、学习预测、controller替换、多目标新关联算法或新全尺寸感知。**

## 3. 同条件合同与进入实验门

在第一条有效run前，提交一份不可变fixture清单，记录以下共同对象的路径、SHA与实际参数readback：

| 对象 | 必须固定的内容 |
|---|---|
| robot geometry | Gazebo机械/link/附件包络、Nav2 footprint/padding、guard footprint；oracle检查完整机械几何。各arm相同，不能用参考八边形替代机械oracle |
| map | 同源地图、分辨率、origin、static/current STVL/inflation参数、global planner/BT；相同地图hash |
| controller | 同一Nav2/MPPI ELF/SDK版本、dt/steps/batch/iteration/temperature/gamma、原标准critic、SG和smoother参数 |
| limits | 同vmax/amax/angular limits、response/stop约束、输出bounds；不通过降低其他arm能力制造优势 |
| actor | 同物理shape、轨迹函数、起始相位、速度/加速度、遮挡时序；truth只进oracle，不给tracker/critic纠偏 |
| task | 同initial pose、goal/action、场景墙体、开始时刻规则、固定超时与尾段；失败不移目标 |
| observation/TF | 同sensor、发布率/噪声、静态剔除输入、canonical TF owner；相同错误注入规则 |
| randomness | 预登记seed集合；验证实际采样noise身份。`regenerate_noises=false`不等于跨进程配对seed，缺证据不得声称strict seed对照 |
| protection | 各arm同普通current-map保护和执行约束；预测附加拒绝纳入R1功能成本，不偷偷移除guard救完成率 |
| oracle | 同独立physical collision/连续净空、raw203、padded正间隙、原bounds与执行证据；与critic实现保持独立 |

不得为结果通过而放松已有门：冻结fixture原要求的机械body净空≥0.05m、padded严格正、raw203/unknown边界、输出bounds及原接受要求继续适用。基线的性能失败照实计入，不挑掉碰撞/timeout baseline；只有配置不一致、损坏/缺失证据或oracle无法判定等协议失败才标INVALID。

进入门只有：共同基线构建/启动、实际配置与topic图、source/rollout epoch、native真实noise/版本、oracle可用。不是重新建设大型证据系统。若在第一周预算内不能形成有效共同基线/可比保护与seed输入，直接C，保留缺失项；不得扩大研究以修建无限fixture。

## 4. 九个代表场景

所有位置/goal/轨迹数值在fixture中一次锁定。以下是必需覆盖，不用一个往复箱体代表所有场景。速度是预登记的仿真试验值，不冒充赛场上界。

| ID | 场景 | 最小设置与验证问题 |
|---|---|---|
| S1 | 静止障碍 | 障碍v=0，存在几何可行绕行；确认阈值与current-only是否足够，预测有无伪运动 |
| S2 | 低速横穿 | v=0.3m/s，直线跨路径；能否合理等待而不绕远/冻结 |
| S3 | 高速横穿 | v=1.5m/s，直线跨路径；预测是否给出可重复的更早避让收益 |
| S4 | 迎面接近 | v=1.0m/s，双方接近后有明确侧向空间；安全停车/避让与任务恢复 |
| S5 | 突然停止 | v=0.8m/s，固定source时刻减速至0，有限加速度一致 |
| S6 | 突然反向 | +0.8→−0.8m/s，固定过渡时序/加速度；旧CV ghost future是否阻塞 |
| S7 | 短暂遮挡 | 相同真值轨迹，固定0.2s与0.5s两个遮挡段；coast/consumer age拒绝及恢复 |
| S8 | 窄通道附近动态目标 | 固定通道宽度/障碍位置，保留base可行空间；几何margin与预测保守代价 |
| S9 | 基线等待后通过 | actor先横跨/占用通道后离开；baseline安全等待并最终完成作为false_block见证 |

S5/S6的stop/reverse不是瞬间赋无限加速度的传送，实际Gazebo真值轨迹必须验证。S7不能仅删除oracle记录模拟遮挡；只按同一输入协议遮挡感知，oracle真值保留。S8/S9至少保留一个baseline安全成功的配对样本，否则不能验证该场景的false_block命题，也不能另挑有利相位直到出现正例。

## 5. 核心指标与 false_block 的可执行定义

| 指标 | 定义/分母 |
|---|---|
| physical_collision | 实际机械几何contact或独立oracle重叠；两种原始结果并报，无法判定标UNKNOWN，不记无碰撞 |
| minimum_physical_clearance | 完整机械对actor与static连续净空下界；另列padded/代理间隙，不能替代physical |
| task_success | action成功且终点/原安全接受门通过；成功action但碰撞为失败 |
| completion_time | 从同一goal开始规则到完成；timeout保留为失败/右删失，不从平均中剔除 |
| stop_wait_time | 距goal>0.3m且实际平移速度≤0.03m/s的累计时间；停止是否必要另作归因 |
| unnecessary_detour | 实际path length相对安全baseline与冻结无动态reference的增量；仅有安全baseline时作因果配对 |
| recovery_count | 原BT/行动的恢复事件，按同action UUID计数；遵守fixture原接受门 |
| optimizer_failures | 原生fail/retry事件次数，按input/TF/grid/CostCritic/all-collision等真实原因分组 |
| no_valid_trajectory_rate | 无满足同oracle条件的候选周期/可完整评估周期；另列原生fail率。采样mask数和输出拒绝率不混用 |
| stale_prediction_usage | 被实际consumer使用的source/observation age超过其声明有效期的次数/消费次数；missing/拒绝单列 |
| CPU / deadline | 同硬件、同采集开关下进程CPU、controller耗时P50/P95/P99/max、miss比例；证据IO另列 |

### false_block

**定义**：STVL baseline有独立证据证明存在可安全执行轨迹，或通过安全等待最终完成同一任务；预测方案因未来占据判定持续无解/拒绝通过。

记录两个层次，避免把猜测混入主指标：

1. `false_block_confirmed`：baseline安全见证成立，candidate仍有同条件的当前几何可行路径/等待策略，真实诊断/离线同周期评分证实future约束是拒绝原因。episode为这种拒绝连续≥1.0s；事件结束于接受安全提案或原因改变。只累计满足证据区间的秒数。
2. `false_block_suspected`：baseline安全完成，但candidate轨迹已偏离、拒绝原因缺失或不能证明当前可执行；单列，不充当confirmed秒数。

“raw中有一条safe”本身不是可执行见证：需要同epoch、controller约束/SG后控制、current occupancy、执行保护可接受等证据。baseline已完成不能自动证明candidate当下任意位置有可行命令。安全停车、真正碰撞威胁、stale/TF拒绝分别记 `necessary_wait` / `input_degradation_block`，后者属于可靠性代价，不偷换为future false_block。

每个事件输出：scenario/seed/action UUID、start/end/source/rollout epoch、baseline安全见证引用、candidate current/future判定、输入age、geometry模式、原生fail/guard原因、成功或timeout结果。缺失cause时保持UNKNOWN。

## 6. 固定预算和阶段安排

预算上限：**120条有效/替补闭环run总计、24小时人工/agent主动研究工时、32小时实验机器占用**，任一到限立即进入结论阶段。证据额外存储上限2GiB，最多2个失败窗口保存完整raw/SG/成本trace，其余保存受限指标、实际配置、轨迹/事件和身份。已有冻结归档只读、不删除；空间不足时停止新增采集并据已有证据选C，不以清历史数据换研究时长。

| 日历阶段 | 截止/预算 | 输出与停止门 |
|---|---|---|
| Week1：共同基线与pilot | 10-10 18:00；最多8小时主动工时、21 runs | 单一fixture、seed/noise/保护/TF/oracle有效；A–G只在一个预登记场景做≤3seed消融。未实现arm记NOT RUN；无共同基线→C |
| Week2：锁定一个R1 | 10-17 18:00；最多6小时、累计≤60 runs | 按预登记顺序选0.5s，只有无收益且无安全失败才看1s；禁止weight/尺寸/安全门搜索。锁定模型、geometry、保护；失败停止该候选 |
| Week3：代表工况对照 | 10-24 18:00；最多6小时、累计≤111 runs | 九场景×A/R1×5seed=90条主对照；其中预先锁2seed为holdout，pilot不用于最终成功率。两周合计进行，不重复算预算 |
| Week4：汇总决策 | 11-03 18:00；最多4小时、最多9条INVALID替补，总≤120 | 只补协议无效run，性能失败不重跑求好结果；结果表、false_block事件和A/B/C结论提交。不得新模型/场景/机制 |

pilot的21 runs是上限；F新surface不可执行就不补新开发或其他scene凑数量。主对照5seed预登记为 `[0,11,23,47,101]`，holdout `[47,101]`；具体native采样身份必须进入fixture，不支持seed API时不能只把这五个数写入报告冒充确定性。时间/存储预算包括前置接线和失败排查，不把这些称“基础设施”后排除预算。

所有run最多90s仿真任务时长+固定3.5s尾段；goal失败/取消/碰撞全部保留。没有新实车、串口接入或无人值守运动安排。未来实现/实验在单一 `experiment/*` 分支、固定小commit节点进行，不逐证据点新建无上限分支；TF/topic/安全变化须同步合同文档。

## 7. 预登记淘汰和三选一规则

原安全门优先，统计收益不能抵销碰撞。发生候选physical collision、原安全接受失败或实际使用stale预测，**停止该候选**，保留证据，不调松门重启。INVALID协议故障可在替补预算内修复并仅重跑对应协议无效run；涉及主线可靠性时按第8节立即暂停。

在同一未放松的fixture接受条件下：

| 结论 | 截止前必须满足的证据 |
|---|---|
| **A — DEPLOY CANDIDATE** | 九场景数据齐、所有候选有效run安全接受；至少5/9场景在净空或任务表现上有稳定改善（≥4/5seed方向一致，并含holdout）；baseline安全成功的场景candidate无新增task failure；false_block无新增≥1s确认episode；无stale usage；资源门通过；复杂度仅一候选且撤回清楚 |
| **B — CONDITIONAL FEATURE** | 达不到A，但预登记的S3/S4等一个或两个明确类别有≥4/5seed且holdout复现收益；该触发类别全部安全接受、无新增false_block/task failure；有可测输入触发条件与失效回退边界。其他场景继续baseline，不能把oracle/truth当触发器 |
| **C — RESEARCH FROZEN** | 任何核心安全失败；无上述稳定收益；保守阻塞/失败/绕行或CPU显著增加；公平对照/holdout/退化边界未建立；预算/截止先到；或更高优先级主线问题阻断 |

“收益”至少满足一个预登记的实用门，并报告代价：baseline净空不达原门而candidate达到；或同安全条件下完成时间减少≥10%且≥1s；或stop/wait减少≥20%且≥1s；或任务失败减少。baseline安全成功的其他场景不能以新增失败补偿收益；median时间/行程/等待不得有>10%的系统性退化。若样本少导致不能稳定判断，选择C，而非以“看起来更安全”通过。

资源门：10Hz基线周期100ms下新增prediction路径CPU median≤一个core的10个百分点、controller P95新增≤10ms且<100ms，deadline miss不高于baseline+1个百分点，无新增感知/控制阻塞。以上是本任务复杂度判据，不是更改正式controller时限。完整原本资源门更严格时沿用原门。

A仅表示隔离验证中的部署候选，**不自动合main/上实车**；B仅表示明确条件的增强候选，也需要单独部署评审；C保留版本代码、文档、复现数据，移出活跃开发。三个结论都必须有具体证据/失败/资源表，不使用“还需要继续研究”逃避判断。

## 8. 主线优先和立即暂停条件

若出现定位、重定位、canonical TF、串口、底盘控制稳定性、基础MPPI或其他正式导航可靠性高优先级问题，**立即暂停动态预测研究，优先处理主线问题**。包括：source-time定位/TF不可信、串口/控制超时、底盘颤抖、基础controller不稳定、同地图基线反复异常。不得为预测实验绕过这些问题。

暂停记录：issue/evidence、发生时间、剩余预算、研究branch/HEAD、未完成run、停止进程和保留档案；不开展依赖该故障输入的新预测调参。截止保持11-03 18:00；无法在余下预算完成有效对照时选C。用户明确结束/要求结论时立即依据已有证据选A/B/C，不继续等满一月。

## 9. 最终交付与本次收尾

最终应在同一研究目录提交 `prediction_exit_decision_20261103.md`（若提前结束则记录实际日期），包含：唯一A/B/C、共同fixture身份、全部run含失败/INVALID、九scene指标、confirmed/suspected false_block事件、holdout、资源/复杂度、复现入口、正式系统保持baseline的方式。目录命名不是自动调度任务；本次未创建自动续跑或提醒。

本次交付三份文档及引用/历史清单；正式main、冻结旧research/tag、已有失败归档和未提交实验开发均保持。**当前选择C；后续允许的是有期限的净收益检验，不是无限继续实现预测。**
