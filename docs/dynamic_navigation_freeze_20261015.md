# 动态导航最终研究冻结（2026-10-15）

**Modify / 研究冻结 / 未通过实际部署验收。** 值得保留的最小候选是 **简单CV + STVL + MPPI时间感知消费**。已有未来信息消费和有限安全收益证据；复杂预测暂未证明比简单CV更值得投入，全面迁移Temporal MPC没有充分依据。可靠消费/可执行安全输出是主要未闭合条件。本轮不运行任何算法或闭环实验。

冻结源码 **40bdc50144ed6b91e0770d60ac930396432d3119**，原分支 `codex/stvl-lifecycle-hold-20261014`，原worktree `/home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014`，annotated tag `research/stvl-temporal-lifecycle-frozen-20261015`。tag固定算法点，不指向之后归档文档。已有63faca/8f367/其他阶段提交全部为可达祖先，原分支/history不移动。

## 统一研究索引

完整SHA、tag peeled identity及本地可达性在 [manifest](research_asset_manifest_20261015.json)；下表不重复复制报告。报告路径是对应Git树内路径，可用 `git show <SHA>:<path>` 恢复；主档9e08122含完整历史索引，研究目录README提供配置/种子/运行入口。

| 路线/固定入口 | 报告与关键证据 | 已验证/未关闭；复用与是否重启 |
|---|---|---|
| R1 future occupancy设计；旧V1 tag f85149e08cb116a53159b0482ae1db5993ae3a06；风险路线e680b1430db6efd8dc601d5585ee207e5f9b42a4 | 9e08122 `docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md`；e680 `research_stage_conclusion_20261001.md`/`standard_output_cost_alignment_20261001.md` | R1完整Nav2未来occupancy层未实现/闭环未运行，空间并集“永久墙”只是机制风险。旧V1时间critic确有实际消费；候选覆盖、硬罚打平/聚合/SG失败有反例。只保留时间/几何诊断，不复活复杂AR/CA/mixture |
| R2 DynamicObstacleCritic：feature/dynamic-obstacle-critic@91d7eda09dbece27c52fb711f0f8a1a11666c84a；9e08122保存后续历史 | `docs/dynamic_obstacle_critic/validation.md`、`observation_diameter_consumption_experiment.md`、native/SDK/weight witnesses | native消费/评分有源码和实际数据；visible centroid/完整形状/未来支持失败，nominal和guard可堵住任务。复用纯几何/连续时间评分，不迁全guard或第二速度owner；无新数据不重启 |
| R3/Temporal MPC：04291a410f193c009e043af88e014cf420e1f68b（同一路线，不双计） | `docs/dynamic_navigation/temporal_mpc_freeze_20261004.md`、gazebo/timing results | solver/异步/重锚确实现并执行；多轮取消/失败、约95.5ms输出gap、未来净空负例、制动dt错误与老化余量有反事实。可读用solve/诊断，不替换MPPI，不恢复冻结开发 |
| R4：bdb8b8d00308be4384ea94e8b060d60fd2173d30 | `docs/dynamic_navigation/r4_final_corridor_research.md`、finite/matched/Pareto报告和cohort | 时间消费库/绝对epoch成立；20/20局部零contact不能忽略混杂。末cohort gate5/5但完整goal2/5、3自身degenerate-path；Stop/Frozen。复用合同/少量代码，不迁全controller/owner/lease |
| HWS/浙江大学参照；NEU/T-DT静态路径 | 9e08122 `hws_migration_audit_20261003.md`、`hws_vs_current_prediction_analysis.md`；T-DT origin/experiment/tdt-planner-phase2及validation状态报告 | 本地复用了HWS风格tracker/KF与静态规划，不是上游整栈同条件验证。静态Path/Minimum-Snap/Sfc不等于未来障碍执行consumer；不整体迁入、不据上游名称宣称更好 |
| PVT：4d59854fb8a2937ab9cf884f684cdc286842932c | `docs/research_history/prediction_value_20261013.md`，experiments/prediction_value：10 pilot、90正式proxy、完整seed/周期 | A_proxy≠完整STVL，A′同consumer。CV/B/KF各18/18安全成功，但B相对CV无安全/效率门优势，负收益保留。复用provider/薄consumer接口，不升级复杂预测 |
| 真实STVL+原生MPPI：d2b5f0714585e1e71abaf20def932937057f644e | [研究报告](research_history/stvl_temporal_20261013.md)，experiments/stvl_temporal，140有效导航及时间消融 | 简单CV在横穿/迎面/释放通道有实际软件安全价值；原C14/14到达、12/14四门，时间修正21/21到达、19/21四门；Oracle仍窄通道碰撞，动力学额外限制。保留最小消费，非部署 |
| Footprint/执行反馈：353fea2a54e6df858664a831dc394fb04e497223 | [失败定位](research_history/stvl_temporal_failure_roots_20261014.md)，experiments/stvl_temporal_roots：48尝试/42可评价 | polygon shortcut、SIMD成本存储、实际odom消费有确定性缺口/修复；Oracle与动态负例保留。限定复用，不将单修复等同安全 |
| 联合安全：039273ae048edeff614199bae8c33583c4f64be6 | [联合报告](research_history/stvl_joint_execution_20261014.md)，8判别用例/9有限复验 | 几何/时空/Unknown区分有局部正确性；只读检查，未控制干预，执行映射Unknown；C′未准入。可复用检查函数，不作为生产硬约束 |
| 执行/实时性：dc7897e703ad4fd19fe1ab040c2419712b03cfff | [实时报告](research_history/stvl_execution_realtime_20261014.md)，33同输入判定、8启动、真实应用/反馈跟踪 | 必要批检查优化和部分计时成立；原C83接触、外层时序/物理边界未闭合。复用诊断/observer，不能凭分项p99相加保实时 |
| C83端到端：a880c7dc01cb424ea865e1fa0f33732396078670 | [因果报告](research_history/stvl_c83_endtoend_20261014.md)，45原周期、617配对周期、full原native/receipt流 | 原safe0仍新控制、迟回退/停车扫掠；重放/四次新安全导航不等于原失效稳定复现或修复。保留原始contact，不重跑求成功 |
| 控制生命周期/重试/制动/active：055f29/f10a52/af39a45/63faca/e0b1ae，最终40bdc501 | [八阶段总报告](research_history/stvl_autonomous_summary_20261014.md)和各报告/README | 16/16 safe0新非零；早制动有机会，迟400ms门限失败、800ms停车后contact；重试清零仅固定库probe；active4闭环无稳定收益。保留最小合同，不捆绑/部署 |
| fresh-coasting/observer/latest pair：8f367ea0bd53eb114f4dfe8ded22b35b84d225a2、65caadeadde16dbd59e64ef837bffb5cc9499047、40bdc501… | [validator](research_history/stvl_coasting_epoch_20261015.md)、[observer](research_history/stvl_observer_export_20261015.md)、[两组闭环](research_history/stvl_coasting_live_20261015.md) | 等时16离线校验支持局部修正；8native update仅非干扰夹具；两C83均安全但没消费等时，快.118s/净空降.115m、失败仍4、101.600ms负例。无在线收益证明，不追加种子 |

R1理论occupancy设计与旧V1真实time critic是两个层面；不能把“R1–R4失败”解释成全部未接线，也不能把所有路线根因拼成一个已经证明的统一原因。HWS/KF不是始终没价值，未来运动信息有用，但准确度提高未保证最终控制更安全。

## 保留的能力等级

| 等级 | 本项目真实完成的范围 | 禁止外推 |
|---|---|---|
| 离线/native调用 | PVT纯输入/信息消融、R2几何、原生重试、fresh等时validator、observer数值非干扰 | 不代表DDS/执行/闭环/实车收益 |
| 代理时间推进闭环 | PVT相同consumer/动力学/预算推进控制；D只作离线可行参考 | A_proxy不是完整STVL；代理尺寸不是新车CAD，D不是实时组/最优证明 |
| 真实软件链+代理物理 | 固定Humble STVL/Nav2/native Omni MPPI/smoother/stub/Gazebo及有限重复 | 软件零contact不保证实物安全，不能提升为新车Integration或Deployment |
| 只读联合安全诊断 | source-time/swept geometry/静态/动态/Unknown检查与执行回溯 | 没有最终命令接受权；不是已接入硬安全约束 |
| 控制干预C′ | **未启用/未准入** | 没有C′闭环结果，不能描述为“修复控制器” |
| 实车 | 当前研究候选 **未验收** | 无真实CAD/底盘响应/MCU命令执行/时钟/物理停止证明 |

## 最终失败和待验矩阵

| 未关闭问题 | 已知事实与最有价值证据 | 缺少什么/能否软件处理/是否等硬件/重新开启条件 |
|---|---|---|
| 原C83不能稳定复现 | corrected_scene原记录goal4.002s、相对5.308s contact；后续同定义独立运行多次安全 | 缺受控同源反馈/调度复现。软件可继续定位但本轮停止；需新可稳定触发数据，不据新安全样本关闭旧接触 |
| NoSafe最终动作合同 | 原16/16零safe仍新非零，300成本仍有限；最新两组各3safe0新非零 | soft评分计数不等于物理全不可行；缺动作接受与制动/停车未来占用证据。软件可局部研究，实际边界须底盘数据；不得自动开发安全控制器 |
| 规划—执行映射 | weighted/constraint/SG/shift之后不保留raw安全身份；smoother/20Hz/延迟/feedback差异 | 缺可信候选epoch→最终指令→轮端反馈映射，checker仍Unknown。软件与实车联合；新同步记录及完整合同才重启 |
| 动力学/加速度/停车位置 | 早刹净空.2656m，迟400ms.04342m，迟800ms停车后contact；部分真实代理加速度/净空门失败 | 代理target限加速度通过不等于物理速度约束全面通过。必须真实轮/整车制动、载荷/摩擦/时延数据 |
| 100ms实时预算/外层调度 | 本轮c112 native99.436/必要99.846/wrapper101.476/DDS101.600ms，NAV间隔192.110ms，计算窗采样移动.04805m；原线程还有warning | 缺完整server/costmap等待/host/MCU执行截止期限；分项计时不够。可软件profiling但现在冻结；真实任务中出现影响任务的瓶颈才另行授权 |
| 等待/效率退化 | PVT Oracle迎面更慢、释放seed29负收益；active横穿慢2.479s；最新pair少.118s但净空降.115m | 没有稳定安全+到达+效率共同优势；不是加大安全间距一直停的Go。软件比较需要明确新任务需求/同条件输入，不扫参数 |
| 联合检查与C′ | 33同输入/离散加扫掠等局部证据；执行Unknown未消除 | 尚未接入C′、无干预安全收益。软件可补最小合同但本轮不接，不以“checker正确”当部署准入 |
| 实车时钟/速度响应/物理停稳 | 新车硬件未到位，已有串口失联设计/软件编码，不是物理停止 | 必须MCU收到/接受/执行、真实source时间、ESTOP/watchdog/host断电/USB延迟及停稳数据；硬件到位八步采集后再评审 |

100.590209ms旧负例来自另一成功运行、只超.590ms；最新101.600ms亦无contact。两者都不能证明CPU超时导致原C83接触；超限真实存在，碰撞因果仍缺。fresh等时关系有离线局部支持，最新两C83未触发；active过滤无稳定收益。十二条用户指定结论全部保持，C′未启、正式导航未迁入研究。

## 源码、证据与未来恢复

算法tag与独立研究ref已远端保存并核对40bdc501；原研究branch不因归档前进。归档分支只新增本文件、导航总表、manifest，不覆盖原报告。[manifest](research_asset_manifest_20261015.json)记录全部ref/full HEAD、子模块、8个关键Git blob的对象哈希核对、约2.45万可达对象无missing、现存raw路径/大小/SHA与依赖。

现存original C83：`/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned`。c83_cache其余no_shadow/normal/shadow/durable及latest active/fresh pair完整流继续保留。新增SHA是当前文件指纹，不能代替每个文件的旧原始基准；近期无损gzip/精选恢复的既有证明在原stage报告。没有重跑科学指标、全量fsck/GC或删除raw。

只读查看冻结实现：

```bash
git show research/stvl-temporal-lifecycle-frozen-20261015:docs/research_history/stvl_autonomous_summary_20261014.md
git rev-parse 'research/stvl-temporal-lifecycle-frozen-20261015^{commit}'
git ls-tree research/stvl-temporal-lifecycle-frozen-20261015 src/fast_lio_multi
```

未来在**新目录且空间足够**恢复源码：clone现存Gitee的 `research/stvl-temporal-lifecycle-frozen-20261015` branch，核对HEAD40bdc501，再按README恢复所需固定R4 overlay/realtime observer和Humble镜像；源码submodule需恢复69f107/0438/2a。旧R2/R3/R4历史报告通过其冻结refs/私有归档恢复。完整构建/实车profile不在本轮执行。本轮没有新空仓完整网络clone，不冒称完成了最新分支的从零运行恢复。

**第二存储转移清单**（未执行；没有授权的异机目标）：

1. 按manifest `raw_evidence.files`原相对路径转移全部现存文件，逐文件SHA256核对。覆盖STVL Oracle/CV、全部故障/超时/工程失败、C83原始与latest两组，而非只复制摘要。
2. 共享Git对象/refs及独立子模块恢复对象，尤其尚未在所查询Gitee入口发现的NEU/Full-base/covariance；原9条WIP与R4 dirty分析的已有备份/恢复引用一并保护。不得改写/移除这些工作区。
3. `/home/qihei/rm2027_git_backups/20261012`原bundle、清单和恢复说明；老私有Release的manifest/restore-proof入口。已有20261013证明复用，本轮两小increment `bundle verify`通过；没有再读/下载大型历史包。
4. 固定runtime image/install资产的恢复来源（manifest dependencies）：源码Git push不保存Docker镜像、ignored R4 install或原1ms native全流。

转移到实际获准的独立设备/异机后再核验sha；不要把本NVMe另一个目录或未知挂载点当灾备。当前完整raw没有经验证的第二份，Gitee只存Git-selected子集。私有历史Release的旧成功不证明新20261013–15 raw已备份。Gitee本次成功保存同时发>80%容量提示；不因此删除历史或改写refs。

## 重启条件与停止

仅在真实底盘响应/制动/时延、新稳定C83失效、明确控制生命周期软件缺陷、真实STVL+MPPI任务瓶颈或具备可信安全/效率/实时共同评测条件时建议评审重启。先固定可验证目标、baseline、预算、门限、无新增证据的停止规则及缺硬件时的暂停规则，再取得新用户授权。

额度充足、仍有Modify、存在高级算法、10月29日到期都不是重启理由。暂停覆盖所有导航定位/规划/控制研究；允许单独报告资产损坏或安全异常，但不自动恢复开发。当前没有需两周内紧急推进的非实车算法工作；优先其他项目，保留恢复入口即可。
