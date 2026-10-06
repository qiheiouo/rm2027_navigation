# RM2027 研究资产归档索引（2026-10-06）

本轮仅做资产保护、筛选、去重与 Gitee 归档，没有运行研究实验、编译或回归，没有修改冻结算法。归档分支 `archive/dynamic-navigation-evidence-20261006` 直接从本地 main `2849cbe4dba5cf7e6548ff3f67e72941c895eef9` 派生。main 的导航功能、配置及原有工作区保持原状。历史文档里的“下一步”和“未 push”描述按其记录时间理解；它们不是恢复研究的授权。

**用户手动认证后，19项已上传并只读核实：17个分支（含归档、R4、R3）及2个冻结tag。数据归档commit为 `ac197e4e`。第20项R2历史被Gitee仓库/单文件容量限制拒绝；剩余16个研究分支tip及1个冻结tag停止上传。当前阻塞是容量，不是认证。详细状态见 [推送记录](push_report.md)。** [branches.csv](branches.csv) 列出所有原本地分支、完整 commit、独有历史规模、筛选依据与最终远端状态；[git_audit.json](git_audit.json) 保存拓扑、tag、worktree 和独有提交清单。没有按分支名直接判定价值，没有使用 push-all、批量 tags、force、历史改写或删除旧资产。

## 方向与结论

| 资产 | 方法、目标与关键实验 | 冻结身份 / 入口 | 结论与材料价值 |
| --- | --- | --- | --- |
| R1 动态预测 | 跟踪/运动模型、CV/CA 与 native MPPI 候选风险消费；预测可靠性与输出链诊断 | 已在远端的 `experiment/dynamic-prediction-v1`；tag `dynamic-prediction-v1-frozen-20260925`，commit `f85149e0` | Research 冻结，未获部署接受；模型失配、候选覆盖与聚合失败可作反例 |
| R1/R2 differential-risk 历史 | 保持 native 优化链的风险、时序 proposal、成本对象与输出映射；38 个已入 Git 的无损实验包 | `codex/dynamic-differential-risk` / `e680b143`；tag `research/dynamic-differential-risk-frozen-20261001`；`docs/dynamic_navigation/research_stage_conclusion_20261001.md` | 冻结；ranking 或采样覆盖局部 PASS 不等于最终安全输出 PASS；保留失败链和逐候选原始证据 |
| R2 STVL/MPPI、observed shape | 静态 stopping、map uncertainty、footprint、native cycle/cost/SG/aggregation、CV support、observation anchor/diameter、surface reveal；独有源码与全部中间 checkpoint | `feature/dynamic-obstacle-critic` 至 `experiment/dynamic-surface-reveal`、`experiment/dynamic-documents-archive-20261004`；逐项见 branches.csv；`docs/dynamic_obstacle_critic/` | 多项合同/观测链研究有独立复用价值；离线保证有条件，不能当成生产安全证明。部分已提交巨大输入/SDK 产物须单独处理远端容量 |
| R3 Temporal MPC | T-DT/Sfc reference/corridor + 1.5s 平移 QP、局部 portfolio、Nav2 selector/执行与 timing 对照 | `experiment/temporal-mpc-main-20261004` / `04291a41`；tag `research/temporal-mpc-frozen-20261004`；`docs/dynamic_navigation/temporal_mpc_freeze_20261004.md` | Research Frozen / 未获部署接受。最新双方 40s 取消、候选仅 1 个通过 MPC 周期，后续重锚终态净空 −14.953267mm；保留 acceptance 撤回、TF/时间/命令间隙反例，不归结为“预测无用” |
| R4 A01–A22 | HWS-style 消费数学、输入合同、复用审计、实际 source/observed shape、world XY 语义及 endpoint/clock 修复 | `experiment/r4-hws-prediction-consumption`；数学冻结 `ccd3eac4`；进度 `docs/dynamic_navigation/r4_hws_prediction_consumption_progress.md`；源码 `experiments/r4_hws_prediction_consumption/` 与 `src/rm_r4_prediction_consumption/` | 前期 harness 与 adapter 属于研究资产；不自动升级为第二套导航链。world XY 与 yaw-invariant footprint 是关键语义修正；角速度扩展路线的失败也保留 |
| R4 A23 | 首次有限 Gazebo 闭环：S0 每组5、S1/S2每组10；输入/真值/接触/轨迹对照 | commit `59fae158`；`experiments/r4_gazebo_comparison/evidence/`；新增原始记录组 `R4/r4_finite_comparison_20261006` | Modify：原碰撞收益存在速度/footprint 混杂，不能单独归因于 prediction consumption |
| R4 A24 | 对齐空场实测速度与共同圆形支持，S0每组3、S1/S2每组5 | commit `1c0b6d73`；`evidence_matched/`；组 `R4/r4_matched_comparison_20261006` | Modify：双方13/13成功零contact，R4 S2更大净空伴随约33%耗时增加；冻结配置后不再改本体 |
| R4 A25 | 安全距离有界校准未达到原目标带；用户授权固定更大净空支配点，5对新S2 | commit `89f9035b`；`evidence_pareto/`、`r4_clearance_efficiency_pareto.md`；组 `R4/r4_pareto_comparison_20261006` | **Stop 当前配置生产化**：双方5/5零contact；STVL+MPPI 净空中位 .38927m / 到达14.628s，R4 .29294m / 17.889s；5/5配对 baseline 更快且净空更大。严格等净空校准未达标仍保留 |
| R4 A26 原 cohort | 单通道短时阻塞；wrapper 原首次 native 异常锁存导致提前结束 | commit `9d005e05`；`evidence_corridor/`；组 `R4/r4_corridor_comparison_20261006` | 原10次提前终止、首次异常 ring、前置可行性/失败全部独立保存；不能与恢复 cohort 合并样本 |
| R4 A26 最终 cohort | 用户授权仅解除 native 异常永久锁存，让原 Nav2 failure_tolerance/BT 恢复运行；R4算法与双方参数不变 | wrapper `8f8d6c80`；最终 `bdb8b8d00308be4384ea94e8b060d60fd2173d30`；`evidence_corridor_recovery/` 与 `r4_final_corridor_research.md`；组 `R4/r4_corridor_native_recovery_20261006` | **Stop 并冻结当前 low-level 路线**：R4 5/5安全通过gate、零contact，仅2/5完整goal成功，3/5近终点自身path合同失败；baseline 0/5成功、5/5contact。保留 WAIT→GO 研究信号，不宣称整体优于 MPPI 或具备生产化条件 |
| T-DT 迁移、定位/重定位/TF/可靠性 | 已有规划迁移、gimbal/LIO动态TF、重定位后端、integrity 状态机及实车链路修复 | 已在 origin 的54项分支 tip及实际改动见 [remote_branch_inventory.json](remote_branch_inventory.json)，例如 `experiment/tdt-planner-phase2`、`feature/phase2j-relocalization-backends`、`feature/navigation-integrity` | 已有独立历史不重复造归档分支；保留各自阶段边界，不凭已上传宣称全部部署验收 |
| 未提交 observed surface 成员取证 | 实际 DDS/TF/member provenance、预算与 corruption 核查，私有旁路默认关闭 | [WIP 快照](wip/observed_surface_members/snapshot.json)，原基点 `1229583f`；完整9文件与 tracked_changes.patch；组 `surface/*` | 精确 WIP 备份，未合并、未在本轮重新验证、不冒充正式功能；对观测形状输入真实性与论文材料有价值 |

所有短 commit 均可在 branches.csv、git_audit.json 或相应研究分支日志解析为完整身份。R4最终冻结tag `research/r4-hws-low-level-frozen-20261006` 已核实在远端，指向 `bdb8b8d0`。R4最终结论只适用于记录的仿真、感知、控制与 footprint 条件，未来真值未反馈给控制器。A26成功样本的到达时间不能代表3个失败样本，gate通过不替代Nav2 goal成功。

## 证据层级与去重

[证据清单](evidence_inventory.md) 区分完整原始记录、精简统计、既有 Git 无损归档与仅留本地的内容。新包采用按原始内容 SHA256 去重的 gzip 对象；[evidence_manifest.json](evidence_manifest.json) 逐文件记录原路径、组、原始大小/校验和、对象路径和压缩大小。未改写 CSV、JSONL、bag 或失败原因。

- 新增选择原件3,324条，去重后2,559个对象；原路径合计530.69MiB，唯一原内容509.72MiB，存储114.08MiB。最大对象4.12MiB，未使用 LFS，未收集镜像或巨大core。
- A23–A26保留140个原始运行目录（含校准、pilot、前置失败），每个都有 prediction、truth、command 三条完整分析流；正式样本集合仍由原protocol/schedule/summary确定，不把140当作正式样本数。
- R3已有15份 manifest（覆盖9组核心证据），1,265项payload校验无缺失/无不符；见 [r3_existing_archives.json](r3_existing_archives.json)。不再复制其约212.8MiB既有 Git 包及可再生构建树。
- 另外31个历史分支的源码/结论/配置小快照共40,449个路径引用、2,661个独有Git blob，以内容去重后仅新增20.55MiB。见 `historical_snapshots.json.gz`；它们是部分文件树备份，不能替代完整Git历史或原始大输入，遗漏清单为 `historical_snapshot_omissions.json.gz`。
- R1/R2已有38个无损实验包索引见 [r1_r2_existing_archives.json](r1_r2_existing_archives.json)。原 Git 大文件不移除、不重写历史。R2差分风险分支原始历史已因容量被拒绝，未上传范围见推送记录；其小型源码/结论/配置快照已随归档commit上云。
- [local_only_exclusions.json](local_only_exclusions.json) 列出新收集范围排除的文件及理由；[ignored_directory_inventory.json](ignored_directory_inventory.json) 是忽略目录总盘点。排除不等于删除，所有原件仍留本地。

## 恢复与复现入口

恢复工具只解压并校验，不启动ROS或实验，不覆盖内容不同的文件：

```bash
# 在归档分支中，仅验证包完整性
python3 docs/research_archive/tools/restore_evidence.py /tmp/rm2027-restored --verify-only
# 恢复A26原Nav2恢复cohort的全套分析记录
python3 docs/research_archive/tools/restore_evidence.py /tmp/rm2027-restored --group-prefix R4/r4_corridor_native_recovery_20261006
```

恢复后的目录与原 source 路径一致。离线重分析使用冻结R4分支的 `experiments/r4_gazebo_comparison/analyze.py <恢复的cohort目录>` 及同目录公平性/场景核查脚本；R3物理真值审计函数来源和依赖在原README有记录。重新开展仿真须另获用户研究授权；历史README记录原环境、依赖commit、配置、构建/运行命令，归档不包含Docker镜像、可再生install或未授权的第三方完整clone。

## 本地保留与保护

原primary observed-shape dirty/untracked、R4 `experiments/r4_rotation_value/analyze.py` dirty及巨大core均保持原状。后者修改来源/独立价值不明确，仅在清单记录，不自动推送。main缺失/prunable的旧worktree登记不清理。没有stash。唯一reflog-only A15前驱与已引用 `13b81775` 基本等价；小差异保存在 `reflog_precursor_difference.patch`，不新建旧算法分支，不删除原对象。保护前后状态与dirty内容校验结果见最终推送记录。

本地索引已按实际上传结果更新。远端归档分支仍为 `ac197e4e`，其中数据已保存，上传前的状态文字属历史记录。用户随后选择调整配额、完整上传原历史；[容量处理与手动续传方案](quota_resolution.md)包含重新计算的18项清单及固定SHA工具。新增管理commit仍仅在本地，代理没有重试第20项、改写历史或推送。
