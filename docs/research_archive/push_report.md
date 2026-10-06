# 本轮归档执行结果（2026-10-06）

**本轮成功新增上传的分支、tag、commit均为0。云端归档未完成。** 本地审计、筛选、去重、源码/WIP快照和归档提交已完成。

首次单独推送 `experiment/r4-hws-prediction-consumption` 的 `bdb8b8d00308be4384ea94e8b060d60fd2173d30` 时，Git返回：`could not read Username for https://gitee.com: terminal prompts disabled`。这是本会话缺少HTTPS身份，尚未进入有身份的远端写入；不是分支冲突或容量拒绝。已遵照用户“权限错误停止对应推送”要求停止同一origin全部后续写入，没有尝试替换认证、改remote、强推或改写历史。原输出见 [first_push_failure.log](first_push_failure.log)。

## 本地归档与筛选量

- 归档分支：`archive/dynamic-navigation-evidence-20261006`，从 main `2849cbe4dba5cf7e6548ff3f67e72941c895eef9` 新建；最终本地归档commit为本分支HEAD（本报告随同该commit保存）。
- worktree：`/home/qihei/rm2027_navigation/build/research_archive_20261006`；内容位于 `docs/research_archive/`。
- R4/observed-surface新原始记录：3,324条路径，2,559个去重对象，原路径合计530.69MiB，唯一原内容509.72MiB，gzip114.08MiB；最大对象4.12MiB。
- A23–A26共140个完整分析运行目录，包括所有正式样本、校准、pilot/前置失败；正式统计仍按原protocol/schedule划分，不混合cohort。
- 31个历史分支的小型源码/结论/配置快照：40,449条路径引用，2,661个不同Git blob；去重后仅新增20.55MiB。它们是部分文件树快照，完整历史和大输入另见原分支；不宣称这份小快照足够复现全部原试次。
- 合计数据对象约134.63MiB；索引/清单/快照映射/工具等另约8.91MiB。归档实际文件合计约143.54MiB（报告新增自身字节及Git对象存储另计）。**上传数据量0。**
- R3已有约212.8MiB的无损Git证据不重复复制，15份manifest的1,265项payload校验全部通过。
- 新增本地R4冻结tag：`research/r4-hws-low-level-frozen-20261006`，target `bdb8b8d0`；未上传。

## 分支逐项状态

32个非main本地分支经独有代码、提交和结论文档审计后选中保留原历史。R4首次推送身份失败，其余31个未尝试同一身份下的重复推送。以下均仍只存在本地的新tip；部分祖先原本已在远端。完整字段/规模见 [branches.csv](branches.csv)。

| 分支 | 保留commit | 本轮远端状态 |
| --- | --- | --- |
| `codex/dynamic-differential-risk` | `e680b1430db6efd8dc601d5585ee207e5f9b42a4` | 因同一认证缺失未尝试 |
| `experiment/controller-odom-contract` | `96799412e0b60a2b55eda2ed567476160d675e9e` | 因同一认证缺失未尝试 |
| `experiment/cv-support-bound-contract` | `ca3a50dcd50bd5059b686f59f00cc50941f5a20c` | 因同一认证缺失未尝试 |
| `experiment/cv-support-factor-audit` | `b17b73a5d887c78945e5db89b95cb7e866ac85eb` | 因同一认证缺失未尝试 |
| `experiment/dynamic-consumption-evidence` | `180a773cd000a338b99ea43841dbb0943af500f6` | 因同一认证缺失未尝试 |
| `experiment/dynamic-documents-archive-20261004` | `1229583ff5811bc42671188f05332d0b3780ebd2` | 因同一认证缺失未尝试 |
| `experiment/dynamic-surface-reveal` | `b5645ecaf6b4575a6537a0a0a9907f4891955955` | 因同一认证缺失未尝试 |
| `experiment/ground-robot-extent-prior` | `e50b5eeb208f438f7a1cf73a8463fa140ae16348` | 因同一认证缺失未尝试 |
| `experiment/hws-migration-audit-20261003` | `652f14cea66edc0fcd77401de7db41cb6ffa7477` | 因同一认证缺失未尝试 |
| `experiment/map-uncertainty-stopping` | `aafd87f6d977c024e86c871cdcdcce3e9200cb74` | 因同一认证缺失未尝试 |
| `experiment/mechanical-footprint-contract` | `a4076027050c806104977317141ea53d21c88631` | 因同一认证缺失未尝试 |
| `experiment/native-cost-accumulator-evidence` | `41e9639936718a46a9b3aecaba51981b2805c4f3` | 因同一认证缺失未尝试 |
| `experiment/native-cost-map-contract` | `3ce395889f565b786f980929ad64b6edd4fe8ac2` | 因同一认证缺失未尝试 |
| `experiment/native-cycle-evidence` | `a12688b2c35b86ebf5ea63dcc86bdc753918f551` | 因同一认证缺失未尝试 |
| `experiment/native-factor-aggregation` | `7c0db1b528ca01624be8ae6ce43ee80eb57ed25a` | 因同一认证缺失未尝试 |
| `experiment/native-factor-safe-mass` | `60e485bdb2ba73b67da5cce50830d6273c4251e8` | 因同一认证缺失未尝试 |
| `experiment/native-individual-stage-contract` | `d688f790f7d8d3223d61a14537de4037146f4c92` | 因同一认证缺失未尝试 |
| `experiment/native-raw-map-contract` | `2a67835d59a3310773091db95f864ddc60c6faa9` | 因同一认证缺失未尝试 |
| `experiment/native-safe-control-witness` | `b89eb1d0ff4cad3ef25d71a64c7d0588e63d985e` | 因同一认证缺失未尝试 |
| `experiment/native-sg-stage-audit` | `ced13e8ae13a90e88c22e08fb9b67169acfa015e` | 因同一认证缺失未尝试 |
| `experiment/native-weight-attribution` | `fea7bf914e42361a3dda0053145e7077687b0a3e` | 因同一认证缺失未尝试 |
| `experiment/observation-anchor-cv` | `bc447533ab4a847059e7ca65c54bf9267236a941` | 因同一认证缺失未尝试 |
| `experiment/observation-diameter-consumption` | `e061a245e3d760f27705af1a1125774c84298c34` | 因同一认证缺失未尝试 |
| `experiment/processed-cv-objective-audit` | `8523d1f47fbb86e6e976f371e43778addcf40eee` | 因同一认证缺失未尝试 |
| `experiment/r4-hws-prediction-consumption` | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 推送身份失败 |
| `experiment/raw-rollout-objective-audit` | `1edb4ee42c1a0ae9b4132d994e5220c92f838469` | 因同一认证缺失未尝试 |
| `experiment/soft-clearance-performance` | `b92cbe0f48e29c3538e01045163262d2e8fb5d7d` | 因同一认证缺失未尝试 |
| `experiment/soft-map-clearance` | `2622371295abfdaefc543e1c346346adce36228f` | 因同一认证缺失未尝试 |
| `experiment/static-stopping-critic` | `faabc5a5df059c9a084c58750079bc789ce0619f` | 因同一认证缺失未尝试 |
| `experiment/temporal-mpc-main-20261004` | `04291a410f193c009e043af88e014cf420e1f68b` | 因同一认证缺失未尝试 |
| `experiment/visible-box-geometry` | `5b5f654434f5c52c35a85275b8a14a26c16ac7e7` | 因同一认证缺失未尝试 |
| `feature/dynamic-obstacle-critic` | `91d7eda09dbece27c52fb711f0f8a1a11666c84a` | 因同一认证缺失未尝试 |

main只审计，不推送：本地 `2849cbe4` 保持不变，远端main仍为 `d735ee12`。本地main的开发规范commit已作为归档/R4历史的祖先保留，没有修改远端main。

## Tag与既有远端核实

| Tag | target commit | 远端状态 |
| --- | --- | --- |
| `dynamic-prediction-v1-frozen-20260925` | `f85149e08cb116a53159b0482ae1db5993ae3a06` | 本轮前已存在，tag对象与peeled commit只读核实一致 |
| `research/dynamic-differential-risk-frozen-20261001` | `e680b1430db6efd8dc601d5585ee207e5f9b42a4` | 仅在本地，认证缺失未推送 |
| `research/r4-hws-low-level-frozen-20261006` | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 仅在本地，认证缺失未推送 |
| `research/temporal-mpc-frozen-20261004` | `04291a410f193c009e043af88e014cf420e1f68b` | 仅在本地，认证缺失未推送 |

远端为 [Gitee rm2027_navigation](https://gitee.com/qiheiovo/rm2027_navigation)。前后各读取一次真实远端refs：均56个分支、1个冻结tag，完整结果 [remote_before.txt](remote_before.txt) 与 [remote_after.txt](remote_after.txt) 字节一致。现有R1分支/tag可访问；**没有本轮新增归档commit可供远端访问，不能以本地commit成功替代上传成功。**

## 未上传、仅留本地与限制

- 本轮全部32个选中研究分支新tip、R2/R3/R4三个未上传冻结tag、归档分支及新数据：身份缺失。逐ref待执行计划为 [push_plan.json](push_plan.json)，不自动续推送。
- 部分R2历史含约2GiB级证据与143.4MiB单blob；保留原始历史，不为绕过远端限额改写。由于认证未通过，本轮没有试出该账户容量/单文件限制，也未确认Gitee LFS支持。
- 有价值的primary observed-surface dirty/untracked：完整9文件+diff已在本地归档WIP备份；实际DDS/member/shape证据按原件保存。R4 rotation analyze.py dirty独立价值不明确，保持原工作区，仅记录，不自动提交/上传。
- surface SDK 113MB级输入bin、原R1/R2/R3/R4完整ignored build、原SDK/devspace、源码/binary物理身份：原件全部保留本地。部分在既有Git无损包已有等价版本，不重复上传。小快照遗漏及排除记录可逐项查询。
- 另查三个原worktree中docs/experiments/src的ignored文件，均为Python缓存或CMake构建记录，明细见ignored_outside_build.json。可再生build/install/log、.o/库/cache、完整第三方clone、Docker镜像、795MB core：不纳入新包，未删除。任何负面结果均不因为Stop而清理。
- 凭据检查：17,639个未推送unique Git blob及61个tar/gzip包，新增原始流、historical小快照、WIP和最终元数据均扫描，未发现需拦截的凭据；扫描为模式审计，范围和结果在credential_audit文件中。
- 没有发现远端分叉冲突；远端写入认证失败已保留。容量未测试，不将其写成通过。

## 原工作区保护及研究结论

[preservation_after.json](preservation_after.json)核实33个原本地分支tip、3个旧tag、3个现存原工作区HEAD/status与dirty内容SHA256一致；main旧prunable登记保持。只新增归档worktree/分支和R4最终tag。没有运行研究实验、编译、回归或sanitizer，没有修改导航/算法配置，没有删除证据。

A25开放S2支持调优STVL+MPPI在当前R4配置上的安全—效率支配；A26保留R4 5/5安全gate通过、零contact、仅2/5完整goal成功与baseline 0/5成功/5/5contact的区别。最终仍为Stop当前low-level路线的生产化；WAIT→GO信号有研究价值，不概括为预测无用，也不宣称R4整体优于MPPI。

本轮到此停止。若恢复Gitee写入认证，可据已审计计划继续逐项上传并重新核实远端；未获得的新导航研究方向不在本任务中开展。
