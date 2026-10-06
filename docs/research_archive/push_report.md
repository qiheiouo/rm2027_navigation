# 研究资产远端归档执行结果（更新于用户手动上传之后）

**认证已经成功；已上传并核实19项（17个分支、2个tag）。第20项因仓库与单文件容量限制被拒绝，剩余17项停止上传。** 此后的状态更新只提交本地，不再次推送。

## 认证与停止原因

最初代理会话缺少HTTPS身份，首次R4推送未启动远端写入，记录仍保留在 `first_push_failure.log`。随后用户在自己的终端输入账号密码，执行 `/tmp/rm2027_push_reviewed_archive_20261006.py --execute`。前19项均得到Git成功返回，脚本逐项核实远端SHA；本次又通过只读 `ls-remote` 独立确认。账号密码这次已被Gitee接受，当前失败不是认证问题。

第20项 `codex/dynamic-differential-risk` / `e680b143` 的实际服务器拒绝：

- `Repo size: 1135.480MB, exceeds quota 1024MB`。这是服务器对本次拟上传历史的容量判定，不等于已独立测得当前已接受仓库占用1135.480MB。
- 对象 `0733a98c73ba25f3168e71784c3eed57e3debf5a`，服务器报告142.713MB，超过100MB限制。已映射至 `docs/dynamic_navigation/evidence/ca_independent_01_capture_20261001/capture.tar.gz`，本地文件150,407,573 bytes（143.44MiB）。服务器与本地的单位/计量值分别保留。
- `pre-receive hook declined`；被拒绝分支的远端tip仍为 `e067c3757e2272128a6af6f015526f893321e5f6`，没有变成 `e680b143`。
- 服务器提示仅有3次push机会。脚本立即停止，代理没有重试、删大文件、改写历史、强推、运行GC或切换存储方式。不要重跑原脚本消耗拒绝机会。

## 已上传的关键成果

- 数据归档分支：`archive/dynamic-navigation-evidence-20261006`，远端commit `ac197e4e1d89e51e4799ac6d8fbafe34bc1ca2df`。其中两次本地归档提交 `6dff0a13` / `ac197e4e` 已保存；本次更新状态的后续commit仅在本地。
- R4完整原Git历史：`experiment/r4-hws-prediction-consumption` / `bdb8b8d00308be4384ea94e8b060d60fd2173d30`。冻结tag `research/r4-hws-low-level-frozen-20261006` 也已核实上传。
- R3完整原Git历史：`experiment/temporal-mpc-main-20261004` / `04291a410f193c009e043af88e014cf420e1f68b`。冻结tag `research/temporal-mpc-frozen-20261004` 也已核实上传，原无损证据不重复复制。
- 其余14个已审计R2研究分支完整历史已上传，名称与SHA见下表（包含tracker/observed shape/原生MPPI witness等源码及原始证据）。
- 新数据包：3,324条原始路径引用、2,559个内容对象，原路径合计530.69MiB、唯一原内容509.72MiB，gzip114.08MiB；A23–A26保留140个完整分析运行目录，正式/校准/pilot/失败按原protocol分组。
- 31个历史分支的小型源码/结论/配置快照：40,449条路径引用、2,661个原Git blob，去重新增20.55MiB；即使原分支后期大历史受阻，这些小快照已经随归档上云，但不能冒充完整历史与全部原始输入。
- 数据对象合计134.63MiB；上传的归档tree含索引/工具约143.54MiB，Git传输pack136.26MiB。19次成功推送的终端pack传输量合计约697.68MiB，包含其他分支的原历史；不把传输量等同仓库存储大小。

## 分支逐项清单

| 分支 | 审计保留commit | 真实远端状态 |
| --- | --- | --- |
| `archive/dynamic-navigation-evidence-20261006` | `ac197e4e1d89e51e4799ac6d8fbafe34bc1ca2df` | 数据归档已上传；本次状态更新仅本地 |
| `codex/dynamic-differential-risk` | `e680b1430db6efd8dc601d5585ee207e5f9b42a4` | 容量拒绝；远端保留旧tip |
| `experiment/controller-odom-contract` | `96799412e0b60a2b55eda2ed567476160d675e9e` | 已上传，远端SHA核实 |
| `experiment/cv-support-bound-contract` | `ca3a50dcd50bd5059b686f59f00cc50941f5a20c` | 容量拒绝后未尝试 |
| `experiment/cv-support-factor-audit` | `b17b73a5d887c78945e5db89b95cb7e866ac85eb` | 容量拒绝后未尝试 |
| `experiment/dynamic-consumption-evidence` | `180a773cd000a338b99ea43841dbb0943af500f6` | 已上传，远端SHA核实 |
| `experiment/dynamic-documents-archive-20261004` | `1229583ff5811bc42671188f05332d0b3780ebd2` | 容量拒绝后未尝试 |
| `experiment/dynamic-surface-reveal` | `b5645ecaf6b4575a6537a0a0a9907f4891955955` | 容量拒绝后未尝试 |
| `experiment/ground-robot-extent-prior` | `e50b5eeb208f438f7a1cf73a8463fa140ae16348` | 已上传，远端SHA核实 |
| `experiment/hws-migration-audit-20261003` | `652f14cea66edc0fcd77401de7db41cb6ffa7477` | 容量拒绝后未尝试 |
| `experiment/map-uncertainty-stopping` | `aafd87f6d977c024e86c871cdcdcce3e9200cb74` | 已上传，远端SHA核实 |
| `experiment/mechanical-footprint-contract` | `a4076027050c806104977317141ea53d21c88631` | 已上传，远端SHA核实 |
| `experiment/native-cost-accumulator-evidence` | `41e9639936718a46a9b3aecaba51981b2805c4f3` | 容量拒绝后未尝试 |
| `experiment/native-cost-map-contract` | `3ce395889f565b786f980929ad64b6edd4fe8ac2` | 容量拒绝后未尝试 |
| `experiment/native-cycle-evidence` | `a12688b2c35b86ebf5ea63dcc86bdc753918f551` | 已上传，远端SHA核实 |
| `experiment/native-factor-aggregation` | `7c0db1b528ca01624be8ae6ce43ee80eb57ed25a` | 容量拒绝后未尝试 |
| `experiment/native-factor-safe-mass` | `60e485bdb2ba73b67da5cce50830d6273c4251e8` | 容量拒绝后未尝试 |
| `experiment/native-individual-stage-contract` | `d688f790f7d8d3223d61a14537de4037146f4c92` | 容量拒绝后未尝试 |
| `experiment/native-raw-map-contract` | `2a67835d59a3310773091db95f864ddc60c6faa9` | 容量拒绝后未尝试 |
| `experiment/native-safe-control-witness` | `b89eb1d0ff4cad3ef25d71a64c7d0588e63d985e` | 已上传，远端SHA核实 |
| `experiment/native-sg-stage-audit` | `ced13e8ae13a90e88c22e08fb9b67169acfa015e` | 容量拒绝后未尝试 |
| `experiment/native-weight-attribution` | `fea7bf914e42361a3dda0053145e7077687b0a3e` | 已上传，远端SHA核实 |
| `experiment/observation-anchor-cv` | `bc447533ab4a847059e7ca65c54bf9267236a941` | 容量拒绝后未尝试 |
| `experiment/observation-diameter-consumption` | `e061a245e3d760f27705af1a1125774c84298c34` | 容量拒绝后未尝试 |
| `experiment/processed-cv-objective-audit` | `8523d1f47fbb86e6e976f371e43778addcf40eee` | 容量拒绝后未尝试 |
| `experiment/r4-hws-prediction-consumption` | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 已上传，远端SHA核实 |
| `experiment/raw-rollout-objective-audit` | `1edb4ee42c1a0ae9b4132d994e5220c92f838469` | 已上传，远端SHA核实 |
| `experiment/soft-clearance-performance` | `b92cbe0f48e29c3538e01045163262d2e8fb5d7d` | 已上传，远端SHA核实 |
| `experiment/soft-map-clearance` | `2622371295abfdaefc543e1c346346adce36228f` | 已上传，远端SHA核实 |
| `experiment/static-stopping-critic` | `faabc5a5df059c9a084c58750079bc789ce0619f` | 已上传，远端SHA核实 |
| `experiment/temporal-mpc-main-20261004` | `04291a410f193c009e043af88e014cf420e1f68b` | 已上传，远端SHA核实 |
| `experiment/visible-box-geometry` | `5b5f654434f5c52c35a85275b8a14a26c16ac7e7` | 已上传，远端SHA核实 |
| `feature/dynamic-obstacle-critic` | `91d7eda09dbece27c52fb711f0f8a1a11666c84a` | 已上传，远端SHA核实 |

## Tag核实

| Tag | target commit | 远端状态 |
| --- | --- | --- |
| `dynamic-prediction-v1-frozen-20260925` | `f85149e08cb116a53159b0482ae1db5993ae3a06` | 已在远端；R1为本轮前已存在，R3/R4为用户本轮新增 |
| `research/dynamic-differential-risk-frozen-20261001` | `e680b1430db6efd8dc601d5585ee207e5f9b42a4` | 仅本地；目标原历史触发同一容量限制，未推送 |
| `research/r4-hws-low-level-frozen-20261006` | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 已在远端；R1为本轮前已存在，R3/R4为用户本轮新增 |
| `research/temporal-mpc-frozen-20261004` | `04291a410f193c009e043af88e014cf420e1f68b` | 已在远端；R1为本轮前已存在，R3/R4为用户本轮新增 |

截至本次核实，远端为73个分支、3个tag。初始56个分支、1个tag新增17个分支与2个tag，原有refs保持；main仍为 `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。没有远端分叉/冲突，实际错误是容量hook拒绝。独立远端快照为 `remote_after_user_push.txt`，机器记录为 `human_upload_verification.json`、`human_push_results.json` 和 `server_quota_rejection.json`。

## 尚仅在本地的范围

- 剩余16个研究分支的最新原始Git历史与1个R2冻结tag没有上传：被拒绝的codex分支加上其后15个未尝试分支。逐项见表/branches.csv/push_plan.json；部分祖先和小源码/报告快照已经在远端，不能称全部历史已上传。
- 较大原始CA capture、后期native完整候选二进制及重复SDK/编译身份等未接受的大历史仍保留本地。原文件与38个R1/R2无损包没有因方案Stop、容量拒绝或归档筛选而删除。
- primary observed-surface九文件WIP与相关原始取证数据的快照已随归档上传，原工作区未提交改动保持。R4 rotation analyze.py dirty独立价值不明确，仍仅保留原工作区，不自动上传。
- core、Docker镜像、可再生build/install/log/cache/完整上游clone不纳入新包；所有原件保留。三个原worktree的docs/experiments/src忽略文件已额外盘点，均为Python缓存或CMake构建记录，不纳入新包。
- 本次上传后的索引、状态与配额报告更新仅提交本地；远端 `ac197e4e` 中的数据已保存，但其上传状态文字是上传前记录，尚未包含本次更新。

## 原资产保护与研究结论

`preservation_after.json` 再次确认全部33个原本地分支tip、旧tag、3个现存原工作区HEAD/status/dirty内容以及main旧prunable登记保持。main本地仍为 `2849cbe4`；只更新独立归档分支的文档，没有运行研究实验、编译、回归/sanitizer，没有更改冻结算法或导航配置。

R4结论保持：A25开放S2中调优STVL+MPPI安全—效率支配当前R4；A26 R4 5/5安全gate通过、零contact，但仅2/5完整goal成功，baseline 0/5成功/5/5contact。保留WAIT→GO信号，仍Stop当前low-level生产化路线，不概括预测无用或宣称R4整体优于MPPI。

本轮代理停止远端写入。用户随后授权选择调整Gitee配额、完整保存原历史，已准备[容量处理与手动续传方案](quota_resolution.md)、18项固定SHA清单及工具。配额生效后由用户手动认证执行；在远端核实前仍不将剩余历史标注为已上传。没有重写历史、迁移LFS、删除证据或代理重试。
