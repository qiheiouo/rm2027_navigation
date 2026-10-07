# 本地工作区与主线说明

审计日期：2026-10-07。仅管理工作区、恢复引用和可重建缓存；不修改导航源码、研究输入、冻结结论或 Git 历史，不上传远端。

## 三条线的实际含义

只读远端检查 `git ls-remote --symref origin HEAD` 确认默认入口仍为 `main`。

| ref | 本次核查版本 | 用途 |
| --- | --- | --- |
| `origin/main` | `b8e965c02c0c2038da6e51fbef92135bdf9d8396` | 远端默认入口、公共基线与项目规范 |
| 本地 `main` 治理前 | `dfd638879081ee6d8659d1ddc8dac0383b80d679` | 已增加研究资产与 Git 规范；本次维护在此继续 |
| `origin/main-old-car` | `820179457b70789b0f46697758dbe3d947ec1d4a` | 老车开发 / 部署主线 |
| `origin/main-new-car` | `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496` | 新车开发 / 部署候选主线，真实硬件仍待验收 |

源码依据：`bd6cf689:docs/branch_mainlines.md`。其“双车型部署主线”规则仍有意义，但“main 与 old-car 指向同一基线”是历史快照；三者现在共同祖先为 `d735ee12`，已经分叉。不能仅因 main 是默认分支，就把它当成最新新车实现。

**后续以新车为目标的源码研究基于 main-new-car；main 保持清晰的默认 / 公共入口。** 本次没有切换远端默认分支、提升新车验收状态、整体合并车型或把冻结研究合入 main。若要统一成单一新车工程主线，需要独立 Integration 审计，显式迁移公共规范及需要的能力。

## 日常与研究入口

| 路径 | 分支 / 状态 | 用途 |
| --- | --- | --- |
| `/home/qihei/rm2027_navigation_main` | `main`，维护完成后 clean | 长期默认 / 公共开发目录；不包含历史 1.7 万 evidence 文档 |
| `/home/qihei/rm2027_navigation/build/neu_localization_baseline` | `experiment/neu-localization-baseline` | 活跃新车 L1；源码固定 `bd6cf689`，治理前检查点 `9e3b0bd5`；不移动、不清理、不合入 main |
| `/home/qihei/.codex/worktrees/temporal-wait-go-replan/rm2027_navigation` | `experiment/temporal-wait-go-replan` @ `fac071b2` | 另一项用户研究入口；保留，不纳入本次清理或归档 |

main 本次治理前 docs 跟踪数为 60；新增加的本说明是小型治理文档。新车源码边界与日常目录边界分别记录，不能混淆。

## 冷归档：仍保留

| 路径 | branch | HEAD | 保留原因 |
| --- | --- | --- | --- |
| `/home/qihei/rm2027_navigation` | `experiment/dynamic-documents-archive-20261004` | `9e08122a` | 17,802 个跟踪 docs 不删除；9 个 surface-member WIP 原样保留，已有归档副本 |
| `/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption` | `experiment/r4-hws-prediction-consumption` | `bdb8b8d0` | 1 个 dirty 分析脚本及约 1.03 GB ignored 内容；不删除 |
| `/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004` | `experiment/temporal-mpc-main-20261004` | `04291a41` | Git clean，但约 1.86 GB ignored 研究输出尚未逐项证明恢复副本；不删除 |

旧主目录另有本地忽略的 `WORKTREE_ROLE.md`，明确标记冷归档。它仍是共享 Git 对象库和若干子工作树的容器，不能直接删除整个目录。

## 已移除的已完成归档工作树

下列三个工作树逐项检查为 clean；全部 ignored 文件只是可重建 Python `.pyc`。移除前已记录 HEAD，保留对应本地 branch，不删除 Git 对象。

| path | 保留的恢复 branch | HEAD |
| --- | --- | --- |
| `/home/qihei/rm2027_navigation/build/research_archive_20261006` | `archive/dynamic-navigation-evidence-20261006` | `8a46ffc87fbcee7064934f0d335f62933f77c28e` |
| `/home/qihei/rm2027_navigation/build/research_archive_github_20261006` | `codex/github-research-archive-20261006` | `e65a2f35ba5c92c064598ad5200bbcb4e6f728cf` |
| `/home/qihei/rm2027_navigation/build/research_archive_github_resume_20261006` | `codex/github-archive-resume-20261006` | `41bd460eea6edfe1f2777dea8853b7a6e559d84c` |

原私有归档已完成独立远端恢复，详见旧主目录 `build/research_archive_audit_20261006/final_archive_status_20261006.json` 和 [私有归档 Release](https://github.com/qiheiouo/rm2027_navigation_research_archive/releases/tag/research-archive-history-20261006-e65a2f35ba5c)。原审计 / 工具树仍可用保存的 branch 重新创建 worktree：8a46ffc8、e65a2f35 已在历史包，41bd460e 续传工具另保存在归档小型索引中。本次不重新下载、打包或运行归档验证。

`/tmp/rm2027-main-research-summary-20261007` 已不存在，仅清除其 prunable 登记；main 提交完整保留并在长期目录检出。旧的 development-policy 临时登记本次未出现，不虚报删除。

## 历史本地 pointer

以下 18 个 branch 的 HEAD 均是保留的冷归档 `9e08122a` 的祖先；原归档目标清单逐个匹配相同 SHA。它们没有独有提交、不属于任何 worktree，已用普通 `git branch -d` 删除本地名字。恢复时可直接从下表 SHA 建立 branch；对应历史仍由冷归档分支和私有包保护。

| 已安全移除的本地 branch | 原 HEAD |
| --- | --- |
| `experiment/cv-support-bound-contract` | `ca3a50dcd50bd5059b686f59f00cc50941f5a20c` |
| `experiment/cv-support-factor-audit` | `b17b73a5d887c78945e5db89b95cb7e866ac85eb` |
| `experiment/dynamic-consumption-evidence` | `180a773cd000a338b99ea43841dbb0943af500f6` |
| `experiment/hws-migration-audit-20261003` | `652f14cea66edc0fcd77401de7db41cb6ffa7477` |
| `experiment/native-cost-accumulator-evidence` | `41e9639936718a46a9b3aecaba51981b2805c4f3` |
| `experiment/native-cost-map-contract` | `3ce395889f565b786f980929ad64b6edd4fe8ac2` |
| `experiment/native-cycle-evidence` | `a12688b2c35b86ebf5ea63dcc86bdc753918f551` |
| `experiment/native-factor-aggregation` | `7c0db1b528ca01624be8ae6ce43ee80eb57ed25a` |
| `experiment/native-factor-safe-mass` | `60e485bdb2ba73b67da5cce50830d6273c4251e8` |
| `experiment/native-individual-stage-contract` | `d688f790f7d8d3223d61a14537de4037146f4c92` |
| `experiment/native-raw-map-contract` | `2a67835d59a3310773091db95f864ddc60c6faa9` |
| `experiment/native-safe-control-witness` | `b89eb1d0ff4cad3ef25d71a64c7d0588e63d985e` |
| `experiment/native-sg-stage-audit` | `ced13e8ae13a90e88c22e08fb9b67169acfa015e` |
| `experiment/native-weight-attribution` | `fea7bf914e42361a3dda0053145e7077687b0a3e` |
| `experiment/observation-anchor-cv` | `bc447533ab4a847059e7ca65c54bf9267236a941` |
| `experiment/observation-diameter-consumption` | `e061a245e3d760f27705af1a1125774c84298c34` |
| `experiment/processed-cv-objective-audit` | `8523d1f47fbb86e6e976f371e43778addcf40eee` |
| `experiment/raw-rollout-objective-audit` | `1edb4ee42c1a0ae9b4132d994e5220c92f838469` |

保留 main、活跃 L1、另一项用户研究、R3/R4 冻结 branch/tag、三个归档恢复 branch、冷归档 branch 以及未在上述白名单中的其余 pointer。没有远端 branch/tag 删除，也没有为了数量进行批量 force 删除。

## 大文件与数据安全

- 曾记录的 RViz2 `core` 为 795,242,496 bytes；本次在旧主目录及已登记工作树未找到该文件，不能声称本次删除或释放这部分空间。
- main `.gitignore` 新增 `core` / `core.*`，避免以后崩溃转储进入日常状态；未改变历史提交。
- 旧主目录 `.vscode/browse.vc.db` 为 1,282,048,000-byte SQLite C++ 浏览索引；无进程占用，属于源文件自动生成缓存，已单独删除，回收约 1.28 GB。研究原始数据、地图、bag 和证据均不因此删除。
- surface-member 9 个 WIP 文件逐个与 `8a46ffc8` 归档内副本比较，字节一致；原文件保持原样。
- R4 dirty `experiments/r4_rotation_value/analyze.py` 已建立恢复引用 `refs/stash` @ `9601e91019b8556caa04de7f732d421ce5df28c8`。只是保存快照，未 stash 掉或改动工作区文件。未来 stash 栈可能变化，应按该 SHA 查询；本轮不删除该恢复引用。
- L1 未提交工具与结果先形成本地 Git 检查点 `9e3b0bd5`；仅保留研究进度，native 核对和正式结论仍由 L1 继续完成，不因治理预判 Go / Modify / Stop。
- **仍存在未逐项确认 Git / archive 恢复副本的 ignored 研究数据**：旧主目录 build、R3、R4 等。按潜在唯一有价值数据处理，全部保留；不宣称整个磁盘已无唯一内容，也不继续清理这些目录。

本地治理不会使所有旧工作树都变 clean。目标是日常入口清晰且 clean，同时保护尚未完成价值审计的内容。
