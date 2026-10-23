# 文档策略

优先更新已有权威文档。每份文档只承担一个主要职责，同一事实只保留一个维护入口。
执行优先级见根目录 [AGENTS.md](../../AGENTS.md)；查找入口见[文档地图](../README.md)。

## 三类文档

| 类别 | 职责 | 更新方式 |
| --- | --- | --- |
| Current | 当前分支的 architecture、contracts、runtime、operations | 随相关代码修改同步；明确车型和实验开关 |
| Research / Validation | 一次研究、实验或验证的条件、结果与限制 | 可带日期；不自动代表当前系统状态 |
| Archive | 历史过程、Phase 里程碑、冻结阅读副本 | 保留知识；不得作为当前接口或运行方式权威 |

当前事实按 branch/profile 理解，不能把旧车或研究配置写成全项目统一事实。
发现文档与源码冲突，说明差异；不要依据历史报告推断新的运行协议或验收结论。
公共接口变化更新对应 contract；启动/配置行为变化更新 operations；局部用法更新 package README。
只改受影响的权威文档，不要求每次同时修改索引、报告和所有 README。

## 不制造文档负担

- 不为每次 Codex 任务或微小改动新建 Markdown。
- 不重复生成 summary + report + final report + freeze report + closeout report。
- 不要求所有任务填写模板、checklist、报告或同一测试矩阵。
- 研究里程碑或需要未来复现的结论才值得单独记录；优先追加到已有路线文档。
- 新目录保持浅层；普通入口约 2～3 层，历史 Phase 分组和原始证据布局允许更深。
- 移动文档用 `git mv`，修复活动文档和 README 的相对链接及正文路径引用。

## 本分支分类与保留例外

`phase1*`、部分 `phase2*` 和 `phase3*` 的历史验收过程进入 `archive/phases/`。
仍承担当前职责的内容直接迁为正式文档：地图 bundle 到 contracts，任务结构到 architecture，
地图部署、建图、重定位、串口 dry-run、双雷达及比赛 bringup 到 operations。
这些页保留原 Phase 来源和历史验收章节，不重复生成一套历史副本。
旧车早期计划/行为完成表进入 `archive/old_car/`，带日期的现场证据集中在 `validation/old_car/`。

根目录保留两处既有目录作为路径例外：

- `dynamic_obstacle_critic/`：研究登记、17,000 余个既有证据/源码快照，以及未提交的用户实验文档；不属于正式运行入口。
- `dynamic_navigation/`：研究总结、历史原文阅读副本、来源清单和复算工具；历史原文不改写。

保留原路径以免破坏证据清单、复算路径和用户进行中的修改；不在文档治理中重组原始证据。
副本内引用原始上下文的链接可能在本 checkout 不存在，按其来源版本恢复，不伪造资产。
本轮未提交/忽略的 `tdt_migration/` 残留也保持原样，不纳入文档入口或 Git 资产。
未来新研究使用 research/ 或 validation/，不继续扩展根目录例外。

## 外部依赖与开源模块

仅对真正引入系统的模块，在 [external](../external/) 记录：upstream URL、branch/tag/commit、license、
引入范围、本地修改、adapter boundary、引入理由、替换或移除方式。
版本信息在该记录维护一次；不要在多份报告重复维护同一 pin。
只读调研记录在 research，不因调研就 vendor、添加 submodule、fork 或重复制作 upstream snapshot。
保留现有许可证、来源和本地修改记录；不为“架构完整”引入外部模块。

## 长期分支差异与同步

2026-10-23 仅检查本地已有 refs，未 fetch，不代表远端此刻状态。
此次只在 `experiment/dynamic-documents-archive-20261004` 整理，未修改其他 worktree 或分支。

| 已检查引用 | 文档治理差异 | 车型/运行事实边界 |
| --- | --- | --- |
| `main`（fcd9a05b） | 有旧 AGENTS 及两份长研究/Git 规范，索引仍平铺 | 公共历史基线及工作区/研究索引，不等同最新部署车型 |
| `main-new-car`（bd6cf689） | 无 AGENTS 和上述两份规范，开发流程仍要求英文提交 | Competition V2、单云台 MID360、航向融合、机构状态、狗洞与专属仿真 |
| `origin/main-old-car`（82017945） | 本地无 main-old-car 分支；该远端跟踪引用同样缺少规范 | 旧串口、旧车外参/AMCL、fresh03、现场操作、HWS/STVL 与运行加固记录 |

三处分支的 contracts、runtime 和能力表不同；新旧车文档还写着 main 与 old-car 同步，
而当前本地 refs 已不同。存在将不同时间/车型的状态当成三份全局“真相”的风险。
这些差异不能靠相互 merge 文档消除，也不能用当前研究分支覆盖任一部署主线。

后续建议：

- 跨分支同步 AGENTS 的通用行为与本目录四份规范；移植时保留各自 worktree 角色，删改本分支特有的冷归档说明。
- 在各长期分支分别迁移自己的入口、Phase 和链接；main 的两份旧规范由短版替代，不并存维护。
- old-car 的现场 SOP、地图、AMCL、旧协议及验证结果继续由 old-car 维护。
- new-car 的 V2、heading/gimbal、姿态/狗洞合同及验证结果继续由 new-car 维护。
- 当前 branch 独有的失败研究、历史原文和大批 evidence 不整体带入 main 或车型分支。
- 结构可共用，系统事实必须就地审查；提交/合并/上传仍由用户另行决定。

研究规范来自 main 的 `research_integration_deployment_principles.md`，资产规则来自
`research_asset_and_git_management_principles.md`；本分支原先缺失这两页。
本次提炼到 research_and_validation 与 git_workflow，并吸收原 development_workflow 的 Git、
文档同步、开源引入规则；不额外保留重叠规范或原先“未来目录”建议。
