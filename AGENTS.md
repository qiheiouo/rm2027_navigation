# 项目执行规则

适用于本仓库的开发者与 AI/Codex。先读本文件，再按任务读取相关权威文档。
本文件约束执行行为；系统知识从[文档地图](docs/README.md)进入。

## 优先级

发生冲突时依次采用：

1. 用户当前明确指令。
2. 本文件及适用的项目级执行规则。
3. 当前分支的 contracts / architecture / operations 事实。
4. development 详细规范。
5. 对应 package README。
6. research / validation 的实验与验证记录。
7. archive 及历史阅读副本。

文档中的“当前”只针对它所属的分支和车型。不得拿研究分支的合同覆盖部署分支。
代码与合同不一致时，报告差异并在任务相关范围内修正，不靠低优先级历史记录猜测。

## 开始工作

- 先确认目标、当前 branch、worktree 角色及工作区状态。
- 保护与任务无关的用户修改，不覆盖、不删除、不混入提交。
- 当前 checkout 是历史研究冷归档；用户明确授权前不在这里恢复冻结研究或开发新功能。
- 能从代码、文档和上下文可靠判断时直接执行；仅对会实质改变结果的关键歧义询问。
- 发现范围外问题时说明影响；不阻塞当前任务则不自动修复。

## 默认实现

- 采用最小充分实现，只修改与目标直接相关的范围。
- 不做 speculative engineering，不为未来可能的需求提前建设框架、抽象或兼容层。
- 不顺手重构相邻模块、统一风格或优化无关代码。
- 不默认添加多层 fallback、重复保护、冗余缓存或无现实故障依据的恢复状态机。
- 新增复杂保护必须能回答“它防止哪个现实故障？”。
- 对真实故障、外部不可信输入、硬件/通信/文件 IO、运动授权及部署要求保留必要保护。
- 不用更多 adapter 掩盖 owner 不清晰；TF、canonical odometry、motion authority、mission goal、serial transport 的 owner 必须明确。
- 参数差异优先用 YAML、launch 参数或 profile，不复制整套节点或近似配置。
- 参数的物理意义、单位、frame 和时间域必须明确；实验不悄悄覆盖正式 baseline。

详细边界见[日常开发原则](docs/development/development_principles.md)。

## 阶段与验证

使用 **Minimum Sufficient Validation**，不追求 Maximum Possible Validation。
优先采用用户或项目声明的阶段；仍不明确时说明假设并采用最低充分验证。

| 阶段 | 首要问题 | 默认验证 |
| --- | --- | --- |
| Research | 方案有没有价值？ | baseline、核心假设、最小可判定实验、关键指标 |
| Integration | 能否正确接入？ | 接口、owner、配置、上下游交互、核心回归 |
| Deployment | 能否正式使用？ | 按实际风险做系统、失败恢复、硬件和必要冻结验证 |

- 新算法探索默认 Research；未经要求不自动升级到 Deployment。
- 文档和其他低影响修改按影响面验证，不因目标分支叫 main 就运行整仓部署矩阵。
- 每次代码修改至少完成一次直接相关的有效验证；无法验证时明确原因与未验证项。
- 测试必须能发现有意义的错误；不以测试数量、代码量、报告数或 commit 数代表进展。
- 不默认运行全仓回归、所有 profile smoke、全套 sanitizer、Docker 重建或硬件验证。
- 不默认执行 SHA/hash 证明链、逐文件 checksum 或构建产物哈希扫描。
- 核心假设失败时先排除明显实现、配置和环境错误，做最小复核。
- 仍失败则给出证据和 Modify / Stop；失败是合法结论，不继续无目的工程化。
- 已停止路线不继续自动补测试、fallback 或兼容层。

阶段、测试价值和 hash 使用范围见[研究与验证](docs/development/research_and_validation.md)。

## ROS 与硬件边界

- TF/topic/action/service/message 语义、serial protocol、motion authority、chassis command、坐标系、timestamp/clock domain 改变时，同步对应 contract。
- 运行组合改变时同步 [runtime profiles](docs/operations/runtime_profiles.md)。
- 不制造第二个 canonical TF、odometry 或最终命令 owner。
- 不因调试便利绕过 motion authority 或改变 TF 让 RViz“看起来正确”。
- real motion 与真实串口必须显式 opt-in，尊重车型、固件、协议和现有授权边界。
- 未确认 firmware / serial contract 时，不推断实车协议。
- Simulation Go ≠ Hardware Go；offline replay Go ≠ field Go。
- build success ≠ functional acceptance；结论必须说明适用范围和未验证项。
- 普通软件任务不自动升级成 safety-critical Deployment。

接口事实见 [contracts](docs/contracts/)；命令和操作边界见 [operations](docs/operations/)。

## Git

- 操作前确认当前 branch、worktree 和 status；提交前查看近期 log 与最终 diff。
- 一次提交一个有意义的逻辑变化，默认使用一行简短中文标题。
- 只提交当前任务相关内容；不提交 generated logs、build、install、cache 或无关格式化。
- 不自动 push、merge、force push、改写共享历史或执行 `git push --all`。
- 不自动删除用户 branch/worktree；删除数据前确认无唯一有价值内容。
- 不为每个 audit/test/evidence 状态创建永久分支或 tag。
- worktree 有明确用途；结束后提出资产提取/保留和清理建议，不擅自删除。

分支、提交及研究资产边界见 [Git 工作流](docs/development/git_workflow.md)。

## 文档与外部模块

- 优先更新 existing authoritative document，同一事实只维护一个权威来源。
- 明确区分 Current、Research / Validation、Archive；历史记录不能作为当前接口权威。
- 不默认每次任务新建 Markdown、总结报告或成套 evidence archive。
- 不为同一任务重复生成 summary/report/freeze/closeout 文档。
- 保留长期知识、关键代码状态和最小关键证据；不无限累积过程资产。
- 引入实际系统依赖时记录来源、版本、license、本地修改和替换边界。
- 不因只读调研就 vendor、添加 submodule、fork 或重复 snapshot。

具体职责与跨分支同步见[文档策略](docs/development/documentation_policy.md)。

## 沟通与收尾

- 默认使用用户当前语言，及时报告会影响结果的关键发现、阻塞与重要假设。
- 完成时简述做了什么、验证了什么、结果及重要限制。
- 不为了显得完整而自动追加新的开发阶段、报告或验证矩阵。
