# 项目开发规范

本文件适用于整个仓库。所有开发者和 Codex 代理在开始开发任务前，必须阅读并遵守：

- [开发流程](docs/development_workflow.md)：分支、提交、文档同步和开源模块引入规则。
- [实验分支与主线开发分级验证原则](docs/research_integration_deployment_principles.md)：Research / Integration / Deployment 的阶段划分、验证范围和决策要求，作为长期强制规范执行。
- [研究资产保留与 Git 仓库管理规范](docs/research_asset_and_git_management_principles.md)：精选长期资产、少量研究入口、简短中文提交及按价值上传，作为长期规范执行。
- [项目结构与文档索引](docs/project_structure_and_documentation_index.md)：模块职责、权威文档和文档适用顺序。
- 与当前修改相关的 `docs/contracts/` 契约及 `docs/runtime_profiles.md` 运行边界。

开始较大开发任务时，先判断当前阶段。未明确说明的新实验性算法默认属于 Research；先定义核心假设、稳定 baseline 和最小可判定实验，按结果决定 Go / Modify / Stop。未经明确要求，不得自动将 Research 提升到 Deployment 标准，也不得用测试数量、文档数量或工程完整性替代效果结论。

准备进入 `main`、实车或比赛配置时，按分级验证原则完成相应集成和部署验证。分级验证不取消稳定主线保护、公共接口所有权或实际运行安全边界。

用户当前任务中的明确要求优先于上述规范。修改时保留工作区内与当前任务无关的已有改动。
