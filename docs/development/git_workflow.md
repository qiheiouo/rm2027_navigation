# Git 工作流与研究资产

本页承接原 development_workflow 的 Git 规则和 main 的研究资产原则。
实现取舍见[日常开发原则](development_principles.md)，实验与验证见[研究与验证](research_and_validation.md)，
文档/开源引入规则见[文档策略](documentation_policy.md)。

## 分支与提交

- 操作前确认 branch、worktree 和 status；保留无关用户修改。
- main 保持可构建，保护已接受功能；新旧车事实按各自长期分支维护。
- 通常按目的使用 feature/*、fix/*、experiment/*、integration/*、docs/*；Codex 工具新建分支默认用 codex/ 前缀，用户指定优先。
- 一个研究方向一般只维护少量重要实验分支，中间 audit/test/evidence 状态用 commit，不永久创建远端 branch。
- 一次 commit 一个有意义的逻辑变化，提交前看近期 log、staged diff 和 status。
- 默认一行简短中文标题，不用长篇 body 或大量 commit 制造工作量。
- 只提交任务相关内容，不混入无关格式化、generated logs、build/install/log 或 cache。

示例：

```text
文档：整理开发规范与文档结构
修复：统一地图到里程计变换所有权
实验：验证TinyMPC急转弯性能
```

## 历史与上传

不默认 force push、rebase 已共享历史、rewrite old history、为统一语言修改旧 commit、
大量创建 tag，或把所有本地分支上传远端；禁止默认执行 `git push --all`。
不自动 push/merge；获得明确授权后只操作确认的目标引用。

有长期价值的代码和唯一证据应形成可恢复 checkpoint，并提出备份需求；这不是自动上传授权。
冻结路线保留少量代表状态，tag 仅用于重要里程碑；不要求每个实验都建 tag 或新的远端入口。

## Worktree 与研究结束

Worktree 必须有明确用途，不充当无限增长的归档入口。
研究成功时只提取真正需要的修改进入干净集成分支，不整体 merge 长期失败实验和 evidence 历史。
失败或暂停时，更新已有路线总结，保留关键代码状态和必要证据，不继续无目的扩充回归。

长期最低资产为：**总结 + 关键代码状态 + 最小关键证据**。
保留解释结论、恢复关键实现和复核决定性结果的信息，不保留全部中间过程。
既有冷备份保持可恢复，不因每次小实验重做完整归档。

清理无价值临时 worktree 前，确认没有唯一 dirty/untracked 数据且有明确授权。
不自动删除用户 branch/worktree，不为“Git 好看”清理或改写历史。

## 大文件

bag、大量日志、build/install/log、大型仿真输出、原始点云、临时录屏和重复数据集不得直接进入普通 Git。
长期必要的大资产按需求用 artifact store、Release、Git LFS 或冷备份，仓库只放索引和必要元数据。
只有小型代表性证据且确有长期价值时才随源码保存。
已有原始证据不因本规范自动删除；删除前确认可恢复性与任务授权。
