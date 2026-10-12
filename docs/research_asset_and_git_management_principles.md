# RM2027 研究资产保留与 Git 仓库管理规范

## 1. 目的

本规范用于统一 RM2027 导航项目后续的：

- 研究资产保留；
- Git 提交；
- 分支管理；
- 研究冻结；
- 云端上传；
- 归档；
- 仓库清理；
- 历史研究复用。

本规范建立在现有：

> Research → Integration → Deployment

分级开发原则之上。

原有开发原则保持不变。

本规范重点解决另一个问题：

> **并不是所有开发过程和研究资产都值得永久保存。**

项目后续应从：

> “尽可能保存所有过程”

逐步转向：

> **“保存真正有长期价值的知识、代码和证据。”**

核心原则：

> **保留知识，而不是无限保留过程。**

---

# 2. 研究资产的长期价值分级

研究资产按照长期价值分为三类。

---

## 2.1 必须长期保留

包括：

### 研究结论

每个重要研究方向应有最终总结，至少说明：

- 为什么开展；
- baseline 是什么；
- 尝试了什么；
- 关键实验结果；
- 成功点；
- 失败点；
- 最终 Go / Modify / Stop 决策；
- 对正式工程的影响；
- 后续如果重新研究，应从哪里开始。

例如：

- R1 Dynamic Prediction；
- R2 DynamicObstacleCritic；
- R3 Temporal MPC；
- R4 HWS Prediction Consumption；
- STVL + MPPI baseline；
- 定位 / 重定位；
- TF 架构；
- 高速旋转定位问题；
- 其他真正改变项目路线的重要研究。

---

### 关键冻结代码

对于重要路线，应保留少量具有代表性的最终状态。

例如：

```text
R1 final
R3 Temporal MPC frozen
R4 low-level frozen
重要 baseline
```

不要求保存每一个中间实验状态。

---

### 关键配置

例如：

- 最终 baseline 参数；
- 关键实验使用的 controller 配置；
- STVL / MPPI 配置；
- MPC 最终实验配置；
- 重要 launch / yaml；
- 复现实验必须使用的环境参数。

---

### 决定研究结论的关键证据

例如：

- 最终成功率；
- collision / contact；
- min clearance；
- latency；
- goal completion；
- representative trajectory；
- 极少量具有代表性的日志或数据。

保存目标是：

> 未来能够理解和必要时复核结论。

而不是：

> 保存研究过程中产生的全部数据。

---

# 3. 建议保留，但不要求保存全部历史

以下资产可以根据实际价值保留：

- 高价值实验脚本；
- 数据分析脚本；
- benchmark 工具；
- replay 工具；
- 特殊仿真 world；
- 后续可能继续使用的 tracker；
- 具有通用价值的诊断工具；
- 少量典型失败案例。

如果某个工具未来很可能继续使用，则保留。

如果只是一次性审计，则不应因为“已经写出来了”而自动永久保存。

---

# 4. 通常不需要长期保留

以下资产默认不作为长期研究资产：

- build；
- install；
- log；
- cache；
- core dump；
- sanitizer 输出；
- 临时 benchmark 输出；
- 大量重复 CSV；
- 每轮实验的中间结果；
- 重复 hash 文件；
- 重复恢复证明；
- 一次性 debug 脚本；
- 临时 mock；
- 自动生成文件；
- 已被最终结论替代的 WIP；
- 多份内容高度重复的阶段报告。

尤其禁止因为“未来也许可能有用”而无条件保存所有内容。

---

# 5. 完整归档与永久归档不是同一概念

完整 bundle / Release / 全历史归档的主要意义是：

> 在重要阶段结束时确认“当前全部研究资产不会立即丢失”。

它属于：

> **灾难恢复级备份。**

但灾难恢复级备份不等于：

> 所有内容都必须永久存在。

例如已经完成的：

```text
2026-10-06 RM2027 完整研究归档
```

可以作为历史冷备份。

后续不需要继续以同样规模不断追加全部：

- refs；
- audit branches；
- evidence objects；
- WIP；
- 中间实验。

---

# 6. 建立精简永久研究档案

后续推荐建立：

> **Golden Archive / 黄金研究档案**

长期真正需要维护的是这一层。

建议结构：

```text
docs/research_history/
    README.md

    R1_dynamic_prediction.md
    R2_dynamic_obstacle_critic.md
    R3_temporal_mpc.md
    R4_hws_prediction.md

    localization_and_relocalization.md
    stvl_mppi_baseline.md
```

必要时：

```text
research_snapshots/
    r1_final/
    r3_final/
    r4_final/
```

以及：

```text
research_evidence/
    representative_results/
```

只保存：

- 最终结论；
- 关键代码；
- 关键配置；
- 少量关键证据。

---

# 7. 一份高质量总结文档优先于大量原始资产

当一个研究方向已经结束时，应优先制作：

> **一份真正能够解释完整研究过程和结论的总结。**

而不是保留几十个：

```text
audit/*
evidence/*
witness/*
contract/*
test/*
```

作为唯一的历史理解方式。

最终文档应该能够让未来开发者在不阅读几百个 commit 的情况下回答：

- 当时为什么做；
- 做到了什么；
- 哪里失败；
- 为什么停止；
- 什么值得复用；
- 如果重启研究，该从哪里开始。

---

# 8. 不采用“文档-only”极端方案

虽然不需要永久保留所有资产，但也不应只留一篇文档、删除所有代码和证据。

最低长期保存标准：

> **总结文档 + 关键冻结代码 + 最小关键证据。**

原因：

未来可能需要确认：

- 某个算法具体是如何实现的；
- 某个结果是否受参数影响；
- 某个实验到底用了什么模型；
- 某条路线是否值得重新启动；
- 论文 / 大创 / 毕设中的数据是否能够复核。

因此：

> 文档负责解释结论。  
> 代码负责恢复实现。  
> 少量证据负责支持结论。

---

# 9. Git 仓库职责

正式工程仓库首先服务于：

> **当前和未来开发。**

而不是承担无限增长的实验档案职责。

正式仓库应优先保持：

- main 清晰；
- 分支数量有限；
- worktree 数量可理解；
- 工作区干净；
- 当前真实状态明确；
- 历史研究可追溯但不干扰日常开发。

---

# 10. main 的职责

`main` 只保存：

- 当前正式工程代码；
- 已接受的功能；
- 当前正式配置；
- 正式长期维护工具；
- 必要工程文档；
- 最终研究结论 / 索引；
- 项目开发规范。

禁止因为“想保存历史”把失败研究整体 merge 进 main。

---

# 11. Research 分支原则

每个新的研究方向，原则上只建立一个主要实验分支。

例如：

```text
experiment/temporal-wait-go-replan
```

不要默认为每个小实验建立永久分支，例如：

```text
experiment/foo-audit
experiment/foo-test
experiment/foo-evidence
experiment/foo-witness
experiment/foo-contract
experiment/foo-v2
```

中间阶段：

> 用 commit 保存即可。

不需要每个 commit 都长期保留 branch pointer。

---

# 12. 研究成功后的处理

如果 Research 证明方案值得继续：

从最新 `main` 建立干净的 Integration 分支：

```text
integration/<topic>
```

只迁移真正需要的代码。

推荐：

- cherry-pick 少量明确 commit；
- 手工迁移核心实现；
- 基于最新 main 重构；
- 重新建立正式接口。

不要默认：

> 整体 merge 长研究历史。

尤其不要把数百个：

- audit；
- failed experiment；
- evidence；
- temporary tool；

全部带入正式工程历史。

---

# 13. 研究失败或冻结后的处理

如果研究最终 Stop：

应：

1. 写最终结论文档；
2. 保留最终冻结点；
3. 创建明确 frozen tag；
4. 确认远端存在；
5. 不再作为活跃开发分支维护。

例如：

```text
research/temporal-mpc-frozen-20261004
research/r4-hws-low-level-frozen-20261006
```

被冻结研究：

> 不再整体 merge main。

未来需要其中成果时：

> cherry-pick / 提取 / 重新实现。

---

# 14. Tag 的用途

Tag 用于保存：

- 冻结研究；
- 正式里程碑；
- 比赛版本；
- 重要 baseline；
- 可长期引用的历史状态。

Tag 不是活跃开发入口。

一个 frozen tag 可以保存整条历史，而不需要长期保留几十个远端实验 branch。

---

# 15. Git 上传原则

## 必须及时上传

以下内容不应长期只存在本机：

- main 的正式提交；
- 正在持续开发的重要研究分支；
- 关键冻结 tag；
- 重要 release；
- 长期有效的研究总结；
- 唯一且有价值的代码；
- 不可替代的实验结果。

---

## 可以暂时只留本地

以下情况允许：

- 刚开始的小型 Research；
- 生命周期很短的测试分支；
- 明确尚未决定是否有价值的 WIP；
- 临时 worktree；
- 短期分析脚本。

但：

> 一旦确认具有长期价值，应及时形成 checkpoint 并上传。

---

# 16. 禁止无选择地上传所有分支

禁止默认执行：

```bash
git push --all
```

尤其在本地存在大量：

```text
experiment/*
codex/*
audit/*
```

临时分支时。

上传前必须判断：

> 这个 ref 是否真的值得长期成为远端入口？

如果只是某个最终分支的祖先：

> 不需要单独上传该 branch。

---

# 17. 禁止为了归档创建大量远端 branch

例如：

```text
native-factor-aggregation
native-factor-safe-mass
native-cycle-evidence
cv-support-factor-audit
observation-anchor-cv
...
```

如果它们属于同一条研究历史：

只需要保存：

> 最终最高节点 / frozen tag。

Git 会保留祖先 commit。

不需要为每个历史阶段建立远端活跃 branch。

---

# 18. 大文件上传规则

以下文件不得直接进入普通 Git 历史：

- rosbag；
- 大型 PCD；
- 视频；
- core dump；
- 数据库；
- 大型二进制；
- Docker image；
- build tree；
- 大型模型权重。

确实需要长期保存时，根据情况使用：

- GitHub Release；
- Git LFS；
- 独立数据仓库；
- 冷备份；
- 其他对象存储。

不要直接 commit 后再删除。

因为：

> 从工作树删除并不会从 Git 历史中删除。

---

# 19. 工作区必须保持可解释

长期工作的主工作区应尽量保持：

```text
nothing to commit, working tree clean
```

不应长期存在：

- 不知道用途的 dirty 文件；
- 数周未处理的 untracked 源码；
- core dump；
- 不知道是否还能删除的临时内容。

未提交内容必须尽快被分类：

- 提交；
- checkpoint；
- 归档；
- 删除。

---

# 20. Worktree 管理

Worktree 是临时开发工具，不是永久归档工具。

研究结束后：

- 冻结 ref；
- 确认远端；
- 确认没有唯一 dirty 内容；
- 再删除 worktree。

不应长期同时维护大量：

```text
build/foo
build/bar
/tmp/foo
/tmp/bar
```

却不知道哪些仍然有效。

---

# 21. Commit 规范

## 21.1 所有新的 commit 默认使用简短中文

从本规范生效后：

> **所有正常项目开发 commit message 统一使用简短中文。**

保持与项目早期已有风格一致。

例如：

```text
功能：增加动态障碍等待状态
```

```text
修复：统一控制器里程计输入
```

```text
文档：补充动态避障研究结论
```

```text
测试：增加控制器切换检查
```

```text
重构：整理动态障碍跟踪接口
```

---

## 21.2 禁止默认生成长篇英文 commit message

除非确有外部工具、上游项目或第三方协作要求，否则不要提交：

```text
docs: add mandatory staged development validation policy
```

这类英文 commit 风格。

后续应改用：

```text
文档：新增分级开发验证原则
```

也不要生成：

- 多段 commit body；
- 过长背景描述；
- 大段实验结论；
- 自动生成式流水账；
- 类似 PR 描述的 commit message。

详细内容应该进入：

- 文档；
- issue；
- research report；

而不是 commit 标题。

---

## 21.3 Commit 标题应简洁

建议：

> 一行即可。

通常控制在：

> 10～30 个中文字符左右。

目标是：

```bash
git log --oneline
```

能够一眼看懂历史。

---

# 22. 推荐中文 commit 前缀

保持简单即可：

```text
功能：
修复：
文档：
测试：
重构：
配置：
构建：
维护：
实验：
归档：
```

不用强制 Conventional Commits 英文格式。

如果现有项目已经有更自然的中文风格：

> 优先保持项目原有风格。

---

# 23. AI / Codex 提交规则

Codex 在执行 Git commit 前必须：

1. 明确本次提交的真实作用；
2. 保证 commit 内容边界合理；
3. 使用简短中文标题；
4. 不自动写长篇英文 commit；
5. 不为每一个极小实验建立新 branch；
6. 不因为做了很多测试就拆出大量 evidence branch；
7. 不自动 push 所有分支；
8. 不自动创建大量长期远端 refs。

例如：

正确：

```text
实验：验证动态障碍等待绕行
```

不推荐：

```text
experiment(dynamic): validate temporal waiting and replanning behavior with additional evidence
```

---

# 24. Commit 应服务于开发，而不是证明工作量

禁止通过以下方式制造“进展感”：

- 一个实验拆十几个 commit；
- 每次保存数据都 commit；
- 每次 audit 都建 branch；
- 每跑一次测试就形成永久历史节点。

Commit 应对应：

> 一个清晰、值得回看的工程或研究状态变化。

---

# 25. 关于旧英文 commit

已经存在的英文 commit：

> 不需要修改。

禁止为了统一语言而：

- rebase；
- force push；
- 改写旧历史。

本规范只约束：

> 从现在开始的新 commit。

---

# 26. 关于旧仓库历史

现有：

- R1；
- R2；
- R3；
- R4；
- dynamic-differential-risk；
- native / critic；
- 大量 audit；
- 完整归档；

视为历史资产。

不要求为了“Git 看起来漂亮”重写。

原则：

> **旧历史保持可恢复即可。**

真正要优化的是：

> 后续开发方式。

---

# 27. 如果未来建立新仓库

允许未来建立新的干净工程仓库。

推荐方式：

> 以当前正式 main 为基础，选择性迁移真正有价值的内容。

旧仓库：

> 作为完整历史档案。

新仓库：

> 作为长期工程开发仓库。

不建议：

> 把所有旧研究 branch、所有 frozen history、所有 230+ commit 研究链全部重新迁入新仓库。

---

# 28. 新仓库迁移原则

如果执行迁移：

优先保留：

- 正式 main；
- 当前仍需要维护的功能；
- 必要工程历史；
- 研究总结文档；
- 极少数关键冻结代码；
- 当前继续开发的 research branch。

不迁移：

- 大量中间 audit branches；
- 已失败的一次性实验 branch；
- build / log / cache；
- core dump；
- 重复 evidence；
- 无长期价值的验证资产。

---

# 29. 旧完整归档的定位

已经建立的完整研究归档：

> 可以作为冷备份存在。

它不需要：

- 每天更新；
- 每次 commit 更新；
- 每个新实验都立即重做 2GB 级全量包。

只有在出现真正重要的阶段节点时，才考虑重新制作完整 archive。

例如：

- 比赛版本冻结；
- 大型研究阶段结束；
- 准备迁移仓库；
- 准备重大重构；
- 准备删除大量旧资产。

---

# 30. 以后归档优先使用“增量总结”

以后研究过程中优先维护：

```text
研究总结文档
+
关键代码 checkpoint
+
关键结果
```

而不是不断扩展大型历史包。

完整大归档属于：

> 低频操作。

---

# 31. 最终原则

项目今后采用：

> **代码服务于工程。**

> **实验服务于决策。**

> **Git 服务于开发历史。**

> **文档服务于知识传承。**

> **归档服务于灾难恢复。**

五者不要混为一谈。

尤其：

> **不因为能够保存，就意味着必须永久保存。**

> **不因为某个文件曾经对实验有用，就意味着它具有长期价值。**

> **不因为某个 branch 曾经存在，就意味着它应该永久存在于远端。**

---

# 32. 长期资产标准

判断一个资产是否值得永久保存时，优先问：

1. 它是否帮助理解最终结论？
2. 它是否包含无法从其他地方恢复的信息？
3. 它未来是否可能被复用？
4. 它是否是某项重要结果的唯一证据？
5. 删除它以后，未来是否真的会后悔？

如果以上问题基本都是否：

> 可以不保留。

---

# 33. 项目最终期望状态

理想状态：

```text
正式工程仓库
    main 清晰
    少量活跃分支
    工作区干净
    commit 简短中文
```

研究历史：

```text
少量 frozen tags
+
高质量研究总结
+
关键代码
+
关键证据
```

完整历史：

```text
必要时保留一份冷备份
```

而不是：

```text
几十个永久实验 branch
+
数百个 audit commit
+
大量重复 evidence
+
无期限 dirty worktree
```

---

# 34. 与既有开发规范的关系

此前已经确定的：

> Research → Integration → Deployment

分级开发验证原则保持不变。

仍然遵循：

> **Explore fast.**  
> **Integrate carefully.**  
> **Deploy defensively.**

本文件只补充：

> **Preserve selectively.**

即：

> **探索要快，集成要稳，部署要严，资产要精选。**

---

# 35. Codex 默认执行要求

今后 Codex 在本项目中默认遵守：

1. 新 commit 使用简短中文；
2. 不自动建立大量实验 branch；
3. 不自动 `git push --all`；
4. Research 中间状态优先用 commit，而非永久 branch；
5. 研究结束后优先总结、冻结，而非无限扩充证据；
6. 只有真正有长期价值的资产才进入永久归档；
7. build / install / log / core 等默认不进入 Git；
8. 冻结研究不整体 merge main；
9. 历史成果需要复用时，优先 cherry-pick / 提取 / 重实现；
10. 不因为归档任务重新启动大规模验证；
11. 不为了整理仓库而无必要地改写旧 Git 历史；
12. 任何删除动作前先确认已有远端或其他可恢复副本。

本规范作为后续 RM2027 项目长期 Git 与研究资产管理原则执行。
