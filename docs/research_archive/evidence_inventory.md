# 证据保存范围

| 类别 | 内容 | 完整性 / 限制 | 保存位置 |
| --- | --- | --- | --- |
| 完整原始分析记录 | R4 A23/24/25/26的140个运行目录，所有控制/预测/真值/事件/plan、首异常与R4自身失败ring、实际参数和scene | 收集所有研究数据流，保留失败与校准；不含可再生plugin_build/research_install和重复ROS日志目录，逐项排除在清单可查 | `evidence_manifest.json`中的五个comparison组，`data/objects/*.gz` |
| 原始观测与bag | A13/A16 shadow 的ROS bag、source-stamp预测、odom/plan、输入与停止诊断，其他A01–22数值记录 | 原先固定yaw失效、world静态语义、32-bit clock失败及修正均保留；部分纯工程目录仅有诊断记录，其源码已在原分支 | 同manifest的shadow/numerics/world_xy/input-applicability等组 |
| 精简统计与论文图 | R4正式summary/trials、fairness、净空/WAIT/arrival图与failure_classifications | 既有Git小证据 + 原始组中的精确副本；统计不冒充原始输入；副本按内容去重 | R4分支 `experiments/r4_*/evidence*`；新manifest |
| 完整既有无损证据 | R3九组核心实验、15份manifest及源码/binary身份、原始JSONL压缩、实际DDS/CDR/故障记录 | 1,265项文件校验无缺失或不符；见原manifest的scope，不把早期debug有限来源写成完美可复现 | R3原分支/tag，`docs/dynamic_navigation/evidence/temporal_mpc*` |
| 完整既有Git历史 | R1/R2源码、合同、脚本、失败输入、native候选与聚合证据，38个冻结实验包及stage2数据 | 部分历史单文件超过100MiB、总新增blob约2GiB；逐分支远端结果单列，不能以本地存在替代云端成功 | 原分支/tag；清单与push_report |
| 未提交WIP | primary observed-surface 9个源码/工具/文档文件、tracked diff、状态/原基点 | 原工作区不动；精确备份，未本轮重跑测试、未部署接受 | `wip/observed_surface_members/` |
| 取证原始记录 | surface/member_ros、budget/corruption、actual-sdk重执行与range标签流/summary | 文本、配置、对照结果无损保存；113MB级SDK输入bin仍留本地，已有等价冻结输入优先用原Git证据 | manifest `surface/*`；原本地`build/dynamic_surface_recovery` |
| 仅留本地，价值不明确 | R4 rotation analyze.py dirty、部分未知扩展/缓存/历史构建目录 | 不自动提交或删除；保护校验记录原状态 | 原worktree；`worktree_state_before.json`、exclusions |
| 仅留本地，可再生或不应上传 | build/install/log缓存、.o/库、SDK/devspace、上游clone、Docker镜像、795MB core | 没有清理或上传；新包不含完整第三方源码clone，来源与许可沿用原intake记录 | 原目录；ignored inventory/exclusions |

原A26首异常终止的10次与最终原Nav2恢复的10次是两个独立cohort；均包含原首次native异常证据，不能混合成功率。A25的20次校准与授权后的10次正式支配验证同样分开。新归档只是保存和去重，没有更改任何历史失败分类。

31个历史分支的小型源码/结论/配置快照见 `historical_snapshots.json.gz`，恢复工具为 `tools/restore_historical_snapshot.py <branch> <destination>`。每个文件保留Git blob、SHA256与commit来源，只复制选定的≤1MiB文件；未保存完整树和大二进制，遗漏可查询对应gzip清单。用户手动认证后，新增数据包及所有小快照均已随归档 `ac197e4e` 上传。部分R2完整Git历史与大输入因容量限制仍只在本地；不能用小快照冒充完整原始记录。

三个原worktree的docs/experiments/src目录中Git忽略文件额外盘点见 `ignored_outside_build.json`；均为缓存或CMake构建记录，未纳入数据包。

上传后真实refs与未上传范围见 `human_upload_verification.json`、`branches.csv` 和 `push_report.md`；原首次认证失败记录保留，不覆盖历史。
