# 冻结周期的原生 CostCritic 碰撞判据与 batch 可行数

状态：**在同一冻结输入下，300 条已经含动态安全且原局部图放行的 rollout；扩大 batch 增加候选数，但 V1 共同首步评分并列仍然存在。** 这是[采样空间对照](batch_sampling_probe_20260926.md)的局部图判据补全，不是完整 MPPI 回放或闭环验收。

## 原生判据一致性

[输入导出器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_fixture.py)只取周期 162 的持锁 raw local costmap、padded 八边形及捕获/新采样的 30 步位姿。[隔离 C++ 工具](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_probe_cpp/src/costmap_mask_probe.cpp)链接镜像中已安装的 Nav2 `FootprintCollisionChecker`，并沿用 CostCritic 的中心点成本检查、局部 inflation layer 在外接半径处的快捷阈值与未知空间判据。当前冻结配置算得快捷阈值为 **203**。先复算原始 300 条：**44 条 CostCritic 碰撞，工具也为 44 条，逐条 mismatch 0/300**。若此对齐失败，审计脚本拒绝报告新增 batch 的联合可行数。

工具在 ID `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6` 的隔离镜像中以 `--parallel-workers 1`、`MAKEFLAGS=-j1`、`CMAKE_BUILD_PARALLEL_LEVEL=1` 构建。镜像及可执行文件、源码、导出器、采样证据和原始周期的摘要在[结果](evidence/costmap_mask_probe_20260926/summary.json)。该工具只复刻**碰撞标记**，没有重算 CostCritic 连续代价、其它 critic、softmax、滤波或命令，也没有增加运行时依赖或替换正式控制器。

## 四组固定随机前缀

每个种子只生成一次 2000 条，四列 batch 取同一数组前缀。以下为 30 步均满足原 0.05 m 动态本体间隙、padded 足迹无动态相交、且原生局部图检查器无碰撞的候选数：

| batch | 种子 0 | 种子 1 | 种子 2 | 种子 3 |
| ---: | ---: | ---: | ---: | ---: |
| 300 | 10 | 13 | 15 | 7 |
| 600 | 24 | 20 | 24 | 18 |
| 1000 | 40 | 40 | 34 | 38 |
| 2000 | 93 | 89 | 82 | 79 |

原实际控制周期是 **16/300**，四组独立高斯种子的 300 前缀均非零。增加 batch 提高可行候选绝对数；这里每组都是同一个固定周期的不同噪声样本，不代表独立闭环成功率。新采样使用与 Nav2 相同的分布和 Omni 积分，但不是其内部同一串随机数。未来 Gazebo 轨迹只用于离线间隙标签。

原 V1 在这个周期的固定首步就对所有轨迹给出同一个硬碰撞项，[逐 critic 排序分解](critic_rank_decomposition_20260926.md)表明路径项在已知窗口通常反向偏好动态不安全轨迹。因此此处可明确排除“约 300 条根本采不到安全控制候选”作为周期 162 的主解释；**仅增加 batch 不能修正已证实的 V1 排序信息缺失**。尚未测得 600/1000/2000 的全部 critic 得分、最终控制及原生完整控制周期耗时，也不能从可行候选数推断这些输出会更安全。

## 复算与边界

1. 在工作分支的 Docker 镜像中对 `costmap_mask_probe_cpp` 单任务构建；不使用旧已构建插件得出新结论。
2. 用导出器生成 `captured.bin` 和四个种子的冻结位姿文件，再在同一镜像中逐个运行 `costmap_mask_probe`。
3. 用[审计器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_audit.py)核对捕获 300 行 mask 完全一致，并计算四组交集；生成结果为上述 JSON。

当前证据足以决定先研究预测评分与滤波后控制对齐，不支持改大默认 batch 或宣布 V1 可部署。完整本机性能上限还需同输入的原生 MPPI 全 critic 回放，随后才能讨论更大 batch 的控制与耗时。
