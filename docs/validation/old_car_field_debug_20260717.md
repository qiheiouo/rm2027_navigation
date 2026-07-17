# Old-Car Field Debug Competition 更新报告

日期：2026-07-17

仓库：`/home/wpie/rm2027_navigation`

分支：`feature/field-debug-competition-inputs`

HEAD：`f58f30ed7e53ad96fed3375c605be02422314206`

容器：`rm2027_navigation_humble`

> 整理说明（2026-07-17）：本报告正文冻结的是 `f58f30e` 时刻的现场
> checkpoint，因此第 6、7、9 节保留了归档当时的工作区描述。随后这些
> dirty 修改已在 `fix/field-debug-linux-consolidation` 上按主题拆成独立中文
> commit，并逐提交完成无硬件构建/测试；提交哈希、测试边界和持久化日志
> 清单见 [证据索引](old_car_field_debug_20260717_evidence.md)。本次整理没有
> 批准 candidate 地图，也没有新增 mapping quality 或定位人工确认功能。

## 1. 结论

真实旧车的主链路已经完成到以下边界：左 MID360、FAST-LIO、TF、candidate 地图部署、AMCL、唯一 `map->odom`、Nav2、Global/Local Costmap、真实串口底盘运动、synthetic referee、mission、home、patrol、hold、mission disable、遥控器手动接管和低血量回家均有实车或运行时证据。

这属于“Old-Car 软件候选和受控实车功能通过”，不是“比赛系统全部验收”。当前不能批准地图资产，也不能把 AMCL readiness 当成全局重定位正确性的证明。真实 referee 协议、右雷达/双雷达、真实比赛点位、最终外参和完整比赛环境仍未验收。

2026-07-17 环境中只有小障碍发生移动，墙面、桌子等主要结构未变。该变化不阻塞离线诊断，也通常不阻塞以主要结构为约束的 AMCL；它可能改变 Local Costmap，并在小物体靠近机器人、窄通道或墙边时干扰局部规划/匹配。正式重建静态地图前仍需清场。

## 2. 能力状态矩阵

| 能力 | 接口/骨架 | Linux 构建 | 模块 smoke | 集成/实车证据 | 剩余门槛 |
|---|---|---|---|---|---|
| 左 MID360 + FAST-LIO | 有 | 已通过既有构建 | 无硬件链已有记录 | 实际点云、里程计、静态/运动对齐通过 | 驱动版本和长期运行仍需比赛环境验收 |
| TF 所有权 | 有 | 已通过 | 已检查 | `lio_adapter` 为 `odom->base_link` owner；`map_odom_from_global_pose` 为 `map->odom` owner | 修改 TF/外参后需重验 |
| candidate 地图部署 | 有 hash/path/schema 门 | 已通过 | 部署通过 | 当前 `old_car_clean_20260715_field01` 可加载和导航 | 地图质量不通过，禁止 approved |
| AMCL 2D | 有 | 已通过 | fake 链已有记录 | 准确初值后点云/地图对齐并完成实车导航 | 错误初值可自信收敛到相同局部峰；不是 place recognition |
| Global/Local Costmap | 有 | 已通过 | 清除/动态障碍 smoke 通过 | 动态障碍出现与清除通过；真实导航通过 | PGM 脏点；inflation 视觉偏大，待清图后再调 |
| Nav2 规划与控制 | 有 | 已通过 | 无硬件 action/速度链已有记录 | 前向、侧向、斜向转角目标通过 | patrol 中出现一次 abort/retry；控制参数仍需赛场调优 |
| 真实串口底盘 | 有 | 已通过 | dry-run 记录存在 | `/dev/ttyACM0` 实际驱动车辆完成多种运动 | 仅当前旧车协议/限速边界；不等于真实 referee 验收 |
| Mission BT | 有 | BehaviorTree.CPP v3 已构建 | hold/service/action/cancel/retry 已测 | home、patrol、disable、低 HP 优先级通过 | 真实比赛点位、完整赛程和长时间稳定性 |
| 遥控器手动接管 | 有物理底盘门 | 不适用 | 不适用 | 用户切手动后车辆停止；随后 mission disabled/hold | ROS mission 在物理手动期间仍会继续，必须及时 disable |
| synthetic referee | 有 | 已通过 | gate/mock 已测 | 400 HP 到 100 HP 后 mission 进入 home | mock 参数是启动时读取；真实协议未验收 |
| 真实 referee | 接口有 | 软件边界已构建 | 未做真实输入 | 未验证 | 被真实协议与现场设备阻塞 |
| 右 MID360 / 双雷达 | 接口有 | 既有无硬件证据 | 本轮保持关闭 | 未做真实旧车验证 | 右侧外参、IMU 隔离与实车安全门 |
| pursuit | 接口有 | 既有无硬件证据 | 本轮保持关闭 | 未做真实目标链 | 上游目标协议/置信度/赛场验收 |

## 3. 实车验证摘要

### 3.1 普通导航

- 直线目标：完成真实运动。
- 侧向目标：完成约 0.47 m；存在有限角速度，用户确认现象正常。
- 斜向转角目标：完成约 0.45 m；用户确认现象正常。
- 动态障碍：Local Costmap 中出现并在移除后清除。
- 用户观察到 inflation 相对点云团偏大；按约定只记录，不在脏地图上调参。

### 3.2 Home 与 Patrol

- 运行时 home：`[0.441, 0.208, -0.030]`。
- patrol P1：`[0.409, 0.752, -0.051]`。
- patrol P2：`[0.811, 0.965, -0.029]`。
- 第一次 home action 成功，结束误差约 0.124 m，小于 0.15 m 容差。
- Patrol 已完成多轮 P1/P2 循环，未发现旧 goal 并发。一次早期 hold 测试脚本误判历史 action ID，导致多跑循环；这是临时测试脚本问题，不是产品代码。
- 第二次 home 遇到断电，用户观察车辆似乎到达 home，但软件 result 不完整；后续低 HP home 已提供第二次完整回家证据。

### 3.3 手动接管

Rosbag 状态链：

`disabled/hold -> enabled/patrol -> disabled/hold`

- 用户在 patrol 运动中切回手动，确认车辆物理停止。
- Mission 随后禁用，最终状态为 `holding`。
- `/cmd_vel` 最后一条为全零；最后非零样本后还有 15 条连续零速样本。
- 物理手动只阻断底盘执行，不自动停止 ROS mission；测试期间 ROS 仍继续 patrol，直到显式 disable。这是操作合同的重要安全要求。

### 3.4 低血量回家

Rosbag 状态链：

`400 HP + patrol -> referee freshness 空窗时 hold/cancel -> 100 HP + home -> goal_complete -> disabled/hold`

- 400 HP 时 `active_branch=patrol`。
- 由于 mock 的 `current_hp` 只在启动时读取，本轮通过停止 400 HP mock、启动 100 HP mock 注入低血量。
- mock 替换造成约 2.4 秒输入 freshness 空窗；mission 先安全 hold/cancel，100 HP 有效后进入 `home`。
- 最新 home action 状态为 `4=SUCCEEDED`。
- 用户观察到“巡逻一段时间后回到 home”。
- `/cmd_vel` 最后一条为全零；最后非零样本后有 16 条连续零速样本。

该测试验证了低 HP 下 home 的高优先级和输入失效时的安全取消，但不是“无缝真实 referee 数据切换”验收。

## 4. 当前地图质量

资产：

`/data/rm27_maps/old_car_clean_20260715_field01/20260715T025927Z/old_car_clean_20260715_field01.bundle.yaml`

状态仍为 `candidate`。定量结果：

- 尺寸 333×398，分辨率约 0.05 m。
- occupied 15,310，free 48,304，unknown 68,920。
- 725 个 occupied 8 邻接连通域。
- 686 个连通域不超过 25 像素，共 1,686 像素。
- PGM 上半部 occupied 密度 3.41%，下半部 19.69%，下半部约为 5.8 倍。
- 稳定 PCD 共 12,750 点；0.10–1.80 m 高度带 5,776 点。
- 7,095 个 occupied 像素在 0.10 m 内没有稳定 PCD 高度带支持。
- 633/725 个 occupied 连通域没有上述 PCD 支持。
- bundle 记录 536 个 accepted clouds，其中 357 个使用 latest-TF fallback。

结论：用户观察到的下半部脏点有数据支持。当前 PGM 来自 OctoMap 最新投影，而 PCD 使用 `min_observations=2` 稳定体素，两条资产路径的持久化合同不同。不能靠缩小 inflation 掩盖，也不能只删除单像素。

本轮只生成诊断统计和 overlay，没有覆盖、修改或重新生成部署资产。

## 5. AMCL 错误初值诊断

受控 bag 含 3 次 `/initialpose`：

- seed 1 `(1.742, 1.208, 0.664)`，末态 `(1.607, 0.083, 0.020)`。
- seed 2 `(2.404, 1.180, 0.417)`，末态 `(2.423, 0.132, 0.033)`。
- seed 3 `(3.354, 1.292, 0.530)`，末态 `(2.477, 0.114, 0.021)`。

seed 2 和 seed 3 的最终位置只差约 0.057 m，但各自离输入种子约 1.05 m 和 1.47 m。最终 x/y/yaw 协方差下降到约 `1e-4`；`amcl_backend_valid` 与 `global_localization_valid` 在记录内没有 false。

当前 validity gate 只验证时间、frame、有限值、姿态和协方差上限，因此无法识别“自信但错误”的匹配。

使用同一真实 LaserScan 的离线 likelihood 对照：

- AMCL 最终落点在原始 PGM 上近似分数 0.809。
- 排除无稳定 PCD 支持的占用格后，分数仍为 0.795。
- 主要局部峰结构仍存在。

因此脏 PGM 会影响整体质量和规划，但不是相同 AMCL 吸引点的唯一原因。当前 AMCL 是带 seed 的局部定位，不应宣称任意错误初值的 place recognition。现阶段启动 mission 前仍需准确 2D Pose Estimate 和人工确认点云/地图对齐。

## 6. 工作区与修改

2026-07-17 没有修改产品代码、地图资产或控制参数。只创建 `/tmp` 离线分析脚本、统计和本报告。

当前工作区已有变更：19 个 tracked 路径（约 +223/-54）、`livox_ros_driver2_humble` 子模块状态变化，以及未跟踪 `core.205`。这些变更属于此前调试过程，未 commit、未 push。完整 diff 见日志。

已知已有改动包括：

- mission 配置文件兼容与运行时配置装载；
- RViz 增加独立静态 `/map` 显示；
- AMCL 静止更新参数；
- global pose validity 传播；
- old-car Nav2 所有速度 producer remap 到 `/cmd_vel_nav`；
- map 导出 unknown 阈值和 all-unknown 拒绝；
- launch 导入、参数与依赖修复。

运行时 `/data/rm27_maps/runtime/mission_old_car.yaml` 已设置 home/patrol 点，并保留 `.bak`。它不在 Git 工作区内。

## 7. 构建与测试边界

最新保存的既有证据包括：

- 全工作区 build 成功：`89_final_full_workspace_build.log`。
- first-party tests：`85_first_party_tests_after_cmd_vel_fix.log`。
- 隔离 test-result：`87_first_party_test_results_isolated.log`，无失败。
- map export threshold build/tests：`94`–`97` 日志。
- map candidate quality gate build/tests：`98`–`99` 日志。

2026-07-17 没有产品代码变化，因此未重复构建。不能用这些记录替代未来地图合同或 AMCL gate 修改后的重建与测试。

## 8. 残留风险与下一步

1. 不批准当前 candidate 地图；清场后再建新 revision，保留旧 revision/hash。
2. 新建图前先决定统一 PGM 与 PCD 的持久化合同，并把 PCD 支持率、连通域、fallback 比例加入 candidate 审查门；不要直接后处理覆盖旧 PGM。
3. 尽量消除 timestamped TF 失败，正式候选不应依赖高比例 latest-TF fallback。
4. AMCL 启动仍要求近似真实初值；readiness true 不能单独授权 mission。应增加人工对齐门，未来若需要无人工冷启动则引入可验证的全局 place recognition/外部地标，而不是把 GICP 或 AMCL 局部收敛描述成全局识别。
5. 清图后再复测 inflation 和窄通道；现在调 inflation 会掩盖静态图污染。
6. 只有地图、AMCL 合同或控制参数发生变化时，才重跑已通过的静态链和三类运动，避免今天小障碍移动导致无意义重复。
7. 真实 referee、右雷达/双雷达、pursuit、比赛点位和长时间赛场运行仍需独立验收。

## 9. 推荐合并策略

- 不要把当前脏工作区整体一次性合并。
- 先审查 `core.205` 与 submodule 状态，避免误带入提交。
- 按最小主题拆分：launch/依赖修复、global-pose validity、mission 配置、Nav2 速度所有权、map-export 合同。
- 地图资产使用独立 candidate revision 和独立人工审查，不与代码提交混合。
- 当前不 commit、不 push；待 diff 审查和对应 build/test 重跑后再决定合并。

## 10. 证据路径

- 更新工作区状态：`/tmp/rm27_field_debug_full_validation/20260717_update/01_git_state.log`
- 完整 worktree diff：`/tmp/rm27_field_debug_full_validation/20260717_update/02_worktree.diff`
- 当前地图统计：`/tmp/rm27_field_debug_full_validation/current_map_analysis/summary.log`
- 地图 PCD/PGM overlay：`/tmp/rm27_field_debug_full_validation/current_map_analysis/pcd_pgm_support_overlay.png`
- initialpose 分段：`/tmp/rm27_field_debug_full_validation/relocalization_analysis/initialpose_segments.log`
- scan likelihood：`/tmp/rm27_field_debug_full_validation/relocalization_analysis/scan_likelihood.log`
- mission bag 汇总：`/tmp/rm27_field_debug_full_validation/final_analysis/mission_bag_summary.log`
- 手动接管原始 bag：`/tmp/rm27_field_debug_full_validation/manual_takeover/patrol_manual_takeover/`
- 低 HP 原始 bag：`/tmp/rm27_field_debug_full_validation/low_hp_return_home/patrol_low_hp_home/`
- 7 月 14 日基础报告与完整日志：`/tmp/rm27_field_debug_full_validation/20260714T130902+0800/`
