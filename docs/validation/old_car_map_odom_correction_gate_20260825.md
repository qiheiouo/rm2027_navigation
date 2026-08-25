# 老车高速自转定位复发与 `map→odom` 修正门

日期：2026-08-25

分支：`fix/old-car-amcl-correction-gate`

状态：修正门现场拦截真实错误峰；自主恢复 Linux/ROS 测试通过；最终实车运行记录
4 次 recovery completed，操作员确认位姿正确恢复。完整 10 次独立验收仍待执行。

## 1. 本次现场结论

问题在仍运行的 `old_car_dog_hole_navigation.launch.py` 中复发。核对实际运行参数后，
确认第一版高速自转候选已经生效，而不是启动错了 profile：

- AMCL 使用 `amcl_2d_spin_robust_candidate.yaml`，`alpha4=0.02`；
- 点云投影使用严格逐点 SE(3) deskew，10 Hz，时间字段为绝对纳秒；
- scan、AMCL、LIO 分别约 9 Hz、9.2 Hz、50 Hz；
- map server、AMCL 与 Nav2 生命周期节点均为 active；
- deskew 只有少量明确丢帧，没有持续降级为未补偿 scan；
- LIO 没有出现与全局跳变同量级的发散。

失败后抽样到的 AMCL raw pose 约为：

```text
x=0.696 m, y=2.971 m, yaw=-1.545 rad
cov_x=1.59e-4, cov_y=1.85e-4, cov_yaw=1.72e-4
```

约 1 秒后仍稳定在同一错误模式，协方差进一步变小。原有 gate 只在启动早期拒绝过
3 次 yaw 协方差超限，之后连续接受位姿；bridge 日志记录了 381 次全局修正更新。
因此本次证据再次确认：错误粒子峰可能形成很小协方差，协方差门不能独立保护
`map→odom`。

现场日志的临时副本保存在：

```text
/tmp/rm2027_amcl_high_spin_20260825_live
```

该目录不是仓库资产，重启或清理 `/tmp` 前应按需另行归档。

同日启用第一版 0.35 m / 0.35 rad 修正门后再次受控自转，得到更直接的现场证据：

- 自转初期 bridge 在 0.414 m / 0.437 rad、0.925 m / 1.866 rad 等候选之间交替
  接受和拒绝，导致 canonical TF 可见性闪烁；
- 随后 AMCL 锁定在相对最后可信修正约 2.06 m / 2.75 rad 的错误峰；
- bridge 持续拒绝，错误修正没有进入 canonical `map→odom`；
- 左雷达点云仍约 52 Hz，`odom→base_link` 持续更新，deskew 没有持续丢帧；
- 因 RViz 固定 frame 为 `map`，canonical TF 被安全撤下后 RobotModel、点云和 costmap
  看似同时消失，实际底层 LIO 链仍在线。

这证明第一版门控隔离有效，但“下一帧回到限制内便自动恢复”会让 TF 闪烁，而且错误
峰稳定后比赛无法依赖人工 `/initialpose`。第二版因此增加锁存和自主局部重播种。现场
日志临时保存于：

```text
/tmp/rm2027_old_car_auto_recovery_20260825
```

同日第三次实车触发验证了自主重播种已运行，但暴露了第一版停车判定不适配老车有限
差分 twist：

- bridge 在停车后成功向 AMCL 自主发送多次 `/initialpose`，AMCL 日志确认逐次接收；
- AMCL raw、通过 covariance gate 的 global pose 和 scan 均持续约 8--9 Hz，backend-valid
  保持 true，不是后端或点云停发；
- 8 秒订阅中 recovery state 在 `latched_waiting_for_stationary_hold` 与
  `latched_waiting_for_stop` 间反复切换，global-valid 始终 false；
- 10 秒、482 帧 `/odometry/lio` 静止采样中，瞬时线速度最大 0.154 m/s、角速度最大
  0.235 rad/s；只有 79.0% 样本同时低于 0.08/0.15 阈值，最长连续低阈值区间仅
  0.396 秒，无法满足 0.6 秒保持；
- 同一批 odom 位姿在任意 0.6 秒窗口的最大平移范围仅 0.034 m、最大 yaw 范围仅
  0.022 rad，证明主要是有限差分 twist 尖峰，而不是底盘仍在持续运动。

因此停车门增加 `recovery_motion_confirmation_sec=0.12`：孤立超阈值样本禁止当帧
重播种，但不会清零静止保持和一致位姿计数；只有超阈值连续维持 0.12 秒才确认为
真实运动并清零。原 0.08 m/s、0.15 rad/s 安全阈值不放宽。

第三次现场日志和两次订阅探针摘要临时保存于：

```text
/tmp/rm2027_old_car_recovery_spike_20260825
```

用户重新 `colcon build` 并启动最终代码后进行第四次受控复验。该轮 bridge 日志记录
4 次 fault latch 后均完成自主恢复；最大被拒绝 correction 创新为 1.928 m / 2.873 rad。
其中 3 次从首次重播种到恢复为 1.052--1.095 秒，另 1 次困难场景需要 11 次重播种、
21.803 秒才进入连续 5 帧可信模式。操作员确认等待后位姿正确，恢复后 8 秒探针中
recovery state 全为 `healthy`、global-valid 全为 true，AMCL/global pose/scan 均持续
约 8--9 Hz。

最终实车日志临时保存于：

```text
/tmp/rm2027_old_car_recovery_success_20260825
```

跨阶段的根因、消融、修复和最终实车数据汇总见
`old_car_high_spin_localization_engineering_report_20260825.md`。本轮通过的是实车恢复
smoke；因存在 11 次重播种长尾且没有精确物理停车 timestamp，仍不能关闭 10 次独立
重复、底盘停权和恢复延迟验收门。

## 2. 本次修改

在 `map_odom_from_global_pose` 中增加可选修正创新门。它对时间匹配后的
`T_map_odom` 候选执行：

```text
translation_innovation = hypot(candidate.x - accepted.x,
                               candidate.y - accepted.y)
yaw_innovation = shortest_angle(candidate.yaw - accepted.yaw)
```

老车 profile 的初始限制为：

```yaml
correction_innovation_gate_enabled: true
max_correction_translation_step_m: 0.35
max_correction_yaw_step_rad: 0.35
```

超过任一限制时：

1. 不写入候选修正；
2. `/localization/global_localization_valid` 立即变为 false；
3. 保留最后一次已接受修正用于比较，但停止续发 `map→odom` TF；
4. 锁存故障，单帧返回正确模式也不恢复 TF，避免可见性闪烁；
5. 等 `/odometry/lio` 线速度不大于 0.08 m/s、角速度不大于 0.15 rad/s 并持续
   0.6 秒；孤立尖峰不清零，超阈值连续 0.12 秒才确认运动；
6. 用 `last_trusted(T_map_odom) × current(T_odom_base)` 计算预测全局位姿，以
   0.09 m² XY 方差和 0.0685 rad² yaw 方差自动向 AMCL 发布 `/initialpose`；
7. 等待 0.5 秒，再要求连续 5 个候选都落在可信修正门内才恢复 canonical TF；
8. 未恢复时每 2 秒最多重播种一次，不做无限频率重置；
9. 人工发布 `/initialpose` 或调用 `/localization/reset_map_to_odom` 仍会明确清除基准，允许
   下一次有效结果建立新基准。

恢复状态使用 transient-local topic：

```text
/localization/correction_recovery_state  std_msgs/String
```

主要值为 `healthy`、`latched_waiting_for_stop`、
`latched_waiting_for_stationary_hold`、`reseeded_waiting_for_consistency` 和
`waiting_for_baseline`。

通用配置的开关默认为 false。只有老车 competition、AMCL 和 GICP 入口默认选择老车
profile；新车与通用 launch 不改变行为。

## 3. 能解决与不能解决的边界

该门针对历史样本中约 95 ms 内 1.07 m 的首次灾难跳变，也针对本次“低协方差错误
模式”向下游传播的问题。它是 fail-safe，不是 AMCL 根因修复：

- 不修改 `/odometry/lio`，不能修复 LIO 发散；
- 不保证发现每次小步累积漂移；
- 第一次基准本身仍需正确初值或人工确认；
- 自主恢复建立在 LIO 仍可靠的前提下，不能从 LIO 发散中恢复；
- 被拒绝后 Nav2 应因 TF/有效性丢失而停车，不能继续执行旧目标；
- 0.35 m / 0.35 rad 是基于既有故障幅度选择的保守初值，仍需实车数据确认误拒绝率。

## 4. 不需要匹配地图的验证

以下工作可在当前地图不匹配时完成，不要为了它重新建图：

1. 构建 `rm_relocalization_bridge` 与 `rm_navigation_bringup`；
2. 运行单元测试，覆盖平移、yaw 和 `±π` 最短角；
3. 用合成 `/odometry/lio`、`/localization/global_pose` 和 backend-valid 消息做注入：
   - 第一帧建立基准，valid=true；
   - 小于限制的修正被接受；
   - 大于限制的修正被拒绝，valid=false，且不发布新 TF；
   - 运动中偶尔回到可信范围不能恢复，canonical TF 不闪烁；
   - 静止保持后自动发布的初值等于最后可信修正乘当前 LIO；
   - 前 4 个一致候选仍保持 invalid，第 5 个才恢复 TF；
   - 发布 `/initialpose` 后，远处新位姿可以建立新基准；
4. 展开老车 launch，确认加载的是
   `map_odom_from_global_pose_old_car_2026.yaml`；展开通用/新车 launch，确认仍使用兼容
   profile 或 gate=false。

PASS：上述五种状态转换全部符合预期，测试无失败。

FAIL：异常候选仍更新 TF、valid 未拉低、普通新车入口被意外开启，或 reset 后不能重建
基准。

2026-08-25 已完成结果：

- 隔离构建 `rm_relocalization_bridge`、`rm_navigation_bringup` 成功；
- `colcon test-result`：23 tests，0 errors，0 failures，0 skipped；
- 增加自主恢复回归后再次单独构建 `rm_relocalization_bridge`：21 tests，0 errors，
  0 failures，0 skipped（包含 1 个完整节点状态机 ROS 测试）；
- 第一版 ROS 注入 10 个检查点全部 PASS；
- 注入从已接受 0.10 m 到 1.00 m 的候选，即 0.90 m 创新，节点按 0.35 m 限制
  拒绝，valid=false，且 output 未出现 1.00 m 修正；
- 第二版自主恢复注入 15 个检查点全部 PASS：运动中不闪烁、0.6 秒静止保持触发一次
  可信预测重播种、4/5 一致时仍 invalid、5/5 时恢复 valid/TF，人工初值兜底仍有效。
- 第三版增加静止 twist 尖峰回归：每 4 帧注入一次 0.12 m/s、0.25 rad/s 短尖峰仍能
  自动重播种并恢复；连续 1.0 rad/s 运动仍禁止重播种；21 tests 全部通过。

因此“不需要匹配地图的验证”已关闭；剩余工作只有下一节实车复验。

## 5. 需要匹配地图的老车复验

只有这部分需要当前环境与地图匹配。若现有地图不匹配，先冻结实车复验；无需仅为
代码验证重新建图。以后使用匹配的 PGM/点云或重新建图后按以下顺序执行。

### A. 静止与初值

1. 架空轮或确保底盘急停可用，禁止任务与自动导航；
2. 启动老车完整导航，发布正确 `/initialpose`；
3. 静止 2 分钟，记录 AMCL raw、global pose、LIO odom、TF、valid 和 deskew diagnostics；
4. 确认没有误触发 0.35 m / 0.35 rad 门。

### B. 低速闭环

1. 先手动低速直行、横移、转向；
2. 再执行短距离导航；
3. 确认 `map→odom` 连续、costmap 正常、没有因正常修正误停车。

### C. 受控高速自转

1. 保持 Nav2 目标和 mission 禁用，场地清空并准备物理急停；
2. 从较低 yaw rate 分档上升，每档只做短时自转并停车观察；
3. 每档记录上述全部 topic，禁止只看 RViz；
4. 若 AMCL 再次跳入错误峰，要求门先拒绝，valid=false，TF 停止续发，底盘不得继续
   接受导航运动；
5. 停止后观察 recovery state 依次进入 stationary hold、reseeded、healthy；正常情况下
   不做人工操作；
6. 要求自动恢复后 RobotModel、点云和 costmap 回到 `map`，且恢复位置与实际位置
   一致；
7. 连续至少 10 次触发，记录恢复耗时、重播种次数、误恢复和无法恢复次数；只有自动
   恢复失败时才使用人工 `/initialpose` 兜底。

实车 PASS：正常运动不误拒绝；灾难候选没有进入 canonical `map→odom`；失效后导航
安全停止；无需人工干预即可恢复，人工兜底仍有效。

实车 FAIL：大跳仍被接受、valid=false 时底盘仍执行导航、正常工况频繁误拒绝，或
AMCL/LIO 本身持续发散、自动恢复到错误位置，或频繁循环重播种。

完成 A/B/C 前，本修复只能称为“代码与合成验证完成”，不能称为高速自转问题已实车
关闭。
