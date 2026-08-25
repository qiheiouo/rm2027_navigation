# 老车高速自转定位复发与 `map→odom` 修正门

日期：2026-08-25

分支：`fix/old-car-amcl-correction-gate`

状态：Linux 隔离构建、单元测试与 ROS topic 注入测试通过；实车复验必须在匹配地图下
进行。

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
4. 后端若返回到原正确模式，可自动恢复；
5. 发布 `/initialpose` 或调用 `/localization/reset_map_to_odom` 会明确清除基准，允许
   下一次有效结果建立新基准。

通用配置的开关默认为 false。只有老车 competition、AMCL 和 GICP 入口默认选择老车
profile；新车与通用 launch 不改变行为。

## 3. 能解决与不能解决的边界

该门针对历史样本中约 95 ms 内 1.07 m 的首次灾难跳变，也针对本次“低协方差错误
模式”向下游传播的问题。它是 fail-safe，不是 AMCL 根因修复：

- 不修改 `/odometry/lio`，不能修复 LIO 发散；
- 不保证发现每次小步累积漂移；
- 第一次基准本身仍需正确初值或人工确认；
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
   - 回到最后基准附近后恢复；
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
- ROS 注入 10 个检查点全部 PASS；
- 注入从已接受 0.10 m 到 1.00 m 的候选，即 0.90 m 创新，节点按 0.35 m 限制
  拒绝，valid=false，且 output 未出现 1.00 m 修正；
- 随后 0.12 m 候选恢复 valid/TF；发布 `/initialpose` 后 1.00 m 成功成为新基准。

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
5. 只有人工确认新位姿正确后才重新发布 `/initialpose`。

实车 PASS：正常运动不误拒绝；灾难候选没有进入 canonical `map→odom`；失效后导航
安全停止；人工重定位能恢复。

实车 FAIL：大跳仍被接受、valid=false 时底盘仍执行导航、正常工况频繁误拒绝，或
AMCL/LIO 本身持续发散。

完成 A/B/C 前，本修复只能称为“代码与合成验证完成”，不能称为高速自转问题已实车
关闭。
