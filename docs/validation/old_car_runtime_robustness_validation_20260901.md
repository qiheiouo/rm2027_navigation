# 旧车导航偶发故障收敛与复验方法（2026-09-01）

## 1. 范围与当前结论

本文只处理近期实车出现过的运行时问题，不重新设计导航架构：

1. 全地形入口的全局速度上限此前没有显式参数；
2. 偶发看到 local/global costmap 不可用；
3. 高速自转后 RobotModel、点云、TF 和 costmap 一起消失；
4. `/mapping/save` 曾一直等待，同时 ROS 图中只剩 `/octomap_server`；
5. 偶发发布初始位姿后仍不能导航。

截至本文版本，已完成三项不依赖实车的修复：

- `global_max_forward_speed` 同时约束 velocity smoother 与真实串口边界，并正确串接狗洞、
  坡道两层 YAML 重写；
- `/system/readiness` 只有在 `/navigate_to_pose`、local costmap 和 global costmap 都存在且
  costmap 最近 3 秒仍有更新时，才报告 Nav2 部分就绪，并分别列出缺失项；该 topic 仍是
  只读诊断，不直接授予或撤销底盘运动权限；
- 建图采样器、OctoMap 或 mapping session 任一关键进程退出时，建图 launch 整体退出，
  不再留下“OctoMap 看似在线、保存服务实际不存在”的降级假象。

costmap 启动顺序目前有明确但尚未构成失败的证据：Nav2 激活 global costmap 时，人工初始
位姿尚未建立 `map -> odom`，所以 planner 会循环等待 `base_link -> map`。现存四次日志均
在约 8--12 秒后出现 `global_costmap: start`，且 lifecycle manager 随后报告
`Managed nodes are active`。2026-09-01 的 R02 又验证了延迟约 115 秒才发布初始位姿仍可
恢复，因此当前不能把“costmap 永久加载失败”归因于固定 60/70 秒超时，也不需要新增
定位有效后再启动 Nav2 的生命周期协调节点。真正的偶发永久故障仍需要按第 5 节冻结现场。

## 2. 显示消失不等于后端全部崩溃

高速自转保护拒绝异常全局修正后，会主动撤下 canonical `map -> odom`。RViz 固定 frame
为 `map`，因此 RobotModel、点云和 costmap 会一起无法变换。这是定位隔离的可见结果；
先检查以下 topic，再判断是否为 MID360、FAST-LIO 或 Nav2 进程故障：

```bash
ros2 topic hz /odometry/lio
ros2 topic hz /livox/left/pointcloud_filtered
ros2 topic echo --once /localization/global_localization_valid
ros2 topic echo --once /localization/correction_recovery_state
```

仓库根目录现有 5 个 core 文件都识别为 `rviz2`。已抽查的回溯位于 X11/Mesa 与 rclcpp
退出路径；容器使用无硬件加速的 llvmpipe。现有 ROS 日志中的 RViz、Livox 和一次投影器
`-11` 也发生在整套 launch 收到 SIGINT 的收尾时刻。它们说明退出路径不够干净、会产生
大 core，但暂时没有证据证明这些进程在导航运行中自行崩溃。不要用这些 core 替代下一次
现场故障的时间对齐证据。

## 3. 当前完整启动命令

```bash
ros2 launch rm_navigation_launch old_car_full_terrain_navigation.launch.py \
  map_bundle_override:=/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_field.bundle.yaml \
  activate_ramp_filter:=true \
  enable_dog_hole_route:=true \
  dog_hole_regions_file:=/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_connected_dog_hole_ramp.regions.yaml \
  dog_hole_route_file:=/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_connected_dog_hole.route.yaml \
  dog_hole_hold_sec:=5.0 \
  global_max_forward_speed:=4.00 \
  mppi_forward_velocity_std:=0.60 \
  ramp_max_forward_speed:=4.00 \
  dog_hole_max_forward_speed:=4.00 \
  ramp_max_yaw_rate:=0.35 \
  dog_hole_max_yaw_rate:=0.35 \
  dog_hole_local_inflation_radius:=0.20 \
  dog_hole_global_inflation_radius:=0.10
```

`global_max_forward_speed` 是全程下游硬上限。坡面 active 时 MPPI 使用
`ramp_max_forward_speed`，否则使用狗洞 profile 的 `dog_hole_max_forward_speed`；最终有效
速度不会超过三者相应的最小约束，也不会绕过加速度、障碍和下位机物理限制。
`mppi_forward_velocity_std` 是采样探索强度，不是硬上限。旧值 `0.20` 即使配合
`vx_max=4.0`，一次迭代也主要探索低速；先用 `0.60` 做清空长直线验证，不能直接设为
`4.0`。当前平滑器前向加速度仍为 `0.8 m/s^2`，短路径达不到 4 m/s 是正常结果。

## 4. 每次启动后的 30 秒检查

发布目标前执行：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /system/readiness
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

PASS 条件：三个 lifecycle 都是 `active`；readiness 中没有 `nav2_action`、
`local_costmap` 或 `global_costmap`；local/global costmap 分别接近配置的 5 Hz/2 Hz。
未发布正确初始位姿前，`global_localization` 缺失是预期行为，不能发导航目标。

注意：readiness 是只读诊断 topic，手工 RViz 目标不会被它自动拦截。操作者仍必须等上述
条件通过。比赛 mission 还有自己的启用和底盘权限门，不能用 readiness 替代。

## 5. 下次故障的现场冻结方法

再次出现“路径有但不动”“模型/点云/costmap 一起消失”或“发布初始位姿无效”时：

1. 立即切手动或急停并保持车辆静止；
2. 不重发 `/initialpose`，不清 costmap，不重启 launch；
3. 记录故障发生的墙钟时间；
4. 执行以下只读检查，或通知 Codex 在线读取：

```bash
ros2 topic echo --once /system/readiness
ros2 topic echo --once /localization/global_localization_valid
ros2 topic echo --once /localization/correction_recovery_state
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info /local_costmap/costmap --verbose
ros2 topic info /global_costmap/costmap --verbose
ros2 topic hz /odometry/lio
ros2 topic hz /localization/scan
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

判定顺序：

- `odom -> base_link` 和点云仍在线、只有 `map -> odom` 消失：定位修正门正在隔离异常；
- TF 都正常、costmap topic 不更新或 lifecycle 非 active：Nav2/costmap 故障；
- 后端 topic 正常但只有 RViz 画面消失：先按 GUI 故障处理，导航后端不应随 RViz 重启；
- LIO odom 本身停发或明显发散：超出 map-to-odom 修正门的恢复能力，禁止自动导航。

## 6. 需要实车配合的复验

### R01：普通冷启动

车辆静止，启动后在 20 秒内发布正确初始位姿，不发送目标。连续做 5 次。每次要求：

- `planner_server`、`controller_server`、`bt_navigator` 都进入 active；
- local/global costmap 持续发布；
- readiness 最终只保留与未启用比赛接口对应的预期缺失项；
- 不需要重启或第二次发布初始位姿。

2026-09-01 实车结果：**5/5 PASS**。五轮均为冷启动后只发布一次正确初始位姿，不发送
导航目标；最终 readiness 均为 navigation true，仅保留未接入的 `referee_state`。从桥接器
接收初始位姿到 global costmap 开始、全部 Nav2 节点 active 的耗时分别为：

```text
轮次    global costmap    Nav2 全部 active
1       0.41 s            1.98 s
2       0.36 s            1.39 s
3       0.18 s            1.34 s
4       0.19 s            1.43 s
5       0.21 s            1.22 s
```

五轮都不需要第二次发布初始位姿或重启 launch，因此启动顺序基线已经满足本轮通过门。

### R02：延迟初始化边界

车辆必须静止且保持人工停车权限。冷启动后故意 70 秒不发布初始位姿，再发布一次正确
初始位姿。此试验只做一次，期间不发目标。若 planner lifecycle 在 60 秒附近失败，且之后
不能自行 active，则证明永久加载问题来自“Nav2 早于定位 TF 激活”；下一步应实现定位有效
后再启动 Nav2 lifecycle，而不是继续加大 costmap 参数。若 70 秒后仍能自动 active，则
保留现有启动方式，不增加额外生命周期协调节点。

2026-09-01 实车结果：**PASS**。车辆全程静止，只发布一次初始位姿：

```text
planner process started                     1788226088.371
planner lifecycle activation began         1788226093.443
manual /initialpose received by bridge      1788226203.511
first corrected global pose accepted        1788226203.613
global costmap reported start               1788226203.948
all Nav2 managed nodes active               1788226205.361
```

初始位姿比 planner 进程晚约 115.14 秒，比 planner activation 晚约 110.07 秒；桥在
0.10 秒内建立修正，global costmap 在 0.44 秒内开始，Nav2 在 1.85 秒内全部 active。
稳定后 controller/planner/BT lifecycle 均为 active，定位恢复状态为 `healthy`，readiness
为 navigation true；local/global costmap 实测约 4.0/1.66 Hz。运行参数核对为：

```text
MPPI FollowPath.vx_max                      4.0
velocity_smoother.max_velocity              [4.0, 0.5, 1.2]
serial_transport.max_vx                     4.0
local/global inflation_radius               0.20 / 0.10
```

结论：保留现有 Nav2 启动方式。启动阶段 planner 暂时 inactive、global costmap 未发布属于
等待首次全局 TF；只有发布正确初始位姿后仍不能在数秒内 active 才按永久故障处理。

### R03：直线与自转恢复

R01 全部通过后再做。先低速直线，再逐级提高角速度，不直接从最高速开始。触发保护后
停车，记录 `correction_recovery_state` 从锁存到 `healthy` 的耗时和自主 `/initialpose`
次数。禁止人工重发位姿，除非超过测试预设停止时间并已终止试次。

2026-09-01 实车基线使用地图 revision `20260828T070722Z`，车辆由遥控器执行动作，mission
保持 disabled。低速直线、低速自转和中速自转均未触发保护；期间 correction recovery
保持 `healthy`，全局位姿连续接受，local/global costmap 持续更新。点云 SE(3) 去畸变在
约 4.1 万包时仅累计 2 次插值失败、4 个 drop，没有时间戳、TF、队列溢出或 odom 拒绝。

高速自转产生了三类结果：

1. 第一次约 2--3 秒/圈的一圈自转触发 0.416 m 平移创新拒绝。停车后第 1 次自主 AMCL
   重播种即恢复；从首次拒绝 `1788273571.627` 到 canonical `map -> odom` 恢复
   `1788273574.768` 共 3.14 秒，恢复后人工确认点云、地图和实车对齐。结果：**PASS**。
2. 同方向持续时间更长的自转触发 0.557 m 创新拒绝。系统保持 fail-closed，但在 0.350 m
   门附近反复失败，共执行 16 次重播种；从 `1788273801.348` 到 `1788273835.228` 共
   33.88 秒才恢复，恢复后人工确认对齐。结果：**PASS with WARN**。现有 2 秒 cooldown
   可能在 AMCL 充分收敛前重复重置，需要离线复核后再调，不能直接放宽创新门。
3. 反方向连续三圈使 `/odometry/lio` 从实车原地附近跳到约
   `(-0.73, 10.86, -0.14)`；首次 correction innovation 达 4.433 m，后续最高约 9.86 m。
   恢复器继续以 `map->odom * 当前 LIO odom` 形成所谓 trusted prediction，播种点因此落到
   约 `(1.2, 10.2)`，已经不是实车位置。到安全结束 launch 前已重播种 24 次，状态仍为
   `latched_waiting_for_stationary_hold`、global localization false。结果：**FAIL**。

第三种情况不是普通 AMCL 漂移，而是底层里程计发生大尺度不连续；它验证了现有算法说明中
“底层里程计本身需基本可靠”的边界。保护层正确禁止导航并拒绝了错误 TF，但当前自动恢复
不能处理 LIO 原点/位置突跳。下一步不能继续无限重播种，也不能把 10 m correction 直接
放行；应先离线区分“AMCL 假匹配”和“odom 不连续”，再为后者设计独立的 fail-closed
重基准流程：以锁存前最后可信全局位姿为候选种子，检测静止及 LIO 已稳定，要求多帧绝对
定位一致后，才允许重新计算新的 `map -> odom`。若 LIO 姿态仍发散或持续移动，必须保持
导航禁用，不能伪装成恢复成功。

本轮证据已保存在宿主机：

```text
artifacts/bags/20260901_r03_spin_recovery_02/
artifacts/logs/20260901_r03_spin_recovery_02_ros/
```

rosbag 已正常关闭并生成 metadata：406.4 MiB、885.116 秒、225085 条消息。它包含
`/odometry/lio`、raw/canonical localization、恢复状态、TF、双 costmap 和去畸变诊断。
此前的 `20260901_r03_spin_recovery_01` 因容器被外部中断且没有 metadata，只能作为中断
残留，不能用于验收。

### R04：建图入口 fail-fast

只在手动控车建图时验证。正常启动应同时存在：

```text
/mapping_pointcloud_sampler
/octomap_server
/mapping_session
```

保存前先确认 `/mapping/sensor_cloud` 有 publisher、`/mapping/recording` 有数据。调用使用
有限等待，防止 CLI 在服务不存在时无限等：

```bash
timeout 15 ros2 service call /mapping/save std_srvs/srv/Trigger '{}'
```

任一关键进程异常退出时，整套建图 launch 必须退出并打印原因；不得继续驾驶后再尝试
保存。正常保存仍只生成新 candidate revision，不覆盖任何旧地图资产。

### R05：目标不动与速度偏低现场

2026-09-01 第一次普通导航目标已经进入 Nav2：controller 在收到路径后运行约 18 秒，随后
报 `Failed to make progress`，完成多轮恢复后 action 以 `ABORTED` 结束。该次故障发生前
尚未开始录包，无法确认最终速度命令是否到达狗洞门控和串口，因此只记录为一次未定根因
的“有路径、无有效运动”事件，不能据此宣称已经修复。下次复现时应立即停车、保留现场且
不要重发目标，再同时采集 action、三段速度、里程计、代价地图和门控状态。

随后从同一运行实例发送的普通非狗洞目标成功完成，证据包保存在宿主机：

```text
artifacts/bags/20260901_no_motion_live
```

该成功试次的量化结果为：

```text
全局路径                         71 poses / 1.786 m
目标后首个非零 /cmd_vel_nav      1.128 s
/cmd_vel_nav 峰值                vx=0.590 m/s, |wz|=0.310 rad/s
/cmd_vel 峰值                    vx=0.590 m/s, |wz|=0.310 rad/s
/cmd_vel_dog_hole_gated 峰值     vx=0.590 m/s, |wz|=0.310 rad/s
目标后的里程计位移               1.674 m
门控状态                         armed，未进入 pause，未判定路径穿洞
```

三段速度的峰值和波形一致，证明本次成功试次没有被 velocity smoother、狗洞门控或串口前级
再次限速。`vx_max=4.0` 仅是允许上限；旧 `vx_std=0.20`、MPPI 单轮采样、平滑器前向加速度
`0.8 m/s^2` 和不足 1.8 m 的短路径共同限制了实际峰值。组合入口现已开放
`mppi_forward_velocity_std`，默认仍为 `0.20`，首次只建议在清空长直线上显式试到 `0.60`。
它不能设成与 `vx_max` 相同的 `4.0`，且提高后必须重新检查贴墙、振荡和制动距离。

## 7. 通过门

只有 R01 五次全过、R02 已通过、R03 至少三次自动恢复且无误停车，才能把本轮描述为
“旧车导航启动与定位保护具备可重复恢复能力”。在此之前，更准确的状态是：核心导航和
自主定位恢复已实现，组合狗洞/坡道已能运行；已修复若干确定的软件边界，但偶发启动故障
仍缺一次冻结现场的根因证据。
