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
`Managed nodes are active`。这说明这些会话是正常等待后恢复，不能据此宣称已复现
“costmap 永久加载失败”。是否会在 Nav2 的初始 TF 等待上限后失效，需要 R02 专门验证。

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

### R02：延迟初始化边界

车辆必须静止且保持人工停车权限。冷启动后故意 70 秒不发布初始位姿，再发布一次正确
初始位姿。此试验只做一次，期间不发目标。若 planner lifecycle 在 60 秒附近失败，且之后
不能自行 active，则证明永久加载问题来自“Nav2 早于定位 TF 激活”；下一步应实现定位有效
后再启动 Nav2 lifecycle，而不是继续加大 costmap 参数。若 70 秒后仍能自动 active，则
保留现有启动方式，不增加额外生命周期协调节点。

### R03：直线与自转恢复

R01 全部通过后再做。先低速直线，再逐级提高角速度，不直接从最高速开始。触发保护后
停车，记录 `correction_recovery_state` 从锁存到 `healthy` 的耗时和自主 `/initialpose`
次数。禁止人工重发位姿，除非超过测试预设停止时间并已终止试次。

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

## 7. 通过门

只有 R01 五次全过、R02 结论明确、R03 至少三次自动恢复且无误停车，才能把本轮描述为
“旧车导航启动与定位保护具备可重复恢复能力”。在此之前，更准确的状态是：核心导航和
自主定位恢复已实现，组合狗洞/坡道已能运行；已修复若干确定的软件边界，但偶发启动故障
仍缺一次冻结现场的根因证据。
