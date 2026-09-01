# 旧车“狗洞后连续爬坡”组合验证

## 1. 本轮要证明什么

本轮不是分别证明“能过狗洞”和“能上坡”，而是验证一条不中断的任务链：

```text
任意上游起点
  -> 坡后目标落入 map-bound goal trigger 区
  -> 公共 NavigateToPose 被改写为固定停止点
  -> Nav2 到达固定停止点
  -> 提交“狗洞出口点 + 原目标”的 NavigateThroughPoses
  -> 最终底盘速度被置零，先留 0.5 s 制动，再完整保持 5.0 s
  -> 老车以这 5 s 模拟下位机变形完成
  -> 原 Nav2 控制流恢复
  -> 进入 committed corridor 后不被本安全门二次打断
  -> 离洞后直接进入相连坡道
  -> 自动坡面过滤保持跟踪，Nav2 到达坡顶目标
```

实现没有增加第二个控制器，也没有复制仿真狗洞控制器。`dog_hole_route_orchestrator`
拥有公共 `/navigate_to_pose` action，但只做目标分段；真实 Nav2 action 被重映射为
`/navigate_to_pose_direct`，Nav2 仍独占规划和路径跟踪。`dog_hole_entry_pause_gate` 只位于
最终 `/cmd_vel` 和串口之间。5 s 定时器是旧车模拟；新车复用时用“变形请求/完成确认/
超时失败”替换该门的定时放行，路由层和 Nav2 不需要重写。

## 2. 场地和地图怎么准备

狗洞与坡道必须按最终验证位置固定，顺序为“狗洞在前、坡道紧接其后”。验证期间不得
移动其中任一结构。不要为了隔离狗洞而移走坡道，这会破坏本轮组合结论。

推荐为该布局生成一个新的 candidate map revision。并非因为算法必须在 PCD 中看到两种
结构，而是为了让 PGM、定位 PCD、语义坐标和现场布局能被同一个 bundle 复现。若只做
当天功能冒烟，并且当前背景地图定位稳定，也可以复用原 PCD、只生成新的 PGM/bundle
revision；不能修改旧 bundle 中的文件后仍沿用旧 hash。

洞顶的处理原则：

- 最终定位 PCD 最好在狗洞、坡道都按实车形态安装后采集，洞顶/立柱是有效静态特征；
- PGM 中必须手工清出洞内中心通道，不能让洞顶的二维投影封住通道；
- 如果建图器因洞顶根本无法生成可用二维图，可以拆顶生成 PGM，但装回洞顶后应以同一
  `map` 原点采集或导出最终 PCD；两次操作之间不能移动狗洞、坡道或地图基准；
- 仅把 PGM 涂白不够。实车 local costmap 仍会看到洞顶，所以组合入口启用现有狗洞
  obstacle-height profile；坡面则由自动坡面过滤器处理。

PGM 应满足：

- 狗洞中心线、狗洞至坡道的连接段、坡面中心线全部为 free；
- 狗洞两侧、坡道两侧、场地边界为 occupied；
- 如果本轮不允许绕行，在 PGM 中直接把绕行口封为 occupied，不用行为树猜测；
- 坡面不要画成黑色障碍，自动坡面过滤只处理实时点云，不能越过静态层；
- 不要用语义多边形掩盖真实墙体或坡上障碍。

PS 完成后创建新 revision，更新 bundle 内 occupancy YAML/PGM 的 SHA256；PCD 若字节未变
可沿用其原 hash。然后先执行：

```bash
ros2 run rm_map_tools validate_map_bundle \
  /data/rm27_maps/connected_test/REV/connected_test.bundle.yaml
```

必须 PASS。当前旧车入口允许 `candidate`，但验证报告要记录 bundle 的绝对路径和：

```bash
sha256sum /data/rm27_maps/connected_test/REV/connected_test.bundle.yaml
```

## 3. 标注狗洞语义区域

坡道不需要语义标注。只给狗洞画 `map` 坐标多边形，复制模板：

```bash
cp src/rm_navigation_launch/config/\
old_car_connected_dog_hole_ramp_regions.example.yaml \
/data/rm27_maps/connected_test/REV/connected_test.regions.yaml
```

在 RViz 选择 `Publish Point`，依次点击矩形四角，并读取：

```bash
ros2 topic echo /clicked_point
```

填写三类区域：

- `dog_hole_entry_a`：主测试方向的洞前停车区。上游边界要给实际制动留余量，建议先按
  当前 `0.20 m/s` 验证；5 s 从 0.5 s 制动等待结束后才开始计时。
- `dog_hole_tunnel`：只覆盖狗洞窄通道，类型必须是 `committed_corridor`；不要把后续整段
  坡道包含进去。
- `dog_hole_entry_b`：仅当允许“坡道侧反向进洞”且该侧有安全停车/变形空间时保留。
  若物理布局只允许狗洞到坡道的单向链路，应删除 B，并在 PGM/任务点设计中禁止反向
  穿越，而不是在坡中强行停车。

`map_binding.map_id`、`map_revision` 必须等于 bundle 字段，
`manifest_sha256` 必须等于上一步 `sha256sum`。验证语义文件：

```bash
ros2 run rm_path_annotations validate_semantic_regions \
  --regions /data/rm27_maps/connected_test/REV/connected_test.regions.yaml \
  --expected-map-id MAP_ID \
  --expected-map-revision REVISION \
  --expected-manifest-sha256 MANIFEST_SHA256
```

任何不匹配都必须拒绝启动；换图后不能继续使用旧多边形。

还需要单独创建目标路由 sidecar：

```bash
cp src/rm_navigation_launch/config/\
old_car_connected_dog_hole_route.example.yaml \
/data/rm27_maps/connected_test/REV/connected_test.route.yaml
```

其中：

- `goal_trigger_polygon` 覆盖“目标点在这里就必须先过狗洞”的坡道及坡后目标区；它不是
  costmap 障碍，也不会改变普通目标；其上游边界应在 `exit_pose` 下游；
- `stop_pose` 是洞前固定停车/变形位，必须在 trigger 外、approach 内；
- `exit_pose` 位于 committed corridor 下游且必须在 trigger 外，机器人会用 Nav2 的连续
  through-poses 路径穿过它，然后继续到用户原目标；
- 三者都使用 `map` 坐标，route 的 map binding 必须和 regions、bundle 完全一致；
- 本方案按“目标区触发”实现。若机器人可能从狗洞下游一侧开始，应另做方向判定，当前
  旧车 smoke 配置只允许从上游进入，不能把“任意摆放方向”写进结论。

启动实车前单独验证 route（参数与 regions 验证使用同一组 bundle 字段/hash）：

```bash
ros2 run rm_dog_hole_entry_gate validate_dog_hole_route \
  /data/rm27_maps/connected_test/REV/connected_test.route.yaml \
  --expected-map-id MAP_ID \
  --expected-map-revision REVISION \
  --expected-manifest-sha256 MANIFEST_SHA256
```

## 4. 构建和启动

```bash
colcon build --symlink-install --packages-up-to \
  rm_dog_hole_entry_gate rm_navigation_launch rm_navigation_bringup
source install/setup.bash
```

首次只检查参数合同，保持轮子离地或底盘急停。完整组合入口为：

```bash
ros2 launch rm_navigation_launch old_car_full_terrain_navigation.launch.py \
  map_bundle_override:=/data/rm27_maps/connected_test/REV/connected_test.bundle.yaml \
  activate_ramp_filter:=true \
  enable_dog_hole_route:=true \
  dog_hole_regions_file:=/data/rm27_maps/connected_test/REV/connected_test.regions.yaml \
  dog_hole_route_file:=/data/rm27_maps/connected_test/REV/connected_test.route.yaml \
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

`enable_dog_hole_route:=true` 会自动启用 entry pause 和旧车狗洞 Nav2/costmap profile，
但不会自动打开坡面 active A/B，所以命令中仍明确写出 `activate_ramp_filter:=true`。
速度由 `ramp_max_forward_speed`（坡面过滤 active 时）或 `dog_hole_max_forward_speed`
（坡面过滤未 active 时）决定，并受 `global_max_forward_speed` 的 velocity-smoother 与
串口全局上限约束；分段路由本身不再增加额外速度上限。`4.00 m/s` 只表示软件允许的
上界，不保证底盘能达到，也不绕过加速度、MPPI、障碍物和下位机限制。初次搭建新场地
仍应从低速开始。`mppi_forward_velocity_std` 控制 MPPI 对更高前向速度的探索范围，不是
速度上限；旧值 `0.20` 的实车短路径峰值约为 `0.59 m/s`，首轮提高只建议用 `0.60`，
不得直接随 `vx_max` 一起设成 `4.0`。未显式提供 bundle、regions 或 route 任一文件都会
拒绝启动。

当前 2026-08-31 实验文件已经生成在：

```text
/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_field.bundle.yaml
/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_connected_dog_hole_ramp.regions.yaml
/data/rm27_maps/old_car_field/20260831T073929Z_ps_corridor/old_car_connected_dog_hole.route.yaml
```

它们的几何来自目测和一次静止定位，只能用于当前 smoke test。

启动后确认单一速度所有权：

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_dog_hole_gated --verbose
ros2 topic echo /dog_hole/pause_state
ros2 topic echo /dog_hole/path_crosses
ros2 topic echo /dog_hole/pause_active
ros2 topic echo /dog_hole/route_state
ros2 topic echo /dog_hole/route_active
ros2 action list -t | grep -E 'navigate_to_pose|navigate_through_poses'
```

预期：Nav2 发布 `/cmd_vel`，安全门是
`/cmd_vel_dog_hole_gated` 的唯一 publisher，真实串口只订阅 gated topic；action 列表应
同时存在公共 `/navigate_to_pose` 和内部 `/navigate_to_pose_direct`。不得出现串口同时订阅
原始 `/cmd_vel`，也不得有第二个节点发布 gated topic。

## 5. 推荐的分阶段实车门

### C00：错误绑定必须失败

把 regions 中的 revision 临时改成错误值。入口必须报 map binding mismatch，串口 gated
topic 没有可运动输出。恢复正确文件后再继续。

### C01：路径不经过狗洞

初始化定位，mission 保持 disabled，给一个不在 goal trigger 内的 RViz 目标。PASS：

- `/dog_hole/path_crosses == false`；
- route 状态短时为 `direct_navigation`；
- 不出现 5 s 暂停；
- `/navigate_to_pose_direct` 只收到原目标，车按普通 Nav2 行驶。

### C02：经过狗洞但尚未到 approach

从至少三个不同的上游起点分别发布坡后的同一个目标。PASS：route 先进入
`navigating_to_stop`，三个试次都先到同一 `stop_pose`，而不是直接规划至原目标。此阶段
`/plan` 不应穿越 committed corridor，gated 速度与 `/cmd_vel` 一致。

### C03：洞前停车 5 s

Nav2 报告 stop stage 成功后，route 进入 `transition_then_traverse` 并提交出口点；pause
状态必须按顺序出现：

```text
armed -> braking -> holding -> released
```

PASS：

- `braking` 开始后 gated 速度立即为零；
- 实际停车位置在 Nav2 goal checker 容差内接近 route `stop_pose`；
- 0.5 s 制动等待结束后，`holding` 连续不少于 5.0 s；
- 用 `/odometry/lio` 复核机器人在 holding 内确实静止；
- 同一方向一次穿越只触发一次；
- 发布新规划但仍穿洞，不能绕过正在执行的 hold。

### C04：狗洞 COMMITTED

放行后状态进入 `committed`。此阶段不要人为杀定位做危险实验；只观察正常短时消息抖动。
PASS：安全门不在洞内二次停车，出洞后依次 `passed -> armed`，对侧 approach 不误触发。

### C05：连续上坡

目标必须位于 route 的 goal trigger 内且在坡顶之后。对用户仍是同一个公共 Nav2 goal，
内部通过 stop 和 exit 两个阶段继续执行。PASS：

- 不重新发目标、不重新发初始位姿、不重启 launch；
- `/diagnostics` 中 `old_car_shared_ramp_filter/automatic_ramp_filter` 在坡前确认坡面；
- local costmap 不再用整片致命障碍封死坡面，坡上额外障碍仍保留；
- 车辆不从坡侧绕行，不在坡上持续左右振荡，不发生定位保护锁存；
- 最终到达坡顶后目标，定位和 TF 仍存在。

### C06：失效与回归

至少验证：路径取消、路径改为不穿洞、未初始化定位、错误 frame、重启后车已位于 corridor
内。最后一种必须进入 `invalid_entry` 并保持零速，不能假装已经做过洞前变形。人工把车
移回所有狗洞区域之外后，只有定位连续有效、新鲜且稳定，且安全区状态连续保持
`dog_hole_invalid_entry_clear_sec`，锁存才可自动清除；不再要求重启 launch。清除前不得
发送穿洞目标。

## 6. 录包与结论门

```bash
ros2 bag record \
  /plan /localization/global_pose /odometry/lio \
  /cmd_vel /cmd_vel_dog_hole_gated \
  /dog_hole/pause_state /dog_hole/pause_active /dog_hole/path_crosses \
  /dog_hole/route_state /dog_hole/route_active \
  /points/obstacles_fused /points/obstacles_ramp_filtered \
  /livox/left/pointcloud_filtered /livox/left/pointcloud_ramp_filtered \
  /local_costmap/costmap /global_costmap/costmap \
  /localization/global_localization_valid /diagnostics /tf /tf_static
```

每个起点单独记录试次，文件名包含 map revision、起点编号和方向。只有 C00--C06 全部
通过，才可以写“旧车在该组合场地完成了语义触发的 5 s 变形模拟，并在同一导航任务中
连续通过狗洞和坡道”。不能据此写“真实变形执行器合同已完成”“任意未知狗洞均可自动
识别”或“比赛级坡道能力已完成”。
