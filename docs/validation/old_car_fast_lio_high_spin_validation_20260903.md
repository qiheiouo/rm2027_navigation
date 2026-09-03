# 旧车 FAST-LIO 高速自转根因候选与实车验收（2026-09-03）

## 1. 结论和边界

2026-09-01 的 R03 已证明：最终一次持续高速自转不是单纯 AMCL 丢失，而是
`/odometry/lio` 在积压后发生约 10 m 的错误平移。已有全局修正门正确撤下了
`map -> odom`，但它只能防止错误位姿进入导航，不能阻止 FAST-LIO 先失效。

本分支从更靠近根因的位置做两项改动：

1. 双 MID360 模式下，FAST-LIO 默认直接接收驱动生成的原生 `CustomMsg`，保留
   `timebase + uint32 offset_time`；PointCloud2 改为从原生消息派生，仅供障碍感知。
2. 旧车 `lio_adapter` 拒绝超龄、未来、静默后突然跳远的后端输出，避免积压输出重新
   发布为 canonical `odom -> base_link`，并提供可录包的运行诊断。

第一项是“防止时序精度/转换链导致高动态失效”的根因候选，第二项是“即使后端仍失效也
不把错误结果交给导航”的安全边界。代码构建和自动化测试通过，但是否能让旧车持续高速
自转一分钟仍不丢定位，必须完成本文实车验收后才能下结论。

新车配置没有启用旧车的 LIO 输入门；本次参数和实车结论不得直接当作新车标定结果。

## 2. 证据链

### 2.1 已确认的现象

R03 录包和 ROS 日志位于：

```text
artifacts/bags/20260901_r03_spin_recovery_02/
artifacts/logs/20260901_r03_spin_recovery_02_ros/
```

离线时间对齐得到以下事实：

- 成功试次角速度约 `5.75 rad/s`；失败片段约 `6.44 rad/s`，说明它是负载/运动强度
  相关边界，而不是每次自转必现。
- 原始 PointCloud2 仍接近 50 Hz，每帧约 1400--3800 点，不能简单归因于雷达完全断流。
- `/odometry/lio` 的录制时刻减消息时间戳最高达到约 `3.495 s`，说明后端正在处理旧数据。
- 同期 FAST-LIO 大量输出 `No point` 和 `Too few downsampled points`，之后 odom 从原地
  跳到约 `(-0.73, 10.86, -0.14)`。
- 全局修正层随后拒绝 4--10 m 创新并保持 fail-closed；这解释了 RViz 中 RobotModel、
  点云和 costmap 在 `map` 固定坐标系下一起消失。

### 2.2 新发现的耦合

旧车单雷达入口此前使用 Livox 驱动原生 `CustomMsg`。启用右雷达/双雷达障碍融合后，入口
会切换到单个双设备驱动；旧实现先输出 PointCloud2，再根据其中的 `FLOAT64` 绝对纳秒值
重建 `CustomMsg` 给 FAST-LIO。

因此“增加右雷达做障碍物感知”曾间接改变左雷达 LIO 的消息表示和转换负载。右雷达没有
进入 FAST-LIO，但系统边界仍产生了隐式耦合。这也能解释早期单雷达版本可能可以高速自转，
而后来的完整导航入口重新出现问题。

这条耦合已被代码检查确认；它是否就是唯一根因仍需 A/B。不能仅凭转换链不同就宣称问题
已经解决。

## 3. 本次实现

### 3.1 双雷达原生时序路径

`dual_mid360_driver.launch.py` 新增参数：

```text
lio_input_mode:=native_custom         # 新默认
lio_input_mode:=reconstructed_custom  # 旧路径，仅供 A/B
driver_publish_freq:=50.0             # 默认值，可做 50/20 Hz A/B
```

`native_custom` 使用驱动 `xfer_format=1`，每个设备输出原生消息到
`/livox/{left,right}/lidar_native`。桥接器仅改 canonical frame id，并原样转发
`timebase`、`point_num`、`lidar_id` 和每点 `offset_time` 到
`/livox/{left,right}/lidar`；同时生成 `/livox/{left,right}/pointcloud`。

单雷达 fallback 保持原有 `xfer_format=4`，没有被本分支改写。

### 3.2 旧车 LIO 输入健康门

旧车参数启用：

```text
最大消息年龄                 0.20 s
允许未来偏移                 0.05 s
后端静默锁存阈值             0.20 s
恢复所需连续新鲜样本         5
恢复样本距最后可信平移上限   0.75 m
```

新增 topic：

```text
/localization/lio_runtime_valid
/localization/lio_runtime_status
```

状态中包含消息年龄、静默时间、拒绝计数、恢复计数、FAST-LIO 计算时间和下采样点数。
锁存时停止发布 canonical odom/TF；不会修改底层 FAST-LIO 状态，也不会伪造恢复位姿。

### 3.3 后端 A/B 入口

完整旧车入口现在可传：

```text
lio_update_method:=bundle    # 当前已用基线和默认值
lio_update_method:=async     # 仅 A/B
lio_update_method:=adaptive  # 仅 A/B
```

上游说明认为 async 更适合 aggressive motion，但当前旧车只在 bundle 上完成了完整零起点
工程验证。因此先验证原生时序；只有原生时序在 50/20 Hz 都失败后，才进行 async/adaptive
静止启动和低速测试，不能直接用未验证后端做最高速试验。

## 4. 构建和无车检查

在仓库根目录执行：

```bash
colcon build --packages-select \
  rm_mid360_driver_bridge rm_localization_adapters rm_lio_bringup \
  rm_navigation_bringup rm_navigation_launch rm_nav_config \
  --symlink-install --cmake-args -DBUILD_TESTING=ON

colcon test --packages-select \
  rm_mid360_driver_bridge rm_localization_adapters \
  rm_navigation_bringup rm_navigation_launch rm_nav_config \
  --event-handlers console_cohesion+
```

2026-09-03 本分支以上构建通过；五个选定包的测试 XML 没有 failure/error。工作区全局
`colcon test-result` 仍会统计 `livox_ros_driver2_humble` 第三方源码的历史 lint 失败，必须
按包区分，不能把它误判为本分支回归。

## 5. 实车启动与录包

当前地图为 `20260828T070722Z`，先使用普通完整导航入口，排除狗洞/坡道 profile 干扰：

```bash
ros2 launch rm_navigation_launch old_car_full_navigation.launch.py \
  map_bundle_yaml:=/data/rm27_maps/old_car_field/20260828T070722Z/old_car_field.bundle.yaml \
  driver_publish_freq:=50.0 \
  lio_input_mode:=native_custom \
  lio_update_method:=bundle
```

启动后、发布初始位姿前先确认消息类型和连接：

```bash
ros2 topic type /livox/left/lidar_native
ros2 topic type /livox/left/lidar
ros2 topic type /livox/left/pointcloud
ros2 topic hz /livox/left/lidar
ros2 topic hz /odometry/fast_lio_raw
ros2 topic echo --once /localization/lio_runtime_status
```

三个类型应分别为 `livox_ros_driver2/msg/CustomMsg`、同类型和
`sensor_msgs/msg/PointCloud2`。50 Hz 设置下，雷达和 raw odom 稳态应接近 50 Hz；若没有
`lidar_native` 或 canonical `lidar`，禁止开始运动。

每个 A/B 试次单独录包：

```bash
ros2 bag record -o /data/rm27_bags/high_spin_native50_bundle \
  /livox/left/lidar_native \
  /livox/left/lidar \
  /livox/left/pointcloud \
  /livox/lio_imu \
  /odometry/fast_lio_raw \
  /odometry/lio \
  /lio/diagnostics/calc_time \
  /lio/diagnostics/point_number \
  /lio/diagnostics/localizability_x \
  /lio/diagnostics/localizability_y \
  /lio/diagnostics/localizability_z \
  /localization/lio_runtime_valid \
  /localization/lio_runtime_status \
  /localization/scan \
  /localization/scan_deskew/status \
  /localization/amcl_pose_raw \
  /localization/global_pose \
  /localization/global_localization_valid \
  /localization/correction_recovery_state \
  /tf /tf_static
```

包名必须体现 `native50_bundle`、`native20_bundle` 或
`reconstructed50_bundle`，不要覆盖旧包。正常按 Ctrl-C 结束并确认存在 `metadata.yaml`。

## 6. 分档实车程序

全程要求空旷场地、遥控器可立即接管、mission disabled。每档之间停车并确认地图对齐。

### H01：静止和普通导航

1. 静止 30 秒；
2. 发布一次正确初始位姿；
3. 做一段低速直线和低速转向；
4. 确认 `lio_runtime_valid=true`、全局定位 healthy、costmap 连续。

### H02：原生 50 Hz 分档自转

按同一方向依次执行，任一档失败立即停止，不跳到下一档：

1. 约 3 rad/s，10 秒；
2. 约 5 rad/s，10 秒；
3. 约 6.4 rad/s，10 秒；
4. 目标比赛最高角速度，30 秒；
5. 目标比赛最高角速度，60 秒。

每档停止后观察 10 秒，再给一个安全的 home 目标。目标要求不是“停车后靠人工重发位姿
恢复”，而是整个自转期间 LIO 不产生积压/跳变、停车后可直接执行 home。

### H03：频率 A/B

仅当 H02 在 50 Hz 仍出现后端积压时，保持其他参数不变，改为：

```text
driver_publish_freq:=20.0
```

重新执行 H01、H02。不能在同一个 launch 内动态改变频率后继续比较。

### H04：旧转换链对照

只有需要证明本次耦合时才使用：

```text
lio_input_mode:=reconstructed_custom
driver_publish_freq:=50.0
lio_update_method:=bundle
```

先做 3 rad/s 和 5 rad/s。若已出现明显延迟、运行门锁存或地图错位，即可结束对照，不必
故意复现 10 m 跳变。

### H05：后端方法 A/B

仅当 native 50 Hz 和 20 Hz 都不能通过时开展。先分别用 `async`、`adaptive` 做静止
30 秒、低速直线、低速自转，并检查启动坐标和 TF 连续性。任何启动偏移、异常 TF 或 odom
跳变都判 FAIL，不能进入高速档。

## 7. PASS/FAIL 门

完整通过需要同时满足：

- 原生消息、raw odom 和 canonical odom 无持续断流；
- `/odometry/fast_lio_raw` 消息年龄不超过 `0.20 s`，没有秒级增长趋势；
- `lio_runtime_valid` 在 60 秒最高速自转中保持 true；
- raw/canonical LIO 平移没有与实车运动不符的跳变；
- 全局修正门不因 LIO 错误触发，RobotModel、点云和 costmap 不因 canonical TF 撤下而消失；
- 停止自转后不等待人工重发初始位姿即可执行 home；
- 停车后地图、点云和实车位置人工核对仍对齐。

以下任一情况为 FAIL：

- 消息年龄超过 0.20 秒并持续增长；
- `raw_odometry_silence`、`stale_raw_odometry` 或
  `post_backlog_translation_mismatch` 锁存；
- raw odom 原地平移超过 0.75 m，或出现历史约 10 m 同类跳变；
- 仅靠 5--10 秒停车或人工 `/initialpose` 才能继续 home；
- 定位状态显示 healthy，但人工检查地图实际不对齐。

安全门触发时，车辆必须先停止。它会防止错误位置继续导航，但不会在车辆仍持续高速自转
时凭空恢复可靠的全局位姿。

## 8. 决策规则

- `native50 + bundle` 通过 60 秒最高速：保留默认，进入 5 次重复验收。
- 50 Hz 失败、20 Hz 通过：暂定旧车 20 Hz，并比较普通导航精度和低速稳定性后再合入。
- native 通过、reconstructed 失败：可确认双雷达消息转换链是主要根因。
- 两种输入模式都积压，且 `calc_time` 明显超过周期：根因在后端计算负载/算法更新方式，
  转入 async/adaptive A/B，不再继续调整 AMCL 门。
- raw odom 健康但只有全局定位失效：回到 AMCL/地图修正层分析，与本轮 FAST-LIO 根因
  分开处理。

## 9. H02 首轮实车结果：native 50 Hz + bundle

2026-09-03 使用地图 `20260828T070722Z` 完成第一轮。现象为画面再次消失，曾较快恢复，
但停车后的持续观测显示 LIO 健康门仍会重复开闭，因此结果为 **FAIL**，不能按“已经恢复”
验收。

现场证据：

```text
输入模式                         native_custom
驱动频率                         50 Hz
后端                             bundle
首次运行中观测的消息年龄         0.322 s
同期 FAST-LIO calc_time          152.648 ms
同期下采样点数                   523
后续停车时消息年龄               0.222 s
后续停车时 calc_time             44.770 ms
后端/雷达短窗平均输出            约 45--46 Hz
短窗最大消息间隔                 约 0.34--0.36 s
```

50 Hz 每帧预算只有 20 ms，而本轮计算时间达到 152.648 ms；即使停车后降至 44.770 ms，
仍高于一帧预算。`lio_adapter` 分别记录到 `stale_raw_odometry`，拒绝计数由 29 增至 45，
并发生 2 次 silence latch。由此可以确认本轮直接触发原因是 FAST-LIO bundle 计算积压，
不是 Livox 完全断流，也不是 AMCL 单独丢失。

本轮还暴露出安全状态没有完全贯通：`lio_runtime_valid=false` 时，旧版
`global_localization_valid` 和 `/system/readiness` 仍可能短暂报告 true。后续补丁已使旧车
`map -> odom` 输出和 readiness 同时依赖 `/localization/lio_runtime_valid`；底层锁存时将明确
报告 `lio_health` 缺失。相关构建及测试通过，下一轮重启后生效。

本轮关键日志已封存在宿主机：

```text
artifacts/logs/20260903_native50_bundle_01/
```

下一步按 H03 执行 `native_custom + bundle + 20 Hz`。若 20 Hz 仍积压，不放宽 0.20 s
安全门，转入 async 后端的静止/低速/高速分档 A/B。
