# Old-Car Debug Knowledge Base

本文档整理 2026 老车验证过程中已经遇到、排查、修复或形成判断的问题。它不是最终 2027 实车标定文档，而是后续实车调试时可复用的故障手册。

适用范围：

- 分支：`experiment/old-car-2026-bringup`
- 旧车单左 MID360 验证入口
- ROS 2 Humble / Docker / Nav2 / FAST-LIO / Livox driver / real serial transport
- RViz、TF、点云、local costmap、短距离落地运动排查

不适用范围：

- 2027 新车最终双 MID360 外参
- 高速自转验证
- 最终比赛策略、行为树、裁判系统
- 新下位机四轮编码器协议验收

## 1. 当前阶段性结论

截至本轮整理，几个关键判断已经明确：

1. `base_link -> mid360_left_frame` 静态雷达外参已经经过机械测量和 RViz `Fixed Frame=base_link` 观察确认，基本可信，不应再用它去抵消其它 TF 问题。
2. `/livox/left/pointcloud` 和 `/livox/left/pointcloud_filtered` 的 `frame_id` 都是 `mid360_left_frame`，这是合理的，关键是 TF 链要正确。
3. 点云在 `base_link` 下地面变平，但在 `odom/map` 下曾经倾斜，根因不是静态雷达外参，而是 `lio_adapter` 错把 FAST-LIO raw odom 的传感器初始系语义当成 canonical odom 处理。
4. `lio_adapter` 已增加 `raw_odom_parent_frame_mode: sensor_initial` 模式，用于老车 FAST-LIO raw odom。修复后 `odom -> base_link` 的 roll/pitch 从十几度降到约 0.x 度。
5. local costmap 的异常黑块主要来自 local `voxel_layer` 对原始/近车点云的 marking。自车点云过滤链路方向正确，当前已阶段性缓解。
6. global costmap 不应重新接入原始动态点云，否则容易形成全局拖影污染。
7. 真实串口链路已经打通，但 Docker 默认容器不会自动看到 `/dev/ttyACM0`，必须用 serial compose override 映射设备。
8. 短目标落地运动出现圆弧/绕行时，当前证据更支持 Nav2/local costmap 误判或目标方向问题，不优先怀疑串口协议或底盘坐标映射。

## 2. 快速故障索引

| 现象 | 最可能来源 | 已知修复或排查方向 |
|---|---|---|
| RViz 中点云在 `base_link` 下倾斜 | `base_link -> mid360_left_frame` 静态外参错误 | 检查静态 TF，使用机械测量和旧代码外参修正 |
| RViz 中点云在 `base_link` 下平，但在 `odom/map` 下倾斜 | `odom -> base_link` 动态 TF 错误 | 检查 `lio_adapter`，启用 `sensor_initial` 模式 |
| `odom -> base_link` roll/pitch 约十几度 | FAST-LIO raw odom parent 语义被误解 | 使用 `T_base_sensor * T_sensor0_sensor * inverse(T_base_sensor)` |
| 机器人周围 local costmap 黑块包围 | local voxel_layer marking 自车点/低空噪声 | 使用 `/livox/left/pointcloud_filtered`，调 self filter 和高度过滤 |
| 移动后 local/global costmap 有残留拖影 | clearing/raytrace、PointCloud2 空射线不足、动态点云进 global | global 不吃动态点云，local 检查 clearing endpoint 和过滤后点云 |
| `/serial_transport_node` 不存在 | 容器内没有 `/dev/ttyACM0` 或节点启动即崩溃 | 用 serial compose override 重建容器 |
| 完整 launch 有路径但机器人不动 | 串口节点未启动、自动模式未开、设备未映射、cmd_vel 未输出 | 先查节点、设备、`/cmd_vel`、遥控自动模式 |
| 机器人短目标走圆弧而非直线 | local costmap 误占、目标方向不准、MPPI 避障输出 | 先看 `/cmd_vel` 分量和 costmap，不先改串口 |
| Livox driver 报 `found lidar not defined` | 网络中有额外 MID360 或 IP 配置不匹配 | 只保留目标雷达，或临时隔离额外 IP |

## 3. RViz 点云倾斜排查逻辑

### 3.1 先分清 PointCloud2 话题

常见话题：

```text
/livox/left/pointcloud
/livox/left/pointcloud_filtered
/lio/cloud_registered
/lio/cloud_registered_body
/lio/cloud_registered_transformed
/lio/map
/local_costmap/clearing_endpoints
```

判断：

- `/livox/left/pointcloud`：左 MID360 原始点云。
- `/livox/left/pointcloud_filtered`：点云自车过滤后的 local costmap 输入。
- `/lio/cloud_registered*`、`/lio/map`：LIO 输出，不应用来直接判断原始雷达外参。
- 原始点云和过滤点云的 `frame_id` 都应保持为 `mid360_left_frame`，不要为了 RViz 显示好看去改 `frame_id`。

检查命令：

```bash
ros2 topic list -t | grep PointCloud2
ros2 topic echo /livox/left/pointcloud --once | grep frame_id
ros2 topic echo /livox/left/pointcloud_filtered --once | grep frame_id
```

### 3.2 RViz 分层观察方法

判断静态雷达外参时：

- `Fixed Frame = base_link`
- 只显示 `Grid`
- 只显示 `/livox/left/pointcloud`
- 关闭 RobotModel、TF、filtered pointcloud、costmap、plan

预期：

- 地面应接近 Grid 平面。
- 墙体应接近竖直。

判断全局 TF 时：

- 再切到 `Fixed Frame = odom` 或 `map`
- 逐层检查 `map -> odom`、`odom -> base_link`、`base_link -> mid360_left_frame`

原则：

- 点云在 `base_link` 下歪，优先查静态雷达外参。
- 点云在 `base_link` 下平，但在 `odom/map` 下歪，优先查 `odom -> base_link`。

## 4. 静态雷达外参问题

### 4.1 初始错误现象

最初检查：

```bash
ros2 run tf2_ros tf2_echo base_link mid360_left_frame
```

输出类似：

```text
Translation: [0.150, 0.140, 0.240]
Rotation Quaternion: [0.000, 0.000, 0.000, 1.000]
RPY degree: [0.000, -0.000, 0.000]
```

问题：

- 只写了平移，没有写真实安装姿态。
- 对斜装 MID360 来说，这是占位外参。
- 它会导致原始点云在 `base_link` 下地面倾斜。

### 4.2 旧车机械测量信息

旧车左前 MID360 机械测量：

```text
x = 193.5 mm
y = -175.3 mm
z = 210.2 mm
normal = (3.91, -3.91, 11.87)
```

坐标注意：

- 旧测量以车头为 x 正方向。
- 当时使用左手坐标系。
- ROS 常用 `x forward, y left, z up` 右手系。
- 因此旧 `y=-0.1753` 转到 ROS 后约为 `y=+0.1753`。
- 旧外部 IMU 后来已经拆除，不能把旧“雷达到外部 IMU”外参直接当成当前“雷达到 MID360 内置 IMU”的外参。
- 这组机械量更适合用于 `base_link -> mid360_left_frame` 这类车体机械外参。

### 4.3 修正后的参考结果

修正后检查：

```bash
ros2 run tf2_ros tf2_echo base_link mid360_left_frame
```

期望接近：

```text
Translation: [0.194, 0.175, 0.210]
Rotation Quaternion: [0.005, 0.216, 0.707, 0.673]
RPY degree: [19.042, 16.475, 95.591]
```

旋转矩阵第三列应接近：

```text
[0.299, 0.299, 0.906]
```

该列与旧法向量 `(3.91, -3.91, 11.87)` 转换到 ROS 右手系并归一化后的结果一致。法向相对竖直方向总倾角约 25 度，不是简单 45 度。

注意：

- 不要只看 RPY 中某一个 pitch 值。
- RPY 是三维旋转分解，单独一个角不一定有直接机械含义。
- 更适合看旋转矩阵中代表雷达轴向/法向的列向量。

## 5. LIO 动态 TF 问题

### 5.1 现象

静态外参修正后，原始点云在 `base_link` 下已经变平，但在 `odom/map` 下仍然倾斜。

检查：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

当时结果：

```text
map -> odom: identity
odom -> base_link: roll ≈ 18.7 deg, pitch ≈ -17.2 deg, yaw ≈ -96 deg
```

判断：

- `map -> odom` 不是问题来源。
- `odom -> base_link` 错误带入了雷达安装角。
- `/tf` 发布者中 `odom -> base_link` 来自 `lio_adapter`。

### 5.2 raw odom 与 lio odom 对比

检查：

```bash
for t in /odometry/fast_lio_raw /odometry/lio
do
  echo "===== $t ====="
  timeout 3s ros2 topic echo $t --once | sed -n '1,80p'
done
```

关键结果：

```text
/odometry/fast_lio_raw:
  header.frame_id: odom
  child_frame_id: body
  orientation 接近单位姿态

/odometry/lio:
  header.frame_id: odom
  child_frame_id: base_link
  orientation 带明显安装角
```

`lio_adapter` 参数：

```yaml
base_frame: base_link
input_sensor_frame: lio_imu_link
backend_child_frame_alias_enabled: true
backend_child_frame_alias_source: body
backend_child_frame_alias_target: lio_imu_link
use_tf_sensor_to_base: true
raw_odom_topic: /odometry/fast_lio_raw
output_odom_topic: /odometry/lio
publish_tf: true
```

### 5.3 根因逻辑

FAST-LIO raw odom 的 parent 语义更接近：

```text
sensor_initial
```

也就是 raw pose 更像：

```text
T_sensor0_sensor
```

如果错误使用：

```text
T_odom_base = T_odom_sensor * inverse(T_base_sensor)
```

启动时 raw pose 接近单位姿态：

```text
T_odom_sensor ≈ I
```

输出就近似为：

```text
inverse(T_base_sensor)
```

这会把雷达安装姿态错误灌入 `odom -> base_link`。

正确处理方式是同时变换 parent 和 child 基：

```text
T_base0_base = T_base_sensor * T_sensor0_sensor * inverse(T_base_sensor)
```

这样启动时：

```text
T_sensor0_sensor ≈ I
T_base0_base ≈ I
```

### 5.4 修复方案

`lio_adapter` 增加 raw odom parent 语义模式：

```text
canonical_odom:
  T_odom_base = T_odom_sensor * inverse(T_base_sensor)

sensor_initial:
  T_base0_base = T_base_sensor * T_sensor0_sensor * inverse(T_base_sensor)
```

默认仍是 `canonical_odom`，避免影响其它配置。

老车配置使用：

```yaml
raw_odom_parent_frame_mode: sensor_initial
```

涉及文件：

```text
src/rm_localization_adapters/include/rm_localization_adapters/canonical_odometry.hpp
src/rm_localization_adapters/src/canonical_odometry.cpp
src/rm_localization_adapters/src/lio_adapter.cpp
src/rm_localization_adapters/test/test_canonical_odometry.cpp
src/rm_lio_bringup/config/lio_adapter_old_car_2026.yaml
```

### 5.5 修复后验证结果

检查：

```bash
ros2 param get /lio_adapter raw_odom_parent_frame_mode
ros2 run tf2_ros tf2_echo odom base_link
```

结果：

```text
raw_odom_parent_frame_mode: sensor_initial
odom -> base_link:
  RPY degree ≈ [0.2, 0.2, -0.4]
  或 [0.3, 0.2, -0.0]
```

对比修复前：

```text
RPY degree ≈ [18.7, -17.2, -96]
```

结论：

- 雷达安装角不再被错误带入 `base_link`。
- `/odometry/lio` 在静止平地启动时接近单位姿态。
- RViz `Fixed Frame=odom/map` 下点云地面恢复正常。

## 6. Local Costmap 自车污染问题

### 6.1 现象

RViz 中 local costmap 初始异常：

- 机器人周围约 1 m 实际无障碍。
- 但 local costmap 中 footprint 周围出现大量黑色障碍/膨胀区域。
- 看起来机器人被一圈障碍包住。
- 移动后还有拖影、残留障碍、旧障碍清不掉。
- 该问题在发导航目标前已经存在，因此不优先怀疑 MPPI/DWB。

### 6.2 已验证的来源

运行时关闭 local voxel layer：

```bash
ros2 param set /local_costmap/local_costmap voxel_layer.enabled false
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
```

现象：

- 黑色障碍块全部消失。
- 恢复 `voxel_layer.enabled=true` 后黑块重新出现。

结论：

```text
local costmap 黑块主要来自 local voxel_layer marking。
```

当时 local voxel layer 输入：

```yaml
observation_sources: cloud
cloud:
  topic: /livox/left/pointcloud
  marking: true
  clearing: true
  min_obstacle_height: 0.18
  max_obstacle_height: 2.0
  obstacle_min_range: 0.45
  raytrace_min_range: 0.0
  raytrace_max_range: 8.0
footprint_clearing_enabled: true
```

### 6.3 根因判断

最可能链路：

```text
/livox/left/pointcloud
  -> local costmap voxel_layer
  -> 近距离点 / 自车点 / 地面点 / 低空噪声 被 marking
  -> RViz 出现贴近机器人黑块
```

原因可能包括：

- MID360 扫到车体、轮子、线材、安装件。
- 原始点云有近距离噪声。
- 地面点或低矮点被当成障碍。
- PointCloud2 clearing 不像 LaserScan 那样有明显的 inf 空射线。
- local costmap 直接吃原始点云，承担了过多清洗工作。

### 6.4 阶段性修复方案

新增点云过滤链路：

```text
/livox/left/pointcloud
  -> pointcloud_self_filter_node
  -> /livox/left/pointcloud_filtered
  -> local costmap voxel_layer
```

原则：

- 不关闭 local voxel_layer。
- 不把 global costmap 重新接回原始动态点云。
- 不修改控制器参数来掩盖感知问题。
- 过滤参数可配置，便于回退和调参。

人工 RViz 验证结果：

- footprint 周围不再像最初那样被一圈异常黑色障碍包住。
- local costmap 仍有一些黑色区域，但多数位于场地障碍、墙体、边界和点云密集区域附近。
- global costmap 没有重新出现明显动态点云拖影污染。
- 当前作为阶段性修复可以保留。

后续可继续调：

- self filter box 尺寸。
- height filter 参数。
- 雷达外参。
- raw/filtered pointcloud 分开显示，避免 RViz 叠加误判。

### 6.5 动态障碍残留与 LaserScan 清除实验

后续实车测试确认：

- 机器人可以通过 RViz 目标点基本准确到达目标，主链路已经可用。
- 修正 footprint 到约 `0.64 m x 0.54 m`、`footprint_padding=0.02` 后，通道可通行性改善。
- local inflation 从 `inflation_radius=0.45, cost_scaling_factor=3.0` 调为 `0.30 / 6.0` 后，1.4 m 通道通过能力明显改善。
- 新的主要问题是人从车前横穿离开后，local costmap 会沿人的轨迹残留黑色障碍，一部分不会自然消失，只有机器人移动到附近甚至 footprint 重合时才消失。

判断：

```text
PointCloud2 + VoxelLayer 可以 marking，但没有 LaserScan inf 空射线那样稳定的“这里已经空了”表达。
残留只有靠近 footprint 后才消失，说明当前很大一部分清除依赖 footprint_clearing_enabled，而不是稳定 raytrace clearing。
```

阶段性实验方案：

```text
/livox/left/pointcloud
  -> pointcloud_self_filter_node
  -> /livox/left/pointcloud_filtered
  -> pointcloud_to_laserscan_node
  -> /local_scan
  -> local costmap obstacle_layer
```

该方案保留原 VoxelLayer YAML，同时新增 scan-based local costmap YAML：

```text
src/rm_nav_config/config/nav2_old_car_2026_left.yaml
  原 VoxelLayer 路径，作为回退基线。

src/rm_nav_config/config/nav2_old_car_2026_left_local_scan.yaml
  实验路径：/local_scan + ObstacleLayer + inf_is_valid。
```

启动时需要同时启用 scan projection 并指定 scan YAML：

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_nav2:=true \
  use_real_serial:=true \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  pointcloud_filter_enabled:=true \
  local_scan_enabled:=true \
  nav2_params:=/workspace/rm2027_navigation/install/rm_nav_config/share/rm_nav_config/config/nav2_old_car_2026_left_local_scan.yaml \
  serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200
```

验证标准：

- `/local_scan` 存在，`frame_id=base_link`。
- `/local_costmap/local_costmap` 订阅 `/local_scan`，不再订阅 `/livox/left/pointcloud_filtered`。
- 人从车前横穿离开后，local costmap 中轨迹残留应在 0.5 到 2 秒内明显消失。
- 静态障碍仍能稳定 marking。
- 1.4 m 通道仍能通过。
- 若效果变差，回退为默认 `nav2_old_car_2026_left.yaml` 并不启用 `local_scan_enabled`。

### 6.6 STVL 动态障碍残留实验结论与后续调参方向

当前 old-car 实车导航主链路已经基本可用：在 RViz 中发布目标点后，机器人可以到达目标。此前通道问题已经确认主要来自 footprint 和 local inflation 偏保守，导致真实约 1.4 m 通道在 local costmap 中被障碍和膨胀层挤满。

已经保留的 old-car 落地调试参数：

```yaml
footprint: "[[-0.32, -0.27], [-0.32, 0.27], [0.32, 0.27], [0.32, -0.27]]"
footprint_padding: 0.02

inflation_layer:
  inflation_radius: 0.30
  cost_scaling_factor: 6.0
```

这组参数更接近实车约 `0.60 m x 0.50 m` 的尺寸，同时仍保留少量安全余量。实测通道通过能力明显改善，应继续保留，暂不把通道问题归因到 controller 或 MPPI critic。

后续主要问题转为 local costmap 动态障碍残留：

- 人从车前经过并离开后，local costmap 中会留下黑色障碍轨迹。
- 部分残留不会自然消失，只有机器人靠近甚至 footprint 压过去后才被清掉。
- 该现象不是 RViz 假象，也不是 unknown space。
- `/local_costmap/costmap_raw` 中确实存在 `lethal=254` 和 `inscribed=253`。
- filtered pointcloud 在动态障碍离开后，机器人 1 m 内 markable 点可以降到很少，但 costmap 仍保留大量 lethal/inscribed。

因此当前根因判断为：

```text
标准 Nav2 VoxelLayer 对 MID360 PointCloud2 hit-only 输入的 clearing 不充分。
近场点云仍会制造障碍，但不是残留不消的唯一原因。
```

已尝试过的方向：

1. 调整 VoxelLayer 参数，例如 `observation_persistence: 0.0`、`mark_threshold`、`obstacle_max_range`、`raytrace_max_range`。有一定改善，但不能根治。
2. 增加 `PointCloud2 -> LaserScan -> ObstacleLayer` 清除链路。效果不稳定，不能作为最终解。
3. 自定义 `TimedObstacleLayer`。方向合理，但实测不理想，维护成本也偏高，暂不作为主线推进。

当前 STVL 实验 profile 已能运行。RViz 观察显示，STVL 对动态障碍残留的清除明显优于默认 VoxelLayer：动态障碍离开后，残留大约在 `0.5 s` 左右开始消失。这说明“带时间衰减的 voxel layer”方向有效。

STVL 的核心价值：

```text
不再完全依赖 PointCloud2 clearing ray 必须穿过旧 obstacle cell；
未被持续观测到的 voxel 会随时间过期；
动态障碍残留因此更容易自然消失。
```

但 STVL 当前不是最终完成状态。实测仍观察到：

- 静态障碍物有时会被清除一瞬间，然后重新生成。
- 这可能说明 `voxel_decay` / `decay_model` / clearing frustum 参数偏激进。
- 也可能说明 MID360 对某些静态障碍的观测不够连续，导致 STVL 将其短暂判为过期。

该现象在实验阶段可以接受，但不能忽略。后续 STVL 调参目标是：

- 动态障碍离开后，残留在 `0.5-1.5 s` 内明显消失。
- 静态障碍持续存在时，不出现明显整块闪烁或周期性清空。
- 1.4 m 通道仍能通过。
- 不破坏 RViz 发布目标点后基本准确到达的能力。

当前策略：

- 默认 `nav2_old_car_2026_left.yaml` 继续作为 VoxelLayer 主线配置。
- STVL 只作为显式实验 profile，例如 `nav2_old_car_2026_left_stvl.yaml`。
- 不把 STVL 直接设为默认。
- 不修改 MPPI、planner、controller、serial、BT。
- 不继续把 `TimedObstacleLayer` 作为推荐主线；如保留，也只能作为历史实验分支。
- `costmap_inspector` 诊断工具仍需保持可用，后续用于定量比较 VoxelLayer 与 STVL 下 lethal/inscribed 数量的变化。

推荐 STVL 后续调参优先级：

1. 先调 `voxel_decay`，从当前偏快衰减逐步放慢，例如围绕 `1.0-2.0 s` 试验。
2. 再检查 STVL 的 clearing frustum，例如 vertical FOV、vertical offset、model type、clearing observation source。
3. 观察静态障碍闪烁是否和点云稀疏、遮挡或 filtered pointcloud 有关。
4. 不要同时改 controller、inflation、footprint、serial，避免混淆变量。

#### global path 穿过 local 黑区的解释

当前 RViz 中有时会看到 global path 直接穿过 local costmap 的黑色障碍区域，但机器人实际会绕可走区域过去。这不是串口问题，也不优先是 controller 问题。

原因是 global/local costmap 信息不一致：

- RViz 的全局路径 `/plan` 来自 `planner_server + global_costmap`。
- 当前 old-car global costmap 基本不包含动态障碍或静态地图信息。
- 因此 global planner 看到的是较自由的空间，可能规划出穿过 local 黑区的直线路径。
- controller/MPPI 使用 local costmap，会根据 local 障碍进行局部避让或停车。

所以现象会表现为：

```text
全局路径看起来穿过障碍；
机器人实际不按这条线硬穿，而是根据 local costmap 绕行或停住。
```

若后续希望 global path 也绕障碍，需要引入全局地图/static layer，或谨慎给 global costmap 加 obstacle layer。但在 dynamic residual 尚未稳定前，不建议急着把动态障碍加入 global costmap，否则可能把局部假障碍扩散成全局假障碍。

## 7. Livox 网络和多雷达干扰

### 7.1 IP 配置不匹配导致 TF 断裂

曾出现：

- `/odometry/fast_lio_raw` 无数据。
- `/odometry/lio` 无数据。
- `odom -> base_link` 不存在。
- TF 树断裂。

根因：

- Livox driver 实际只发现 `192.168.1.3`。
- 配置中写的是 `192.168.1.166`。
- driver 不发布 `/livox/left/lidar` 和 `/livox/lio_imu_raw`。
- FAST-LIO 没有 raw odom。
- lio_adapter 无法发布 `odom -> base_link`。

修复：

```text
src/rm_mid360_driver_bridge/config/left_mid360_config.json
ip: 192.168.1.166 -> 192.168.1.3
```

验证：

- `/livox/left/lidar` 约 50 Hz。
- `/livox/lio_imu_raw` 约 200 Hz。
- `/odometry/fast_lio_raw` 约 50 Hz。
- `/odometry/lio` 约 50 Hz。
- `odom -> base_link` 出现。

### 7.2 多余 Livox 设备干扰

曾出现：

- driver 发现未配置的 `192.168.1.166`。
- 日志有 `found lidar not defined`、`Can not get index`、`Storage point data failed`。
- 点云/costmap 可能受污染。

临时处理：

- 物理上只保留目标雷达。
- 或临时使用宿主机 iptables 隔离额外 IP。

隔离后观察：

```text
found lidar not defined = 0
Can not get index = 0
Storage point data failed = 0
```

注意：

- 临时 iptables 规则验证结束后应移除。
- 最终方案应尽量通过物理连接和明确 IP 配置解决。

## 8. 真实串口和 Docker 映射问题

### 8.1 现象

完整 launch 中想启真实串口：

```bash
use_real_serial:=true
serial_device:=/dev/ttyACM0
```

但检查：

```bash
ros2 node info /serial_transport_node
```

返回：

```text
Unable to find node '/serial_transport_node'
```

### 8.2 根因

单独启动 serial transport 后发现：

```text
open /dev/ttyACM0: No such file or directory
```

宿主机能看到：

```bash
ls -l /dev/ttyACM0
stat -c '%g %G' /dev/ttyACM0
```

输出：

```text
crw-rw---- 1 root dialout ... /dev/ttyACM0
20 dialout
```

但容器内看不到。根因是默认 Docker compose 没有映射串口设备。

### 8.3 修复流程

使用 serial override：

```bash
ls -l /dev/ttyACM0
export SERIAL_DEVICE=/dev/ttyACM0
export DIALOUT_GID=$(stat -c '%g' /dev/ttyACM0)
sudo -E docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  up -d --force-recreate rm2027_nav
sudo docker exec -it rm2027_navigation_humble bash
```

容器内确认：

```bash
ls -l /dev/ttyACM0
python3 - <<'PY'
import os
print(os.path.exists("/dev/ttyACM0"), os.access("/dev/ttyACM0", os.R_OK | os.W_OK))
PY
```

再启动：

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_nav2:=true \
  use_real_serial:=true \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200
```

### 8.4 正常状态

正常时：

- `/serial_transport_node` 存在。
- 节点订阅 `/cmd_vel`。
- 日志有 `REAL SERIAL TRANSPORT ENABLED`。
- 无 `Failed to write`。
- 不存在 `/serial/mock_tx`，因为这是真实串口，不是 dry-run。

## 9. 底盘运动和 Nav2 输出判断

### 9.1 已确认链路

已经打通：

```text
真实 MID360
  -> FAST-LIO
  -> lio_adapter
  -> Nav2
  -> /cmd_vel
  -> serial_transport_node
  -> /dev/ttyACM0
  -> 下位机
```

真实串口使用：

```text
legacy_v1_no_crc
baudrate = 115200
device = /dev/ttyACM0
```

### 9.2 自动模式前提

用户现场确认：

- 不是发出串口指令后底盘就一定动。
- 需要遥控器切到自动模式。
- 切回手动模式会由下位机控制停止。
- 测试时必须由用户明确确认“已进入自动模式”后再发目标。

### 9.3 圆弧运动问题

短目标曾出现：

- 目标为当前正前方 0.3 m。
- 机器人实际走约四分之一圆弧，逆时针。
- `/cmd_vel` 中 `vy` 较大，`wz` 也非零。

当时观察：

```text
vx ≈ 0.20
vy ≈ 0.28
wz ≈ 0.09
```

结合现场：

- 真实串口链路是通的。
- 底盘运动方向与 `/cmd_vel` 中大 `vy` 和正 `wz` 大体一致。
- 问题不优先指向串口协议或底盘坐标。
- 更可能是 local costmap 误占、点云污染、目标相对方向不准或 controller 基于障碍绕行。

### 9.4 不要急着调控制器

如果发目标前 local costmap 已经异常，控制器输出绕行是合理后果。

优先级：

1. 点云和 TF 是否正确。
2. local costmap 是否干净。
3. `/cmd_vel` 是否符合目标方向。
4. 真实串口是否正确发送。
5. 最后才考虑 MPPI/DWB 参数。

## 10. 低速死区和开环测试结论

开环只启动 `serial_transport_node` 时确认：

- `vx=0.30` 稳定前进。
- `vx=-0.30` 稳定后退。
- `vy=0.30` 稳定向左。
- `vy=-0.30` 稳定向右。
- `wz=0.60` 当时不动。
- `wz=1.0` 可见稳定旋转。

判断：

- 三分量方向映射基本正确。
- 低速死区明显，尤其角速度。
- 但比赛常用速度通常高于这些极低值，低速死区不是当前主要阻塞项。
- 老车雷达与底盘固连，不建议用原地自转作为 2027 云台雷达方案的有效验证。

## 11. 开机后建议流程

### 11.1 宿主机检查

```bash
cd /workspace/rm2027_navigation  # 按现场实际路径
ls -l /dev/ttyACM0
stat -c '%g %G' /dev/ttyACM0
```

若没有 `/dev/ttyACM0`，先处理 USB/下位机连接，不要启动真实串口测试。

### 11.2 重建带串口的容器

```bash
export SERIAL_DEVICE=/dev/ttyACM0
export DIALOUT_GID=$(stat -c '%g' /dev/ttyACM0)
sudo -E docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  up -d --force-recreate rm2027_nav
sudo docker exec -it rm2027_navigation_humble bash
```

### 11.3 容器内准备

```bash
cd /workspace/rm2027_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ls -l /dev/ttyACM0
```

### 11.4 启动老车完整链路

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_nav2:=true \
  use_real_serial:=true \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  pointcloud_filter_enabled:=true \
  serial_protocol_profile:=legacy_v1_no_crc \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200
```

### 11.5 发目标前检查

```bash
ros2 node list | grep serial_transport
ros2 node info /serial_transport_node
ros2 topic hz /odometry/lio
ros2 run tf2_ros tf2_echo odom base_link
ros2 param get /lio_adapter raw_odom_parent_frame_mode
```

RViz 检查：

- `Fixed Frame=map`。
- RobotModel 正常。
- filtered pointcloud 正常。
- local costmap 没有把机器人包住。
- global costmap 没有动态拖影污染。

### 11.6 安全发目标

1. 遥控器在手。
2. 场地周围清空。
3. 先发零命令或确认机器人静止。
4. 用户确认切到自动模式。
5. 只发很短目标，例如正前方 0.2 到 0.3 m。
6. 观察方向不对立即 cancel 并补零。

## 12. 重要诊断命令集合

### TF

```bash
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link mid360_left_frame
```

### 点云

```bash
ros2 topic list -t | grep PointCloud2
ros2 topic echo /livox/left/pointcloud --once | grep frame_id
ros2 topic echo /livox/left/pointcloud_filtered --once | grep frame_id
ros2 topic hz /livox/left/pointcloud
ros2 topic hz /livox/left/pointcloud_filtered
```

### LIO

```bash
ros2 topic hz /odometry/fast_lio_raw
ros2 topic hz /odometry/lio
ros2 topic echo /odometry/fast_lio_raw --once
ros2 topic echo /odometry/lio --once
ros2 param get /lio_adapter raw_odom_parent_frame_mode
```

### Nav2 和 costmap

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info /local_costmap/costmap -v
ros2 topic info /global_costmap/costmap -v
ros2 param get /local_costmap/local_costmap voxel_layer.enabled
```

### 串口

```bash
ls -l /dev/ttyACM0
ros2 pkg executables rm_serial_driver
ros2 node info /serial_transport_node
ros2 topic echo /cmd_vel --once
```

## 13. 回退和排查原则

### 不要做的事

- 不要为了让 RViz 好看而改点云 `frame_id`。
- 不要用错误的静态外参抵消 `odom -> base_link` 动态 TF 错误。
- 不要关闭 local voxel_layer 作为最终方案。
- 不要让 global costmap 重新吃原始动态点云。
- 不要在 costmap 明显异常时优先调 MPPI/DWB。
- 不要直接复制 2026 `serial_task` 到 2027 主线。

### 推荐分层排查

1. `base_link -> mid360_left_frame`：只看 raw 点云，Fixed Frame=`base_link`。
2. `odom -> base_link`：检查 lio_adapter 输出，Fixed Frame=`odom`。
3. `map -> odom`：确认是否 identity 或由重定位桥接正确发布。
4. 点云过滤：对比 raw 和 filtered，不要同时叠加误判。
5. local costmap：确认只订阅 filtered pointcloud。
6. `/cmd_vel`：确认 Nav2 输出是否符合目标方向。
7. 串口：确认节点存在且设备映射正确。
8. 底盘：确认遥控自动模式和现场运动。

## 14. 后续待办

- 将当前 old-car 参数保持为阶段性验证配置，不作为 2027 最终标定。
- 继续用 RViz 分别观察 raw pointcloud 和 filtered pointcloud，精调 self filter box。
- 检查 filtered pointcloud 中是否仍有车体、地面或线材点。
- 在 costmap 干净后再做短距离真实运动目标。
- 对 2027 新车重新测量最终外参，不沿用 old-car 外参。
- 新车下位机协议和轮速反馈仍需单独验收。
