# 旧车无标注坡道选择性验证

## 结论与来源

旧车不重新实现整套地形导航。本分支复用 `main-new-car` 已有的坡面几何和仿真资产：

- `eda100c`：地图绑定 PointCloud2 坡面过滤和 11°/15° 合成点云；
- `070b5f2`：狗洞/斜坡统一仿真入口和可碰撞三维坡道；
- `1e8f44a`：仿真 LaserScan 主动过滤和 Nav2 物理穿越。

在此基础上增加无标注自动检测。它没有迁移 HWSentry 的空间/Kino A*、MINCO、轮腿
FDDP、terrain planner 或 terrain FSM。

## 默认工作方式

```text
带时间戳的 odom <- cloud_frame / base_link TF
                    +
机器人附近未经语义标注的三维点云
                    ↓
融合点云在 odom 中积累 300 ms，10 Hz RANSAC 拟合重力系斜面
                    ↓
坡度、宽度、长度、高差、残差和点数门控
                    ↓
同一 odom 平面 5 个检测周期确认，短时漏检后自动过期
                    ↓
共享模型同时过滤融合/左雷达流；坡上凸起继续作为障碍
```

不需要填写坡道坐标、多边形、坡脚高度、方向或坡度。默认输出：

- `/perception/ramp/localization_filtered_shadow`
- `/perception/ramp/obstacles_filtered_shadow`

shadow 点云不被 AMCL、costmap、Nav2、串口或底盘消费。缺少精确时间 TF、点云格式
异常、候选尚未确认或候选超时，节点都原样直通。显式 active A/B 才会把左雷达过滤
点云送给 AMCL/global scan，把融合过滤点云送给 local STVL，并使用低速、前向优先的
DiffDrive MPPI 候选。

自动检测只改变实时点云，不能越过 Nav2 静态层：PGM 中坡道对应通道必须本来就是可
规划区域。否则 global planner 仍会拒绝路径，这是正确的 fail-closed 行为。

## 通用阈值而非场地标注

默认参数在 `old_car_automatic_ramp_filter.yaml`。这些参数描述机器人和传感器能力，可以
跨场地使用：

- 搜索范围：车前约 3 m、左右各 1.2 m；
- 可接受坡度：5°–25°；
- 最小坡面：长 0.60 m、宽 0.55 m、高差 0.08 m；
- 平面拟合容差 0.025 m，实际滤除容差 0.05 m；
- 每 0.10 s 检测一次、积累 0.30 s 点云；
- 确认 5 个检测周期，pending 允许漏 2 个周期，confirmed 连续漏 8 个周期后撤销。

先改机器人能力阈值，不要为某一张地图填坐标。人工 `map_regions` 仅保留为重复实验或
比赛场地已知坡道的确定性回退，不是默认前置条件。

## 构建与静态测试

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_navigation_bringup rm_navigation_launch rm_nav_config
source install/setup.bash

colcon test --packages-select rm_mid360_driver_bridge rm_nav_config
colcon test-result --verbose
```

必须确认自动检测的未标注 11°/15° 坡面、纯平地拒绝、窄斜面拒绝和坡上障碍保留测试
通过，并确认 launch 默认是 `automatic + SHADOW`。

## R00：平地零误触发

不需要准备区域文件：

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py
```

该入口现在使用独立的 `map_bundle_override` 参数；留空时继承全功能入口用户配置区中的
地图，不会再把空字符串传成 `map_bundle_manifest`。临时换图时使用：

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py \
  map_bundle_override:=/absolute/path/field.bundle.yaml
```

在平地执行静止、直线、转弯和原地旋转，每项至少 30 秒。PASS：

- 日志显示 `SHADOW/automatic`；
- 原导航继续消费原始话题；
- 诊断可以短时出现 candidate，但 `confirmed` 不得持续变成 true；
- shadow 输出与原点云点数基本一致；
- 节点不发布 `/cmd_vel`、TF、Path 或 Nav2 action。

平地、矮台阶、墙脚或机器人自身点云被连续确认为坡道是硬 FAIL，必须先收紧检测条件。

## R01：静止坡面自动识别

将车停在坡前，不启动 mission。RViz 同时显示原始和 shadow 点云，并记录：

```bash
ros2 bag record \
  /livox/left/pointcloud_filtered \
  /points/obstacles_fused \
  /perception/ramp/localization_filtered_shadow \
  /perception/ramp/obstacles_filtered_shadow \
  /odometry/lio /localization/global_pose \
  /localization/global_localization_valid /tf /tf_static /diagnostics
```

PASS：约 0.5--1.0 s 后 `confirmed=true`，诊断坡度接近实物、宽度/长度超过下限；坡面主体
显著减少，坡外固定结构不消失。确认前的原样直通是预期行为。

## R02：坡上障碍保留

在坡面放置至少 0.12 m 高的固定障碍，重复 R01。PASS：坡面回波减少，但凸起点保留，
过滤后的 local costmap 候选仍形成致命障碍。坡面和障碍一起消失是硬 FAIL。

## R03：接近、上坡和候选连续性

保持 shadow，人工遥控以 0.10–0.20 m/s 接近、驶上、停在坡中，再退出。PASS：

- 候选在 `odom` 中保持相近中心、方向和坡度；
- 进入坡面后不会因为 `base_link` 俯仰立即丢失；
- 短时遮挡允许保留，超过约 0.8 s missed-cycle TTL 后恢复原样直通；
- 定位完整性和 TF 不受影响。

## R04：低速主动 A/B

前置条件：R00–R03 通过、路线清场、PGM 通道可规划、遥控/急停能立即夺权、mission
保持 disabled，先用 RViz 单目标测试。

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py \
  activate_filter:=true \
  max_forward_speed:=0.20 max_yaw_rate:=0.40
```

按顺序执行平地回归、坡前停车、0.20 m/s 上坡、0.20 m/s 下坡；全部通过后才尝试
0.35 m/s。PASS：

- global/local costmap 不再把已确认坡面主体标为不可通行；
- 坡上障碍仍能令控制器停车或绕行；
- `/localization/global_localization_valid` 全程为 true；
- 没有 correction gate 锁存、TF 消失或 AMCL 重定位；
- 没有明显横移、坡面自转、触底或持续打滑；
- 停车后位姿仍与地图对齐。

## R05：失效保护

分别断开 `odom <- cloud_frame` TF、回放零时间戳/缺字段点云，并让坡面离开视野超过 8
个检测周期（默认约 0.8 s）。PASS：诊断说明原因、删除点数为 0、原始点云直通，Nav2 把未确认结构继续当作
障碍，而不是错误放行。

## 可选：人工地图合同回退

只有自动检测无法稳定区分某个固定场地结构时，才复制并测量
`old_car_ramp_regions.example.yaml`，然后运行：

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py \
  detection_mode:=map_regions \
  regions_file:=/data/rm27_maps/old_car_ramp_regions.yaml \
  active_map_id:=MAP_ID active_map_revision:=REVISION
```

地图 ID/revision 或时间戳 TF 失配时同样原样直通。当前合同未绑定 manifest SHA256，
因此该回退文件仍不能自动称为比赛正式语义地图。

## 2026-08-30 实车录包进展

- 已取得 123.94 s 旧车真实斜坡包并复现两套 tracker 分歧；
- 已改为融合点云单一检测源和共享模型，同包回放实现平地零误删、两路同步滤坡；
- 详细证据、参数和复验步骤见
  `old_car_ramp_live_tracking_report_20260830.md`；
- 修复后的车辆主动 R00--R05 尚未重新执行，因此不能宣布比赛级通过；
- 自动几何只能判断“像可通行坡面”，不能证明机械离地间隙、抓地力或承载能力；
- 没有自动地形方向规划、弧长速度剖面或坡道 COMMITTED 状态；
- 没有下位机专用爬坡/悬挂模式合同。

因此本分支完成的是无标注感知、隔离接入和可复现实验合同，不是宣布旧车或新车已经
具备比赛级跨地形能力。
