# 旧车坡道选择性验证

## 结论与来源

旧车不重新实现坡面算法。本分支选择性复用 `main-new-car` 上已经完成 Linux smoke 的
斜坡链路：

- `eda100c`：地图绑定的 PointCloud2 坡面过滤和 11°/15° 合成点云验证；
- `070b5f2`：狗洞/斜坡统一仿真入口和可碰撞三维坡道；
- `1e8f44a`：仿真 LaserScan 主动过滤，使 Nav2 能规划并物理穿越坡道。

旧车分支只吸收其中通用的平面几何库、PointCloud2/LaserScan 过滤节点和单元测试，
再增加隔离的旧车接入。没有迁移 HWSentry 的空间/Kino A*、MINCO、速度剖面、轮腿
FDDP 或 terrain FSM。

## 当前能力边界

```text
已测量的坡道多边形、坡脚高度、上坡方向、坡度
                    +
带时间戳的 map <- cloud_frame TF
                    ↓
       只删除预期坡面容差内的回波
                    ↓
       坡上凸起障碍和区域外点保持不变
```

默认 shadow 模式发布：

- `/perception/ramp/localization_filtered_shadow`
- `/perception/ramp/obstacles_filtered_shadow`

它们不被 AMCL、costmap、Nav2、串口或底盘消费。显式 active A/B 模式才会将：

- 左雷达过滤点云送入 AMCL 的点云转 LaserScan，因此也成为 global costmap 的动态扫描；
- 双雷达融合过滤点云送入 local STVL；
- MPPI 限制为低速、前向优先的 DiffDrive 候选，避免坡面上横移和大角速度。

地图 ID/revision 不一致、缺少精确时间 TF、无区域或点云格式异常时，过滤器原样直通。
因此错误合同不会把未知结构从障碍物输入中删除，但也意味着 Nav2 会继续把坡面视为
障碍并拒绝通行。

## 准备区域文件

复制：

```bash
cp install/rm_navigation_launch/share/rm_navigation_launch/config/old_car_ramp_regions.example.yaml \
  /data/rm27_maps/old_car_ramp_regions.yaml
```

对每个测试坡道测量并填写：

1. 当前 bundle 的 `map_id` 和 `revision`；
2. `map` 坐标系内覆盖完整坡面的多边形；
3. 坡脚中心或坡面参考点 `origin_xyz`；
4. 从低处指向高处的 `ascent_yaw_deg`；
5. 实测 `slope_deg`；
6. 初始保留 `surface_tolerance=0.04 m`，不得为了“滤干净”盲目扩大。

当前合同只绑定 map ID/revision，尚未绑定 manifest SHA256。区域文件和 bundle 必须作为
同一份验收资产归档；在补齐 hash 绑定前不能称为比赛正式语义地图。

## 构建与静态测试

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_navigation_bringup rm_navigation_launch rm_nav_config
source install/setup.bash

colcon test --packages-select rm_mid360_driver_bridge rm_nav_config
colcon test-result --verbose
```

必须确认 `test_ramp_plane_filter` 和旧车 ramp launch 合同测试通过。

## R00：默认不接管

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py \
  regions_file:=/data/rm27_maps/old_car_ramp_regions.yaml \
  active_map_id:=MAP_ID active_map_revision:=REVISION
```

PASS：

- 日志明确显示 `SHADOW`；
- `/localization/scan` 和 `/points/obstacles_fused` 的原消费关系不变；
- 两个 shadow 输出存在；
- 节点不发布 `/cmd_vel`、TF、Path 或 Nav2 action。

## R01：静止坡面 shadow

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

PASS：坡面主体显著减少，坡外固定结构没有消失，诊断显示地图合同有效且没有 TF
passthrough。若原点、方向或坡度不准，修正测量值，不放宽容差。

## R02：坡上障碍保留

在坡面放置至少 0.12 m 高的固定障碍，重复 R01。PASS：障碍点保留，且过滤后的 local
costmap 候选仍能形成致命障碍。任何“坡面和障碍一起消失”均为硬 FAIL。

## R03：低速主动 A/B

前置条件：测试路线清场、PGM 中坡道通道可通行、遥控/急停可立即夺权、mission 保持
disabled，先用 RViz 单目标测试。

```bash
ros2 launch rm_navigation_launch old_car_ramp_validation.launch.py \
  activate_filter:=true \
  regions_file:=/data/rm27_maps/old_car_ramp_regions.yaml \
  active_map_id:=MAP_ID active_map_revision:=REVISION \
  max_forward_speed:=0.20 max_yaw_rate:=0.40
```

按以下顺序执行，每次只改变速度：

1. 平地同路线往返，确认没有定位/避障回归；
2. 坡前 0.5 m 目标，确认对正且可停车；
3. 0.20 m/s 上坡一次；
4. 0.20 m/s 下坡一次；
5. 通过后才尝试 0.35 m/s。

PASS：

- global/local costmap 不再把坡面主体标为不可通行；
- 坡上障碍仍能令控制器停车或绕行；
- `/localization/global_localization_valid` 全程为 true；
- 没有 correction gate 锁存、TF 消失或 AMCL 重定位；
- 车体没有明显横移、坡面自转、触底或轮胎持续打滑；
- 停车后位姿与地图仍对齐。

## R04：失配保护

使用错误的 `active_map_revision` 启动 active 模式。PASS：诊断报告合同不匹配、删除点数
为 0、原始点云直通，Nav2 仍把坡面当作障碍而不是错误放行。

## 尚未完成

- 当前场地没有坡道，因此没有旧车真实 R01-R04 数据；
- 没有自动地形方向规划、弧长速度剖面或坡道 COMMITTED 状态；
- 没有下位机专用爬坡/悬挂模式合同；
- 没有证明普通四轮旧车的机械离地间隙、抓地力或允许坡度。

因此本分支的“完成”含义是：复用算法和旧车隔离接入完成，等待有坡道时验证；不是宣布
旧车或新车已经具备比赛级跨地形能力。
