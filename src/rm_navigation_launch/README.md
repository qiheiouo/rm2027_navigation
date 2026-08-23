# rm_navigation_launch

这个包只保存面向日常使用的启动入口，不实现驱动、定位、规划、串口或行为树逻辑。
实际节点仍由各自原包拥有，避免在用户 launch 中复制实现。

## 日常入口

| 用途 | launch |
| --- | --- |
| 实车建图 | `old_car_mapping.launch.py` |
| RMUC 2026 场地新车仿真（NUC 轻量版） | `field_geometry_simulation.launch.py` |
| RMUC 2026 场地新车仿真（高性能版） | `field_geometry_simulation_full.launch.py` |

仓库中的其他 launch 是上述入口复用的底层组件或回归测试入口，不需要在日常调试时
直接选择，也不代表会同时启动实车和仿真硬件。

### 旧车建图

编辑 `launch/old_car_mapping.launch.py` 顶部的用户配置区，然后运行：

```bash
ros2 launch rm_navigation_launch old_car_mapping.launch.py
```

默认启动左 MID360、FAST-LIO、OctoMap/地图保存节点和 RViz；不会启动 Nav2、串口或
mission。`FEATURES.add(...)` 每行都是一个模块，注释对应行即可关闭。

### 内部基线：旧车全功能导航

`old_car_full_navigation.launch.py` 是保留的旧车内部基线，不作为新车日常入口。
它集中保存旧车硬件、定位、Nav2、mission 和安全门的共有配置。

主要路径为：

- `MAP_BUNDLE_YAML`：地图 bundle YAML；它内部引用真正的 occupancy YAML/PGM/PCD；
- `MISSION_TREE_XML`：行为树 XML，留空使用三点巡逻自转候选；
- `MISSION_CONFIG_YAML`：home、巡逻点和任务参数；
- `NAV2_CONFIG_YAML`：Nav2 参数；
- `RELOCALIZATION_CONFIG_YAML`：AMCL/GICP 参数；
- `SCAN_PROJECTION_CONFIG_YAML`：点云转扫描及 deskew 参数。

为保留现有地图 hash/path/schema 门，入口不直接接受裸 `map.yaml`。应把该文件放入
`rm_map_tools` bundle，再将 `*.bundle.yaml` 填入 `MAP_BUNDLE_YAML`。

`FEATURES` 中注释某一行会关闭该模块。若它是必要上游，依赖模块也会自动关闭。例如
注释 `relocalization` 后，Nav2、真实串口和 mission 会一起关闭，而不是创建未经全局
定位的可运动系统。

默认全功能入口使用实车高限速候选，但 `mission_startup_enabled` 固定为 false。启动、
发布 2D Pose Estimate 并确认定位后，仍需显式调用 mission 服务。裁判 gate 默认不使用
mock；同一 `serial_transport_node` 使用已确认的 `hpm_crc_v1` 接收当前下位机 45 字节
状态帧并发布 `/referee/state_raw`。

### RMUC 2026 场地仿真（两个性能入口）

两者共用 `rmuc2026_navigation_field.sdf`、坐标、机器人模型、斜坡过滤配置和狗洞
流程，功能上没有删减。轻量版适合当前 NUC：

```bash
ros2 launch rm_navigation_launch field_geometry_simulation.launch.py
```

高性能电脑使用：

```bash
ros2 launch rm_navigation_launch field_geometry_simulation_full.launch.py
```

| 参数 | 轻量版 | 高性能版 |
| --- | ---: | ---: |
| Gazebo 物理步长 | 2 ms | 1 ms |
| 平面雷达 | 360 beams @ 10 Hz | 720 beams @ 15 Hz |
| 全局代价地图 | 0.10 m @ 1 Hz | 0.05 m @ 2 Hz |
| MPPI batch | 400 | 1000 |
| BT loop | 50 ms | 10 ms |

两个入口默认都打开 Gazebo GUI 与 RViz，生成 `28 x 15 m` 导航级简化场地，包括
带 `0.80 x 0.25 m` 开口的有顶道路、11°/15°道路斜坡、10.5°中心高地、17°飞坡
和若干主要不可通行区域。默认 `auto_start:=false`，机器人保持静止便于检查几何；
需要执行红方道路狗洞接近、对齐和穿越流程时使用：

```bash
ros2 launch rm_navigation_launch field_geometry_simulation.launch.py \
  auto_start:=true
```

两个入口都只启动仿真，不打开真实 MID360 或串口。默认使用 `path_aligned` 控制配置，并在
精确匹配合成地图 ID/revision 时把已知斜坡表面的二维扫描回波从 Nav2 `/scan` 中剔除。
在 RViz 中，斜坡中心线应是空闲区；实体边缘附近仍可能保留黑色膨胀安全带。如果中心线
也是黑色，则不应认为斜坡功能已经通过。可用 `active_ramp_filter:=false` 复现未过滤基线。

当前 NUC 上轻量版已完成从 `(-11, -3)` 到红方 15° 坡中段及返回的低速导航验证。
高性能版不建议在 NUC 上启动；它应在用户的高性能电脑上完成首次运行和负载记录。

狗洞参数仍来自
`rm_dog_hole/config/dog_hole_sim.yaml`，也可以用 `dog_hole_config:=/absolute/path.yaml`
覆盖。运行时不会直接加载 1.25 GB 的官方 STEP；SDF 有意省略螺丝、电路板、纹理、
机构内部件和粗糙地形微小凸起，官方 STEP 只保留为后续离线几何复核来源。详细边界见
`docs/rmuc2026_navigation_field_simulation.md`。
修改源码中的 launch 文件后需要重新构建并 source：

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_simulation rm_navigation_launch
source install/setup.bash
```
