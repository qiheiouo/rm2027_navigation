# rm_navigation_launch

这个包只保存面向日常使用的启动入口，不实现驱动、定位、规划、串口或行为树逻辑。
实际节点仍由各自原包拥有，避免在用户 launch 中复制实现。

## 入口

### 旧车建图

编辑 `launch/old_car_mapping.launch.py` 顶部的用户配置区，然后运行：

```bash
ros2 launch rm_navigation_launch old_car_mapping.launch.py
```

默认启动左 MID360、FAST-LIO、OctoMap/地图保存节点和 RViz；不会启动 Nav2、串口或
mission。`FEATURES.add(...)` 每行都是一个模块，注释对应行即可关闭。

### 旧车全功能导航

编辑 `launch/old_car_full_navigation.launch.py` 顶部的用户配置区，然后运行：

```bash
ros2 launch rm_navigation_launch old_car_full_navigation.launch.py
```

主要路径为：

- `MAP_BUNDLE_YAML`：地图 bundle YAML；它内部引用真正的 occupancy YAML/PGM/PCD；
- `MISSION_TREE_XML`：行为树 XML，留空使用三点巡逻自转候选；
- `MISSION_CONFIG_YAML`：home、巡逻点和任务参数；
- `NAV2_CONFIG_YAML`：Nav2 参数；
- `RELOCALIZATION_CONFIG_YAML`：AMCL/GICP 参数；
- `SCAN_PROJECTION_CONFIG_YAML`：点云转扫描及 deskew 参数。

这些路径也可以在命令行临时覆盖：

```bash
ros2 launch rm_navigation_launch old_car_full_navigation.launch.py \
  map_bundle_yaml:=/absolute/path/field.bundle.yaml \
  mission_tree_xml:=/absolute/path/tree.xml \
  mission_config_yaml:=/absolute/path/mission.yaml \
  nav2_config_yaml:=/absolute/path/nav2.yaml
```

为保留现有地图 hash/path/schema 门，入口不直接接受裸 `map.yaml`。应把该文件放入
`rm_map_tools` bundle，再将 `*.bundle.yaml` 填入 `MAP_BUNDLE_YAML`。

`FEATURES` 中注释某一行会关闭该模块。若它是必要上游，依赖模块也会自动关闭。例如
注释 `relocalization` 后，Nav2、真实串口和 mission 会一起关闭，而不是创建未经全局
定位的可运动系统。

当前旧车现场分支默认启用左/右 MID360 障碍点云融合，并选择
`nav2_old_car_2026_dual_stvl.yaml`。双雷达只增强局部 STVL 避障；FAST-LIO、AMCL
扫描和用于动态堵路重规划的全局 `ObstacleLayer` 仍由左 MID360 供数。启用所依赖的
旧车右雷达外参已由操作员于 2026-08-23 确认完成实车标定，但原始标定产物仍需
归档，且该值不得用于 2027 新车。

默认全功能入口使用双雷达低速现场候选（MPPI 前向/侧向上限 `0.50 m/s`），但
`mission_startup_enabled` 固定为 false。启动、
发布 2D Pose Estimate 并确认定位后，仍需显式调用 mission 服务。裁判 gate 默认不使用
mock；同一 `serial_transport_node` 使用已确认的 `hpm_crc_v1` 接收当前下位机 45 字节
状态帧并发布 `/referee/state_raw`。

### 旧车狗洞全功能验证

狗洞入口完整复用上述双雷达驱动/融合、LIO、AMCL、串口、裁判、mission 和安全门，只覆盖狗洞
候选地图、`0.55 m` 点云高度门以及前向穿越控制参数：

```bash
ros2 launch rm_navigation_launch old_car_dog_hole_navigation.launch.py
```

默认导航前进上限为 `0.80 m/s`、角速度上限为 `0.80 rad/s`。双雷达基线的
`velocity_smoother` 分别保留 `3.0 m/s` 和 `1.2 rad/s` 上限，因此不会截断默认过洞值；
超过该范围的显式参数仍会被 smoother 截断。可在保持其余全功能配置
不变的前提下临时调整：

```bash
ros2 launch rm_navigation_launch old_car_dog_hole_navigation.launch.py \
  max_forward_speed:=1.0 max_yaw_rate:=0.8
```

该入口只用于现有假狗洞和旧车验证。`obstacle_ceiling_height:=0.55` 会有意忽略更高的
悬空回波；真实狗洞、新车几何和最终雷达安装确定后必须重新测量，不能直接作为比赛值。

修改源码中的 launch 文件后需要重新构建并 source：

```bash
colcon build --symlink-install --packages-select rm_navigation_launch
source install/setup.bash
```
