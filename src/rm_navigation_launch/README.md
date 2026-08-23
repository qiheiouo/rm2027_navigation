# rm_navigation_launch

这个包只保存面向日常使用的启动入口，不实现驱动、定位、规划、串口或行为树逻辑。
实际节点仍由各自原包拥有，避免在用户 launch 中复制实现。

## 日常入口（只使用这三个）

| 用途 | launch |
| --- | --- |
| 能过洞的实车全功能导航 | `old_car_dog_hole_navigation.launch.py` |
| 实车建图 | `old_car_mapping.launch.py` |
| 有顶狗洞与 11°/15°斜坡的新车仿真 | `field_geometry_simulation.launch.py` |

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

`old_car_full_navigation.launch.py` 是过洞实车导航入口复用的内部基线，不再作为第四个
日常入口直接启动。它集中保存旧车硬件、定位、Nav2、mission 和安全门的共有配置；
`old_car_dog_hole_navigation.launch.py` 在此基础上只覆盖狗洞相关参数。

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

### 有顶狗洞与斜坡仿真

```bash
ros2 launch rm_navigation_launch field_geometry_simulation.launch.py
```

默认打开 Gazebo GUI 与 RViz，并生成带碰撞顶板的 `0.80 x 0.25 m` 狗洞、蓝色 11°
斜坡和橙色 15°斜坡。默认 `auto_start:=false`，机器人保持静止便于检查几何；需要执行
狗洞接近、对齐和穿越流程时使用：

```bash
ros2 launch rm_navigation_launch field_geometry_simulation.launch.py \
  auto_start:=true
```

该入口只启动仿真，不打开真实 MID360 或串口。狗洞参数仍来自
`rm_dog_hole/config/dog_hole_sim.yaml`，也可以用 `dog_hole_config:=/absolute/path.yaml`
覆盖；斜坡实体模型用于候选几何验证，不是最终比赛场地尺寸。
修改源码中的 launch 文件后需要重新构建并 source：

```bash
colcon build --symlink-install --packages-select rm_navigation_launch
source install/setup.bash
```
