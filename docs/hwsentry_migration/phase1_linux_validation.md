# Phase 1 Linux 与旧车验证流程

## 0. 已完成基线

2026-08-16 已在代码提交 `65fa728` 上使用无网络、只读源码挂载的 Docker/Humble
环境完成隔离回归：20 个首方包构建和 test 命令通过；去除 CTest wrapper 重复后，
底层 pytest/gtest/xunit 共 152 项、0 failure/error。与本阶段直接相关的结果为：

- `rm_dynamic_obstacle_tracking`：11 项通过；
- `rm_map_tools`：53 项通过，其中 ray evidence CLI 14 项；
- 外部 `fast_lio_multi`、`livox_ros_driver2` 未计入本轮通过范围。

同一代码基线的隔离 ROS runtime smoke 也已通过：默认 launch 不创建 tracker；
`enabled:=true` 时节点只发布 shadow markers、diagnostics 以及标准 ROS 参数/日志，
不发布 TF、`/cmd_vel`、costmap、plan 或 goal。

尚未完成的是带真实 `/map + /local_scan` 的旧车无运动 smoke、D01-D07 和真实 ray
sidecar 地图验证。只有在 `65fa728` 之后相关源码、依赖或工具链变化时才需要重复
Linux build/test；纯文档变化不触发重跑。

## 1. 获取与构建

在上述基线之后有相关代码变化时执行：

```bash
cd /home/wpie/rm2027_navigation
git fetch origin
git switch feature/hwsentry-selective-migration-phase1
git pull --ff-only

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rm_dynamic_obstacle_tracking rm_map_tools
source install/setup.bash

RM27_PHASE1_RESULTS=$(mktemp -d /tmp/rm27_phase1_results.XXXXXX)
colcon test --test-result-base "$RM27_PHASE1_RESULTS" --packages-select \
  rm_dynamic_obstacle_tracking rm_map_tools
colcon test-result --test-result-base "$RM27_PHASE1_RESULTS" --verbose
```

要求 build 成功，新增测试 0 failure/error。独立 result base 用于隔离工作区中其他
分支或外部包留下的历史结果。Windows 纯算法测试不能替代该步骤。

## 2. 无运动 smoke

先按已经验证的 old-car 地图/定位/Nav2 入口启动，但保持：

```text
use_real_serial:=false
use_serial_dry_run:=false
local_scan_enabled:=true
```

地图 bundle、AMCL/STVL profile 和雷达 IP 使用现场已经验证的参数，不在本实验
中改写。不要发 goal。确认：

```bash
ros2 topic hz /local_scan
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

另开终端启动 shadow tracker：

```bash
source /opt/ros/humble/setup.bash
source /home/wpie/rm2027_navigation/install/setup.bash
ros2 launch rm_dynamic_obstacle_tracking \
  dynamic_obstacle_tracking_shadow.launch.py enabled:=true
```

确认只有两个输出 topic，且没有新的 TF、costmap、plan、goal 或 cmd_vel 发布者：

```bash
ros2 node info /dynamic_obstacle_tracker_shadow
ros2 topic info /perception/dynamic_obstacles_shadow/markers -v
ros2 topic info /perception/dynamic_obstacles_shadow/diagnostics -v
ros2 topic info /perception/dynamic_obstacles_shadow/predictions -v
ros2 topic info /tf -v
ros2 topic info /cmd_vel -v
```

## 3. 录包与 RViz

```bash
mkdir -p /tmp/rm27_dynamic_tracker_shadow
ros2 bag record -o /tmp/rm27_dynamic_tracker_shadow/d01_d07 \
  /map /local_scan /odometry/lio \
  /tf /tf_static \
  /perception/dynamic_obstacles_shadow/markers \
  /perception/dynamic_obstacles_shadow/diagnostics \
  /perception/dynamic_obstacles_shadow/predictions
```

RViz Fixed Frame 使用 `map`，一次只增加 shadow MarkerArray 和必要的 map/scan
显示。不要把原始点云、filtered cloud、scan 和 markers 全部同色叠加后下结论。

## 4. 场景

依次执行 `phase1_design.md` 的 D01-D07。D01-D06 不需要底盘自动运动；D07
必须在遥控器人工接管可用、场地清空且已有高速自转安全流程时执行。tracker 不
会主动发运动命令。

初始实验门如下，后续只能基于录包证据调整：

- D01：confirmed false tracks 不超过 1/min，且不能持续 1 秒以上；
- D02：目标在可见后 0.5 秒内 confirmed，速度方向与人工路线一致；
- D03：靠近/远离方向正确，0.5 秒预测误差不超过 0.4 m；
- D04：沿静态墙边行走不产生持续静态墙 track；
- D05：一次双目标交叉不超过 1 次 ID switch，记录是否必须升级 Hungarian；
- D06：不超过 0.5 秒的遮挡后优先保持原 ID，离场后不永久保留 ghost；
- D07：平移/自转静态环境 confirmed false tracks 不超过 1/min；
- callback P95 小于 20 ms，输入不能出现持续 drop。

这些是 shadow 候选门，不是控制安全认证。任何一项失败都只调整 tracker 参数/
算法，不接 MPPI。

## 5. 离线清图原型

包测试已在上述代码基线完成，不要重复执行没有新输入的真实清图试验。真实 CLI 还需要
`rm_map_ray_observations/v1` sidecar；当前 Phase 2I 不生成该数据。没有 sidecar
时必须报告：

```text
MAPPING SIDECAR RECORDER NOT IMPLEMENTED
REAL MAP VALIDATION REQUIRED
```

禁止用最终 PCD 伪造 rays，禁止覆盖已有 PCD/PGM/bundle，也不应为了验证本阶段
临时改正式 mapping session。

## 6. 收尾报告

报告至少包含 commit/status、build/test、实际输入频率、TF/时间错误、D01-D07
逐项结果、false tracks、确认延迟、ID switches、coasting/deletion、预测误差、
CPU/RSS/P95 latency、topic/TF 污染检查和日志路径。结论只能是通过、部分通过或
失败；通过 tracker shadow 不代表允许接入 MPPI。
