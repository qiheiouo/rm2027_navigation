# 老车无标注坡道实车录包分析与共享跟踪修复报告

日期：2026-08-30

车辆：2026 旧车，双 MID360，FAST-LIO，AMCL，Nav2 MPPI/STVL

代码分支：`fix/old-car-ramp-live-tracking`

状态：**原故障已由实车包复现，修复已通过同包离线 A/B 和软件测试；修复后的主动上车重复验收仍待执行。**

## 1. 结论

现场现象不是“坡道检测器完全没有工作”。原实现中的融合障碍点云 tracker 能确认坡面并
过滤，但左雷达定位点云的另一套独立 tracker 在同一段数据中一次也没有达到五帧确认，
所以 AMCL 链仍然看到完整坡面；两套 tracker 的状态分歧会让局部障碍与全局定位行为不
一致，也是车辆有时上坡、有时绕行以及坡上左右修正的一个直接软件原因。

原实现还把确认和失效时间绑定到输入点云帧率：融合点云约 100 Hz 时，八个 missed
frames 只有约 0.08 s；同时两个节点分别逐帧运行 RANSAC 并逐帧发布诊断。实车包中自动
坡道诊断合计约 153 Hz，这既放大了偶发漏检，也产生了没有必要的计算和诊断负载。

本次修复收敛为单一坡道事实源：

```text
/points/obstacles_fused（主检测源）
        ↓  odom 中积累 300 ms，10 Hz RANSAC
        ↓  五次检测周期确认，允许两个 pending 漏检周期
同一个 confirmed odom 坡面模型
        ├── 过滤融合障碍点云 → local STVL
        └── 过滤左雷达定位点云 → AMCL/global scan
```

修复后的同包 2× 回放中，前 50 s 平地基线两路均为零误删；看到坡道后，融合障碍流有
4,295 个共同时间戳帧发生有效剔除，左雷达定位流有 2,252 帧发生有效剔除。诊断频率降至
4.53 Hz。该结果证明原来“两路不一致”的软件故障已被修复，但不替代修复后车辆真实
上、下坡的主动安全验收。

## 2. 原始证据和可复现输入

实车原始包保存在宿主机，不纳入 Git：

```text
artifacts/bags/ramp_detector_real_20260830_live01/
  metadata.yaml
  ramp_detector_real_20260830_live01_0.db3
```

属性：

- 时长：123.936997851 s；
- 消息数：84,325；
- DB3 大小：3,543,158,784 bytes；
- SHA256：`133524ba034d1498cbf0125559df5c962d31154b1a94fe23de74f542c6372d15`；
- 点云：融合输入 12,100 帧、左雷达输入 6,185 帧；
- 另含 `/tf`、`/tf_static`、LIO、AMCL、定位有效性、local costmap 和原实现诊断；
- `/plan` 与 `/cmd_vel` 为零帧，因为本段主要是人工分段停留和遥控取样，不用于评价
  MPPI 跟踪质量。

分段动作包括平地原位、坡下观察、坡上停留和返回坡下。离线按 LIO 高度和俯仰复核：
前约 50 s 高度中值约 0.008 m、俯仰约 -1.3°；约 75--90 s 高度约 0.23 m、俯仰约
-18.7°，与车辆位于斜坡上的动作一致。

## 3. 原实现复现结果

### 3.1 两个 tracker 产生分歧

| 原实现链路 | 诊断样本 | candidate | 最大确认计数 | confirmed 样本 | 有效剔除点云帧 |
|---|---:|---:|---:|---:|---:|
| 左雷达定位 | 6,189 | 1,969 | 4 | 0 | 0 / 6,182 common |
| 融合障碍 | 12,316 | 4,244 | 6 | 4,819 | 3,508 / 12,072 common |

左雷达链并非看不到斜面：候选坡度中值为 18.22°、长度中值 1.46 m、宽度中值
1.21 m，但确认计数最高只到 4，刚好达不到五帧门。融合链候选坡度中值 16.72°，能够
确认并剔除坡面。由此排除“阈值完全不适配这个实物坡道”，将根因收敛到独立输入采样与
时序跟踪。

### 3.2 为什么现场看起来是偶发的

两套输入的帧率、视角、点密度和融合时刻不同；逐帧 RANSAC 又允许某一帧因遮挡或采样
稀疏而没有候选。原 pending candidate 遇到一次空检测就归零，而 confirmed model 的
八帧 TTL 在融合流中只有约 80 ms。因此相同接近路径的细小时间差就可能决定是否凑够
五个连续候选，表现在 Nav2 上就是有时把坡面清掉、有时仍把它标成黑色障碍并绕行。

## 4. 修复内容和安全边界

### 4.1 单一共享 tracker

`automatic_ramp_filter_node` 新增可选 secondary 输入/输出。主融合点云独占检测和跟踪
状态，左雷达流只使用已确认模型做过滤，不再运行第二套 RANSAC。人工 `map_regions`
回退仍保持两个无状态过滤节点，行为不变。

### 4.2 时间化检测和短时积累

默认配置为：

```yaml
detection.update_period_sec: 0.10
detection.accumulation_window_sec: 0.30
diagnostics.publish_period_sec: 0.20
tracking.confirmation_frames: 5
tracking.pending_max_missed_frames: 2
tracking.max_missed_frames: 8
```

每帧仍先在机器人坐标系 ROI 内取二维体素最低回波，再把候选点变换到 `odom` 做 300 ms
积累。RANSAC 只按 10 Hz 检测周期运行。确认大约需要 0.5 s，confirmed 模型在连续约
0.8 s 没有同轨候选后撤销，不再随 50/100 Hz 输入话题改变物理时间含义。

### 4.3 保留的 fail-closed 合同

- 未确认、禁用、TF 缺失、零时间戳或点云缺字段时原样直通，不错误放行；
- 只删除确认平面多边形内、表面容差范围内的点；高于坡面的凸起继续保留；
- 不发布 TF、速度、路径、Nav2 action，不修改 PGM/PCD；
- shadow 仍不被 AMCL、costmap 或底盘消费；
- active 只改变既有的 AMCL 点云输入和 local STVL 点云输入，静态 PGM 若把坡道画成
  occupied，global planner 仍会拒绝路径。

## 5. 修复后同包 A/B

使用隔离的 `ROS_DOMAIN_ID`，只回放原包中的融合点云、左雷达点云、`/tf` 和
`/tf_static`，不启动串口、Nav2、mission 或底盘。以 2× 回放比较相同 header stamp 的
输入和新输出：

| 修复后链路 | 输入帧 | 输出/输入共同时间戳 | 有效剔除帧 | 单帧剔除中值 | 最大值 |
|---|---:|---:|---:|---:|---:|
| 融合障碍 | 12,100 | 10,876 | 4,295 | 369 | 1,171 |
| 左雷达定位 | 6,179 | 6,087 | 2,252 | 376 | 1,171 |

2× SensorDataQoS 回放时部分融合帧没有形成共同时间戳，因此通过 common 集合评价是否
剔除，不把回放丢帧误写成节点功能失败。真实输入速率只有该回放的一半。

关键分箱结果：

| 原包时间 | 场景 | 融合剔除率 | 左雷达剔除率 |
|---|---|---:|---:|
| 0--50 s | 平地基线 | 0.000 | 0.000 |
| 50--55 s | 进入坡面视野的过渡 | 0.009 | 0.008 |
| 55--75 s | 坡下/接近 | 0.716--0.763 | 0.732--0.760 |
| 75--95 s | 坡上 | 0.481--0.511 | 0.368--0.448 |
| 95--124 s | 返回坡下/持续观察 | 0.725--0.785 | 0.708--0.776 |

新诊断共 295 条、覆盖 65.16 s 的 2× 回放，频率 4.53 Hz；168 条有 candidate，164 条
为 confirmed，最大确认计数为 5。平地零误删和两路同步剔除均达到离线门槛。

## 6. 软件验证

在 ROS 2 Humble 容器执行：

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_navigation_launch
colcon test --packages-select rm_mid360_driver_bridge rm_navigation_launch
```

结果：两个包构建通过；`rm_mid360_driver_bridge` 的 5 个 test target 全部通过，包括
17 个 C++ case 和 3 个 runtime pytest。新增 runtime 用例证明：一次空检测不会清空
pending candidate；第三次匹配后主流和 secondary 流同时过滤；坡面上方障碍在两路
输出中均保留。

注意：若直接对整个历史 `build/` 运行无范围的 `colcon test-result --verbose`，会列出
Livox 上游第三方目录遗留的 lint 失败；它们不是本次测试产生的失败。应使用本次包的
测试结果目录或先清理历史结果再汇总。

## 7. 仍需执行的实车验收

修复后的代码还没有重新驱动车辆经过斜坡，所以当前只能称为“软件完成、待主动实车
验收”。重新接电后按以下顺序执行。

### 7.1 构建与启动

```bash
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_navigation_launch
source install/setup.bash

ros2 launch rm_navigation_launch old_car_full_terrain_navigation.launch.py \
  activate_ramp_filter:=true
```

发布初始位姿后先不要发 Nav2 目标。确认只有一个自动节点：

```bash
ros2 node list | grep ramp_filter
```

自动模式预期只有 `/old_car_shared_ramp_filter`；同时确认以下输出持续更新：

```bash
ros2 topic hz /points/obstacles_ramp_filtered
ros2 topic hz /livox/left/pointcloud_ramp_filtered
```

### 7.2 V00 平地回归

平地静止、直行、转弯各 30 s。PASS：`confirmed` 不持续为 true、两路删除点数为零、
定位有效性不丢失，local costmap 不出现由过滤器造成的新空洞。

### 7.3 V01 坡前静止

车停在坡前 10 s。PASS：约 0.5--1.0 s 内确认；两路删除点数同时大于零；原始点云仍
显示坡面，过滤点云中坡面主体减少；PGM 可通行且 local costmap 的黑色坡面逐步清除。

若诊断已经 `confirmed=true` 且融合删除点数大于零，但 local costmap 仍一直全黑，应
检查 STVL 订阅和体素衰减；若 global costmap/PGM 本身为黑，则应修 PGM，而不是放宽
坡道检测阈值。

### 7.4 V02 障碍保留

在坡面固定至少 0.12 m 高的障碍，只观察不自动行驶。PASS：坡面被删，但障碍仍在过滤
点云和 local costmap 中形成 lethal cell。障碍一起消失为硬 FAIL。

### 7.5 V03 主动重复

清场、有人持急停、mission 保持 disabled，从单个 RViz 目标开始。以 0.20 m/s 完成至少
3 次上坡和 3 次下坡，每次坡中停车约 5 s。PASS：不再选择绕过坡道，不持续左右振荡，
不发生定位 invalid/自主重定位，停车后位姿与地图一致。通过后才能提高到 0.35 m/s。

建议同时录制：

```bash
ros2 bag record \
  /livox/left/pointcloud_filtered \
  /livox/left/pointcloud_ramp_filtered \
  /points/obstacles_fused \
  /points/obstacles_ramp_filtered \
  /odometry/lio /localization/global_pose \
  /localization/global_localization_valid \
  /local_costmap/costmap /plan /cmd_vel_nav \
  /tf /tf_static /diagnostics
```

## 8. 完成度

- 原故障复现和根因证据：100%；
- 共享 tracker、时间积累、运行时测试和同包 A/B：100%；
- 修复后旧车主动坡道重复验收：0%；
- 新车机械/坡道能力与比赛级跨地形合同：不在本分支范围。

按“老车本次无标注坡道修复”计，当前约 **85%**；剩余约 15% 是不可由离线包替代的
修复后实车 V00--V03。即使这些通过，也只代表旧车对该类坡道的验证，不代表已有
HWSentry terrain planner、轮腿 FDDP 或新车下位机专用爬坡合同。
