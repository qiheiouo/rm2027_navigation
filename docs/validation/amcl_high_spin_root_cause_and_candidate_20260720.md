# AMCL 高速自转漂移：从问题发现、根因实验到第一版候选实现

日期：2026-07-20  
代码基线：`fix/field-debug-linux-consolidation`，`ed6c7ddb7110cdca99c37120461fcfa3f30e5665`  
当前结论状态：**离线根因证据强，第一版实验候选和无硬件基础验证完成；实车高速自转尚未验证。**

> 2026-08-25 更新：第一版候选在老车现场发生过一次低协方差错误模式复发。新的
> 下游隔离措施、现场证据和复验方法见
> `old_car_map_odom_correction_gate_20260825.md`；本文件中的根因实验记录保持不改写。

## 1. 摘要

旧车在高速原地自转后出现过 AMCL 米级位置跳变。问题不是 FAST-LIO 同步产生了
同量级平移：失败期间 FAST-LIO 只保留约厘米级真实位移，主要跳变出现在 AMCL，
随后错误的全局修正进入 `map→odom`。失败后的 AMCL 协方差还可能很小，因此“小
协方差”不能证明位置正确。

最终离线证据把首发机制收敛为：约 `8.35 Hz` 的观测更新使高速段一次 AMCL 更新
累积约 `0.837 rad` 旋转；Omni 模型的 `alpha4=0.2` 把这个旋转量映射为前向和侧向
各约 `0.374 m/update` 的平移粒子噪声，二维平移 RMS 约
`0.529 m/update`。scan 帧内运动畸变和场地重复结构使错误峰更有竞争力，随机 seed
决定具体哪一次触发，重采样再把错误峰固化。

固定 trial02 的离线消融中，baseline 为 `18/30 catastrophic`；只把
`alpha4` 降到 `0.02` 后为 `22 safe / 8 transient / 0 catastrophic`；只做完整
SE(3) 去畸变为 `0 safe / 30 transient / 0 catastrophic`；组合
`alpha4=0.02 + SE(3)` 在唯一 seeds 1–100 上达到 `100/100 safe`。这证明该组合
值得进入现场候选，但同一 bag 的 100 个 seed 不能替代不同工况和实车验收。

本次实现没有改正式比赛配置，而是新增独立候选：

```text
baseline  = alpha4 0.2  + 当前无 deskew 投影
candidate = alpha4 0.02 + LIO 插值逐点完整 SE(3) deskew
```

`resample_interval=1`、`recovery_alpha_fast=0.0` 和
`recovery_alpha_slow=0.0` 均保持不变。候选失败时采用严格整帧丢弃和 invalid 状态，
不静默输出未经补偿的 scan。

## 2. 问题是怎样发现的

现场首先观察到：车辆停止或完成快速旋转后，RViz 中机器人位置可能继续出现不合理
漂移；在更明确的高速自转失败样本里，AMCL 会发生米级跳变。调查时特别区分了三条
状态链：

```text
雷达/IMU -> FAST-LIO -> /odometry/lio
PointCloud2 -> LaserScan -> AMCL -> /localization/amcl_pose_raw
AMCL/global pose -> map→odom -> RViz/Nav2 全局坐标
```

这样做避免把 RViz 中的全局跳变直接归因于 LIO。后续证据显示：

- FAST-LIO yaw 与 IMU 积分高度一致，累计角差约 `0.77%`；
- trial02 高速段保留约 `2 cm` 的真实底盘细小平移，不应人为冻结为零；
- AMCL 首次严重错误可在约 `95 ms` 内产生约 `1.07 m` 跳变；
- 错误 AMCL 位姿会驱动 `map→odom` 跳变；
- 粒子群可以在错误位置形成小协方差，出现“自信地错”。

所以本问题的根因分析对象是 AMCL 的运动更新、观测几何、地图峰和重采样共同作用，
而不是简单把静止时看到的全局漂移等价为“雷达坏了”或“FAST-LIO 漂了”。

## 3. 调查和实验时间线

1. 从现场日志分离 LIO、AMCL、`map→odom` 与 RViz 表现，确认错误最先在 AMCL 层
   变成米级修正。
2. 保存 rich trial02 bag，包含 raw/filtered PointCloud2、LaserScan、LIO odom、TF、
   AMCL pose、粒子云和全局修正。正式紧凑输入含 532 帧 scan 与 2,981 帧 odom；
   高速段约 16.60 秒。
3. 分析粒子云，观察正确粒子在重采样附近从约 `92.4%` 跌到约 `0.2%`，随后错误峰
   获胜并形成低协方差。
4. 通过 `LD_PRELOAD` 固定 AMCL 内部两处 `srand48()` seed，使相同输入、参数、
   地图、初始位姿和 seed 可确定性重放；同一 seed 两次结果 JSON 字节一致。
5. 运行 baseline seeds 1–30，得到 `0 safe / 12 transient / 18 catastrophic`。
6. 分别消融 `alpha4`、旋转 deskew、完整 SE(3) deskew、50 Hz scan、
   `resample_interval`、classic recovery 和“小连通域清图”。
7. 将保守的一数量级 `alpha4=0.02` 与完整 SE(3) deskew 组合，唯一 seeds 1–100
   全部 safe，零 timeout；灾难概率的 Wilson 95% 上界仍为约 `3.70%`。
8. 核对在线链路，发现 self-filter 用 `setPointCloud2FieldsByString("xyz")` 重建消息，
   正是在这里删除了 `intensity/tag/line/timestamp`。
9. 核对 Livox driver2 源码和 trial02：raw schema 的逐点字段为
   `timestamp@offset18/FLOAT64`；值来自 `PointXyzlt::offset_time`，是绝对纳秒；cloud
   header 为 `pkg.base_time`，与首点时间一致，点跨度中位数约 `20.20 ms`。
10. 实现独立 candidate profile、字段保留、在线 SE(3) deskew、诊断状态、单元测试和
    无硬件 trial02 短回放。本轮未进行任何实车高速自转。

## 4. 核心公式与正确单位

Nav2 Humble AMCL 1.1.20 的 Omni 运动模型使用：

```text
delta_trans = sqrt(delta_x² + delta_y²)
delta_rot   = delta_yaw

sigma_trans  = sqrt(alpha3 * delta_trans² + alpha4 * delta_rot²)
sigma_strafe = sqrt(alpha4 * delta_rot² + alpha5 * delta_trans²)
sigma_rot    = sqrt(alpha1 * delta_rot² + alpha2 * delta_trans²)
```

trial02 高速段的中值输入为：

```text
delta_rot   = 0.8370408076 rad/update
delta_trans = 0.0106820600 m/update
alpha1..5   = 0.2（baseline）
```

代入得到：

```text
sigma_trans  = 0.3743665103 m/update
sigma_strafe = 0.3743665103 m/update
sigma_rot    = 0.3743665103 rad/update
2D translation RMS = 0.5294341961 m/update
```

`0.374` 的正确单位是当前这一次运动更新中的 `m/update`，不能写成
`0.374 m/rad`。它是粒子滤波主动注入的分布宽度，不是传感器测得的真实平移误差。

## 5. 离线消融结果

| 组 | 唯一变量 | safe / transient / catastrophic | 结论 |
|---|---|---:|---|
| baseline | 当前全部配置 | 0 / 12 / 18 | 60% 灾难率 |
| `alpha4=0.02` | 只降低 alpha4 | 22 / 8 / 0 | 消除灾难，仍有瞬态 |
| `alpha4=0.002` | 更激进降低 | 30 / 0 / 0 | 全 safe，但低速泛化风险更大 |
| `alpha4=0` | 诊断下界 | 30 / 0 / 0 | 只证明剂量关系，不作为候选 |
| rotation deskew | 只补偿逐点旋转 | 0 / 29 / 1 | 灾难显著降低，仍普遍瞬态 |
| full SE(3) deskew | 只完整补偿 | 0 / 30 / 0 | 无灾难，仍全为瞬态 |
| 50 Hz current | 只提高更新率 | 30 / 0 / 0 | `delta_rot` 中值降到 0.141 rad |
| `resample=2` | 只改重采样间隔 | 0 / 15 / 15 | 不能保护正确模式 |
| `resample=3` | 只改重采样间隔 | 0 / 4 / 26 | 灾难率升到 86.7% |
| classic recovery | `0.001/0.1` | 0 / 9 / 21 | 不预防首发，还可能更远更久 |
| clean map CC≤4 | 删除 334 个小连通域 | 0 / 10 / 20 | 灾难率 66.7%，不是根因修复 |
| `alpha4=.02 + SE(3)` | 第一版组合 | 100 / 0 / 0 | 固定 trial02 唯一 seeds 1–100 |

完整 SE(3) 几何代理也改善：高速段 current scan 的地图得分中值约 `0.891`，SE(3)
约 `0.954`，148 帧中 127 帧改善；最近邻距离中值从约 `7.02 cm` 降为
`4.69 cm`。该代理不是 AMCL 内部 likelihood，因果结论仍以完整 AMCL 多 seed 回放为
主。

## 6. 每个因素扮演的角色

| 角色 | 因素 | 证据边界 |
|---|---|---|
| 触发条件 | 高速自转 | 低速/静止不等价于本次失败工况 |
| 主要诱因 | 大 `delta_rot × Omni alpha4` | 剂量响应和 50 Hz 对照证据最强 |
| 观测退化因素 | scan 帧内运动畸变 | deskew 改善几何和灾难率，但单独使用仍全 transient |
| 竞争峰来源 | 场地/地图平行与重复结构 | 真值附近和 2.4–3.0 m 外都有高分峰 |
| 随机触发因素 | 粒子随机采样 seed | 同 bag 同参数只改 seed 会得到不同严重度 |
| 放大/锁死机制 | 重采样 | 正确粒子可在一次重采样附近被清空；增大间隔更差 |
| 恢复机制 | classic recovery | 影响失败后探索，不阻止首次错误峰胜出 |
| 错误传播隔离层 | global pose gate / map→odom 边界 | 可减少下游破坏，但不是 AMCL 根因修复 |

因此不能把“清理 PGM 脏点”“开启 recovery”或“自转后强行恢复旧 xy”当成本问题的
主修复。当前地图有竞争结构，但盲目删小连通域已经得到反例。

## 7. 修改前的在线点云链路

```text
/livox/left/pointcloud
  fields: x/y/z/intensity/tag/line/timestamp
  -> pointcloud_self_filter_node
     旧实现重建 xyz-only PointCloud2
  -> /livox/left/pointcloud_filtered
     fields: x/y/z
  -> pointcloud_to_laserscan_node
     整帧只查一次 header-stamp TF，无逐点补偿
  -> /localization/scan
  -> AMCL
```

字段删除点已经明确定位到
`rm_mid360_driver_bridge/src/pointcloud_self_filter_node.cpp`。它不是 Livox 驱动随机
缺字段，也不是 bag 反序列化问题。

## 8. 第一版候选实现

### 8.1 profile 隔离

正式 baseline 文件保持不变：

```text
rm_relocalization_bridge/config/amcl_2d.yaml
rm_relocalization_bridge/config/pointcloud_to_scan_2d.yaml
```

新增实验文件：

```text
rm_relocalization_bridge/config/amcl_2d_spin_robust_candidate.yaml
rm_relocalization_bridge/config/pointcloud_to_scan_2d_spin_robust_candidate.yaml
rm_navigation_bringup/launch/old_car_2026_amcl_spin_candidate.launch.py
```

实验 AMCL 只改变 `alpha4: 0.2 -> 0.02`。`alpha1/2/3/5`、粒子数、laser model、
KLD、beam、`update_min_a/d` 均保持 baseline。特别保留：

```yaml
resample_interval: 1
recovery_alpha_fast: 0.0
recovery_alpha_slow: 0.0
```

正常 `old_car_2026_amcl_relocalization.launch.py` 新增两个可选文件参数，但默认仍指向
baseline。比赛 launch 没有引用候选文件，deployment manifest 也没有修改。

### 8.2 PointCloud2 字段保留

self-filter 不再创建 xyz-only 消息。它先按原 xyz 和自车包围盒决定保留索引，再按
`point_step` 逐条复制完整原始 record。输出继续保留：

```text
fields、datatype、offset、count、point_step、endianness
x/y/z、intensity、tag、line、timestamp
```

被保留点的 `timestamp` 字节来自同一个原点，不重编号、不重算，也不把 header stamp
复制到每个点。输出改为紧凑的 `height=1`，去掉输入行 padding，但每个 point record
内部布局保持不变。

### 8.3 已确认的时间语义

Livox driver2 的实际实现为：

```text
cloud.header.stamp = pkg.base_time
point.timestamp    = double(pkg.points[i].offset_time)
point.offset_time  = packet.time_stamp + i * point_interval
```

因此当前 old-car 数据的明确合同是：

```text
字段名：timestamp
datatype：FLOAT64
含义：绝对纳秒时间
header：包 base_time，与首点时间一致
参考时刻：cloud header stamp / cloud start
```

trial02 raw cloud 每帧点跨度中位数约 `20.20 ms`，最大约 `32.59 ms`；首点相对
header 的误差仅为浮点表示量级。候选只接受这个已经验证的
`absolute_nanoseconds` 模式，不自动猜测 `time/t/offset_time` 或相对秒。

### 8.4 SE(3) deskew

`/odometry/lio` 已由适配器规范成 `T_odom_base(t)`，输入点位于 cloud 的 sensor
frame。对点 `p_sensor(t)`，先用一次正确方向的 `T_base_sensor` 外参，再补偿到 header
参考时刻：

```text
p_base(t) = T_base_sensor · p_sensor(t)
p_odom    = T_odom_base(t) · p_base(t)
p_ref     = inverse(T_odom_base(t_ref)) · p_odom
```

实现不会把雷达到车体外参重复应用。平移在相邻 LIO pose 间线性插值；旋转使用四元数
SLERP，并在点积小于零时翻转后一端四元数，保证跨 `±π` 走短路径。补偿后的点已经位于
参考时刻 `base_link`，随后才做高度、量程、角度过滤和 LaserScan 最近距离投影。

输出语义为：

```text
LaserScan.header.stamp    = 输入 cloud header / t_ref
LaserScan.header.frame_id = base_link
```

几何参考时刻与下游 AMCL 查 TF 的 scan stamp 一致。

### 8.5 缓冲、异常和严格失败策略

候选订阅 `/odometry/lio`，保留 3 秒 pose 缓冲，并最多等待 0.20 秒让 odom 覆盖
header 到末点的整帧区间。插值 gap 上限为 0.10 秒；cloud 点时间跨度和末点相对 header
上限也各为 0.10 秒。

以下情况不发布该帧 LaserScan，并发布 error/inactive：

- 缺少 `timestamp`，或 datatype/count 不是 `FLOAT64/1`；
- 空点云、截断 record、NaN/Inf 点时间；
- header 为零、点时间早于或晚于已验证边界；
- odom 没有完整包围参考时间和所有有效点时间；
- odom parent/child 不是 `odom -> base_link`；
- pose 非有限、四元数零范数、插值 gap 过大；
- timestamped `sensor -> base_link` TF 不可用；
- pending queue 溢出。

重复 odom timestamp 会用新样本替换；时间回退会清空 odom/pending buffer，防止两个
时间 epoch 混用。xyz NaN/Inf 点会被丢掉；零运动、零点时间跨度和所有点被几何过滤
不会崩溃。候选不实现“deskew 失败后退回 raw scan”，所以不会把 degraded 输出冒充
deskew active。

### 8.6 频率和诊断

第一版仍以 `max_publish_rate_hz=10.0` 运行，没有切到 50 Hz。新增：

```text
/localization/scan_deskew/active  std_msgs/Bool（transient local）
/localization/scan_deskew/status  diagnostic_msgs/DiagnosticArray
```

诊断包含 enabled/active、字段名、单位/解释、参考时刻、odom 来源、失败策略、输入/
输出/丢弃/限频帧数、timestamp/插值/TF/coverage/rollback 计数、odom buffer 长度、
pending 长度、累计点数、最近一帧输入/输出/丢弃点数、纯处理时间、总等待+处理延迟、
最后成功时间和最后失败原因。日志按 2 秒节流，不逐点打印。

## 9. 修改文件清单

### 新增

- `src/rm_mid360_driver_bridge/include/rm_mid360_driver_bridge/pointcloud_deskew.hpp`
- `src/rm_mid360_driver_bridge/src/pointcloud_deskew.cpp`
- `src/rm_mid360_driver_bridge/test/test_pointcloud_deskew.cpp`
- `src/rm_relocalization_bridge/config/amcl_2d_spin_robust_candidate.yaml`
- `src/rm_relocalization_bridge/config/pointcloud_to_scan_2d_spin_robust_candidate.yaml`
- `src/rm_relocalization_bridge/test/test_spin_candidate_profiles.py`
- `src/rm_navigation_bringup/launch/old_car_2026_amcl_spin_candidate.launch.py`
- 本文档。

### 修改

- `src/rm_mid360_driver_bridge/src/pointcloud_self_filter_node.cpp`
- `src/rm_mid360_driver_bridge/src/pointcloud_to_laserscan_node.cpp`
- `src/rm_mid360_driver_bridge/CMakeLists.txt`
- `src/rm_mid360_driver_bridge/package.xml`
- `src/rm_relocalization_bridge/CMakeLists.txt`
- `src/rm_relocalization_bridge/package.xml`
- `src/rm_navigation_bringup/launch/old_car_2026_amcl_relocalization.launch.py`

没有修改 `amcl_2d.yaml`、`pointcloud_to_scan_2d.yaml`、比赛 launch、地图、deployment
manifest、Livox 子模块内容或 `core.205`。

## 10. 已实际完成的无硬件验证

### 10.1 证据和配置

- 读取并复核最终离线报告、实验矩阵、100-seed summary、资产 inventory、公式证据和
  `SHA256SUMS`。
- 所列五个核心证据文件的 SHA-256 与归档清单一致。
- Python `py_compile`：两个 launch 和配置隔离测试通过。
- YAML `safe_load`：baseline/candidate 共四个配置通过。
- `ros2 launch ... --show-args`：独立候选入口可解析；其父 launch 默认仍是 baseline。
- 实际启动未 configure 的 AMCL lifecycle node 并读取参数：

```text
alpha4 = 0.02
resample_interval = 1
recovery_alpha_fast = 0.0
recovery_alpha_slow = 0.0
```

### 10.2 构建、测试和 lint

容器内构建：

```text
colcon build --symlink-install --packages-select \
  rm_mid360_driver_bridge rm_relocalization_bridge rm_navigation_bringup
```

结果：3 个包全部成功。

测试覆盖：

- 新增 9 个 C++ case：空/非法查询、平移、零运动、跨 `±π` SLERP、旋转+平移+外参、
  绝对纳秒解释、缺字段/NaN/空 cloud、字段逐字节保留、organized row padding、scan
  stamp/frame；全部通过。
- `rm_relocalization_bridge` 既有 12 个 C++ case 全部通过。
- 新增 4 个 profile 隔离 case：baseline 不变、candidate 只改选定 motion term、
  deskew opt-in/10 Hz、独立 launch；全部通过。
- `colcon test-result` 的两个包分别为 `10 tests, 0 failures` 和
  `19 tests, 0 failures`（其中含 CTest runner 记录）。
- 受影响 C++ 文件 `ament_uncrustify` 通过；`ament_cpplint` 在仅忽略仓库既有的
  copyright-header 规则后通过。
- 3 个新增/修改 Python 文件 `ament_flake8` 通过。

### 10.3 trial02 短 bag smoke

只启动 old-car robot description、self-filter、实验投影器和观察器，回放：

```text
/livox/left/pointcloud
/odometry/lio
/tf
```

没有启动 AMCL、串口、底盘、controller、mission、Nav2 或发布 `cmd_vel`。最终观察
窗口结果：

```text
raw cloud observed                     269
filtered cloud observed                268
filtered schema == matching raw        268/268
filtered has FLOAT64 timestamp         268/268
deskew LaserScan output                 47
scan stamp matches filtered cloud       47/47
scan frame == base_link                 47/47
deskew active true events               47
timestamp/interpolation/TF failures      0/0/0
time rollback / queue overflow           0/0
startup odom-coverage frame drop         1
latest processing time                   0.688 ms
latest wait + processing latency         10.421 ms
```

首个 cloud header 比 bag 的首个 odom 样本早约 `0.3 ms`，严格策略按设计丢掉 1 帧；
之后 active。观察器结束并截断 bag 后，projector 日志还记录了一个预期的尾帧 coverage
drop；它来自人为停止 replay，不属于连续运行统计。

将在线结果与历史 Python 离线 SE(3) 产物比较，最终 run 找到 9 个同 stamp scan、
2,615 个共同有限 bin：绝对量程差中位数 `0`，P99 约 `2.38e-7 m`，最大约
`4.77e-7 m`。保存的 smoke 证据：

```text
/tmp/rm2027_amcl_spin_candidate_smoke_final_readable/
observer.json SHA-256:
b55c8cbfd5c609d353785d4e2189846fbeee6e2f45a98ef12914e0a92b08ff8c
```

## 11. 尚未验证的内容

本轮没有把以下项目写成“通过”：

- 实车静止长时间稳定性；
- 低速直线前进和后退；
- 横移和斜向运动；
- 普通转弯；
- Nav2 约 `1.2 rad/s` Spin；
- 人工约 `7 rad/s` 高速自转；
- 自转结束后的持续稳定性；
- LIO 短时丢帧、异常 pose 或 TF 抖动；
- 实车实时 CPU、最坏延迟和长期内存；
- 不同场地位置、不同重复结构和不同初始姿态；
- 与 baseline 的同日、同位置、同运动 A/B；
- 正式比赛导航、controller、mission 和自动模式集成。

因此当前只能说“候选实现符合已验证的离线算法并可进入人工现场观察”，不能说“实车
问题已经最终解决”或“完整比赛可用”。

## 12. 后续实车人工验证步骤

### 12.1 启动前

1. 清空车辆周围，确认车辆静止；先不要自转。
2. 确认本轮只验证定位，不运行串口、controller、mission 或 Nav2 motion。
3. 若未来要测试 Nav2 Spin，那已经是导航功能：**必须先明确提醒用户开启自动模式**，
   并使用单独的运动验收流程；本候选 launch 本身不会启动 Nav2。
4. 在容器中重新 build/source 当前工作区。
5. 使用当前 fresh03 candidate manifest；不要修改地图 approval 或 deployment manifest。

候选无运动定位入口：

```bash
export MAP_MANIFEST=/data/rm27_maps/old_car_fresh03_field_validation/20260720T012237Z/old_car_fresh03_field_validation.bundle.yaml

ros2 launch rm_navigation_bringup old_car_2026_amcl_spin_candidate.launch.py \
  map_bundle_manifest:=$MAP_MANIFEST \
  map_acceptance_policy:=allow_candidate \
  use_driver:=true \
  use_lio_backend:=true \
  use_rviz:=true
```

这个命令会在用户主动执行时启动真实 MID360/LIO 以供观察，但不会启动串口、Nav2、
controller、mission 或发送 `cmd_vel`。本轮 Codex 没有执行它。

### 12.2 必须先确认 deskew 真正 active

另开终端：

```bash
ros2 topic echo --once /localization/scan_deskew/active
ros2 topic echo --once /localization/scan_deskew/status
ros2 param get /amcl alpha4
ros2 param get /pointcloud_to_laserscan_node deskew.enabled
```

只有同时满足以下条件才继续：

```text
active.data = true
alpha4 = 0.02
deskew.enabled = true
timestamp_interpretation = absolute_nanoseconds
timestamp_failures = 0
interpolation_failures = 0
tf_failures = 0
last_processing_ms 和 last_success_stamp_sec 持续更新
```

若 active 为 false、odom coverage 持续失败或输出 scan 频率异常，保持停车并结束候选，
不要开始高速自转。

### 12.3 观察顺序

1. 在 RViz 发布准确的 `2D Pose Estimate`。
2. 静止观察至少 20–30 秒，确认点云/scan/地图和机器人模型重合。
3. 做很短的低速前进、后退；停车观察。
4. 做很短的低速横移、斜向；停车观察。
5. 做普通速度转弯；停车观察。
6. 再做较快但非极限的自转；停车观察。
7. 前面均正常后，最后才进行人工高速自转；不要由本 launch 自动发运动命令。
8. 出现明显位置跳变、错误朝向、deskew inactive 或 scan 异常时立即停车。
9. 停车后保持静止，不要立即重发 initial pose，让失败后的 AMCL、粒子云和
   `map→odom` 状态留在 bag 中。
10. 保存现场位置、转向、持续时间和实际操作方式，并与 baseline 做同位置 A/B。

建议 bag topics：

```text
/livox/left/pointcloud
/livox/left/pointcloud_filtered
/odometry/lio
/localization/scan
/localization/scan_deskew/status
/localization/scan_deskew/active
/localization/amcl_pose_raw
/localization/global_pose
/localization/map_to_odom
/particle_cloud
/tf
/tf_static
```

## 13. baseline 回退

回退不删除代码、不恢复文件，只停止 candidate 后换回原入口：

```bash
export MAP_MANIFEST=/data/rm27_maps/old_car_fresh03_field_validation/20260720T012237Z/old_car_fresh03_field_validation.bundle.yaml

ros2 launch rm_navigation_bringup old_car_2026_amcl_relocalization.launch.py \
  enable_relocalization:=true \
  map_bundle_manifest:=$MAP_MANIFEST \
  map_acceptance_policy:=allow_candidate \
  use_driver:=true \
  use_lio_backend:=true \
  use_rviz:=true
```

该入口的默认文件仍是：

```text
amcl_2d.yaml                         alpha4=0.2
pointcloud_to_scan_2d.yaml           deskew.enabled 缺省=false
```

self-filter 仍会保留额外字段，但 baseline LaserScan 几何继续使用原整帧 header TF 投影；
因此回退不依赖重新编译或手工撤销字段保留代码。

## 14. 已知风险和停止条件

- `alpha4=0.02` 只在固定 trial02 组合回放中通过；可能低估某些真实横移误差。
- deskew 依赖 `/odometry/lio` 的时间、frame 和短时连续性；严格策略会在输入不完整时
  降低 scan 更新率，而不是发布 degraded scan。
- 当前 old car 外参是 2026 旧车测量值；不能作为 2027 最终机器人标定。
- 10 Hz 下在线处理本次约 0.69 ms，但实车长期最坏值未测。
- bag 边界首帧因无更早 odom 丢弃是预期；若连续运行仍频繁 coverage drop，则必须停止
  现场自转并调查时间同步/队列，不得放宽成静默外推。
- global pose gate 仍可能接受“低协方差但错误”的 AMCL pose；物理可达性创新门属于后续
  独立安全层，不在本次候选里。
- candidate launch 不是比赛 launch，不能用其成功代替 Nav2/mission 全栈验收。

## 15. 证据、哈希和版本控制边界

最终离线归档：

```text
/home/wpie/rm2027_validation_archives/20260720_amcl_spin_offline_final/
```

关键文件 SHA-256：

```text
d929fdc2806a2660d5eb8ca3689666d497d84b19fdb9bdca1471b01d1d61a67d  离线报告
dc1a2b73cf93df045f3ab82eec51365162c8dcd2048877e799deb8a061160c5b  实验矩阵
f49fe56709716df0bdcfa707ce25c593273aadb3ca297928ade0994d59d98c8c  100-seed summary
35803aca0ff59b686cf1f6ceae1edfe8122409da11047abf847f6aaceb7c8de9  asset inventory
2e4334a79a8c55911327c13e69ff0592bbf2f4385e815f21310752a89d368644  Omni 公式证据
```

配置快照 SHA-256：

```text
ee85fea9dca9a3925f63cb4a16246daa0130eee8e61b99ebede8c2d48f88d505  baseline AMCL
1ac4de66fc0f669f3501ff6e4eee8202189e76442973a8adf89878bd3b373130  candidate AMCL
0c80ebd7574b2a71645d6107e714a0371ddcd278e62ddc1e274a597a90751b23  baseline scan
da473d3d8c68c021d1dc6247616e40378d1b541f0d26b354121693f352570909  candidate scan
```

实现时工作树原本就包含用户内容：

```text
src/livox_ros_driver2_humble
  src/src/call_back/livox_lidar_callback.cpp modified
  src/src/comm/pub_handler.cpp modified
core.205 untracked
```

它们均未被本次实现修改、清理、暂存或提交。本轮没有 commit、没有 push；没有修改
任何地图、PCD 或 deployment manifest。

## 16. 最终结论

已经完成从问题定位、确定性回放、单变量消融、组合验证到第一版实验候选落地的闭环：

```text
问题现象：高速自转后 AMCL 米级跳变并可能低协方差锁错
最强根因：大 delta_rot × Omni alpha4 造成过强平移粒子扩散
重要共因：scan 运动畸变 + 重复结构竞争峰
放大机制：随机采样相位 + 重采样固化
被反证的单独修复：增大 resample、classic recovery、盲目删 PGM 小脏点
第一版候选：alpha4=0.02 + 逐点 LIO SE(3) deskew
实现状态：代码、profile、诊断、测试和短 bag smoke 完成
现场状态：尚未实车高速自转，不是正式比赛批准配置
```

本轮只完成候选代码实现和无硬件基础验证。实车高速自转效果尚未验证，由用户后续
手动完成。
