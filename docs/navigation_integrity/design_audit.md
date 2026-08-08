# Navigation Integrity 设计审计

## 基线与范围

- 开发基线：`fix/old-car-global-dynamic-replanning @ 9b608b9`。
- 实现分支：`feature/navigation-integrity`。
- 第一阶段严格为 shadow mode，不改变定位、TF、Nav2、mission 或底盘控制。
- Windows 工作区没有 ROS 2，也没有仓库内可用的历史 rosbag；Linux build、replay
  和实车结论均不能在本阶段伪造。

## 当前定位数据流

```text
MID360 PointCloud2
  -> self filter / optional per-point deskew projector
  -> /localization/scan
  -> AMCL (tf_broadcast=false)
  -> /localization/amcl_pose_raw
  -> amcl_pose_gate
  -> /localization/global_pose
  -> map_odom_from_global_pose + timestamp-matched /odometry/lio
  -> /localization/map_to_odom + canonical map->odom

FAST-LIO raw odometry
  -> lio_adapter
  -> /odometry/lio + canonical odom->base_link
```

3D GICP 使用相同公共边界：后端先发布 raw pose 和算法有效状态，再经过公共
global-pose gate，最终仍由 `map_odom_from_global_pose` 独占 canonical TF。

## 当前 global pose gate 已检查内容

`global_pose_gate` 已检查：

1. `frame_id=map`；
2. 时间戳非零、不过期且不过度超前；
3. pose 和 covariance 全部有限；
4. 四元数有效；
5. z、roll、pitch 符合平面定位边界；
6. x/y/yaw covariance 非负且不超过配置上限；
7. 接受流超时后将 backend validity 置 false。

这些检查能拒绝格式、时间、平面语义和定位器自报不确定度异常，但不能证明 pose
与当前传感器观测、静态地图和连续里程计一致。

## 仍可能穿透的错误

历史高速自转已经证明，AMCL 可能收敛到错误峰，同时报告较小 covariance。因此以下
错误仍可穿透当前 gate：

- 低 covariance 的错误峰；
- 相邻 global pose 或候选 map->odom 的异常跳变；
- 与静态地图明显不一致、但 AMCL 自身仍接受的 scan；
- scan、pose、odom 各自新鲜但彼此时间不匹配；
- 动态云台条件下用错误时刻 sensor TF 形成的几何一致性假象；
- 地图重复结构造成的竞争峰；
- 输入点过少、scan dropout 或地图不可用时的不可判定状态。

单独的 pose jump 也不能证明错误：人工重定位、reset 后首次恢复可能产生合法大修正。
因此第一版只把 jump 当作一类证据，不单独产生 shadow `REJECT`。

## map->odom 安全边界

当前 bridge 已经具备以下重要保护：

- 使用 global pose 的 source timestamp 匹配 `/odometry/lio` cache；
- 无匹配 odometry 时进入有界 pending queue，超时丢弃；
- odom 时间回退时清 cache、清 pending 并使 correction invalid；
- 可跟随 backend-specific valid topic；
- 无输入时不发布 identity fallback；
- `map_odom_stub` 与真实 bridge 互斥；
- 只有 bridge 发布 canonical `map->odom`。

Integrity 第一阶段位于该边界旁路，观察 `/localization/global_pose` 并独立计算候选
correction 指标，但不接入 bridge 输入和输出。

## TF 与时间戳审计

- `pointcloud_to_laserscan_node` 的普通投影使用 cloud header stamp 查询 TF；严格
  deskew candidate 使用逐点时间和 timestamped odometry/TF，失败时丢整帧。
- `lio_adapter` 对非零 raw odometry timestamp 查询对应时刻 sensor TF；只有明确的
  零时间戳兼容分支使用 latest TF。
- `map_odom_from_global_pose` 按 global pose 时间匹配 odometry，而不是使用 latest。

新增 verifier 必须对选中的 LaserScan 使用 scan header timestamp 查询
`base_link <- scan_frame`。不能使用 `TimePointZero`。未来单雷达动态云台仍可复用该
实现，只需提供时间正确的云台 TF 和标准 `/localization/scan`。

## 可复用 diagnostics

- scan deskew candidate 已发布 `diagnostic_msgs/DiagnosticArray`；
- GICP 已发布 registration valid、fitness 和 map id；
- readiness monitor 汇总依赖是否在线，但不判断几何正确性；
- LIO 保留计算时间和 localizability 等 backend-private diagnostics。

项目没有统一的 `diagnostic_updater` wrapper，但已有直接发布 DiagnosticArray 的风格。
Integrity 继续使用标准 `/diagnostics`，不新增复杂消息体系。

## 历史高速自转资产

仓库保留了完整分析文档、candidate 参数、严格 deskew 实现、测试和 Linux 证据路径。
报告中的关键工况包括约 `7.051 rad/s` 中值、`7.348 rad/s` 峰值和单次 AMCL
`0.837 rad` 角增量。

仓库中没有 `.db3` 或 `.mcap`。文档引用的 bag 和分析输出位于历史 Linux `/tmp`
或外部 validation archive，当前 Windows 工作区无法访问。因此本轮不声称完成 replay，
必须重新取得历史 bag 或按现场计划采集。

## 最合理的插入位置

```text
/map ---------------------------+
/localization/scan ------------+|
/odometry/lio ----------------+||
/localization/global_pose ---+|||
                              vvvv
                 localization_integrity (shadow)
                         -> /diagnostics
                         -> optional JSONL evidence
```

不直接订阅 raw AMCL 是有意设计：公共 `/localization/global_pose` 让 AMCL 与 optional
GICP 共用同一监测边界。后端专有指标仍由各后端 diagnostics 提供。

## 包结构决定

新增单一 `rm_navigation_integrity` ament_python 包：

- `core.py`：无 ROS 依赖的 SE(2)、距离场、scan-map 指标和状态判定；
- `localization_integrity_node.py`：只读 ROS shadow node；
- `regression.py` / `evaluate_regression.py`：JSONL 原始指标汇总和 baseline delta；
- `config/`：shadow 参数和场景定义；
- `launch/`：默认 `enabled=false` 的独立入口；
- `test/`：Windows 可运行的纯算法测试。

不放入 `rm_system_monitor`，因为 readiness 的职责是输入是否齐备，而 integrity 的职责
是时间与几何证据是否互相支持。两者可以同时消费标准 diagnostics，但不互相授权。

## 新车影响

可原样复用：pose/odom consistency、distance-field 算法、diagnostics、JSONL、回归定义。

仅需参数化：topic、frame、频率、阈值、地图和 scan 投影 profile。

必须新车实测：云台 angle timestamp/延迟、scan deskew、动态 sensor TF、正常运动指标
分布和阈值。雷达数量、IP、左右命名和固定安装角不得进入 integrity 核心代码。
