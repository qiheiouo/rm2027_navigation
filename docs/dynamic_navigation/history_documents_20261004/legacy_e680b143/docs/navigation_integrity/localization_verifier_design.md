# Shadow Localization Verifier 设计

## 目标

第一版回答两个独立问题：

1. global pose 是否突然与连续 odometry 产生异常 correction；
2. 与该 pose 时间接近的 LaserScan endpoint 是否得到静态 occupancy map 支持。

节点只记录证据。`REJECT` 表示“若未来启用 gate，当前证据组合应被拒绝”，实际导航
仍使用原来的 global pose 和 `map->odom`。

## 输入与输出

输入全部参数化：

- `/map` (`nav_msgs/OccupancyGrid`)；
- `/localization/scan` (`sensor_msgs/LaserScan`)；
- `/localization/global_pose` (`PoseWithCovarianceStamped`)；
- `/odometry/lio` (`nav_msgs/Odometry`)；
- timestamped `base_link <- scan_frame` TF。

输出：标准 `/diagnostics` 中名为 `navigation_integrity/localization` 的 status，以及仅在
显式配置路径时写出的 JSONL。节点没有 TF broadcaster、action client、pose publisher
或 velocity publisher。

## 候选 correction

对 source timestamp 匹配的 odometry：

```text
T_map_odom_candidate = T_map_base_candidate * inverse(T_odom_base)
```

记录相邻 global pose 和相邻 candidate correction 的平移/yaw 跳变及变化率。首次样本
没有 delta；时间回退会清除比较历史并产生独立 reason code。

## 2D scan-map agreement

地图载入时，从 occupied cells 建立 8 邻域距离场。每次 global pose 更新时：

1. 从 bounded cache 选择时间最接近的 scan 和 odometry；
2. 用 scan header timestamp 查询 `base_link <- scan_frame`；
3. 计算 `T_map_base_candidate * T_base_scan * endpoint_scan`；
4. 查询每个 endpoint 到最近 occupied cell 的距离；
5. 输出 map 内 endpoint 数、支持数、agreement、mean residual 和 p95 residual。

该算法没有执行 ICP/GICP，因此不输出 Hessian、信息矩阵、condition number 或
observability。距离场是近似 8 邻域欧氏距离，适合低成本 shadow 对照，不是新的定位器。

## 状态与原因

- `GOOD`：所需证据齐备，且未触发 provisional 异常；
- `UNKNOWN`：地图、scan、odom、TF、时间匹配或有效点不足，无法判断；
- `SUSPECT`：一项跳变、rate 或 scan-map 指标异常；
- `REJECT`：硬 correction/pose jump 与硬 scan-map disagreement 同时出现。

低 agreement 单独只产生 `SUSPECT`，因为动态障碍、地图缺口、玻璃、稀疏 scan 或
错误地图 revision 都可能降低 agreement。大 pose jump 单独也只产生 `SUSPECT`，因为
人工重定位可能合法。

原因码保持离散，不合成未经标定的 confidence score。所有参数阈值均标记为 shadow
provisional；历史失败数值只用于提供初始观测范围，不构成安全认证。

## Timing Integrity

- pose/scan/odom 都以 message stamp 比较；
- scan TF 严格以 scan stamp 查询；
- static TF 返回零 stamp 时记录为零差值；
- TF lookup 失败不会 fallback 到 latest；
- cache 有界；
- timer 会在没有新消息时继续更新 freshness 状态。

这使节点天然兼容未来动态云台。若上游 `/localization/scan` 本身用错误时刻生成，
verifier 无法修复它，只能结合 scan deskew diagnostics 判断输入是否可信。

## 计算成本与限制

- 距离场仅在地图消息到达时构建；
- 每次最多抽样 `max_scan_points` 个 endpoint；
- 默认在 global pose 更新时计算，diagnostics 仅 2 Hz；
- Windows 尚未测得 ROS 实际 CPU，Linux 必须记录 map build time 和 callback latency。

该方法不能在完全重复结构中证明唯一位置，也不能识别与错误 pose 同样吻合的对称地图
峰。若实车数据证明 2D agreement 无法区分失败，再评估独立 scan matcher 或 optional
GICP shadow verifier；不能先引入重型 3D 依赖。

## Gate Mode 前置条件

本轮代码拒绝非 `shadow` mode。未来增加 WARN/GATE 必须先满足：

1. 正常与失败 bag 都存在；
2. 指标在独立路线、地图区域和速度下可重复；
3. false positive/negative 有量化结果；
4. reset、人工 initial pose 和地图切换有明确生命周期；
5. gate 与 bridge 的单一 TF ownership 经过单独设计和测试。
