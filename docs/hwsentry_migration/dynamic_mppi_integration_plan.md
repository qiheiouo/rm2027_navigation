# 动态目标与 Nav2 Humble MPPI 集成评估

## 当前结论

动态目标代价可以作为独立 MPPI critic 插件实现，原则上不需要 fork 或
修改 `nav2_mppi_controller` core。Nav2 Humble 的 `CriticManager` 通过
pluginlib 加载 `mppi::critics::CriticFunction`，critic 可读取批量轨迹、代价、
时间步长和 costmap 数据。

本阶段不实现、不加载该 critic。`rm_dynamic_obstacle_tracking` 仍为
shadow-only，MarkerArray 仅用于观察，不是控制器稳定接口。

参考源码：

- [CriticFunction interface](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp)
- [CriticManager plugin loading](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critic_manager.cpp)
- [CriticData](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/critic_data.hpp)

## 未来稳定输入

进入控制器前应定义小型、只读的预测消息，至少包含：

```text
header(frame_id=map, source timestamp)
track_id
state
current pose and footprint
velocity
prediction dt
predicted poses or footprints
confidence/freshness
```

MarkerArray 不提供稳定的语义、协方差或版本合同，不应被 critic 解析。

## Critic 计算

对采样轨迹的第 `k` 个时间步：

```text
t = k * model_dt
robot_footprint = sampled_robot_state(k)
object_footprint = interpolate(track_prediction, t)
cost += proximity_or_overlap(robot_footprint, object_footprint)
```

第一版只考虑 confirmed track。coasting track 应降低权重并随时间衰减；
tentative、lost、过期、错误 frame 或未来时间戳数据应当无效果，而不是阻塞
控制周期。是否使用硬碰撞失败标志必须在 shadow score 和仿真通过后决定。

## 并发与时序

订阅回调只构造不可变预测快照并原子替换。`score()` 不应等待订阅锁，也不应
在采样批次内读取两版目标集合。预测数据必须在统一 frame 下，且明确区分：

1. 观测源时间戳；
2. tracker 处理完成时间；
3. MPPI 当前求解时间；
4. 每个 trajectory step 的相对时间。

过期策略必须 fail-open 到“本 critic 不加分”，并通过 diagnostics 报警；不能
因为 tracker 掉线凭空制造障碍，也不能重放最后一帧预测。

## 计算量控制

直接复杂度近似为：

```text
batch_size * time_steps * active_tracks
```

实现前应限制 active track 数和 horizon，并优先用 Eigen 张量批量计算。进入
实车前必须证明 P95/P99 controller 周期没有显著回归。

## 分阶段门

1. tracker shadow：验证 false track、ID、速度和预测。
2. offline critic score：对录包轨迹计算代价，不接 Nav2。
3. loaded-with-zero-weight：插件加载但权重为 0，验证时延。
4. simulation A/B：验证避让收益、振荡和不可达行为。
5. old-car limited field A/B：限速、人工接管、逐场景放行。

在第 1 阶段没有老车证据前，不进入后续阶段。

