# HWSentryNav26 选择性迁移 Phase 1

## 基线与来源

- 开发分支：`feature/hwsentry-selective-migration-phase1`。
- 基线：`feature/navigation-integrity @ 64870dc`。
- 参考仓库：<https://github.com/Polyacetone/HWSentryNav26>。
- 审计提交：`20ef3d14ac0dc10e331691407b07663887cd0a73`。
- 最新复核提交：`f5f9412`。`20ef3d1..f5f9412` 没有 tracker、planner、
  LIO 或地图核心源码变化，无需重复迁移。
- 上游许可证：MIT。
- 本轮没有 vendoring、submodule 或逐文件复制。实现是依据算法边界重新编写的独立代码。

选择性迁移遵守现有 canonical TF、Nav2 action authority 和安全默认。外部系统的
`latest TF`、自定义未来代价图、自研整套 planner/executor 不进入当前主线。

本轮重点源码锚点：

- HWSentry tracker：`map_server/src/object_tracker.cpp`、
  `map_server/src/map_server_node.cpp`；
- HWSentry 离线射线清理：
  `utils/cpp/offline_mapping_optimizer/src/raycasting_filter.cpp`；
- HWSentry execution：`nav_executor/task_manager.cpp`；
- HWSentry LIO：`small_glim`；
- 当前 mission：`src/rm_competition_mission/src/competition_mission_node.cpp`；
- 当前 mapping：`src/rm_map_tools/rm_map_tools/mapping_session_node.py`。

## 选择结论

| HWSentry 能力 | 结论 | 本轮交付 |
|---|---|---|
| 动态点提取、聚类、CV Kalman、轨迹生命周期、短时预测 | 值得迁移 | 独立 shadow 包 |
| future costmap 直接驱动自研 FDDP/MPC | 不直接迁移 | 仅保留 MPPI critic 设计 |
| 逐帧点云+位姿的 3D-DDA raycasting 清图 | 值得迁移，但依赖数据 | sidecar 驱动的候选 PCD CLI |
| TaskManager generation/latest-wins/COMMITTED | 作为合同参考 | 覆盖矩阵和狗洞状态机设计 |
| Spatial A*、Kino A*、MINCO、速度剖面 | 与轮腿地形强耦合 | 仅形成 annotated path 设计 |
| small_glim factor-graph LIO | 有实验价值 | 可执行 A/B 计划，不替换 FAST-LIO |

## 动态跟踪输入决策

### A. 2D scan + occupancy map

选为 Phase 1 输入：

```text
/map + timestamped /local_scan
  -> 静态地图近邻剔除
  -> 欧氏聚类
  -> gated one-to-one association
  -> [x, y, vx, vy] CV Kalman
  -> tentative / confirmed / coasting / lost
  -> velocity-decay prediction
```

优点是标准接口、计算量小，且雷达数量、点云过滤、云台运动补偿均留在上游。旧车可
复用现有 PointCloud2-to-LaserScan 节点；新车只要提供时间正确的 `/local_scan` 即可。

局限是 2D 投影丢失高度，地图脏点会制造漏检，静态物体附近的动态目标会因 subtraction
容差而被部分吞掉。它是 shadow 验证起点，不是最终安全感知。

### B. 3D cloud + global PCD

对物体高度、悬空结构和多雷达信息保留更好，但要求可靠 PCD、3D NN 索引、逐点运动补偿
和更高算力。当前 occupancy-only 场景与新车地图条件不能保证这些前提，因此留作第二阶段。

### C. costmap 或 obstacle representation

不采用。costmap 已包含 inflation、clearing 残留和 layer combination 结果，不能可靠恢复
“这次观测到的动态点”。把它作为 detector 输入会将已有假障碍进一步固化。

## 算法选择

- 静态剔除：endpoint 必须落在 map 已知 free 区，并与最近静态 occupied cell 的距离大于
  `static_distance_threshold`。阈值参数化，不硬编码旧车尺寸。
- 聚类：metric bucket 加速的欧氏连通聚类。最小点数、容差和最大外接尺寸均参数化。
- 关联：带距离门的一对一贪心匹配。目标数量少时更简单、可审计且无 SciPy 依赖。
  若 D05 交叉工况出现不可接受的 ID switch，再切换 Hungarian，而不是提前增加复杂度。
- 滤波：x/y 两个独立 1D 常速度 Kalman，整体状态等价于 `[x,y,vx,vy]`。
- 生命周期：连续命中确认；短暂丢失进入 coasting；每次关联前先按源时间戳清理超过
  `max_coast_time_sec` 的旧 track，避免长断流后错误复用 ID，也避免 10 Hz/50 Hz
  输入产生不同寿命。时间回退清空全部 track，防止 bag loop 或时钟重置后继承旧状态。
- 预测：常速度加可选指数速度衰减，预测步长、周期、最大速度均参数化。

## 输出与权限

当前只发布：

- `/perception/dynamic_obstacles_shadow/markers`；
- `/perception/dynamic_obstacles_shadow/diagnostics`。

MarkerArray 用于 RViz 显示 ID、状态、包围盒、速度箭头和未来轨迹。它不是稳定控制 API。
在 shadow 数据通过现场门之前，不新增 controller 消费消息，不修改 local/global costmap、
planner、MPPI、goal、TF 或 `/cmd_vel`。

## Old-car D01-D07 验证

1. D01 空场静止 60 秒：统计 false track、callback latency、CPU/内存。
2. D02 单人横穿：确认延迟、速度稳定性、离场后的 coasting/deletion 时间。
3. D03 单人径向靠近/远离：确认 scan 稀疏方向的速度和预测误差。
4. D04 静态墙、箱体旁经过：测 map subtraction 的误检/漏检。
5. D05 两人交叉：统计 ID switch，决定是否需要 Hungarian。
6. D06 遮挡后重现：统计 track 生命周期和错误重连。
7. D07 机器人平移/自转：验证 timestamped TF、输入丢帧和假速度。旧车高速自转只在安全
   条件下进行，新车动态云台需另行验收。

每轮保存 scan、map、TF、markers、diagnostics 和人工标注。至少报告 false tracks/min、
确认延迟、ID switches、轨迹寿命、速度方差、0.5/1.0/1.5 秒预测误差、CPU、RSS、P95
callback latency 与输入 drop。

可执行命令和 provisional PASS/FAIL 门见
`docs/hwsentry_migration/phase1_linux_validation.md`。

## 状态

- 纯算法与 ROS wrapper：已实现。2026-08-16 在代码提交 `65fa728` 上完成隔离的
  Docker/Humble 构建与测试；tracker 11 项、map tools 53 项均通过。
- MPPI/costmap 接入：未实现，明确禁用。
- 隔离 ROS runtime/default-off/topic authority smoke：已通过。默认 launch 不创建
  tracker；显式启用后不发布 TF 或 `/cmd_vel`。带真实 `/map + /local_scan` 的旧车
  无运动 smoke 仍未执行。
- old-car D01-D07：未执行。
- 新车单雷达/动态云台：接口可复用，时间与外参必须实车重验。
