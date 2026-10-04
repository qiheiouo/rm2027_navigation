# HWSentryNav26 Phase1 Linux Validation Report

验证日期：2026-08-24（Asia/Shanghai）

验证对象：旧车、左侧 MID360、真实实验室 `/map + /local_scan`

验证性质：shadow-only 集成与 D01-D07 实车准入验证，不是控制安全认证

> 本报告第 1-10 节保留 2026-08-24 首轮实车验证快照。第 11 节记录随后在
> `fix/hwsentry-tracker-field-gates` 上完成的最小修复与同一批 bag 的离线 A/B。
> 修复候选显著改善 D01、性能和退出行为，但 D05、D07 仍未过冻结门；总体结论仍为
> **FAIL**，不能把离线重放等同于修复后实车验收。

## 1. 环境信息

| 项目 | 结果 |
| --- | --- |
| Host workspace | `/home/wpie/rm2027_navigation` |
| Container workspace | `/workspace/rm2027_navigation` |
| Container OS | Ubuntu 22.04 |
| Kernel | Linux 6.8.0-124-generic x86_64 |
| ROS | ROS 2 Humble，`ROS_VERSION=2`，Python 3 |
| ROS_DOMAIN_ID | `0` |
| ROS setup | `/opt/ros/humble/setup.bash` 与 workspace `install/setup.bash` 已加载 |
| `ros2 --version` | Humble CLI 不支持该参数；以 `ROS_DISTRO=humble` 和已安装包路径确认 |
| Tracker install prefix | `/workspace/rm2027_navigation/install/rm_dynamic_obstacle_tracking` |
| 地图 | `lab_dog_hole / 20260820T040437Z`，candidate，map frame |
| 地图分辨率 | 0.05 m/cell |
| Bundle SHA256 | `5c2656e3763778e466855da80f9abefb7be0d478381567a686bb7dad4a7b3d91` |
| Tracker 参数 SHA256 | `b388debaa5ee25d09e2195228a5897309ddedfd95c032f7ef7ec6a665addd9a7` |

地图 bundle 声明 PCD/occupancy 共用 map origin，但仍为 `candidate`，且
`review_status=requires_human_landmark_review`。本轮已由操作者在 RViz 对现场地图和扫描进行
检查；D06 临时加入泡沫板后不再属于完全静态匹配场景，因此 D06 结果单独注明夹具干扰。

## 2. Git 状态

- 当前分支：`integration/old-car-consolidation`
- 验证基线 HEAD：`4198443f93e23d4010a6a5888007cacf203918ea`
- 最终 tracker 修复提交：`cc254b4`（本报告提交前生成）
- 原 Phase1 四个提交 `22117ce`、`d0ca960`、`0944c99`、`c4f064c` 均为当前 HEAD 的祖先，
  迁移包、清图工具和文档均存在。
- submodule：`fast_lio_multi@e7864a6`、`ikd-Tree@0438b0d`、
  `livox_ros_driver2_humble@2a2029a`。
- 验证结束时工作区不 clean：
  - `M src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py`
  - `?? core.105`（488,452,096 bytes，RViz core dump）
  - 本报告为新增验证记录。

唯一源代码差异是把 tracker 从单线程 `rclpy.spin()` 改为两线程
`MultiThreadedExecutor`。原实现被 50 Hz scan callback 内的阻塞 TF lookup 占满，TF
订阅无法及时执行，持续输出 `tf_unavailable`。这是进入实车测试所需的最小必要改动，已经
build/test，测试期间没有主动扩展算法。报告原始指标对应 `4198443 + 修复差异`，最终经审查、
隔离复验后以 `cc254b4` 提交；并不等价于对原 clean HEAD 的通过结论。

## 3. Colcon Build 结果

| 项目 | 结果 |
| --- | --- |
| 当前 tracker build | PASS，`rm_dynamic_obstacle_tracking` 完成安装 |
| 当前 tracker tests | PASS，11 passed / 0 failure / 0 error |
| 当前 map tools build | PASS，`rm_map_tools` 1 package finished |
| 当前 map tools tests | PASS，213 tests / 0 failure / 0 error / 0 skipped |
| `package.xml` | PASS，ament_python 与运行依赖存在 |
| `setup.py` / entry point | PASS，`dynamic_obstacle_tracker_node` 已安装并实际启动 |
| launch/config install | PASS，安装空间可找到 launch 和 YAML |
| ROS package discovery | PASS，`ros2 pkg prefix rm_dynamic_obstacle_tracking` 返回安装路径 |

Tracker 当前构建日志：`log/build_2026-08-24_02-54-26/`；测试日志：
`log/test_2026-08-24_02-54-31/rm_dynamic_obstacle_tracking/stdout_stderr.log`。
Map tools 当前隔离结果目录：`/tmp/rm27_hwsentry_20260824_map_tools_results`。

## 4. ROS 节点启动结果

`dynamic_obstacle_tracking_shadow.launch.py enabled:=true` 能正常创建
`/dynamic_obstacle_tracker_shadow`。节点接收 `/map`、`/local_scan`、`/tf`、`/tf_static`，
运行阶段 diagnostics 可恢复并持续为 `ok`。

原 clean HEAD 的单线程 executor 在真实 50 Hz 输入链上发生 TF callback 饥饿，不能作为
本轮通过基线；现场当时应用尚未提交的两线程最小差异后才完成 D01-D07。该差异及 SIGINT
收尾修复现已纳入 `cc254b4`，并通过第 11 节所列隔离复验；原始现场异常保留在 P2-01 作为
取证时间线，不代表最终提交仍有该异常。

## 5. Topic 接口验证

Tracker 的业务发布者只有：

- `/perception/dynamic_obstacles_shadow/predictions`
- `/perception/dynamic_obstacles_shadow/markers`
- `/perception/dynamic_obstacles_shadow/diagnostics`

`/local_scan` 正式验证高度范围恢复为 0.18-2.00 m，实测约 8.96-9.18 Hz。D07 中
`/local_scan` 2416 条、predictions/diagnostics/markers 各 2419 条；时间戳单调，最大 scan
间隔 0.140 s，prediction `complete=false` 为 0。

## 6. Shadow-only 约束验证

PASS。`ros2 node info /dynamic_obstacle_tracker_shadow` 表明：

- tracker 只订阅 map、scan、TF；
- 不发布 `/tf` 或 `/tf_static`；
- 不发布 `/cmd_vel`，且 D07 开始前 `/cmd_vel` 不存在；
- 不发布 local/global costmap、plan 或 goal；
- 没有 action server/client，未接管 Nav2、mission 或底盘。

迁移提交没有修改 `nav2_controller`、`nav2_planner` 或 `nav2_costmap`。当前 mission 的
generation/latest-wins、旧 goal cancel、过期 callback 屏蔽和 retry/backoff 逻辑仍位于
`src/rm_competition_mission/src/competition_mission_node.cpp:362-448`，本轮只做静态审计，
未在 D01-D07 中运行 mission。

未来 MPPI `DynamicObstacleCritic` 在架构上仍可作为 pluginlib 扩展，但按冻结门，在 tracker
D01-D07 全部通过前不得实现或接入控制链。

## 7. RViz 验证

PARTIAL。

- RViz Fixed Frame 使用 `map`，现场用于检查 map、scan 和 MarkerArray；marker 包含 track
  box、ID/state、速度箭头和预测线。
- 操作者完成了 D02-D07 的现场路线，但没有保存固定机位视频或 RViz 截图，因此 ID switch
  和预测误差没有独立视觉真值。
- 验证期间生成一次 `core.105`，core 指向 `/opt/ros/humble/lib/rviz2/rviz2`；之后 RViz
  能继续运行，但该崩溃尚未稳定复现。

因此 RViz 仅作为观察工具，不用于替代 bag 数值和外部真值。

## 8. Offline bag 验证

证据目录：`log/hwsentry_d01_d07_20260824T1052CST/`。所有场景均独立保存 map、scan、
LIO odometry、TF、predictions、markers 和 diagnostics。

| 场景 | 原始记录与指标 | 判定 |
| --- | --- | --- |
| D01 空场静止 | 61.442 s；520/520 diagnostics=`ok`；38 个 confirmed ID，约 37.1/min；最长连续 0.477 s；callback P95 69.545 ms | **FAIL**：误检率大于 1/min，P95 大于 20 ms |
| D02 单人横穿 | 3 次往返/6 次横穿；6/6 检出；confirmed 延迟 0.220-0.481 s；方向正确；离场 coasting/delete 0.52-0.58 s；首轮 ID 149->154 碎片化 | **CONDITIONAL PASS**：确认、方向、删除通过；连续性和背景误检不通过整体门 |
| D03 径向运动 | 至少 5 次往返；方向 137/138=99.275%；同轨迹 0.5 s 一致性误差 P95 0.371 m、max 0.522 m | **CONDITIONAL PASS**：P95 门内，但没有外部位置真值，仅能证明内部一致性 |
| D04 静态结构边缘 | 86.897 s；755/755 `ok`；未出现“持续>=1 s、span<=0.3 m、mean speed<=0.2 m/s”的墙 track；5 次人员通过每次碎成 2-3 个 ID | **PASS** 静态墙门；人员连续性另列问题 |
| D05 双人交叉 | 75.848 s；661/661 `ok`；52 个 confirmed ID；双人场景最大 3 个 confirmed、7 个 total；多轮出现碎片化/方向反转 | **FAIL**：连续性不满足，触发 Hungarian 评估；无身份视频使精确 switch 数只能下界估计 |
| D06 短遮挡 | 128.769 s；1136/1136 `ok`；5 次中 3 次同 ID 保持，观测 gap 0.360/0.360/0.460 s；2 次碎片化 | **PARTIAL PASS** |
| D06 长遮挡补录 | 180.056 s；1615/1615 `ok`；4 组可审计断流约 5.34/7.02/5.96/5.56 s；全部使用新 ID，无过期 ID 复用；2 组重现碎片化 | **PASS** 长断流 ID 生命周期门 |
| D07 机器人运动 | 272.876 s；2419/2419 `ok`；识别 12 个运动段/38.10 s；运动阶段 24 个 confirmed ID，约 37.8/min；最长 0.321 s；scan/prediction 最大 gap 0.140 s；callback P95 70.546 ms | **FAIL**：无持续 drop 且没有运动诱发放大，但误检率和 P95 均超过门 |

D07 额外结果：运动阶段 confirmed 输出占比 53/343，静止阶段为 436/2076；运动误检率
没有高于静止基线。最大单步位移 0.035 m、最大单步 yaw 0.0478 rad，往返后首尾净差约
0.144 m / 0.020 rad，未发现定位离散跳变。

性能快照（测试完成后、旧 NUC）：tracker 约 55.5% CPU、RSS 70,180 KiB。用户已说明新车
计算平台更强，但冻结文档规定的 P95<20 ms 门仍按本次设备结果记为失败，不能用未来硬件
假设改写当前结论。

## 9. 清图工具验证

软件合同 PASS，真实地图效果 DEFERRED。

- 当前代码 `rm_map_tools` build PASS，213/213 tests PASS。
- 输入合同要求 sensor origin 与 endpoint；3D traversal 位于
  `src/rm_map_tools/rm_map_tools/ray_evidence_cleanup.py:72-121`。
- `origin -> voxel traversal -> hit/pass evidence update` 位于同文件 `124-206`。
- 候选 PCD 必须显式 `--write-candidate`，输出路径和输入必须隔离，位于同文件
  `317-340`；不可变输出、写失败和 symlink race 测试位于
  `src/rm_map_tools/test/test_immutable_output.py:11-65`。
- 工具不会覆盖源 PCD，不修改 PGM、现有 bundle 或部署文件，只能生成新的 candidate/report。
- 当前 `lab_dog_hole` bundle 没有与建图同会话的 `ray_observations` artifact；依照
  `docs/hwsentry_migration/phase1_linux_validation.md:121-137`，不得用最终 PCD 伪造 rays，
  所以真实 PCD 清理效果没有执行。

结论：3D-DDA 和不可覆盖合同 **PASS**；真实 ray-sidecar 地图效果 **DEFERRED**。

## 10. 发现的问题

### P0：阻塞使用

无。Tracker 保持 shadow-only，不会直接造成底盘控制失效。

### P1：必须修复

#### P1-01 静态环境 confirmed false track 远超准入门

- 证据：D01 约 37.1/min，D07 运动阶段约 37.8/min；门为 <=1/min。
- 代码锚点：
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:127-175`；
  `src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml:11-18`；
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:281-292`。
- 复现：使用本报告 map/参数启动 no-motion AMCL chain 和 shadow tracker，清空场景并静止
  60 s，录制 D01 话题，统计 diagnostics 中进入 `confirmed` 的唯一 track ID。
- 影响：D01、D07 正式 FAIL，动态准入和 MPPI 解冻条件不成立。

#### P1-02 callback latency 超过 20 ms 门且旧 NUC CPU 占用高

- 证据：D01 P95 69.545 ms，D07 P95 70.546 ms；运行快照 CPU 55.5%。
- 代码锚点：逐 endpoint 搜索 occupied cell 的嵌套循环位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:127-151`，每帧调用链位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:260-307`。
- 复现：正式高度 0.18-2.0 m、约 9 Hz `/local_scan` 下录制 diagnostics，读取
  `latency_ms` 的 P95，同时用 `ps` 记录节点 CPU/RSS。
- 影响：当前平台不满足冻结资源门；新平台仍需完整复测，不能直接假定通过。

#### P1-03 D05 双目标交叉发生不可接受的 ID 碎片化

- 证据：五轮双人交叉中出现多次 ID 重建/方向反转，75.8 s 内生成 52 个 confirmed ID。
- 代码锚点：当前按距离排序的一对一贪心关联位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:370-388`。
- 复现：两人以可区分起点沿交叉路线通过至少五轮，录制 predictions 和固定机位身份视频，
  按每轮统计 ID switch、错误合并和拆分。
- 影响：按设计文档条件，应只解冻关联模块评估 Hungarian/更稳健代价，不应接控制链。

#### P1-04 原验证基线的单线程 executor 会造成 TF callback 饥饿（已由 `cc254b4` 修复）

- 证据：真实输入启动后持续 `tf_unavailable`；切换两线程 executor 后 D01-D07 diagnostics
  可持续 `ok`。
- 代码锚点：TF listener/阻塞 lookup 位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:139-140,215-244`；
  修复后的 executor 与 shutdown 路径位于同文件 `13,538-555`。
- 历史复现：在 `4198443` 撤去 executor 差异、保留约 50 Hz 上游点云和约 9-10 Hz scan，
  启动 tracker；TF 实际存在但 diagnostics 持续为 `tf_unavailable`。应用两线程差异后恢复。
- 处置：修复已在独立分支完成审查、隔离构建、18 项测试和 D01 实车复测，并提交为
  `cc254b4`。该项不再阻塞 shadow-only 代码合并，但不改变 D05、D07 的算法准入结论。

### P2：可以后续优化

#### P2-01 现场两线程候选 SIGINT 收尾异常（已由 `cc254b4` 修复）

- 现象：Ctrl+C 时出现 `KeyboardInterrupt`，随后 `rcl_shutdown already called`，launch 报
  node exit code 1。
- 代码锚点：
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:543-548`。
- 复现：启动 `dynamic_obstacle_tracking_shadow.launch.py enabled:=true`，稳定运行后 Ctrl+C。
- 处置：最终实现显式关闭 executor、销毁节点并只在 context 仍有效时 shutdown；第 11 节的
  timeout/SIGINT 隔离复验未再出现 traceback 或 `rcl_shutdown already called`。

#### P2-02 D02/D04/D06 的单目标轨迹也存在碎片化

- 现象：D02 首轮 ID 149->154；D04 每次人员通过常被拆成 2-3 个 ID；D06 短遮挡仅 3/5
  保持同 ID，长遮挡两组重现轨迹碎片化。
- 代码锚点：关联和 tentative 删除分别位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:370-422`，参数位于
  `src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml:25-32`。
- 复现：按 D02、D04、D06 bag 路线运行并绘制每帧 track ID/position/state。

#### P2-03 RViz 单次崩溃并遗留 466 MiB core

- 证据文件：`/home/wpie/rm2027_navigation/core.105`，core exec 为
  `/opt/ros/humble/lib/rviz2/rviz2`。
- 启动锚点：`src/rm_navigation_bringup/launch/old_car_2026_validation.launch.py:511-516`。
- 复现记录：以 `use_rviz:=true` 启动旧车 AMCL no-motion chain；本轮发生一次，随后未稳定复现。
- 影响：占用工作区空间并使工作区不 clean；在用户确认前未删除。

#### P2-04 缺少固定机位身份视频和外部位置真值

- 要求锚点：`docs/hwsentry_migration/frozen_validation_handoff.md:122-147`。
- 复现：仅依赖 bag/marker 对 D03 预测误差或 D05 身份切换下结论时，无法得到独立 ground
  truth，只能计算同轨迹内部一致性和 ID switch 下界。
- 影响：D03 只能 CONDITIONAL PASS，D05 无法给出精确每轮 switch 数。

### P3：设计建议

#### P3-01 新车动态云台需独立重验时间与外参链

- 设计锚点：`docs/hwsentry_migration/phase1_design.md:99-113`。
- 复现：让云台转动并保持底盘静止，记录原始雷达 timestamp、云台 yaw timestamp、
  `map -> odom -> base_link -> gimbal -> lidar` TF 和 `/local_scan`；检查静态背景假速度。
- 建议：仿真先覆盖云台恒速、加减速、时间偏移和外参偏差，之后在新车实雷达上完整重跑
  D01、D04、D07。

## 11. 修复后离线 bag 复用与 A/B（2026-08-24）

### 11.1 修复范围

当前修复分支为 `fix/hwsentry-tracker-field-gates`，验证基线为
`4198443f93e23d4010a6a5888007cacf203918ea`，最终 tracker 修复提交为 `cc254b4`。修复仅涉及
`rm_dynamic_obstacle_tracking`：

- 预索引占用栅格中心，避免每个 endpoint 重复遍历局部栅格和计算地图旋转；当前实现位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:107-193`。
- 聚类后按 centroid 再做一次 0.35 m 静态地图近邻过滤，位于
  `core.py:280-297` 和
  `dynamic_obstacle_tracker_node.py:296-313`。
- tentative 轨迹除连续 3 次命中外，还必须使“当前观测”相对创建点移动至少 0.15 m；不再
  使用曾经出现过的历史最大跳变，位于 `core.py:577-589`。
- 增加可选 gated global minimum-cost assignment，位于 `core.py:300-392,535-567`；
  D05 实包 A/B 无改善，因此生产 YAML 明确保持 `false`。
- 两线程 executor 解除阻塞 TF lookup 对 TF subscription 的 callback 饥饿，并在 SIGINT 时
  避免二次 shutdown，位于
  `dynamic_obstacle_tracker_node.py:558-572`。
- 最终参数锚点：
  `src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml:12-39`。

没有修改 Nav2、mission、costmap、TF 发布或底盘控制权限。

### 11.2 bag 是否可以复用

**可以，而且本轮已复用。** D01-D07 bags 保存了 `/map`、`/local_scan`、时间戳 TF 和
LIO odometry，因此可对相同传感器输入确定性重跑静态扣除、聚类、跟踪和生命周期，特别适合
修复前后 A/B、参数扫描和回归检查。离线脚本只读 sqlite3 bag，没有改写原始 `db3` 或
`metadata.yaml`。

它不能替代三类证据：

1. 修复代码在实时 ROS executor 下的 CPU/RSS 和 callback latency；
2. 没有固定机位视频时的 D05 人员身份真值；
3. 环境、外参、时间同步或新车动态云台发生变化后的实车结果。

因此 bags 可用于修复开发和回归，但最终关闭 P1 仍需要短实车复测。

### 11.3 最终候选 A/B 结果

最终候选使用 endpoint 静态距离 0.25 m、cluster centroid 静态距离 0.35 m、当前位移确认
0.15 m、3 次连续命中、0.60 m 关联门、0.60 s coasting，global assignment 关闭。

| 场景 | 原始实车输出 | 同 bag 最终候选离线重跑 | 结论 |
| --- | --- | --- | --- |
| D01 | 38 confirmed，约 37.1/min；callback P95 69.545 ms | 0 confirmed，0/min；core reprocess P95 17.148 ms | **离线 PASS**；需短实时复测 |
| D02 | 6/6 横穿，确认 0.220-0.481 s | 6 条主要运动轨迹，确认 0.201-0.439 s；总计仍有 21 个 confirmed ID | 主要检测门 **PASS**；碎片/背景仍未关闭 |
| D03 | 方向 99.275%，0.5 s 内部一致性 P95 0.371 m | 9 条跨度 >=0.5 m 的主要轨迹仍存在，首次确认多数 0.20-0.46 s，1 条 0.56 s；core P95 17.835 ms | 无外部真值，仍为 **CONDITIONAL** |
| D05 | 52 confirmed ID，明显碎片化 | 27 confirmed ID；Hungarian 与 greedy 均为 27；cluster tolerance 0.25/0.30 m 分别为 24/26，但主要轨迹无一致改善 | **FAIL**；检测断续是主要根因，不能靠关联器或简单调参关闭 |
| D06 short | 5 次中 3 次保持 ID | 14 confirmed，至少 1 条跨度 >=0.5 m 的主要轨迹；core P95 15.684 ms | 只证明检测/生命周期未崩；身份保持需按视频/原输出复测 |
| D06 long retry | 长断流均使用新 ID | 7 confirmed，0.60 s 生命周期配置未放宽；core P95 16.944 ms | 长断流语义未回归 |
| D07 | 运动阶段 24 confirmed，约 37.8/min；callback P95 70.546 ms | 全包 17 confirmed，3.781/min；4 个触及运动段、14 个触及静止段（1 个跨阶段）；0 TF drop；core P95 23.883 ms | **FAIL**：仍高于 1/min 和 20 ms 门 |

D07 阶段分离表明剩余误检主要不随底盘运动出现，而更像环境中新出现的静止物体、地图残差
或簇 centroid 抖动；不能用放宽关联门限解释。已否决的配置包括：

- `min_displacement=0.50 m`：D07 可到 0.667/min，但 D02 多次确认变为 0.54-0.66 s；
- 连续 7 次命中：D07 可到 0.890/min，但约 9 Hz 输入下理论/实测确认超过 0.5 s；
- `min_speed=0.30 m/s` 试验：D07 仍为 1.335/min，D02 有 0.54/0.70 s 确认；该参数已从代码和 YAML 撤回；
- association gate 0.70/0.80 m、coasting 0.75 s：D05 仍为 25-27 confirmed，且会侵蚀 D06 长断流边界。

### 11.4 构建、测试与运行 smoke

- 隔离构建：PASS，`1 package finished`；结果目录
  `/tmp/rm27_hwsentry_tracker_fix_verified.brhlHZ`（容器内）。
- 隔离测试：PASS，18 tests / 0 error / 0 failure / 0 skipped。
- 提交前最终隔离复验：`/tmp/rm27_hwsentry_final.KDOkNp`（容器内），
  `rm_competition_interfaces`、`rm_dynamic_obstacle_tracking` 共 2 packages build PASS；
  tracker 18 tests / 0 error / 0 failure / 0 skipped。
- `git diff --check`：PASS。
- 隔离 `ROS_DOMAIN_ID=77`、D01 以 sim clock 5 倍速重放：节点输出
  `diagnostics=ok`、`authority=shadow_only`、`confirmed=0`；录包旧 predictions/markers/
  diagnostics 已重映射，不与新输出混淆。
- timeout 发送 SIGINT 后日志没有再出现 `rcl_shutdown already called` 或 traceback；回放首尾
  各有少量 TF 边界 drop，稳定段恢复为 `ok`。5 倍速首个 diagnostics latency 99.104 ms
  仅是压力 smoke，不作为冻结性能证据。

### 11.5 修复后仍未关闭的问题

#### P1-R1 D05 检测断续/ID 碎片化

- 路径与行号：`src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:220-277,507-635`；
  参数位于 `config/dynamic_obstacle_tracking_shadow.yaml:16-39`。
- 复现：用最终候选参数重放
  `log/hwsentry_d01_d07_20260824T1052CST/bags/D05`；统计 confirmed unique ID 得 27；
  开/关 `tracker.use_global_assignment` 结果相同。
- 需要的新增证据：固定机位视频或人工标注的每轮两人轨迹，用于区分检测丢失、错误拆分与真实
  ID switch。没有身份真值前不继续堆参数。

#### P1-R2 D07 静态/运动混合场景误检与离线 P95 仍超门

- 路径与行号：`core.py:196-297,457-635`；运行调用链
  `dynamic_obstacle_tracker_node.py:296-325`。
- 复现：用最终候选参数重放
  `log/hwsentry_d01_d07_20260824T1052CST/bags/D07`；得到 17 confirmed、3.781/min、
  core P95 23.883 ms。
- 下一步：先确认 D07 场景中 D06 泡沫板或人员是否仍在扫描范围；随后录制 60 秒严格空场静止
  和 60 秒严格空场低速运动短包。若新包仍复现，保存 detections/centroid 与 map distance 调试
  旁路，再设计“静止新障碍”和“真实慢速动态目标”的可分辨证据，不能直接提高位移/速度门限。

#### P2-R1 修复后实时资源门已复测但未全部通过

- 路径与行号：占用栅格索引 `core.py:107-193`；callback
  `dynamic_obstacle_tracker_node.py:252-325`。
- 复现：正式 1× `/local_scan` 下运行 D01_fix 61.315 s，读取 diagnostics `latency_ms`，
  同时记录 CPU/RSS；CPU 约 23%、RSS 74048 KiB，但 callback P95=60.259 ms。
- 影响：CPU 已明显改善，在线 P95 仍失败；TF 到达相位根因及 deskew 对照见 12.2、P1-R3。

## 12. 修复候选实车短复测与 deskew 对照（2026-08-24 下午）

### 12.1 运行边界与证据

- 仍使用 `fix/hwsentry-tracker-field-gates`；实车录制时的基线为
  `4198443f93e23d4010a6a5888007cacf203918ea + 修复差异`，该差异最终提交为 `cc254b4`。
- 地图为 `lab_dog_hole/20260820T040437Z`，定位链不启动 Nav2、串口、controller 或
  mission；检查期间 `/cmd_vel` 不存在。
- 新包位于
  `log/hwsentry_fix_retest_20260824T1600CST/bags/{D01_fix,D07_fix,D07_deskew}`；
  `D07_deskew` 额外保存 `/livox/left/pointcloud_filtered` 和 deskew diagnostics，约
  875 MiB。所有分析均只读，原 `db3`/`metadata.yaml` 未改写。
- 重新发布 initial pose 后，`map -> base_link` 可按 scan timestamp 查询；`/local_scan`
  恰有一个发布者。tracker node 只有 `/map`、`/local_scan`、TF subscriptions 和三个
  shadow 输出，没有控制或导航接口。

### 12.2 实时结果

| 场景 | 实测 | 判定 |
| --- | --- | --- |
| D01_fix 严格空场静止 | 61.315 s；523 scans、547 predictions/diagnostics；全部 diagnostics=`ok`；0 confirmed；CPU 约 23%，RSS 74048 KiB | **PASS** 误检门；资源较原始运行明显改善 |
| D01_fix callback | 在线 P50/P95/P99/max = 34.695/60.259/68.365/78.392 ms；同包纯 core 离线 P95=18.187 ms | **FAIL** 在线 P95<20 ms 门 |
| D07_fix baseline projection | 70.18 s；运动段 52.06 s；运动 confirmed 12 个，约 13.83/min；在线 P95=59.874 ms | **FAIL**；但实际并非低速：线速度 P95 1.09 m/s、角速度 P95 2.60 rad/s |
| D07_deskew | 209.28 s；运动段 74.35 s；运动 confirmed 8 个，约 6.46/min；在线 P95=28.156 ms；core P95=17.327 ms | **FAIL** 冻结门，但相对 baseline 运动误检约下降 53%，在线 callback P95 约下降 53% |

D01_fix 已把 P1-01 的严格空场静止误检从约 37.1/min 关闭为 0/min。该结果是最终候选的
实时实车证据，不再只是离线 A/B。

实时 TF 探针在 120 个预热后样本上得到：scan callback 到达时，仅 3/120 能立即查询到
timestamped TF；阻塞 lookup P50/P95/max 为 20.660/42.233/43.699 ms。它与 core P95
约 16-18 ms 相加后，准确解释 baseline 在线约 60 ms 的 P95。复现位置为
`src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:230-255`：
以 baseline `/local_scan` 运行 tracker，分别用 0 timeout 和 0.08 s timeout 查询
`map <- base_link`，统计 lookup wall time。

### 12.3 deskew 与底盘颤抖影响

对照仅用临时参数启用仓库已有严格逐点 SE(3) deskew，没有修改正式投影配置。实现与参数锚点：

- `src/rm_mid360_driver_bridge/src/pointcloud_to_laserscan_node.cpp:235-241,369-430`；
- `src/rm_mid360_driver_bridge/src/pointcloud_deskew.cpp:74-148`；
- `src/rm_relocalization_bridge/config/pointcloud_to_scan_2d_spin_robust_candidate.yaml:19-38`。

正式 D07_deskew 录制期间：deskew 始终 active，counter delta 为 10520 clouds received、
1884 output、8636 rate-throttled、0 dropped；timestamp bounds、interpolation、odom coverage、
TF、queue overflow、rollback 和 odom rejected 均为 0。deskew 单帧处理 P50/P95/max 为
0.838/1.289/5.641 ms。它将 scan 推迟到完整 odometry coverage 可用后再发布，因此也消除了
tracker 大部分 TF 阻塞等待。停止录包很久后曾有一帧 timestamp bounds invalid，被严格策略
按设计整帧丢弃；它不在正式 D07 bag 内，也没有 raw fallback。

操作者报告“低速下底盘持续颤抖”，bag 定量支持这一点：运动样本线速度 P50/P95/max 为
0.285/0.471/0.701 m/s，绝对角速度 P50/P95/max 为 0.534/1.571/2.169 rad/s；约 74 s
运动内发生 386 次有意义的角速度方向反转，角加速度 P95 为 22.576 rad/s²。在线新出现的
运动 confirmed 中，4 个发生在 `|yaw rate| >= 1 rad/s` 的仅 11.53 s 暴露区间，说明
底盘颤抖显著放大静态背景残差。因此该包有效地证明了压力工况问题，但不满足冻结文件要求的
“平稳低速 D07”前提，不能单独代表新车或正常控制器下的通过/失败率。

### 12.4 已评估但未落地的运动自适应门控

离线试验按 timestamped `map <- base_link` 相邻变换估计自身运动，并只在自身运动时把 cluster
centroid 静态容差从 0.35 m 自适应扩大。候选可把 D07_deskew 运动 confirmed 降到 1 个/
74.35 s（约 0.81/min），同时保留 D02 6/6、确认 0.201-0.440 s。它没有增加 odometry
topic 依赖。

该候选**没有写入正式 tracker**：D06_short 同包回归中，主要轨迹最大 span 从约 0.51 m
变为约 0.38 m；由于本轮没有固定机位身份视频，不能证明这是正确去除静态残差还是人员轨迹
被过度分段。根据冻结原则，不用单包调参掩盖未区分的检测问题。

#### P1-R3 baseline scan 与 timestamped TF 的到达相位导致在线等待

- 路径与行号：同步 lookup 位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py:249-255`；
  baseline 投影路径位于
  `src/rm_mid360_driver_bridge/src/pointcloud_to_laserscan_node.cpp:235-240,493-542`。
- 复现：baseline `/local_scan` 下执行上述 120 样本 TF 探针；立即成功 3/120，阻塞 P95
  42.233 ms；同时读取 tracker diagnostics 得 callback P95 约 60 ms。
- 影响：旧 NUC 正式实时门仍失败；更快 CPU 只能缩短 core 时间，不能消除 TF 到达相位等待。
- 后续：保留严格 deskew 为明确 opt-in 候选；在新车时间源、LIO 和动态云台 TF 确定后重新
  测试 end-to-end prediction age，而不是只优化 diagnostics 中的 callback wall time。

#### P1-R4 抖动底盘下运动背景误检仍超门

- 路径与行号：静态扣除/cluster filter 位于
  `src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py:196-297`，确认门位于
  `core.py:577-589`；输入投影见上节 deskew 锚点。
- 复现：只读分析
  `log/hwsentry_fix_retest_20260824T1600CST/bags/D07_deskew`；用最终候选参数得到运动段
  8 confirmed / 74.35 s，且 deskew diagnostics 全程无错误。
- 影响：D07 仍不能正式 PASS；在该输入用于动态 clearance 或 MPPI 之前必须保持
  shadow-only/冻结。
- 后续：先修复或更换底盘低速控制，在平稳低速下重录 D07；新车动态云台另做等价测试。
  若仍超门，再配固定视频真值评估 motion-aware static margin，不直接提交本轮离线候选。

## 总体状态

**FAIL**

构建、安装、消息接口、shadow-only 权限、长遮挡生命周期和静态墙门通过；修复后 D01 已由
实时实车复测关闭误检门，资源占用也明显改善。但 baseline 在线 callback P95、D05 ID 连续性
和 D07 抖动底盘运动误检仍未达到冻结门；deskew 明显改善 D07 和 TF 等待，却没有把运动误检
压到 1/min 以下。当前结果不能作为算法准入 PASS，也不能解冻 dynamic clearance COMMITTED
policy 或 dynamic MPPI critic。

### 代码合并处置

形式化算法准入状态保持 **FAIL**；但本分支的实际代码可以按 **CONDITIONAL MERGE** 处置：

- launch 的 `enabled` 默认仍为 `false`；
- 输出 authority 保持 `shadow_only`，没有 TF、costmap、plan、goal 或 `/cmd_vel` 权限；
- 只合并已经通过构建、单元测试和 D01 实车复测的 tracker 修复；
- 不合并本轮离线尝试的 motion-aware static margin，也不把 deskew 改成通用默认；
- 合并后继续冻结 dynamic clearance COMMITTED 和 MPPI dynamic critic，直到新车重新完成门测。

因此“允许默认关闭的 shadow 代码进入主线”和“允许动态结果参与控制”是两个独立决策；本报告
只允许前者。

## 下一步建议

1. 保留本轮所有 Linux bags；它们已经证明可以用于确定性 A/B。只按上一节边界合并默认关闭的
   tracker 修复，不启用任何下游消费链。
2. D01 严格空场已完成；不再针对冻结老车的低速颤抖修改导航代码。新车具备稳定控制后，用最终
   候选 + 明确 deskew A/B 重录 D07，同时记录 P95、CPU、RSS 和 end-to-end prediction age。
3. D05 下一次同步固定机位视频与地面标记真值；在明确检测断点之前不再切关联算法或扩大 gate。
4. D01/D07 过门后再完整重跑 D02-D06；在新车计算平台重复资源门，P95<20 ms 后才关闭性能项。
5. 新车动态云台仿真与实车分别验证 TF 外参、timestamp 对齐和静态背景假速度。
6. 只有需要真实清图效果时才重新采集同一 mapping session 的 ray sidecar；不得用最终 PCD
   伪造 origin/endpoint。
7. D01-D07 全部通过后，再规划动态通道准入接入和 Phase2 MPPI offline score；当前保持冻结。
