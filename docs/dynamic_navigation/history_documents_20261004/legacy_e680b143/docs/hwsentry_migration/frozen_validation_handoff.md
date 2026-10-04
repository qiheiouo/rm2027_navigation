# 开源选择性迁移冻结与验证交接

## 0. 2026-08-24 老车代码冻结与合并处置

HWSentry Phase 1 迁移和本轮旧车实车取证结束后，2026 旧车进入代码冻结状态。旧车只作为
算法鲁棒性、回放和对照验证平台，不再因为旧底盘尺寸、低速颤抖、狗洞或旧计算平台性能去
修改导航核心算法。现场复现的问题应保存 bag、参数、地图 revision 和报告，在新车或仿真中
验证通用性后再决定是否修改公共代码。

允许的后续范围：

- 修复阻塞安全停车、数据记录或可复现性的缺陷；
- 增加不改变既有导航默认行为的诊断、bag 采集和验证入口；
- 未来追击功能使用独立分支、显式启用和独立验收，不直接改写冻结的老车导航入口；
- 新车尺寸、动态云台、时间同步、狗洞变形和控制策略在新车/仿真分支实现并验收。

本轮 tracker 修复可以按“默认关闭、shadow-only、无下游控制权限”合并公共代码；这只是代码
归档和分支收敛，不表示 D01-D07 正式准入 PASS。D05、D07、callback latency 和新车动态云台
仍是解冻 dynamic-clearance COMMITTED policy 或 MPPI dynamic critic 前的硬门。禁止因为代码
已经合并就把 shadow predictions 接入 costmap、planner、controller、mission 或 `/cmd_vel`。

## 1. 冻结状态

自 2026-08-20 起，本轮 HWSentryNav26 及相关开源候选进入“代码冻结、等待现场
验证”状态。恢复时以远端 `feature/dynamic-clearance-shadow` 最新提交为统一基线，
先记录提交和工作区状态，不从旧功能分支重复构建或重新迁移代码。

冻结期间：

- 不实现 dynamic MPPI critic；
- 不把动态准入或 COMMITTED 合同接入 mission、狗洞执行器或底盘；
- 不新增 NDT、factor-graph LIO、ERASOR2、DDA 清图或地图编辑功能；
- 不替换 Nav2 planner、现有 localization owner、FAST-LIO 或 HWSentry 完整 FSM；
- 不以调参代替基线录包；任何参数改动都要保留改动前后的独立结果。

本批外部项目的冻结结论如下，后续不需要重新做同一轮源码筛选：

| 来源 | 已吸收内容 | 冻结结论 |
| --- | --- | --- |
| HWSentryNav26 | tracker、GICP 退化质量思想、语义路径、动态通道准入与隔离设计 | 等本文件的现场门，不迁完整 planner/FSM/FDDP |
| fast_lio_based_localization | `map -> odom` 分层与 NDT candidate 思路 | 现有 AMCL/GICP 同图 A/B 出现缺口才实现 NDT shadow |
| Odin1/MID360 标定与融合工具 | 仅保留为硬件专项参考 | 实际采用 Odin1 且现有标定/去畸变证据失败时再评估，不进入通用导航主链 |
| ERASOR2 | 动态残影清理的外部候选 | 日常可清场静态建图，当前不集成 |
| map_edit | 语义区域思想 | 语义 sidecar 已独立实现，不迁完整编辑器或重复 PGM 编辑功能 |

日常建图没有冻结。日常或临时场地先清场，在环境静止时使用现有 Phase 2I 流程建
匹配地图；比赛场地可使用标准图纸导出的同源 PCD/PGM，PGM 微调由操作者人工控制。
普通日常建图不要求开启 ray sidecar。详细建图流程见
[`phase2i_managed_mapping.md`](../phase2i_managed_mapping.md)。

## 2. 恢复验证所需条件

同时满足以下条件再开始，不满足时保持冻结：

1. 有一份与测试现场匹配、经过人工叠图检查的地图；GICP 还要求 bundle 内有 PCD；
2. `/map`、`/local_scan`、`/odometry/lio` 和 timestamped TF 稳定；
3. 有一名安全员、可用的人工遥控/急停和足够的录包空间；
4. D02-D06 至少有一名行人，D05 最好有两名行人；
5. D01-D06 禁止 Nav2 自动运动，D07 只允许安全员控制的低速运动；
6. 测试期间不运行任何消费 shadow 结果的 controller 或 mission 代码。

建图和动态验证应分开：先在无人移动时完成地图，再保持地图不变并引入动态目标。
旧地图与当前环境不匹配时不得据此判定 tracker、GICP 或动态准入通过/失败。

## 3. 软件基线与构建

```bash
cd /home/wpie/rm2027_navigation
git fetch origin
git switch feature/dynamic-clearance-shadow
git pull --ff-only
git rev-parse HEAD
git status --short --branch
git submodule status --recursive

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  rm_competition_interfaces \
  rm_map_tools \
  rm_dynamic_obstacle_tracking \
  rm_path_annotations \
  rm_dynamic_clearance \
  rm_gicp_relocalization \
  rm_relocalization_bridge \
  rm_navigation_integrity
source install/setup.bash

# --packages-up-to intentionally builds declared dependencies such as
# rm_mid360_driver_bridge and the locked livox_ros_driver2 submodule. Keep
# external-package lint outside the first-party acceptance result below.

RUN_DIR=$(mktemp -d /tmp/rm27_open_source_validation.XXXXXX)
mkdir -p "$RUN_DIR/bags"
colcon test --test-result-base "$RUN_DIR/test_results" --packages-select \
  rm_competition_interfaces \
  rm_map_tools \
  rm_dynamic_obstacle_tracking \
  rm_path_annotations \
  rm_dynamic_clearance \
  rm_gicp_relocalization \
  rm_relocalization_bridge \
  rm_navigation_integrity
colcon test-result --test-result-base "$RUN_DIR/test_results" --verbose
```

保存 `RUN_DIR`、HEAD、地图 manifest SHA256、参数文件 SHA256 和 submodule 状态。
必须是 0 failure/error 才进入实车；不要删除或清理验证前已经存在的本地修改。

## 4. 地图与传感器 preflight

日常场地先按 Phase 2I 流程在静态环境完成地图并保存新 revision。比赛 CAD 地图则
确认 PCD 与 PGM 来自同一坐标基准；人工修改 PGM 后保存为新 revision。两类地图都要：

1. 在 RViz 以 `map` 为 Fixed Frame 检查 `/map` 与 `/local_scan`；
2. 静止观察至少 60 秒，墙、柱、箱体等主要结构不能存在明显的整体平移或旋转偏差；
3. 检查 `map -> odom -> base_link -> lidar` 链连续，禁止用 latest-TF 回退掩盖时间问题；
4. 分别记录 `/local_scan` 频率、TF lookup 错误和 scan age；
5. GICP 使用的 manifest 必须是 `occupancy_with_pcd`，并记录 PCD/PGM/manifest 哈希。

基础检查命令：

```bash
ros2 topic hz /local_scan
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /tf -v
```

preflight 失败只修地图、TF 或输入链，不调整 tracker/GICP 阈值。

## 5. 动态 tracker D01-D07

按现有旧车 validation 入口启动地图、定位和 `/local_scan`，保持自动运动与两个串口模式
关闭；另开终端显式启动旁车 tracker：

```bash
ros2 launch rm_dynamic_obstacle_tracking \
  dynamic_obstacle_tracking_shadow.launch.py enabled:=true
```

每个场景单独录包，文件名使用 `D01` 至 `D07`：

```bash
ros2 bag record -o "$RUN_DIR/bags/D01" \
  /map /local_scan /odometry/lio \
  /tf /tf_static \
  /perception/dynamic_obstacles_shadow/markers \
  /perception/dynamic_obstacles_shadow/diagnostics \
  /perception/dynamic_obstacles_shadow/predictions
```

用地面标记、量尺和固定机位视频记录目标路线与时间。预测误差以目标实际到达位置为
基准，不用 RViz marker 的视觉印象代替测量。

| 场景 | 操作 | 主要记录 | provisional PASS 门 |
| --- | --- | --- | --- |
| D01 空场静止 | 机器人和环境静止 60 秒 | false track、CPU/RSS、callback latency | confirmed false track 不超过 1/min，且不持续超过 1 秒 |
| D02 单人横穿 | 单人沿标记直线横穿视野，至少 5 次 | 首次可见到 confirmed 的时间、方向、离场删除 | 0.5 秒内 confirmed，速度方向正确，无永久 ghost |
| D03 径向运动 | 沿雷达径向靠近和远离，各至少 5 次 | 速度符号及 0.5/1.0/1.5 秒预测误差 | 方向全部正确，0.5 秒预测误差不超过 0.4 m |
| D04 静态结构边缘 | 人沿墙、箱体等静态结构旁通过，至少 5 次 | 墙体误检、人员漏检 | 不产生持续静态墙 track；人员结果单独报告 |
| D05 双人交叉 | 两人交叉通过，至少 5 轮 | 每轮 ID switch 和错误合并/拆分 | 每轮不超过 1 次 ID switch；失败才评估 Hungarian |
| D06 遮挡与离场 | 分别做 0.3-0.5 秒短遮挡及超过 0.8 秒长遮挡 | coasting、重连、旧 ID 删除 | 短遮挡优先保持 ID；长断流不复用已过期 ID |
| D07 机器人运动 | 空场中人工低速平移、转向，再按既有安全流程决定是否自转 | 静态背景假速度、TF/drop、latency | confirmed false track 不超过 1/min，输入无持续 drop |

全部场景还要满足 callback P95 小于 20 ms，prediction 数量未越界，时间戳不 stale/future，
且 tracker 没有新增 TF、`/cmd_vel`、costmap、plan 或 goal 发布者。逐项原始指标必须写入
报告；“看起来正常”不能记为 PASS。现有更短的执行入口见
[`phase1_linux_validation.md`](phase1_linux_validation.md)。

## 6. GICP 退化质量门 A/B

GICP 和 AMCL 必须分时运行，禁止两个 backend 同时拥有 `map -> odom`。先做无运动
定位，不启用 Nav2 或真实串口：

```bash
ros2 launch rm_navigation_bringup \
  old_car_2026_gicp_relocalization.launch.py \
  enable_relocalization:=true \
  use_driver:=true \
  use_lio_backend:=true \
  selected_side:=left \
  use_rviz:=true \
  map_bundle_manifest:=/absolute/path/to/manifest.json \
  map_acceptance_policy:=allow_candidate
```

若使用已批准地图，将 policy 改为 `approved_only`。通过 RViz `/initialpose` 提供粗初值，
至少在三处几何条件不同的位置各重复三次：

1. 接近正确的初值；
2. 平移误差约 0.2、0.5、1.0 m；
3. yaw 误差约 10、30 度；
4. 走廊、大片平面或重复结构等预期退化位置；
5. 超过配置 jump gate 的错误初值，用于确认 fail-closed，而不是要求其收敛。

完成 GICP 后彻底停止该 launch，再用同一地图、位置、初值偏差和重复次数运行 AMCL
baseline，两个 backend 不得同场并行：

```bash
ros2 launch rm_navigation_bringup \
  old_car_2026_amcl_relocalization.launch.py \
  enable_relocalization:=true \
  use_driver:=true \
  use_lio_backend:=true \
  selected_side:=left \
  use_rviz:=true \
  map_bundle_manifest:=/absolute/path/to/manifest.json \
  map_acceptance_policy:=allow_candidate
```

AMCL 使用独立 bag 和 integrity JSONL，至少记录 `/localization/amcl_pose_raw`、
`/localization/amcl_backend_valid`、`/localization/global_pose`、`/localization/scan`、
`/odometry/lio`、TF 和 diagnostics。A/B 期间不得在两组之间修改地图、LIO、初值真值、
地图坐标或完整性阈值。

同步启动完整性旁车并保存 JSONL：

```bash
ros2 launch rm_navigation_integrity localization_integrity_shadow.launch.py \
  enabled:=true \
  profile:=old_car_2026 \
  metrics_output_path:="$RUN_DIR/gicp_integrity.metrics.jsonl"

ros2 bag record -o "$RUN_DIR/bags/GICP" \
  /initialpose /odometry/lio \
  /localization/gicp_pose_raw \
  /localization/gicp_registration_valid \
  /localization/gicp_backend_valid \
  /localization/gicp_fitness_score \
  /localization/gicp_overlap_ratio \
  /localization/gicp_min_information_eigenvalue \
  /localization/gicp_information_condition_number \
  /localization/global_pose /tf /tf_static /diagnostics
```

先记录 gate 开启的结果。如需校准，只复制参数文件到 `RUN_DIR` 后把
`quality_gate_enabled` 关闭做独立 shadow 对照，不修改仓库默认值。报告每次的收敛时间、
最终位置/航向误差、fitness、overlap、最小特征值、condition number、correction jump、
错误接受和错误拒绝。硬门是：过期、TF 缺失、明显错误 correction 和退化解不得发布为
有效 global pose；阈值只能根据多位置分布确定，不能只用一处成功样本调到通过。

## 7. 语义路径与动态通道准入

先复制 `semantic_regions.example.yaml`，填写当前地图的 map id、revision、manifest SHA256，
再添加测试用 `dog_hole_approach` 和 `committed_corridor` 多边形。先离线验证：

```bash
ros2 run rm_path_annotations validate_semantic_regions \
  --regions /absolute/path/to/regions.yaml \
  --expected-map-id FIELD_ID \
  --expected-map-revision REVISION \
  --expected-manifest-sha256 MANIFEST_SHA256
```

启动 tracker、语义注释和动态准入三个旁车节点：

```bash
ros2 launch rm_path_annotations semantic_path_annotation.launch.py \
  enabled:=true \
  regions_file:=/absolute/path/to/regions.yaml \
  expected_map_id:=FIELD_ID \
  expected_map_revision:=REVISION \
  expected_manifest_sha256:=MANIFEST_SHA256

ros2 launch rm_dynamic_clearance dynamic_clearance_shadow.launch.py enabled:=true
```

录制 `/plan`、`/navigation/annotated_path`、predictions、clearance report、TF 和诊断。
先用 mock/replay 覆盖以下合同，再做通道入口现场测试：

- 空的新鲜 prediction 为 `CLEAR`；
- confirmed 目标与 ETA 位置重叠为 `BLOCKED`；
- tentative、时域不足、stale/future、TF 缺失、revision/frame/schema 不一致均为
  `UNKNOWN`；
- coasting 目标使用额外 margin；
- 没有动态准入区域时为 `CLEAR/no_admission_required`；
- prediction `complete=false` 时不得输出进入许可。

现场依次做空通道、人员横穿、目标停留、短遮挡和预测时域不足，每项至少五轮。
测量实际车速、机器人等效半径、目标 footprint、prediction P95/P99 age 和覆盖整个通道
所需 horizon。阻挡场景出现任何错误 `CLEAR` 即失败；`UNKNOWN` 必须解释为不得进入。
shadow 节点必须始终没有 action、TF、costmap 或底盘控制权限。完整场景表见
[`dynamic_clearance_shadow_validation.md`](dynamic_clearance_shadow_validation.md)。

## 8. 报告与解冻门

每轮报告至少保存：

- 日期、操作者、机器人、传感器、HEAD、submodule 状态和全部本地差异；
- 地图 id/revision、manifest/PCD/PGM/regions/参数文件 SHA256；
- 启动命令、ROS domain、topic 频率、TF/时间错误和安全配置；
- 每个场景的原始 bag、视频、人工真值、逐项指标和 PASS/FAIL；
- CPU、RSS、P95/P99 latency、输入 drop、prediction age/horizon；
- 所有参数改动及改动前后的独立结果。

只按以下规则解冻：

1. D01-D07 全部通过，才允许单独开分支把动态准入报告和 COMMITTED policy 接入
   `main-new-car` 已有狗洞执行器；仍不得新增第二套 FSM；
2. tracker 通过且录包证明预测与资源预算可靠，才允许从 offline score 开始实现 dynamic
   MPPI critic；顺序必须是 offline、零权重加载、仿真、低速实车；
3. GICP 只有在同图同场景 A/B 中优于或补足 AMCL，才考虑改变定位策略；否则保持现状；
4. NDT、factor-graph LIO、MID360 驱动加固只由对应的实测缺口触发；
5. 静态日常建图无法得到可用地图时，才重新评估 ERASOR2、DDA 或其他清图开发。

任一验证失败不会自动扩大迁移范围。先保留 bag 和失败指标，只对被证据定位的最小模块
解冻，修复后完整重跑受影响场景。
