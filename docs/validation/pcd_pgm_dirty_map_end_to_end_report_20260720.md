# PCD/PGM 地图脏点诊断、收敛与交付总结报告

日期：2026-07-20

仓库：`/home/wpie/rm2027_navigation`

分支：`fix/field-debug-linux-consolidation`

当前 HEAD：`ed6c7ddb7110cdca99c37120461fcfa3f30e5665`

对应提交：`ed6c7dd 功能：新增地图质量诊断与回放分析`

## 1. 报告结论

本轮工作已从最初的“地图看起来很脏”，收敛为一个有明确输入链、可回放
证据、自动指标、失败门和回滚路径的工程问题。

最终结论为：

1. PGM 不是由保存后的 PCD 直接转换而来；PCD 和 PGM 是两条并行资产链。
2. PGM 行翻转、YAML origin/阈值、灰度语义和 Nav2 `map_server` 没有凭空制造
   脏点；脏点已经存在于 `/mapping/projected_map` 上游。
3. 受控临时障碍实验确认的主要残留机制是：人员/物体进入静态建图区时
   产生少量高层瞬态 endpoint，它们在 OctoMap 三维树中持久占用；后续 free ray
   因空间错开未命中同一 voxel，二维高度坍缩又把它们投影到 PGM。
4. 低层立方体本体实际已被 free rays 清除。这不是 Nav2 导航期间动态障碍
   clearing 失效，也不否定早期导航清障测试。
5. hit/miss 参数、endpoint-only 门、精确 min2、邻域门和投影概率门都未能
   同时满足“脏点减少”和“真实障碍保护”。最终原生派生 exact-min2 原型在
   loop r1/r2 的 0.10 m 保护召回率仅为 96.64%/96.11%，低于 99% 硬门，
   因此被否决并从产品链彻底回滚。
6. 没有为了让地图“看起来更干净”而提交一个过度过滤的生产算法。当前
   产品侧最小可用收敛方案是静态建图过程中严格使用 `/mapping/stop`、
   `/mapping/start` 和 `/mapping/recording`，跳过人员进入、搬运物体和移除物体的
   transition 段，恢复后重新覆盖受影响视角。
7. 用户决定先使用视觉效果更可接受的 fresh03 冻结地图，后期再替换。该
   地图已通过严格解码、`map_server` 逐格一致、AMCL/RViz 对齐、静态规划、
   人工低速移动和一次限速短距离 Nav2 验收。它仍是 `candidate` /
   `candidate_for_field_validation`，不是 `approved`。

所以，如果以后来缩减的目标“尽快结束建图环节，交付一张可受控使用、可回滚的
地图并进入后续功能”评估，本轮工作已完成。如果以最初的理想目标“交付一个
通用、自动、通过 99% 真实障碍保护门的 PGM 去污生产后端”评估，该部分没有
完成，而是因证据证明存在过滤真障碍风险而主动停止上线。

## 2. 最初计划与安全边界

最初执行计划分为八个阶段：

1. 只读冻结工作区、版本、环境和 baseline；
2. 补齐两段以上可重放 bag 与现场真值；
3. 为 raw、filtered、registered、稳定 PCD 和 PGM 生成分层投影与机器可读证据；
4. 受控区分原始点云污染、建图累计污染、投影算法和参数问题；
5. 只在两段独立 bag 和 ROI 真值完整后执行受约束单变量搜索；
6. 仅选择一个由证据唯一支持的最小修复；
7. 验证 PGM/YAML/坐标/origin/`map_server`/无运动规划；
8. 最后才进入 RViz 和实车验收，并且只能由人工 reviewer 批准。

全程遵守以下约束：

- 不覆盖任何 PCD、PGM、YAML、bundle 或 deployment manifest；
- 所有诊断输出使用独立 run ID，大型 bag 和日志保留在仓库外证据目录；
- 不假设主机一定有 ROS、PCL、numpy 或完整测试数据；
- 不用 PCD support ratio 单独删点或批准地图；
- 不把 PCD 链的 latest-TF fallback 直接归因为 PGM 问题；
- 不从 final `octomap_full` 反推精确 hit/miss 次数，精确次数必须标记为回放推导；
- 不为通过数字门而降低真障碍召回标准；
- 不触碰用户原有 Livox 子模块修改和 `core.205`。

## 3. 环境、基线和真实调用链

### 3.1 环境核对

- 仓库中未发现项目级 `AGENTS.md`；
- 主机当时没有可直接使用的 ROS/colcon/numpy 工具链；
- `rm2027_navigation_humble` 容器提供 ROS 2 Humble、colcon、numpy、OpenCV、
  `octomap_server 2.3.1` 和 `nav2_map_server 1.1.20`；
- 诊断脚本因此不依赖 `pcl_viewer` 等 PCL 命令行程序。

### 3.2 真实 PGM 链

```text
/livox/left/pointcloud
  -> old_car_left_pointcloud_filter
  -> /livox/left/pointcloud_filtered
  -> mapping_pointcloud_sampler (<=5 Hz)
  -> /mapping/sensor_cloud
  -> octomap_server
  -> /mapping/projected_map
  -> mapping_session_node
  -> occupancy_values_to_pgm() (OccupancyGrid Y 行翻转)
  -> PGM + YAML
```

### 3.3 并行 PCD 链

```text
/lio/cloud_registered_transformed
  -> mapping_session_node (0.20 s 采样)
  -> 5 cm 三维体素累计
  -> 每帧同体素只计一次
  -> min_observations=2
  -> ASCII XYZ PCD
```

两条链只共享建图时段，不共享输入、ray tracing、稳定观测门和持久化规则。
所以“PGM 格子附近没有稳定 PCD 点”只能作为可疑特征，不能等价为虚假障碍。

### 3.4 冻结 baseline

```text
map_id/revision: old_car_clean_20260715_field01/20260715T025927Z
PGM: 333 x 398, 0.05000000074505806 m/cell
origin: [-6.200000190734864, -8.450000190734864, 0.0]
occupied/free/unknown: 15310 / 48304 / 68920
occupied components: 725
components <=25 cells: 686, totaling 1686 cells
PCD points: 12750
PCD points in 0.10-1.80 m: 5776
occupied cells without <=0.10 m stable-PCD support: 7095
```

```text
PCD:      f0232b2f2c1560e901af03bbec6b32b6d80f5d94dc1a226178220ddb237a5b47
PGM:      af476fe72d2eb790ce3ec67c17618d1b510761079aa1c7ff2baa4dd99478a155
YAML:     f065edea59d3d682a89ca5564100b91fdfc389920e359b5efc90e92bf1321fa4
manifest: 142f93ddfa44fef6c61c183ccc7f9a78345add99c65e2931f2cb90a19e8958b7
```

## 4. 执行时间线

### 4.1 诊断基础设施

首先新增只读地图质量工具，将“看起来脏”转换为可重复指标：

- `analyze_map_quality`：严格解码 ASCII PCD、P2/P5 PGM 和 YAML，生成 8 个 Z 层、
  密度图、连通域、支持距离、ROI/障碍/墙/地标指标与 JSON/CSV/PNG；
- `sweep_map_projection plan`：生成 baseline + 单变量候选，不做全排列，证据不足时
  必须保持 blocked；
- `sweep_map_projection rank`：先执行关键障碍 100%、总障碍 99%、墙缝、地标、
  known-free 和双数据集复验硬门，再排序脏点指标；
- `verify_map_server`：把 live `/map` 的 frame、尺寸、resolution、origin 四元数和每一个
  OccupancyGrid cell 与离线解码逐项比较。

分析器精确复现了 baseline 的所有主要数字。工具只允许在
`/tmp/rm27_pcd_pgm_diag/` 下创建新 run 目录，拒绝覆盖 bundle，拒绝静默误读
binary/binary-compressed PCD。

### 4.2 现场数据采集

按照“需要移车时必须明确告知”的规则，与操作员分步完成：

- 三个静止位置 `static_p1/p2/p3`，每段约 29--30 秒；
- 两段独立近似闭环 `loop_r1/r2`，约 119.5 秒和 94.7 秒；
- 一个边长约 20--30 cm 的泡沫立方体五阶段实验：
  `baseline_clean`、`place_transition`、`object_present`、
  `remove_transition`、`post_remove`；
- 后续清场重建的 fresh03：197.873 秒、66,227 条消息、十个必需话题全部
  非零、`/mapping/sensor_cloud` 760 帧；
- known-free 四点 ROI，墙和柜子两段，小箱子/机器人/大箱子场景障碍，
  以及三个固定桌角 F1/F2/F3。

完整 bag 包含 raw、filtered、sensor cloud、registered cloud、odometry、TF、
projected map、full OctoMap 和 recording 状态。所有大型 bag 在删除容器临时
重复前先做主机/容器哈希对比。现在完整证据归档约 14 GiB、1767 个文件。

### 4.3 四类根因的受控区分

| 假设 | 最终证据结论 |
|---|---|
| PCD/原始点云污染 | filtered/sampled endpoint 中确实存在少量瞬态空间点，尤其在人员/物体过渡段；但证据不支持把所有脏点简化为雷达硬件原始噪声或仅用 self-filter 解决。 |
| 建图累计污染 | 已确认。单帧/少帧 endpoint 可在三维树中长时保留，后续射线因 voxel 错开未能清除。 |
| 投影算法问题 | 已确认为放大因素。不同 Z 的三维残留被折叠到同一二维列，使 PGM 显示附近脏点。 |
| 参数问题 | 普通 hit/miss/高度/邻域门只能部分缓解；能明显减少脏点的组合又会损失受保护结构，没有通过生产硬门。 |

辅助结论：PCD 累计中的 latest-TF fallback 是 PCD 质量风险，但它不能被直接
当作 PGM 脏点根因。

### 4.4 受控立方体的关键证据

`object_present` 中提取到 6 格低障碍证据块，每格有 13--41 个独立帧支持。
表面上它们在 `post_remove` 后仍有 4/6 二维格 occupied；但 full OctoMap 逐列检查
证明，六个低层立方体 voxel 已获得大量 free rays 并全部清除。真正残留是
同列/邻列约 `z=0.775--1.225 m` 的八个高层瞬态叶节，通常只有一帧命中。

跳过 `place_transition` 和 `remove_transition`，只向标准 OctoMap 回放三个稳定段后：

```text
baseline_clean: 6/6 exact free
object_present: 5/6 exact occupied, 6/6 within 0.05 m
post_remove:    6/6 exact free, 0/6 occupied within 0.10 m
```

这是最终选择“暂停静态建图、跳过人员/物体过渡段”作为最小产品修复的
直接实验依据。

### 4.5 算法原型筛选与否决

先后筛选了：

- hit/miss 参数变体；
- endpoint-only min-frame gate；
- 保留 free rays 的 ray-aware min-frame 方案；
- 精确 3D min2；
- 跨帧空间邻域 min2/min3/min4；
- 二维竖直列多帧门；
- `occupancy probability >= 0.71` 投影门；
- 单帧 radius outlier 邻域门；
- 直接派生自原生 `octomap_server` 的投影期 exact-min2 C++ 原型。

最后一个原型完整处理 fresh03 760/760 帧：

```text
native standard occupied: 15674
exact-min2 occupied:      11291
occupied reduction:       4383 (27.96%)
known coverage change:    -0.2902 percentage points
known-free ROI:           593/593 free
```

但在两段独立闭环的双遍保护代理上：

| 指标 | loop r1 | loop r2 |
|---|---:|---:|
| protected proxy cells | 2,469 | 2,752 |
| 0.10 m recall | **96.64%** | **96.11%** |
| 要求 | >=99% | >=99% |

它还在 r1 把小连通域像素从 606 增加到 728，显示结构碎片化风险。因此结论
不是“数字改善，所以上线”，而是“清污有效，但可能删掉真障碍，所以否决”。
原型、launch、reset、package 和 build/install 残留都已从产品链撤回，源码与结果只保存
在外部否决归档。

### 4.6 fresh03 冻结、文件验证与现场验收

应用层最后没有恢复被否决的算法，而是把历史已生成的 fresh03 终态格栅单独
封装为 occupancy-only immutable candidate：

```text
map_id/revision: old_car_fresh03_field_validation/20260720T012237Z
status: candidate / candidate_for_field_validation
approved: false
size: 381 x 397
resolution: 0.05000000074505806 m/cell
origin: [-4.599999809265137, -8.15, 0.0]
occupied/free/unknown: 11291 / 52975 / 86991
```

```text
PGM:      3c45f3cd97b6d82f5c83ef9d8a98e32b4ec54385fc9ace6180aa03aab68d8e8a
YAML:     2934040273915b22d70c0a7b3eb1ee15e37fc35521d54a3ea159888637f60d41
manifest: fa3144f85a9a0c2cfac1898de4e3e424403c8423c730831c1c69aae1819e9909
```

已完成：

- P5 payload、`0/205/254`、trinary/negate/阈值、Y 行翻转、resolution、origin 和哈希；
- live `/map` 共 151,257 格与离线解码完全一致，`data_mismatches=0`；
- home、P1、P2 和 known-free 中心点的无运动 `ComputePathToPose`；
- 左 MID360、FAST-LIO、fresh03、AMCL、Nav2、真实串口、裁判 mock、mission hold 集成启动；
- 用户发布 `2D Pose Estimate` 后，定位有效，用户目视确认地图/扫描/方向对齐；
- 用户人工低速移动并停车后，对齐仍正常；
- 用户明确开启自动模式后，执行一次 0.15 m/s 限速短距离 Nav2 目标，
  action `SUCCEEDED`、recovery 0，用户确认车辆正常前进并停车；
- 结束后任务仍为 `disabled/hold`，3 秒内无 `/cmd_vel`，定位仍有效，测试
  限速已清除。

已知限制：AMCL 曾偶发“90% 以上观测不在地图”警告，但用户在静止和移动后都
目视确认对齐正常。该结果支持受控临时使用，不构成正式批准。

### 4.7 代码收敛与 Gitee 交付

工作区中曾存在大量临时回放、原型、build/install 和证据输出。它们不等价于
“大量产品代码修改”。回滚和归档后，正式提交收敛为 14 个文件、3131 行新增，
包含三个 CLI、核心诊断算法、测试、标签模板和文档；不包含原型后端和地图资产。

验证结果：

```text
rm_map_tools build: passed
rm_map_tools tests: 39 passed, 0 failed, 0 skipped
Python compileall: passed
ament_flake8: passed
package.xml xmllint: passed
CLI help smoke: passed
```

提交已上传 Gitee：

```text
branch: fix/field-debug-linux-consolidation
commit: ed6c7ddb7110cdca99c37120461fcfa3f30e5665
subject: 功能：新增地图质量诊断与回放分析
local/remote divergence after push: 0 / 0
```

Gitee 中只有源码、测试和文档；fresh03 PGM/YAML/bundle、bag、日志和大型证据没有上传。

## 5. 交付物清单

### 5.1 已提交工具

- `src/rm_map_tools/rm_map_tools/map_quality.py`
- `src/rm_map_tools/rm_map_tools/analyze_map_quality.py`
- `src/rm_map_tools/rm_map_tools/sweep_map_projection.py`
- `src/rm_map_tools/rm_map_tools/verify_map_server.py`
- `src/rm_map_tools/config/map_quality_labels.example.yaml`
- `src/rm_map_tools/test/test_map_quality.py`
- `src/rm_map_tools/test/test_mapping_pause_contract.py`
- `src/rm_map_tools/setup.py`、`package.xml`、`README.md` 对应入口和依赖。

### 5.2 已提交文档

- `docs/phase2i_managed_mapping.md`：静态建图暂停/恢复 SOP 与诊断入口；
- `docs/validation/pcd_pgm_map_quality_implementation_20260717.md`：实施、数据、
  原型筛选和回滚全记录；
- `docs/validation/old_car_fresh03_field_validation_20260720.md`：fresh03 哈希、
  现场验收、自动模式规则和启动命令；
- `docs/validation/old_car_field_debug_20260717_evidence.md`：外部证据索引。

### 5.3 未入 Git 的必要资产

- baseline：`artifacts/maps/old_car_clean_20260715_field01/20260715T025927Z/`；
- fresh03：`artifacts/maps/old_car_fresh03_field_validation/20260720T012237Z/`；
- 现场运行证据：`artifacts/maps/runtime/fresh03_field_validation_20260720/`；
- 完整外部归档：
  `/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/`；
- 候选冻结报告：
  `diag/20260720_fresh03_candidate_freeze/FREEZE_REPORT_20260720.md`；
- 被否决原型：`diag/20260720_native_min2_rejected/`。

外部归档当前约 14 GiB、1767 个文件。它们是项目证据，不是应删除的脏残留。

## 6. 当前运行和部署状态

### 6.1 当前临时地图

fresh03 由用户决定先行使用，后期再替换。启动必须使用：

```text
relocalization_backend:=amcl_2d
map_acceptance_policy:=allow_candidate
map_bundle_manifest:=/data/rm27_maps/old_car_fresh03_field_validation/20260720T012237Z/old_car_fresh03_field_validation.bundle.yaml
```

该地图不含共同生成的 PCD，所以不能用它启动 GICP 三维重定位。

### 6.2 当前实验室启动配置边界

已证实可同时启动：真实串口、左 MID360、FAST-LIO、fresh03、AMCL、Nav2、
referee mock、readiness 和 mission node。但当前启动档是现场调试档：

- `use_referee_mock:=true`；
- `mission_startup_enabled:=false`；
- mission 启动后保持 `disabled/hold`；
- 它不是真实裁判数据驱动的最终比赛自动启动档。

完整开机命令已记录在 `old_car_fresh03_field_validation_20260720.md`。

### 6.3 必须遵守的遥控器规则

用户明确要求：以后每次准备发送任何可能驱动车辆的 Nav2 目标前，都必须先
明确告知用户开启自动模式，说明目标、移动方式、停车条件和回滚，并等待
用户确认“自动模式已开启”和“区域已清空”。

导航结束后必须检查 action 结果和零速；不再需要自主运动时，要求用户切回手动
模式。

## 7. 回滚和不可破坏项

### 7.1 地图回滚

- fresh03 从未替换正式 deployment manifest；
- 不启动 candidate 时，无需修改任何正式文件；
- 候选 map_server 已启动时，先停止当前 launch，再使用冻结归档中的
  `ROLLBACK_OLD_MAP.sh`；
- 旧 baseline 哈希已冻结，回滚时不重新导出、不覆盖原 revision。

### 7.2 必须保留

- `src/livox_ros_driver2_humble` 中的用户修改：
  `livox_lidar_callback.cpp`、`pub_handler.cpp`；
- 未跟踪 `core.205`，逻辑大小 480,518,144 bytes；
- 完整 bag、原型否决证据、fresh03 source 和现场运行日志。

本轮没有清理、暂存、提交或修改上述用户内容。

## 8. 完成度评估

| 范围 | 状态 | 说明 |
|---|---|---|
| 调用链确认 | 完成 | 已明确 PGM 与 PCD 两条并行链。 |
| 基线与自动诊断 | 完成 | 数字、哈希、分层图、连通域和坐标合同可重现。 |
| 完整现场数据 | 完成 | 三个静止位置、两段 loop、立方体五阶段和 fresh03 已归档。 |
| 四类根因区分 | 完成 | 主因为瞬态 endpoint + 3D 持久化 + 2D 高度坍缩。 |
| 参数/原型筛选 | 完成 | 已执行且有明确否决依据。 |
| 通用生产去污后端 | **未交付** | 候选未过 99% 真障碍保护门，已主动回滚，不以过滤安全性换视觉清洁度。 |
| 静态建图最小治理 SOP | 完成 | pause transition + 重新覆盖已通过受控立方体回放。 |
| fresh03 冻结与文件链 | 完成 | immutable candidate、hash、map_server 和无运动规划已通过。 |
| fresh03 有限实车验收 | 完成 | 静态/手动移动对齐和一次限速短导航已通过。 |
| 正式地图批准 | 未执行 | 用户决定临时使用 candidate，未将它标记为 approved。 |
| 建图阶段收尾 | **完成** | 已有可用图、回滚、证据、SOP 和启动命令，停止新算法研发。 |
| 完整比赛功能 | 本报告范围外 | 真实裁判串口转发、最终比赛点位和整场 soak 等仍需在后续阶段完成。 |

## 9. 后续建议

### 9.1 近期不再做的事

- 不继续新增参数扫描、证据门变体、投影器、代理真值和连通域删点算法；
- 不恢复被否决的 `rm_octomap_evidence` 或 exact-min2 产品链；
- 不用 YAML threshold、inflation 或按小连通域删除的方式掩盖脏点；
- 不重复已通过的普通直线/侧向/转角基础导航测试。

### 9.2 后期换图或人工修图

后期如使用图像工具人工清理：

1. 复制为新 revision，不覆盖 fresh03；
2. 不缩放、不旋转、不裁剪画布，不改 resolution 和 YAML origin；
3. 严格保持 PGM `occupied=0`、`unknown=205`、`free=254`，关闭抗锯齿和有损压缩；
4. 只删除有现场证据支持的虚假占用，不删墙、桌柜、低矮障碍和 unknown 边界；
5. 重新计算哈希并生成新 manifest，重跑 bundle 校验、`map_server` 逐格核对、
   无运动规划和一次低速实车验收。

### 9.3 进入后续比赛功能

建图环节不再是近期主阻塞。后续优先顺序应转为：

```text
获取下位机裁判转发协议和真实样帧
-> 在现有串口链增加非阻塞接收与 parser
-> 无运动验证 referee state/freshness
-> 复核最终 home/patrol 比赛点位
-> 真实 referee 驱动 mission 的无运动与低速验收
-> 冷启动、断线恢复和整场时长 soak
```

更完整的旧车导航后续路线见
`docs/validation/old_car_navigation_status_and_roadmap_20260720.md`。

## 10. 证据索引

- 详细实施记录：
  `docs/validation/pcd_pgm_map_quality_implementation_20260717.md`
- fresh03 现场验收与启动命令：
  `docs/validation/old_car_fresh03_field_validation_20260720.md`
- 建图 SOP：`docs/phase2i_managed_mapping.md`
- 现场证据索引：
  `docs/validation/old_car_field_debug_20260717_evidence.md`
- 断网/关机检查点：
  `/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/diag/CHECKPOINT_20260718.md`
- fresh03 冻结报告：
  `/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/diag/20260720_fresh03_candidate_freeze/FREEZE_REPORT_20260720.md`
- 被否决原型归档：
  `/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/diag/20260720_native_min2_rejected/`
- 现场运行报告：
  `artifacts/maps/runtime/fresh03_field_validation_20260720/FIELD_VALIDATION_RECORD_20260720.md`

---

本报告是总结文档，未修改产品代码、参数、地图资产或 deployment manifest。
