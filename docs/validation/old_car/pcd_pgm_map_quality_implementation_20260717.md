# PCD/PGM 地图脏点诊断工具实施记录

日期：2026-07-17（2026-07-18 断线后续验）

分支：`fix/field-debug-linux-consolidation`

基线 HEAD：`e1a822a85262f06c0c218442ea1a41fdfd87274a`

## 1. 实施边界与结论

本次实施完成了不改变建图结果的离线诊断、证据门、受约束 sweep 计划、
PGM/YAML 坐标验证和运行中 `map_server` 逐格核对。没有修改
`mapping_octomap.yaml`、点云过滤参数、mapping launch、bundle schema 或任何
PCD/PGM/YAML/manifest，也没有批准或替换 candidate。

后续已补齐三个静止位置、两段独立近似闭环和五阶段临时立方体 bag，并完成确定性
ray-aware replay。当前主因已收敛为：少量高层瞬态 endpoint 在 OctoMap 三维树中
持续占用，后续 free rays 因空间错开无法命中这些 voxel，二维高度坍缩再把它们投影
为 PGM 脏点。低层立方体本体实际已经清除；exporter、YAML 和 map_server 不是脏点
来源。简单 hit/miss、endpoint-only gate、精确 min2、跨帧邻域候选，以及后续直接
派生自原生 `octomap_server` 的投影期 exact-min2 原型，均未同时通过清污与 99%
双遍保护门，因此仍没有提交生产修复参数或后端。fresh03 已补齐 known-free、墙/柜、
场景障碍和三个固定地标标签；原生派生原型在该标签集上没有损失，但在 loop r1/r2
的双遍保护代理上仅达到 96.64%/96.11%，已按回滚规则从产品链撤销。任何当前候选
都不能生成或批准地图资产。

## 2. 已复核的实际资产链

PGM 不是由保存后的 PCD 转换而来。当前二维链为：

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

并行 PCD 链为：

```text
/lio/cloud_registered_transformed
  -> mapping_session_node (0.20 s 采样)
  -> 5 cm 三维体素累计
  -> 每帧同体素只计一次
  -> min_observations=2
  -> ASCII XYZ PCD
```

两条链只共享建图时段，不共享输入、投影、ray tracing 或持久化规则。保存后的
PCD 没有 sensor origin，工具明确禁止把它单独解释为 free/unknown 真值。

`mapping_session` 的 `latest_transform_fallback_clouds` 只直接描述 registered
cloud 到 PCD 累计链。除非另行证明 OctoMap 插入 sensor cloud 时也使用了错误或
latest TF，否则该计数不能解释 PGM 脏点。类似地，`octomap_full` 只保存最终
posterior node 状态；精确 endpoint hit 和 free-ray 次数必须标为 bag replay
推导值，不能从 final OctoMap 反推。

## 3. 新增工具与接口

### `analyze_map_quality`

输入 immutable `occupancy_with_pcd` bundle，输出：

- P2/P5 PGM、trinary YAML 和 ASCII scalar XYZ PCD 的严格解码；
- occupied/free/unknown 直方图、8 邻接连通域和 4 边周长；
- 八个固定 Z 层的点数、图内点数、栅格数和 log-density PNG；
- 0.05/0.10/0.25 m PCD 支持统计和 support overlay；
- 上下半区、四象限密度和显式 world/grid/PGM 坐标合同；
- 可选 known-free、关键障碍、墙体连续性和地标验收；
- 可选 rosbag metadata 话题覆盖检查。

输出必须是 `/tmp/rm27_pcd_pgm_diag/` 下不存在的新目录。CLI 拒绝写入 bundle
目录、拒绝覆盖已有输出、拒绝 binary/binary-compressed PCD，不会生成删除 mask。

### `sweep_map_projection`

`plan` 生成 baseline 加单变量候选，不做全排列。当前生成 23 个候选，固定：

- resolution 0.05 m；
- `incremental_2D_projection=false`；
- 禁止改 YAML threshold“清图”；
- 禁止按连通域面积删障碍；
- 禁止覆盖 baseline。

`rank` 先执行过度过滤硬门，再比较噪声指标。缺少以下任何证据都会将候选标为
不可选：有效 known-free/障碍/墙/三个地标标注、关键低障碍 100% recall、总体
障碍 99% recall、逐帧时序证据、两段独立数据集复验。

### `verify_map_server`

订阅 transient-local `/map`，读取 `/map_server/get_state`，并把 live
`OccupancyGrid` 的 frame、尺寸、resolution、origin position/quaternion 和每一个
cell 与 bundle 的 PGM/YAML 离线解码逐项比较。它只验证地图加载合同，不批准地图。

## 4. 冻结 baseline 与复现结果

主 baseline：

`artifacts/maps/old_car_clean_20260715_field01/20260715T025927Z/`

哈希在实施前、map_server 验证后和断线续验时均保持：

```text
PCD      f0232b2f2c1560e901af03bbec6b32b6d80f5d94dc1a226178220ddb237a5b47
PGM      af476fe72d2eb790ce3ec67c17618d1b510761079aa1c7ff2baa4dd99478a155
YAML     f065edea59d3d682a89ca5564100b91fdfc389920e359b5efc90e92bf1321fa4
manifest 142f93ddfa44fef6c61c183ccc7f9a78345add99c65e2931f2cb90a19e8958b7
```

新分析器精确复现：

```text
PGM: 333 x 398, 0.05000000074505806 m/cell
origin: [-6.200000190734864, -8.450000190734864, 0.0]
occupied/free/unknown: 15310 / 48304 / 68920
occupied components: 725
components <=25 cells: 686, totaling 1686 cells
PCD points: 12750
PCD points in 0.10-1.80 m: 5776
occupied cells unsupported within 0.10 m: 7095
fully unsupported occupied components: 633
```

上述支持统计只表示 PCD/PGM 合同差异，不能单独证明这些格子是虚假障碍。

## 5. 自动测试与 ROS 集成结果

新增测试覆盖：

- 非对称 3x2、PGM 行翻转和非零 origin yaw；
- 半开 Z 层、密度投影、8 邻接组件和欧氏距离；
- binary PCD 明确拒绝；
- known-free、关键低障碍、墙体断口和地标；
- rosbag 必需话题缺失；
- immutable 输出和 bundle 写入保护；
- map_server 逐格对比；
- sweep 单变量约束和过度过滤门；
- 当前 field baseline 精确回归。

容器内结果：`rm_map_tools` build 成功，39 tests、0 errors、0 failures，新增
Python 文件 `ament_flake8` 无问题。

独立 `map_deployment.launch.py` 使用 `allow_candidate` 加载 baseline：

```text
map_server lifecycle: active
live/offline cell mismatches: 0 / 132534
live occupied/free/unknown: 15310 / 48304 / 68920
other cell values: 0
```

随后只启动 `planner_server`、global static layer、现有 0.65 m inflation 和临时
静态 `map -> base_link`，没有启动 controller、serial、driver 或车辆运动。以现场
记录的 home `(0.441, 0.208)` 为 start，home、patrol P1 `(0.409, 0.752)` 和
patrol P2 `(0.811, 0.965)` 三个 `ComputePathToPose` 均 `SUCCEEDED`。这只证明
当前文件和旧车 footprint/config 能完成无运动静态规划，不等于现场地图质量通过。

## 6. 证据路径

证据已从主机 `/tmp` 原子移动到持久化归档：

`/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/`

以下原 `/tmp/rm27_pcd_pgm_diag/` 路径现在都位于归档的 `diag/` 子目录；现场 bag
位于 `field_capture/`，临时 C++ 原型分别位于 `octomap_column_inspector_ws/` 和
`spatial_evidence_ws/`：

- baseline 分析：
  `diag/20260718_final_baseline_e1a822a/`
- 受约束 sweep 计划：
  `diag/20260718_final_sweep_no_complete_bag/`
- map_server 逐格核对：
  `diag/20260717_map_server_reload_e1a822a/`
- planner 无运动 smoke：
  `diag/20260717_planner_smoke_e1a822a/`

这些证据不是部署资产，不提交 Git；11 个 bag 数据库、参数 dump、JSON/CSV、日志和
可视化均保留在上述外部目录，并记录 SHA-256。

## 7. 分阶段状态与下一步门槛

| 阶段 | 当前状态 | 继续条件 |
|---|---|---|
| 0 冻结 baseline | 完成 | 修改前后继续核对哈希和 dirty 文件 |
| 1 补齐 replay 数据 | 完成 | 三个静止位置、两段 loop 和临时障碍五阶段 bag 已归档 |
| 2 分层诊断 | 完成 | fresh03 人工标签和 r1/r2 双遍保护代理均已加入验收 |
| 3 四类根因区分 | 主因已确认 | 高层瞬态 endpoint + 3D 持久化 + 2D 高度坍缩 |
| 4 参数/原型搜索 | 已执行，无通过候选 | 原生派生 exact-min2 双遍召回仅 96.64%/96.11% |
| 5 最小修复 | 回滚到采集工作流 | 被否决后端已撤销；保留 `/mapping/stop`/`start` 过渡段规则 |
| 6 文件/map_server | 完成 | 新 candidate 生成后重复同一验证 |
| 7 RViz/实车 | 阻塞 | 清场、雷达/LIO 在线、准确初值和人工 reviewer |

不得把当前 7,095 个低支持 occupied cells 直接删除，也不得把任何已筛选候选写入
生产配置。当前结论确认的是受控残留机制，不代表所有现场脏点都只有这一种来源。

## 8. 回滚

没有修改或回滚产品参数和地图资产。代码层保留新增诊断模块、CLI、测试、模板和
文档；原生派生 exact-min2 后端只为验证短暂加入工作树，在双遍硬门失败后连同 launch、
reset 和依赖改动一起撤销。被否决源码与结果先保存为本地临时证据归档，待外部归档
写权限恢复后再复制。运行时验证进程已经停止，容器 ROS 图恢复为空。
`src/livox_ros_driver2_humble` 的用户子模块状态和未跟踪 `core.205` 全程未清理、暂存
或修改。

## 9. 2026-07-18 现场采集进度

用户清理出一片可用于诊断的区域后，先执行了不保存地图的安全预检。只启动左
MID360、FAST-LIO、self-filter、OctoMap 和 mapping session；Nav2、controller、
serial、mission 和 RViz 均未启动。左右雷达地址均可达，但 integrated mapping
仍按已验证合同只消费左雷达 `192.168.1.3`。

7.46 秒 preflight bag 的十个必需话题全部非零，随后完成当前位置正式
`static_p1`：

```text
duration: 30.455314459 s
continuous-topic receive-time overlap: 29.220624 s
messages: 10226
raw pointcloud: 1471
filtered pointcloud: 1522
registered transformed cloud: 1523
mapping sensor cloud: 153
odometry/lio: 1473
projected_map: 110
octomap_full: 106
tf: 3866
tf_static: 1 (latched)
mapping/recording: 1 (latched)
```

证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_static_p1/`

```text
metadata SHA-256: 234c2e979628b2da5b588f57eb9b2c50df8c884c4dccfd5a0ab2e8e31e8dbaed
bag DB SHA-256: 86398320d51b1f58f93aaef415d7aa9e9731343705019b7b756bb5ecb33701f4
```

采集期间 PGM 链正常生成 known cells。PCD 累计链仍出现较高比例的 latest-TF
fallback，但该现象单独列为 PCD 质量风险，不作为 PGM 根因。没有调用
`/mapping/save`，`static_p1` 没有生成 candidate。

车辆从位置 1 前进约 1--2 m 并逆时针旋转约 45--60 度，完全静止后完成第二段
`static_p2`：

```text
duration: 30.162044967 s
continuous-topic receive-time overlap: 27.965952433 s
messages: 9950
raw pointcloud: 1434
filtered pointcloud: 1509
registered transformed cloud: 1429
mapping sensor cloud: 141
odometry/lio: 1428
projected_map: 148
octomap_full: 145
tf: 3714
tf_static: 1 (latched)
mapping/recording: 1 (latched)
```

证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_static_p2/`

```text
metadata SHA-256: cb8ea8eb99ff7c7e9bc297744cf6e83e6c60a22a930ff77eac698a5048f474df
bag DB SHA-256: edff496aebf9a279f8bac8274679e161a86a5a3a13c045a9cd9dd6a527480ab3
OctoMap parameter SHA-256: ac70abb192109b5a8c011c6e3406262bc6e164dd658c7d9a88922b5a0f9ca19c
self-filter parameter SHA-256: 2dc6ce9632fd4f743b1e9ef118e009191ceae735ce15a896db0823578d9f9a77
```

十个必需话题均非零，连续话题的接收时间相交；采集进程已停止，且同样没有调用
`/mapping/save` 或生成 candidate。位置 2 期间的 latest-TF fallback 仍只作为 PCD
累计链风险记录。

位置 3 完全静止后完成第三段 `static_p3`：

```text
duration: 29.013439342 s
continuous-topic receive-time overlap: 28.400504365 s
messages: 10033
raw pointcloud: 1446
filtered pointcloud: 1451
registered transformed cloud: 1460
mapping sensor cloud: 143
odometry/lio: 1446
projected_map: 150
octomap_full: 145
tf: 3790
tf_static: 1 (latched)
mapping/recording: 1 (latched)
```

证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_static_p3/`

```text
metadata SHA-256: e0abea6242add98ac4b451f8c4d2ac7c0bd183ea34850affbb1c4dd13c938a19
bag DB SHA-256: 15b537b1297c49cab9c645b4052248a6d76889abe15b78f626b8109f0e44ebea
OctoMap parameter dump SHA-256: 9427b0099c98a71e8287adc7c6f4c80509a8e91e095c11f07ff235b847ca9b74
self-filter parameter dump SHA-256: 625f165e39c5c69c712b736a1f941cb353b651275b3f3eef03e83db8e552afb1
```

位置 3 的十个必需话题也全部非零，连续接收时间相交，采集进程已停止，没有调用
`/mapping/save` 或生成 candidate。位置 2/3 参数 dump 的逐行差异仅为文件末尾空行；
忽略空白后的内容完全一致，不构成参数漂移。三个静止位置已经采齐，下一步需要执行
两次相同路线的连续闭环 bag，并补齐现场 ROI/低障碍/墙体/地标标注；在此之前参数
sweep 保持阻塞。

随后从位置 3 出发，按位置 3 -> 位置 2 -> 位置 1 -> 位置 2 -> 位置 3 完成第一遍
连续闭环 `loop_r1`。各位置没有地面标记，用户明确说明返回位置为人工近似，因此该
bag 可用于累计/投影/TF 分层诊断，但不能作为厘米级重复路线真值：

```text
duration: 119.542965017 s
continuous-topic receive-time overlap: 119.135015762 s
messages: 41133
raw pointcloud: 5972
filtered pointcloud: 5978
registered transformed cloud: 5972
mapping sensor cloud: 597
odometry/lio: 5978
projected_map: 530
octomap_full: 526
tf: 15578
tf_static: 1 (latched)
mapping/recording: 1 (latched)
```

LIO 轨迹的起终点平面距离为 0.228254 m，航向差为 -6.128 度，最大离起点距离为
3.921692 m；这支持“已形成回环”，但不是外部测量真值。逐样本积分路径长度
21.400407 m 对高频位置抖动敏感，只保留为诊断值，不解释为人工实际里程。

证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_loop_r1/`

```text
metadata SHA-256: a170f000e8406d8c9a2727271ad287a3cd52bea21b0644b82dc9441a838e9824
bag DB SHA-256: 05aceafd1f4b1438201ad0520ce2ad7579aadf18fd4c691d8602c0543645166e
```

容器原件与主机副本的两个哈希完全一致后，为给第二遍采集释放空间，仅删除了容器
内临时重复目录 `/tmp/rm27_loop_r1_no_save`；上述主机证据仍完整保留。第一遍同样
没有调用 `/mapping/save` 或生成 candidate。

以第一遍返回后的当前停车点和朝向作为新基准，沿相同位置顺序完成第二遍连续闭环
`loop_r2`。路线仍为人工近似，但运动尺度与第一遍接近：

```text
duration: 94.742082742 s
continuous-topic receive-time overlap: 93.434445814 s
messages: 32471
raw pointcloud: 4738
filtered pointcloud: 4739
registered transformed cloud: 4738
mapping sensor cloud: 474
odometry/lio: 4687
projected_map: 398
octomap_full: 394
tf: 12301
tf_static: 1 (latched)
mapping/recording: 1 (latched)
```

第二遍 LIO 轨迹起终点平面距离为 0.200977 m，航向差为 4.695 度，最大离起点距离
为 4.307325 m，逐样本积分路径长度为 18.372236 m。两遍都形成有效回环，但由于
没有外部定位或地面路线标记，只能称为独立近似重复路线，不能声称路径逐点一致。

证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_loop_r2/`

```text
metadata SHA-256: 1c59af5447c969ae2fdc45e9d7c1c3f7e6c9b63635c450ede2de703812fb6443
bag DB SHA-256: c99da317f0d614dd5c1f2fd1b759d51f9307128c83e55b773f2def69250cb3d7
```

第二遍主机副本与容器原件哈希一致后，仅删除容器临时重复目录
`/tmp/rm27_loop_r2_no_save`。主机证据完整保留；没有调用 `/mapping/save`，没有生成
candidate。至此三个静止噪声位置和两段独立、话题完整的连续回环 bag 已采齐；阶段
1 仍缺人工 known-free ROI、受保护障碍、墙体和地标标签，以及临时障碍放入/移除
对照，故参数 sweep 继续阻塞。

之后将机器人移回大致位置 1，在车辆始终静止且同一个 LIO/OctoMap 会话持续运行的
条件下，使用边长约 0.20--0.30 m 的泡沫立方体完成临时低矮障碍对照。立方体被
放在车辆正前方约 1.5--2.0 m、原先为空且与其他障碍保持间隔的地面上；位置和尺寸
均为人工估计，尚未用卷尺或外部定位复测。五段数据为：

| 阶段 | 时长 (s) | 连续话题重叠 (s) | 消息数 |
|---|---:|---:|---:|
| `baseline_clean` | 25.222557781 | 23.943660467 | 6,003 |
| `place_transition` | 59.578794041 | 59.199211042 | 20,538 |
| `object_present` | 25.568730245 | 23.696951435 | 8,521 |
| `remove_transition` | 59.509935992 | 57.954858848 | 20,425 |
| `post_remove` | 25.527923389 | 23.944678613 | 8,376 |

每段的十个必需话题均非零；transition 段保留人员进入/离开和物体放置/移除过程，
稳定段只在人员离开雷达视野后采集。证据路径：

`/tmp/rm27_pcd_pgm_field_capture/20260718_temp_cube/`

```text
baseline metadata: 5b829194633b496c5fcbe0b8f52d25195499ff73037d9949424c226106ac712f
baseline DB:       4fa09f3f6124c63c7e9068f0af391778bb044f170f3dcf92677a956e4fadc37e
place metadata:    50b89fdc1a6e28f2821c5a4cf6fc583945546bf3a990cf92552accb1b6417d44
place DB:          24dc10cae4c362d6bc490dcaaf8dadde2eb338535687daa6fc649cf62514e8ae
present metadata:  13e0a17b1573111551e59ee103979d32966c4b5f4f673661ca739ae48c9765db
present DB:        e411f928c7166ad7a7c768ede68818111bad516ebd2215f65359cbb5d51af7b8
remove metadata:   b1c5b8f148b0a5a2d4e9e23908d4dc278cede732b9eebd7100ec60a0e4b42dc1
remove DB:         34f72ac5a2460b7fd02b1262e6ab0a1b8b88c1b59be28eb748b53a2c953c466e
post metadata:     ab518c9ff29fbca633c2b578a56b3278960594f5f6f5061187df5eb62492c996
post DB:           0414b752bc0c7753d3d0f8d56abb8052cfb669ea6a3d1cac10c1911ab4cbcabc
```

每段主机副本都与删除前的容器原件逐文件哈希一致。为避免 `/tmp` 写满，仅删除了
容器内已经核验的重复 bag 和最终临时运行目录；主机端约 3.0 GiB 的五段原始证据、
参数 dump、节点和话题清单完整保留。实验没有调用 `/mapping/save`，没有生成或覆盖
任何 PCD、PGM、YAML 或 candidate。该对照补齐了临时障碍证据，但自动验收所需的
known-free ROI、墙体/地标和立方体精确 world-frame mask 仍需从离线投影图配合人工
标注建立；完成标签前参数 sweep 仍保持阻塞。

## 10. 首轮 bag 增量分析

临时立方体实验在统一 world 网格上比较了各稳定段的末张
`/mapping/projected_map`。宽松的车辆前方 ROI 在 `object_present` 相对
`baseline_clean` 新增 205 个 occupied cells，其中 191-cell 主连通块范围约为
1.20 x 0.90 m，明显大于立方体；该主块混入了人员放置路径和其他瞬时端点，不能
作为立方体真值。

进一步只保留在 `object_present` 的 0.05--0.50 m 高度带内由至少 5 个独立
`/mapping/sensor_cloud` 帧支持、而在 clean/post 段最多 1 帧支持的端点，得到一个
6-cell 主证据块：

```text
world bbox: x=1.975..2.025 m, y=-0.025..0.175 m
centroid:   x=2.017 m, y=0.067 m
per-cell present support: 13..41 independent frames
baseline map state: 6 free
present map state:  6 occupied
post-remove state:  4 occupied, 2 free
```

最终 `/mapping/octomap_full` 的逐列检查修正了上述表面现象：六个低层立方体 voxel
在移除后获得大量 free rays，实际均已清除；4/6 二维格仍 occupied，是因为同列或
相邻列 z 约 0.775--1.225 m 的八个更高叶节点仍占用。它们通常仅由一个独立帧
命中，概率多为 0.70；后续射线与节点相差约 0.06--0.40 m，无法精确 clearing。
二维高度坍缩随后把高层瞬态点投到立方体附近。因此已确认的是 **高层瞬态 endpoint、
三维持久化和二维投影残留**，不是低层立方体本体未清除；它不等价于 Nav2
costmap 动态障碍 clearing 失效，也不否定此前 observation/clearing layer 实车测试。

两段闭环 bag 都在 mapping 会话初始化后才开始录制，因此全图中“最终 occupied 但
bag 内无端点”的数量会混入 recorder 启动前已经累积的节点，不能用于根因判断。
可靠分析只比较 bag 首张投影之后新出现、末张仍 occupied 的增量：

| 指标 | loop r1 | loop r2 |
|---|---:|---:|
| 新增 occupied cells | 2,759 | 3,297 |
| 0 帧支持（0.10 m 内） | 0 | 0 |
| 1 帧支持 | 51 | 107 |
| 2--3 帧支持 | 171 | 299 |
| >=4 帧支持 | 2,537 | 2,891 |
| 从 initial-free 变 occupied | 1,226 | 1,235 |
| 其中仅 1 帧支持 | 9 | 15 |
| 新增小连通域中的低支持 components/cells | 37 / 46 | 55 / 78 |

这证明 recorded interval 内的新 occupied 都能追溯到 filtered/sampled sensor
endpoint，二维投影没有无输入地凭空生成它们。单帧 endpoint 持久化在两段 bag 中
均可复现，但只占新增 occupied 的约 1.8% 和 3.2%；低支持小连通域的高度又分布在
0.10--1.80 m 全带，并非只集中在地面边缘。与此同时，>=4 帧支持的格子占新增量约
92.0% 和 87.7%，其中既可能是真实墙体/障碍，也可能是重复可见的环境杂物、人员或
系统性输入异常；没有 ROI 真值时不能把它们自动删除。

首轮可视化输出保存在独立诊断目录
`diag/20260718_field_bag_analysis_v3/`，没有写入任何地图 bundle：

| 文件 | SHA-256 | 内容 |
|---|---|---|
| `cube_before_present_post.png` | `51e9177c000a4783157f0759dbbaa1442af4c89aa17afcae16c573ce37bffbf5` | clean/present/post-remove 三阶段、低层证据与高层残留修正 |
| `cube_temporal_truth_detail.png` | `d2946a58ded390646fef70c5f2e460c9518fc10740c7b6d1cff29f1407ec1d01` | 六格时序真值和高层瞬态 endpoint 细节 |
| `loop_r1_endpoint_support.png` | `2ce87fa849997885a42ed9c375f45f13989af2d221601f303f5a262e5db8d9db` | 第一遍闭环新增 occupied 的独立帧支持分层 |
| `loop_r2_endpoint_support.png` | `edb55223feca17f5da03204562445fb98840dc622ca3db6bde9c7d9016ef663b` | 第二遍闭环新增 occupied 的独立帧支持分层 |

这些图只用于定位人工标注对象和核对统计，不把视觉上孤立的像素自动当作 false
positive；known-free、protected obstacle 和地标仍须由现场真值确定。

当前证据支持的暂定排序为：

1. 瞬时人员/障碍及其 OctoMap endpoint/三维投影残留已经由受控实验直接证实；
2. filtered sensor cloud 内的单帧空间端点会形成一小部分可复现脏点；
3. 大多数新增 occupied 是多帧观测，是否属于真实障碍仍需人工 ROI/地标标签；
4. PGM exporter/map_server 不是脏点来源；
5. 仅调 `occupancy_min_z` 或只删除小连通域缺乏证据，且可能删除 20--30 cm 低障碍。

在 known-free/protected-obstacle 标签完成前，不执行生产参数修改或 candidate 批准。

## 11. 受约束 replay 与候选结论

完整筛选报告位于外部证据归档：

`diag/20260718_spatial_ray_screen/SCREENING_REPORT_20260718.md`

关键结论：

- hit/miss 单变量不能清除受控 4/6 残留；降低 hit 只会无差别减少全图 occupied；
- endpoint-only 过滤会损失 free-ray/覆盖合同或重新放过相邻瞬态点；
- 始终保留 free rays 的精确 3D min2 能清除受控残留，但 loop r1 过度拆分结构；
- 跨帧邻域 min2/min3 分别在 post-remove 残留 3/6、1/6；min4 达到 6/6 free，
  且低障碍在 0.05 m 容差内召回 100%；
- min4 在 loop r1/r2 分别减少 643/1,057 个 occupied，已知覆盖率不下降，且在
  24,523 个双遍共同 free 格内没有新增 occupied；
- 但双遍一致 protected proxy 的 0.10 m 召回仅 97.71%/98.12%，低于 99% 硬门。

因此阶段 4 的结论是“找到有效但可能过度过滤的实验候选”，不是“修复已完成”。
后续曾把一个更小的原生派生 C++ 原型临时接入工作树进行 byte-parity、launch/reset
和三段 bag 验证；第 13 节记录其失败与回滚。最终产品树仍不包含该后端，未修改
生产参数、未生成 candidate，也未降低验收阈值迁就结果。

## 12. 投影候选淘汰与暂停过渡段结论

在新增清场闭环和现场标签后，继续筛选了三类隔离原型：二维竖直列多帧门、只在
二维投影时应用的 `occupancy probability >= 0.71` 门，以及单帧半径邻域过滤。
完整淘汰报告位于：

`diag/20260718_projection_screen_rejected/PROJECTION_AND_LOCAL_NEIGHBOR_AUDIT_20260718.md`

关键反证如下：

- 竖直列 `min3/exact XY` 虽能使 cube post-remove 达到 6/6 free，但 loop r1 的
  双遍 protected proxy 0.10 m 召回仅 96.49%，occupied components 从 115 增至
  156；允许一个 XY 邻格后，受控残留又恢复为 4/6。
- 投影概率门同样通过 cube，但 loop r1 保护召回仅 95.93%，components 增至 176。
- 能按派生体素键精确回溯的六个高层残留事件全部来自 `place_transition` 的
  约 1.4 秒窗口；它们在单帧 0.10 m 内仍有 5--9 个邻点。真实低 cube 点的邻域
  分布与之重叠，因此它们不是可由简单 radius outlier 门安全删除的孤立噪声。

这组证据把受控残留进一步收敛为“人员/物体进入时的短时动态表面被静态 OctoMap
接收”，而不是 self-filter 漏掉的典型孤点。随后使用标准 `octomap_server 2.3.1`
和原参数做暂停过渡段回放：保留 `baseline_clean`、人员离开后的
`object_present`、人员离开后的 `post_remove`，跳过放入和移除过渡段。结果为：

```text
baseline_clean: 6/6 exact free
object_present: 5/6 exact occupied, 6/6 within 0.05 m
post_remove:    6/6 exact free, 0/6 occupied within 0.10 m
```

成功报告位于：

`diag/20260718_paused_transition_replay/PAUSED_TRANSITION_REPLAY_REPORT_20260718.md`

因此当前最小修复不需要修改 OctoMap、self-filter 或地图文件，而是严格使用现有
`/mapping/stop`、`/mapping/start` 和 `/mapping/recording` 合同：人员进入、搬运
物体或改变静态场景前暂停；人员离开且场景稳定后恢复，并重新覆盖受影响视角。
该规则只约束静态建图采集，不否定 Nav2 导航期间已经验证的动态障碍 clearing。

## 13. fresh03 标签与原生派生 exact-min2 最终否决

2026-07-19 在清理区域重新采集 fresh03：正式 bag 197.873 秒、66,227 条消息，十个
必需话题全部非零，`/mapping/sensor_cloud` 共 760 帧。用户明确说明闭环终点停车只
是人工近似，因此起终点差异不作为定位或建图失败硬门。归档位置：

`field_capture/20260719_clean_area_fresh03/`

```text
DB3 SHA-256:      54bd27ef00733cac31cb3dee5cb2935be595c9d35f96c36741caab7ac6c33bd0
metadata SHA-256: a8643ad7841a7923988a40a6f8ca09e2564dc59fcd75f829ac2dcaf9b066a999
```

跨 session 标签使用一次性拟合的刚体变换 `dx=+0.425 m`、`dy=+0.275 m`、
`yaw=-1.75 deg`。known-free 593 格全部为 free；墙 187/187、柜 135/135 个采样点在
0.05 m 内连续；F1/F2/F3 和大箱子均在 0.05 m 内有占用支持。小箱子标签为人工粗点，
基线和候选最近支持都约 0.1176 m，只能证明候选没有继续损失，不能冒充严格 0.10 m
自动批准真值。

随后实现并隔离验证了一个直接继承 Humble `octomap_server::OctomapServer` 的最小
原型。它保留原生三维 hit/miss、ray clearing、tree、free 投影和地图边界，只在二维
投影 occupied leaf 时要求同一个 full-resolution endpoint voxel 来自至少两个独立
采样帧。`minimum_independent_frames=1` 为无条件原生旁路；fresh03 前 100 帧与上游
server 的 OccupancyGrid 数据 SHA-256 完全一致：

`3ce7f87ec86b0e197ca9a7ee62d78b97fcd97530b0c6f7cfe9f2e8b365434c65`

完整 fresh03 逐帧 ACK 对比均处理 760/760 帧：occupied `15,674 -> 11,291`，known
coverage 仅下降 0.2902 个百分点，标签墙/柜/地标/known-free 无差异。这只允许进入
第二数据集复验，不构成批准。

旧 loop r1/r2 使用相同的逐 cloud timestamped projected-map ACK 合同重新验证。r1
为 597/597 帧，occupied `5,081 -> 3,693`；r2 为 474/474 帧，occupied
`5,983 -> 4,050`。以“双遍 baseline occupied 在另一遍 0.10 m 内也有 occupied 支持”
建立保护代理后：

| 指标 | loop r1 | loop r2 |
|---|---:|---:|
| protected proxy cells | 2,469 | 2,752 |
| exact-min2 0.10 m recall | **96.64%** | **96.11%** |
| 非一致 baseline cells | 2,612 | 3,231 |
| 非一致格删除比例 | 27.87% | 38.63% |

两遍共有 24,336 个 exact common-free 格，候选都没有新增 occupied；但保护召回明显
低于 99% 硬门，且 r1 小连通域像素从 606 增至 728，说明结构碎裂风险不能用总点数
下降掩盖。因此原型被正式否决：不进入产品 package/launch/reset，不请求车辆现场
测试，不生成 candidate。

完整否决报告和原型归档：

```text
diag/20260720_native_min2_rejected/
20260720_native_r2_confirmation/NATIVE_MIN2_REJECTION_REPORT_20260720.md
20260720_rejected_native_min2_archive.tar.gz
archive SHA-256: 75a718be981bb8323a9041724b0876a3596d8337eff906f9725cd60ab0a220bc
```

外部归档在用户重置用量并明确要求重试后完成：

`diag/20260720_native_min2_rejected/`

初次复制的 44/44 个结果文件以及原型 tar 均完成哈希核对；随后追加 5 个回滚后
默认 mapping smoke 文件，归档现共 50 个文件。关键报告、双遍 JSON、原型 tar 和
回滚 reset response 的本地/外部 SHA-256 逐项一致。回滚后增量构建成功，
`rm_map_tools` 39/39 测试通过，`rm_octomap_evidence` 不再可被 ROS package index
发现，默认 mapping smoke 通过。当前可交付的最小修复仍是第 12 节已经通过受控
立方体回放的暂停过渡段采集合同。

## 14. fresh03 冻结候选与有限现场验收

exact-min2 实现的产品化结论仍为“否决”，但用户后续要求将已存在的
fresh03 `11,291` occupied 结果单独冻结为可回滚的现场候选，用于临时地图
验收，不恢复原型到产品链。该 bundle 保持 `candidate` /
`candidate_for_field_validation`，未修改正式 deployment manifest，也未自动
批准。

2026-07-20 的有限现场检查完成了 PGM/YAML 严格解码、live `/map`
151,257 格逐格一致、AMCL/RViz 静态对齐、人工低速移动后对齐，以及一次
0.15 m/s 限速的短距离 Nav2 → 真实串口链路。短距离 action 以零 recovery
返回 `SUCCEEDED`；后续复核为任务 `disabled/hold`、3 秒内无
`/cmd_vel`、定位仍有效，测试限速已清除。AMCL 曾偶发“90% 以上观测
不在地图”警告，因此该结果只支持用户决定的临时使用，不将 candidate
提升为 approved。

候选哈希、现场证据、操作员自动模式规则和临时启动清单见
[fresh03 候选有限现场验收](old_car_fresh03_field_validation_20260720.md)。
