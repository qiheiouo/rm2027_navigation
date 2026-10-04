# 离线射线证据地图清理原型

## 数据依赖结论

HWSentryNav26 的离线清理使用每帧传感器原点、局部点云和优化位姿，通过
3D-DDA 统计某个占用 voxel 被后续自由射线穿过的证据。它不是对最终 PCD 做
半径滤波，也不是完整因子图地图优化。

对应源码为
`utils/cpp/offline_mapping_optimizer/src/raycasting_filter.cpp`。Phase 2I 依然不会
尝试从合并后的 map-frame PCD 反推射线；该 PCD 只保留体素均值/计数，
无法恢复每帧 origin、endpoint 和自由空间路径。

## 已实现的采集合同

`mapping_session_node` 已可选记录 `rm_map_ray_observations/v1` sidecar。开关
`record_ray_observations` 默认为 `false`；关闭时不订阅射线点云、不创建
spool，也不增加正常建图、Nav2、costmap、TF 或底盘的运行路径。

开启后只能订阅 `/mapping/sensor_cloud`，并且必须为该帧时间戳查询
`map <- <physical_lidar_frame>` TF：

- `/mapping/sensor_cloud` 保留物理雷达坐标系中的真实回波，与 OctoMap
  使用同一条最多 5 Hz 的采样路径；
- `/lio/cloud_registered*` 已经是世界配准点云，丢失了完整的每帧传感器
  原点合同，严禁用它代替射线输入；
- 射线不允许 latest-TF fallback。用其他时刻的位姿搭配当前端点会伪造
  pass/hit 路径；查不到同时刻 TF 时只计数并丢弃该帧。

旧车入口会将物理雷达帧锁定为 `mid360_left_frame`：

```bash
ros2 launch rm_navigation_launch old_car_mapping.launch.py \
  record_ray_observations:=true \
  output_root:=/data/rm27_maps \
  map_id:=old_car_field
```

通用 `mapping.launch.py` 使用同名开关，开启时必须显式传入
`ray_source_frame:=<physical_lidar_frame>`。不要把配准点云的 `map`/`odom`
frame 填入该参数。

## 资源上限与中断语义

通用建图入口默认每 0.20 s 最多接收一帧；旧车入口为降低资源压力默认
每 0.50 s 一帧，并把周期、量程、voxel 和全部上限暴露为 launch 参数。
两者都仅保留 0.30--12.0 m 的真实 return，用 0.10 m endpoint voxel 做
确定性去重。超过最大距离的回波
会被丢弃，不会被截短成伪 hit。默认硬上限为：

- 10,000 帧；
- 每帧 10,000 条去重射线；
- 整个 session 10,000,000 条射线；
- sidecar 总大小 536,870,912 bytes（512 MiB）。

记录时只向
`<output_root>/.ray_sidecar_sessions/*.records.partial` 写有界 records-only spool。
该 partial 文件没有 v1 header，不会伪装成可用 sidecar。执行
`/mapping/save` 时才生成不可覆盖的 JSONL snapshot，然后与同一次导出的
PCD/PGM/YAML 一起绑定到 candidate bundle。manifest 记录 sidecar 的
SHA-256、字节数、帧数、射线数和 `offline_evidence_only` authority；
sidecar header、source details 与 artifact 还必须具有相同的 32 位
`capture_id`，防止跨独立 recorder 实例误配。`validate_map_bundle` 会同时校验 payload
和这些声明。运行时地图 resolver 完全跳过该离线 artifact，因此大 sidecar
不会进入 Nav2/map-server 启动路径。

`complete` 表示 snapshot 是从未触发致命合同/资源错误的 spool 生成；它不等于
TF 零丢帧、视角完整或地图已验收。frame 变化、时间倒退、单帧/总量/字节
上限或 spool I/O 错误会立即 halt recorder、发布
`/mapping/recording=false` 并要求 `/mapping/reset`。显式开启 ray 后，缺 recorder、
零接受帧、snapshot 失败或 `incomplete` 都会让 `/mapping/save` 默认失败，避免一次
重建图被误报为已取得证据。只有操作者决定抢救基础地图/诊断数据时，才可临时执行：

```bash
ros2 param set /mapping_session ray_allow_degraded_save true
ros2 service call /mapping/save std_srvs/srv/Trigger {}
ros2 param set /mapping_session ray_allow_degraded_save false
```

该动态覆盖不会由 launch 暴露，且会写入 manifest。它可能附带 `incomplete`
sidecar，也可能只保存基础 candidate；两者都不得进入正式射线清图流程。

完整 sidecar 已随 bundle 保存、且此后没有新增观测时，正常关闭节点会删除本次
已导出的 records-only spool。未保存、保存失败、incomplete 或异常退出的 partial
会保留供诊断；它没有 v1 header，绝不能直接传给清图 CLI。

## 离线清理

`rm_map_tools ray_evidence_cleanup` 对每帧 hit/pass 去重，只在 pass frame 数达到
阈值且显著超过 hit frame 时移除候选 voxel。它总是生成 JSON report；
只有显式指定 `--write-candidate` 才写新 PCD。candidate 会先发布；若随后
report 的不可覆盖发布失败，完整 candidate 会保留，但整个命令仍返回失败。

真实场地数据必须使用 manifest mode，以防止人工拼错 PCD 和 sidecar：

```bash
ros2 run rm_map_tools ray_evidence_cleanup \
  --input-manifest /data/rm27_maps/old_car_field/REVISION/old_car_field.bundle.yaml \
  --report /tmp/ray_cleanup/report.json \
  --output-pcd /tmp/ray_cleanup/cleaned_candidate.pcd \
  --write-candidate
```

`--input-pcd` 和 `--observations` 直接输入仅保留给合成/旧版测试。生成 candidate
时默认拒绝 `incomplete` sidecar；
`--allow-incomplete-observations` 是会写入 report 的显式不安全实验
覆盖，真实场地验收不得使用。

输入 bundle 始终只读；report 和 candidate 先在目标目录写完并同步，再用
原子 no-replace 发布。工具不生成 PGM 或 bundle，不修改部署 manifest，
也不改变任何地图的 `candidate`/`approved` 状态。

## 重新建图前的门

先按 [Phase 2I managed mapping](../phase2i_managed_mapping.md#ray-sidecar-two-minute-preflight)
完成 2 分钟软件/资源 smoke，确认没有 halt、字节增长有界且磁盘余量
满足保存峰值，再通知操作者重新建图。旧地图与当前环境不匹配时，
不得用它执行动态 tracker D01--D07 结论性验收；必须先采集与当前
场地一致的新 candidate、验证 bundle 并完成人工地标/占用回顾。

当前状态为：

```text
BOUNDED SIDECAR RECORDER AND BUNDLE CONTRACT IMPLEMENTED
LINUX CLEAN BUILD/TEST PASSED 2026-08-16 (213 TESTS, 0 FAILURES, 0 SKIPPED)
DEFAULT-OFF AND EXPLICIT-ENABLE NODE CONSTRUCTION SMOKES PASSED
2-MINUTE SENSOR/TF/RESOURCE SMOKE REQUIRED BEFORE REMAP
REAL MAP RAY-CLEANUP VALIDATION REQUIRED
```
