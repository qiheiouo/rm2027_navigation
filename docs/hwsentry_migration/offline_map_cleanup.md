# 离线射线证据地图清理原型

## 数据依赖结论

HWSentryNav26 的离线清理使用每帧传感器原点、局部点云和优化位姿，通过
3D-DDA 统计某个占用 voxel 被后续自由射线穿过的证据。它不是对最终 PCD 做
半径滤波，也不是完整因子图地图优化。

对应源码为
`utils/cpp/offline_mapping_optimizer/src/raycasting_filter.cpp`；当前数据能力
来自 `src/rm_map_tools/rm_map_tools/mapping_session_node.py` 的实际持久化内容。

当前 Phase 2I 最终 PCD 只保留 map-frame 体素均值/计数，无法恢复每帧 ray
origin、endpoint 和自由空间路径。因此仅凭现有合并 PCD 不能等价复现该算法。

## 本阶段实现

`rm_map_tools ray_evidence_cleanup` 是可选离线原型。它读取：

1. 一个 immutable candidate ASCII XYZ PCD；
2. 一个显式的 `rm_map_ray_observations/v1` JSONL sidecar；
3. 体素和证据阈值。

sidecar 首行为：

```json
{"schema":"rm_map_ray_observations/v1","frame_id":"map"}
```

其后每行是一帧严格递增的观测：

```json
{"type":"observation","stamp":1.0,"origin":[0,0,0],"endpoints":[[1,0,0]]}
```

工具对每帧 hit/pass 去重，只在 pass frame 数达到阈值且显著超过 hit frame 时
移除候选 voxel。它总是先生成 JSON report；只有显式指定
`--write-candidate` 才写新的 PCD：

```bash
ros2 run rm_map_tools ray_evidence_cleanup \
  --input-pcd /maps/source.pcd \
  --observations /evidence/rays.jsonl \
  --report /tmp/ray_cleanup/report.json \
  --output-pcd /tmp/ray_cleanup/cleaned_candidate.pcd \
  --write-candidate
```

输入、报告和输出已存在时均拒绝覆盖。工具不会生成 PGM、bundle、approved
状态，也不会修改部署 manifest。

## 尚未实现

Phase 2I mapping session 还没有写上述 sidecar。本阶段没有为了一个实验工具增加
正式建图磁盘负担。若未来决定采集，应通过默认关闭的参数保存降采样 keyframe
origin/endpoints，并设置 session 大小上限、哈希和中断行为。

当前能力只能用合成数据验证算法边界，状态为：

```text
REAL MAP VALIDATION REQUIRED
MAPPING SIDECAR RECORDER NOT IMPLEMENTED
```
