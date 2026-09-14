# P2B 精确端点连接修复与验证

开发开始：2026-09-14；继续验证：2026-09-15。开发基线 `78648e2`，工作区
`/home/wpie/worktrees/rm2027_tdt_phase2`。本轮由开发者执行测试和仿真。

## 依据与设计

static_v4 的构建、46 项测试及统一 sanitizer 链已通过，但 T-DT 的两个首例均在
行驶期间出现端点拒收并导航失败。原始报告和 272 份 v4 文件已冻结于
[evidence/endpoint_connectors_20260914/preserved_static_v4.json](evidence/endpoint_connectors_20260914/preserved_static_v4.json)。
旧 v2 的 166 份、sanitizer 修复保留集的 238 份以及现场三个配置/launch 文件均核对通过。

在发布的历史地图中，目标 (4.3,0) 到最近占用闭方格常为 0.460977 m，大于本轮
padded 外接圆 0.432782 m + clearance 0.02 m；但原算法要求目标所在整格都安全，
距离变换额外预留整格对角线后会拒绝该点。部分地图距离只有 0.430116 m，小于
同一半径要求，仍须拒收。目标到已知仿真墙角的几何距离为 0.5 m，但传感器栅格化
占用与理想墙体并不完全相同，不清除这些占用来通过试验。

[前端诊断](evidence/endpoint_connectors_20260914/static_v4_astar_endpoint_inspection.json)与
[QP 诊断](evidence/endpoint_connectors_20260914/static_v4_qp_endpoint_inspection.json)
使用告警前后发布的地图：它们不等于插件锁内 snapshot，分辨率 float32 和相邻姿态
也存在精度/时序差异，因此不能仅靠这些记录把每一次告警判为误报。

实现只改变 Nav2Master 的端点接入：

1. 用原始 254/255/边界闭方格和不变的 radius+clearance 检查精确端点；253 禁止
   中心进入或擦边。数值保护为正的 1e-7 m，接触也拒收。
2. 在端点周围寻找距离不超过 3*resolution 的最近自由格中心，并检查连接段的
   完整圆扫掠。3 格仅限定短连接范围，不是可移动目标的 tolerance。
3. A*/QP 仍在原保守膨胀掩码内连接两锚点；精确端点在优化后接回。最终所有
   输出段再次按原始地图检查。QP 失败仍只回退到本次经过检查的 A*。
4. 保留请求的起终点、目标 yaw 和 snapshot 更新/超时拒收。PreparedGrid 保存
   原始输入副本，确保 prepared 与 raw 路径/安全语义相同。默认离线输入策略不变。

局限：最近锚点策略不保证搜索所有可行接入；真实圆碰撞、没有锚点、内部无解等仍返回
空路径。修复并不证明 MPPI 跟踪稳定、任意朝向矩形通行或动态安全。

## 验证进展

新增 5 项核心测试覆盖安全端点双向连接、真正碰撞拒绝、raw/prepared 一致性及不可变性、
1,500 组线段与独立 oracle 比对、旧离线整格策略保持。新增 1 项工具测试区分
整格拒收与点碰撞。原有断言保持，预期当前总数 52（30 core + 4 plugin + 14 tools + 4 ROS）。

开发日志位于 `build/tdt_p2b/endpoint_connectors_logs/`。截至仿真开始前，52 项完整 CTest、四配置哈希
核对、统一 sanitizer 审计及 30 项核心均通过。日志索引见
[development_checks.json](evidence/endpoint_connectors_20260914/development_checks.json)。
static_v5 尚待执行，未运行结果不记为通过。

复现入口：`experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh`，按
build → check → sanitizers → profiles → 四组首例 → 通过组重复 → summarize 顺序执行。
使用 `P2B_SERIES=static_v5`；每次运行新容器、网络隔离、无设备。组失败即保留首例并
停止该组重复。旧系列不回填，.05 m 间隙、终点/yaw、零恢复和停止命令门保持。

移动障碍、LIO/双 STVL/MPPI 全负载和实车验收仍不在本轮范围。设备性能只记录。
