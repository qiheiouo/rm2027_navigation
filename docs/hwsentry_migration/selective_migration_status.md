# HWSentryNav26 选择性迁移状态

本文记录已经确认的迁移边界，避免后续因重新发现上游模块而重复实现或扩大范围。

## 已落地

- 默认关闭的动态障碍 shadow tracker：静态剔除、聚类、Kalman 跟踪和短时预测；
- 离线 3D-DDA 射线清图与默认关闭的有界 sidecar 采集链；
- GICP 后验质量门：重叠率、中心化 point-to-plane SE(3) 信息矩阵最小特征值和
  condition number；
- 默认关闭的语义区域与 annotated path sidecar：地图/区域语义哈希绑定、确定性
  path revision、弧长分段、限速/航向/no-spin/准入/COMMITTED 数据合同；
- dynamic MPPI、factor-graph LIO、特殊通道准入与 COMMITTED 执行接线的隔离设计。

以上实现都不改变 Nav2、MPPI、costmap、底盘或 canonical TF 的默认所有权。

## 已冻结

建图、射线清图、ERASOR2、CAD 地图生成和地图编辑工作冻结。场地 PCD/PGM
由操作者从同一 CAD 来源导出并人工维护；现有 Phase 2I 代码和文档保留，但不继续扩展。
需要匹配地图的 sidecar、DDA 效果和 tracker D01-D07 实车验收延后执行。

## 后续明确项

1. 特殊通道在进入前做预测占用准入，进入后遵守已落地 sidecar 的 COMMITTED
   traversal policy；接入已有狗洞执行器，
   不新增第二套 FSM/action owner；
2. tracker D01-D07 使用匹配场地的地图通过后，动态障碍 MPPI 才按 shadow score、
   零权重、仿真、低速实车顺序推进。

## 条件触发项

- 只有 FAST-LIO 完整性证据表明现有里程计不可满足需求时，才启动 factor-graph LIO A/B；
- 只有现有 AMCL/GICP 在同图同包对比后仍有明确缺口，才 clean-room 增加 NDT shadow；
- 只有实际驱动日志证明 CRC、时间或数据有限性保护不足，才补 MID360 驱动鲁棒性；
- 离线 pose graph、ERASOR2 等地图工具保持外部候选，不进入当前主线。

## 永久排除

不迁移 HWSentry 的轮腿 FDDP、整套 planner、完整 FSM、small_glim 全量替换或自有
`map -> odom` 发布链。
