# Navigation Integrity 第一版实现报告

## 当前状态

实现以 `fix/old-car-global-dynamic-replanning @ 9b608b9` 为基线，在
`feature/navigation-integrity` 开发。第一版新增单一 `rm_navigation_integrity` 包，
没有修改现有 localization、TF、Nav2、BT、mission 或 serial 运行逻辑。

## 已实现

- shadow-only localization integrity node；
- global pose 与 timestamp-matched odometry 的 candidate correction 指标；
- timestamped sensor TF 下的 2D scan-map endpoint agreement；
- GOOD/SUSPECT/REJECT/UNKNOWN 与离散 reason codes；
- 标准 `/diagnostics` 输出；
- 参数显式开启的 JSONL 证据记录；
- JSONL raw summary 与 baseline delta CLI；
- T01-T15 regression scenario contract；
- mission/Nav2 stale-result coverage audit；
- Linux build/replay 和 old-car 现场验证计划。

## 未采用

- 未使用 GICP/ICP，因此不伪造 Hessian、condition number 或 observability；
- 未设计综合 confidence score；
- 未启用 gate/warn authority；
- 未新增动态目标 tracker；
- 未新增 action stress sender；
- 未改变 map->odom 或 odom->base_link owner。

## HWSentry 思想吸收

吸收了“定位器自信不等于外部观测可信”、多证据 reason code 和 stale-result coverage
审计思想。没有迁移 HWSentry 的 GICP localizer、custom TF、FSM、FDDP、Kino A*、MINCO
或 time-indexed costmap。

## Windows 验证边界

Windows 已完成 Python compile、XML/配置静态检查和标准库单元测试：13 项通过，
1 项依赖 PyYAML 的场景目录测试跳过。当前环境没有 ROS 2、colcon 或 PyYAML，
因此不能运行 ROS 节点、launch、完整 YAML parse 或正式 `colcon test`。

正式结果必须在 Linux 更新：

```text
LINUX BUILD REQUIRED
LINUX COLCON TEST REQUIRED
FIELD VALIDATION REQUIRED
```

历史高速自转报告仍在仓库，但 bag 不在当前工作区，因此本实现尚未回答“哪些指标稳定
区分正常与历史失败”。这必须由 field plan 的 A/B 数据补齐。

## 新车继承

核心算法、diagnostics 和 regression CLI 可原样使用。新车只需配置标准 topic/frame、
提供时间正确的动态云台 TF，并重新采集正常指标分布。阈值、scan density、允许的运动
变化率和 CPU 性能必须重新验收，不能继承老车数值后直接进入 gate mode。
