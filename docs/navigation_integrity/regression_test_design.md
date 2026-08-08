# Navigation Regression Framework

## 目标与边界

框架把同一场景产生的 integrity JSONL 转换为稳定 JSON summary，并可与先前 summary
计算 delta。第一版只自动化 localization integrity 原始指标，不把尚未实现的 costmap、
action 或 path 指标伪装成已完成能力。

```text
ROS run / replay
  -> localization_integrity optional JSONL
  -> evaluate_navigation_regression
  -> summary JSON
  -> optional baseline delta
```

输出明确写入：

```text
evaluation_status: RAW_METRICS_ONLY
comparison_status: DELTA_ONLY_NO_PASS_FAIL
```

在现场分布和阈值获得审查前，不自动生成 PASS/WARN/FAIL。

## 已自动提取指标

- GOOD/SUSPECT/REJECT/UNKNOWN 计数；
- reason code 计数；
- 最大 global pose translation/yaw jump；
- 最大 candidate map->odom translation/yaw jump 与 rate；
- 最小 scan-map agreement；
- 最大 mean/p95 endpoint residual；
- pose/scan/odom/TF 时间差和 freshness；
- 最小有效 scan 点数和 map 内点数；
- 样本数和 source-stamp 时间范围。

Baseline comparison 对同一 selector 只计算 `current - baseline`。平台不同或场景不同的
summary 不应比较；工具不会替用户判断它们是否可比。

## 场景目录

`config/regression_scenarios.yaml` 定义 T01-T15 的 required topics、required metrics、
expected behavior 和 automation 状态。

当前已具备 metrics automation：T01、T05、T14、T15 的 localization 部分。

需要现场观察或 bag 分析：T02-T04、T08、T11-T13。

尚需专用 extractor/probe：T06-T07 costmap latency、T09-T10 action lifecycle、T12 recovery
duration。它们保留为接口合同，不在本轮生成虚假数值。

## Old-Car 与 New-Car Baseline

平台通用 baseline 可比较：

- frame/time contract；
- scan/global pose/odom freshness；
- correction jump；
- scan-map agreement；
- action lifecycle 和 stale result count。

平台特有值必须分别保存：

- 角速度和加速度；
- chassis response/cmd_vel tracking；
- scan density；
- gimbal motion envelope；
- CPU latency。

推荐目录在仓库外：

```text
/data/rm27_navigation_regression/<platform>/<scenario>/<run_id>/
```

每个 run 保存 bag、`.metrics.jsonl`、summary、commit、launch command、map hash 和参数
dump。只提交经过审查的小型 baseline summary 时，应使用单独文档/配置变更，绝不提交
bag 或自动生成图表。

## 使用

```bash
export INTEGRITY_SHARE=$(ros2 pkg prefix rm_navigation_integrity)/share/rm_navigation_integrity
ros2 run rm_navigation_integrity evaluate_navigation_regression \
  --input /data/.../T05/run.metrics.jsonl \
  --scenario T05 \
  --definitions "$INTEGRITY_SHARE/config/regression_scenarios.yaml" \
  --output /data/.../T05/run.summary.json
```

有可比 baseline 时附加：

```bash
  --baseline /data/.../T05/verified_baseline.summary.json
```

## 后续最小扩展顺序

1. 用真实 T05 bag 校准并验证现有 localization metrics；
2. 增加只读 costmap transition extractor，完成 T06/T07；
3. 如手工 action stress 发现真实缺口，再增加 T09/T10 probe；
4. 只有数据稳定后，才为同平台同场景定义审查过的 pass/warn/fail 门。
