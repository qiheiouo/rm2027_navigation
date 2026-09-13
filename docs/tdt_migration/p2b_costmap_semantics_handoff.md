# P2B：Nav2 代价语义修复与 static_v3 交接

状态：**static_v2 已复核；重复膨胀修复与回归用例已编写，尚待 Terra 编译及运行验证。**
开发基线 `e85b076760c1367385b4f92a84ee24b14985c3c0`，分支
`experiment/tdt-planner-phase2`；修复交接提交以设计者交付号为准。
本轮继续由设计者负责分析/实现、用户操作的 Terra 负责测试/仿真/结果回传。

## 已有结论与本次诊断

[static_v2 回传](evidence/p2b_static_v2_20260913/validation_20260913.md) 的构建和
39 项测试通过已核对。Navfn/Smac2D 各 5/5 通过有限静态门；T-DT 两组首例均失败、
各恢复 16 次。166 个已审计原始文件、aggregate 和回传报告的哈希已复核并保存于
[原证据清单](evidence/costmap_semantics_20260913/preserved_static_v2.json)。
旧报告保留原样，不回填或改写 static_v1/static_v2。

原始事件中，每个 T-DT 组各有一次 `costmap or footprint changed`，另有 A* 15 次、
QP 14 次 `start or goal outside conservative free space`。后两项不是并发拒收次数。
本次不放宽 snapshot 更新拒收，不延长 time_budget，不更改重试或恢复行为。

已确认本地适配层存在重复膨胀：它把 Nav2 的 253 内切区和 254 实体障碍均作为
占用种子，再加 padded footprint 外接圆、clearance 和整格对角线余量。内切区已经
包含一轮 footprint 膨胀，不能再被解释成真实障碍表面。

依据固定 Humble/Nav2 1.1.20 的
[代价值定义](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp)、
[InflationLayer::computeCost](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp)
和 [发布转换](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_publisher.cpp)：
master 253/254/255 在 OccupancyGrid 中对应 99/100/-1。本轮只核对其语义，未复制
Nav2 源码或升级任何依赖。

历史数据分析工具 `inspect_simulation_endpoints.py` 对 29 条端点拒收前后的发布地图
计算端点格中心到实体/未知/边界种子和内切区的距离。仿真 padded 外接圆为
`hypot(0.33,0.28)=0.4327817002 m`，clearance=0.02 m、分辨率约 0.05 m，
原格中心距离阈值约 **0.523493 m**（含数值容差）。

| 观测 | 目标原格值 | 到实体/未知/边界最近格中心 m | 到内切区最近格中心 m | 旧模型 / 修正种子模型 |
| --- | --- | --- | --- | --- |
| A* 首次拒收后的 8.949 s 地图 | 0 | 0.672681 | 0.424264 | 阻塞 / 自由 |
| QP 首次拒收前的 8.636 s 地图 | 0 | 0.721110 | 0.460977 | 阻塞 / 自由 |
| A* 末次拒收前的 29.034 s 地图 | 0 | 0.670820 | 0.403113 | 阻塞 / 自由 |

A* 拒收前的 15 份对应地图中 14 份、拒收后的 14 份中 14 份呈现这一现象；QP
前后各 14 份均呈现。对应项可能使用同一幅发布地图，不能当作独立样本。
完整数值与输入哈希见 [A*](evidence/costmap_semantics_20260913/tdt_astar_endpoints.json)、
[QP](evidence/costmap_semantics_20260913/tdt_qp_endpoints.json)。

这些是**历史数据分析与种子模型预测**，不是重新执行规划器：发布地图并非插件锁内
精确 snapshot；开始位置来自附近 ground truth；OccupancyGrid 的 float32 分辨率
可能影响格线取整。A* 首次拒收前的地图尚未阻塞目标，恰好说明不能用发布快照断言
精确回调时序。证据支持优先修正重复膨胀，尚不证明全部失败已解决。

## 代码语义与保留的约束

新增显式输入标记 `Grid::cost_interpretation`，没有新增 ROS 调参开关：

| 输入 | 膨胀种子 | 路径中心禁区 |
| --- | --- | --- |
| 默认 `ObstacleSeeds`，既有离线 API/P2A | 253/254/255 和地图边界 | 种子膨胀结果及原 253/254/255 |
| Nav2 插件明确设置 `Nav2Master` | 254/255 和地图边界 | 种子膨胀结果，另外保留每个 253 格为禁区 |

因此 253 不会被清成可直接穿越的中心格；即使孤立 253 周围没有 254，中心禁区也
保留。实体/未知区的完整 padded 外接圆、clearance、整格对角线和全段 supercover
仍有效。QP 失败仍只能回退到本次已经验证的 A*，不能复用旧路径。原 master bytes
不修改；prepared snapshot 保留输入解释及不可变性。未声明来源的离线输入保持原
保守语义；P2A 原始二值输入与历史结果不因本修复变成失效证据。

新失败日志保留原统计前缀，并区分 `start=free/blocked/outside_map`、`goal=...`、
原格代价，附该次请求坐标、原点、尺寸、分辨率、radius/clearance 和输入模式。
仍可能由于真实障碍、未知区、外接圆模型或格余量拒收目标，遇到这些结果须继续分析，
不能删检查、缩 footprint、移目标或把未知区改为自由。

## 验证步骤

先核对修复提交、当前目录与旧证据。不要 checkout 主实车工作区或删除构建目录。

```bash
cd /home/wpie/worktrees/rm2027_tdt_phase2
git branch --show-current
git log -1 --oneline
git status --short
pkg="$PWD/experiments/tdt_planner/rm_tdt_planner"
work="$PWD/build/tdt_p2b"
export P2B_SERIES=static_v3
set -o pipefail
mkdir -p "$work/handoff_logs"
logs=$(mktemp -d "$work/handoff_logs/costmap_semantics_20260913.XXXXXX")
printf '%s\n' "$logs"
```

相关源码必须与交付版本一致；`static_v3` 若已存在，先核查执行记录与版本，不覆盖
或换名掩盖失败。用清单只读检查旧证据：

```bash
python3 - <<'VERIFY'
import hashlib, json
from pathlib import Path
root=Path.cwd()
m=json.loads((root/'docs/tdt_migration/evidence/costmap_semantics_20260913/preserved_static_v2.json').read_text())
for name, expected in m['files'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Original evidence changed: '+name)
print('Original static_v2 evidence preserved:', len(m['files']))
VERIFY
```

依次执行以下步骤，每步记录命令、起止时间、退出码和日志；失败即停止依赖步骤：

```bash
bash "$pkg/tools/p2b_validation.sh" build 2>&1 | tee "$logs/build.txt"
bash "$pkg/tools/p2b_validation.sh" check 2>&1 | tee "$logs/tests.txt"
bash "$pkg/tools/p2b_validation.sh" sanitizers 2>&1 | tee "$logs/sanitizers.txt"
bash "$pkg/tools/p2b_validation.sh" profiles 2>&1 | tee "$logs/profiles.txt"
```

- 依赖沿用固定前缀，不需重新下载。常规构建包含 6 个仿真包与实验包。
- CTest 预期 **4 入口、45 用例**：core 24（新增 5）、plugin 4（新增 1）、
  工具 13、ROS 消息 4。新增测试覆盖重复膨胀、全段扫掠圆独立 oracle、孤立 253
  禁区、实体/未知/边界余量、prepared 等价/不可变/footprint 不匹配、诊断和插件接线。
- 本轮修改了核心碰撞输入处理，需运行新增 `sanitizers` 入口：在
  `build/tdt_p2b/costmap-semantics-sanitizer/` 独立 Debug 构建，预期 24 个核心
  GTest 通过且无 ASan/UBSan/LSan 报告。本地 core/vendor 被编译器插桩；已安装
  求解器依赖未重新插桩。它不包含 ROS 插件、仿真或系统实时性验收。
- 四 profile/场景/控制参数不变，哈希应与 static_v2 一致。若不符应停止并报告，
  不覆盖旧 profiles。现有 3,200 次 P2A 比较无需无理由重跑；45 和 24 均是待验预期。

然后按 [完整交接](p2b_terra_handoff.md) 的相同静态门，逐条执行四组首例，读完每次
结果再进入下一次，不先批量循环：

```bash
bash "$pkg/tools/p2b_validation.sh" run navfn 1
bash "$pkg/tools/p2b_validation.sh" run smac2d 1
bash "$pkg/tools/p2b_validation.sh" run tdt_astar 1
bash "$pkg/tools/p2b_validation.sh" run tdt_qp 1
```

首例通过且人工 TF、costmap、SVG 复核无问题的方案，才各补第 2–5 次。所有新矩阵
试次必须来自同一修复提交；static_v2 的 10 次基线通过可以引用为历史证据，但不填进
static_v3 的完整矩阵。记录原车体间隙至少 0.05 m、padded footprint 无接触、终点
位置/yaw、恢复次数、停止命令，完整静态条件不变。

若 T-DT 继续失败，停止该组重复，并返回完整新端点诊断及前后 costmap、路径、命令。
如怀疑 snapshot 变化，应单独数 `costmap or footprint changed`，不混计端点拒收。
单次拒收出现不等于该安全门有缺陷；持续拒收的功能结果需单列。明确的设备性能问题
只归档待新设备验证，不修改参数、不判方案淘汰。

无论完整或中断，保存新汇总：

```bash
bash "$pkg/tools/p2b_validation.sh" summarize 2>&1 | tee "$logs/summarize.txt"
```

如需复查历史端点数值，可在主机把以下输出写到新日志目录；工具只分析旧记录：

```bash
python3 "$pkg/tools/inspect_simulation_endpoints.py" "$work/runs/static_v2/tdt_astar_1/observation" \
  --radius 0.43278170016764805 --clearance 0.02 --output "$logs/tdt_astar_endpoints.json"
```

用 [回传模板](p2b_result_template.md) 创建
`docs/tdt_migration/evidence/p2b_static_v3_<实际日期>/validation_<实际日期>.md`。
写实际执行模型、HEAD、45/24 计数、四配置哈希、运行矩阵、aggregate 路径/SHA、
端点与更新拒收分项、未覆盖内容。原 static_v2 中 LIO teardown 的 -11 另案保留，
不得归因规划算法或擅自改定位；若再次出现应保存同次时间顺序与日志。

设计者本轮只做源码审阅、历史数据分析、静态语法与证据哈希核对，记录见
[authoring_checks.json](evidence/costmap_semantics_20260913/authoring_checks.json)。
未执行新编译、测试、sanitizer、Gazebo 或实车。本次继续停留 P2B，不宣布部署验收。

## 直接交给 Terra 的任务

> 在 `/home/wpie/worktrees/rm2027_tdt_phase2` 核对用户提供的修复提交后，执行
> `docs/tdt_migration/p2b_costmap_semantics_handoff.md`。保留 static_v1/v2，用
> static_v3。你负责验证/回传，不改源码、断言、目标、footprint、超时或安全门。
> 依次增量构建、45 项常规测试、24 项核心 sanitizer、四 profile 核对，之后逐组
> 首例和按门重复。失败记录精确端点日志、costmap 与停止/恢复结果。设备性能问题
> 只记录待新设备；不做移动障碍、实车、push/merge。新报告明确实测与未覆盖部分。
