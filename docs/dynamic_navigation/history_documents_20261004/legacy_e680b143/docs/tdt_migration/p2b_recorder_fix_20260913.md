# P2B 记录器修复复核与 static_v2 重验交接

历史交接：记录器修复已在 e85b076/static_v2 验证通过，本文保留原交接预期。
当前执行入口为 [代价语义修复交接](p2b_costmap_semantics_handoff.md)。
修复基线 `b1c40e68f08771cec763d6bc30ff5fa7072365a8`，分支
`experiment/tdt-planner-phase2`。修复交接提交号以设计者交付的最终提交为准。
本文记录源码复核与重验要求，不是新的运行通过报告。

## 对回传结果的复核

已读取持久目录中的原始 observer、launch、构建与测试日志。认可原版本构建成功、
19 个 core + 3 个插件 + 13 个工具测试通过，以及共享记录器失败后停止全部仿真的
处置。没有可用导航轨迹或 action 结果，四个规划器的静态结论均未建立。

[原始回传报告](evidence/p2b_static_20260913/validation_20260913.md) 中“首个 costmap
回调出错”和 `map_cb` 行号定位需要更正。旧 `observer.log` 与 `summary.json` 只有
异常字符串和 `simulation_clock=0.0`，没有 traceback；五个流均为空，不能证明收到过
costmap，也不能仅靠空流判断哪个回调先运行。

只读检查与回传相同的 Humble 镜像发现：

| 实际安装定义 | Python 类型与本问题关系 |
| --- | --- |
| `rcl_interfaces/msg/Log.msg` 的 `uint8 level` | 生成的 `Log.level` setter 接受 `int` |
| 同一消息的 `byte WARN=30` | 生成的 `Log.WARN` 为 `b'\x1e'` |
| 原 `log_cb` 的 `msg.level >= Log.WARN` | 比较 `int` 与 `bytes`，与原日志错误方向完全一致；INFO 等低等级消息同样触发比较 |
| `nav_msgs/OccupancyGrid.data` 的 `int8[]` | 生成绑定为 `array.array('b')`，迭代元素是整数；未知值 `-1` 有意义 |
| `rclpy.logging.LoggingSeverity` | 使用 rcutils 等级的 `IntEnum`，可与消息整数等级比较 |

因此，本次修正的是源码中明确存在的 **rosout 日志等级类型不匹配**。原日志未保存
调用栈，历史回调位置无法直接追溯；不能把源码推断写成已经取得旧运行栈，也不能
声称本轮已复现成功。已保存安装文件路径、SHA256 与片段，见
[类型定义证据](evidence/recorder_fix_20260913/installed_ros_source.json)。
本轮仅读取镜像内源码，没有执行 ROS 消息回调、单测或 Gazebo。

## 修改与覆盖范围

- 记录器改用 `LoggingSeverity.WARN` 比较，继续保留 WARN 及以上事件。
- 捕获运行异常时，`observer.log` 保留完整 traceback，`summary.json` 新增
  `error_type`、`traceback`，明确两个通过标志均为 false，仍返回退出码 2。
- 新增 `simulation_ros_messages` CTest 入口，共 4 个 unittest，使用实际安装的
  ROS 消息与 serialize/deserialize 往返，再调用生产回调和 JSON 写入逻辑。
  测试自身不创建 DDS 节点或 Gazebo；主入口异常测试仅替换 ROS 初始化与节点构造。
- 4 例分别覆盖时钟/costmap 之前的低等级日志、WARN/ERROR/FATAL 等级与 JSON
  留存、costmap 有符号未知值及占用阈值、回调异常的调用栈/无效状态/退出码/清理。
- 默认 series 改为 `static_v2`，已有目录拒绝覆盖的规则继续生效。

没有必要把 costmap 转成无符号字节；其回调及 `-1` 未知值语义保持原样。算法、
MPPI、场景、目标、footprint、0.05 m 间隙门、超时和四 profile 均沿用原值。
35 项原测试没有覆盖 ROS 生成消息与真实 observer 回调边界，这是本次覆盖缺口。
新增测试只覆盖记录器边界，不代表 readiness、TF、墙体感知或导航验收通过。

## 1. 核对版本和旧证据

后续全部操作在主机持久实验 worktree 内进行：

```bash
cd /home/wpie/worktrees/rm2027_tdt_phase2
git branch --show-current
git log -1 --oneline
git status --short
pkg="$PWD/experiments/tdt_planner/rm_tdt_planner"
work="$PWD/build/tdt_p2b"
export P2B_SERIES=static_v2
set -o pipefail
mkdir -p "$work/handoff_logs"
logs=$(mktemp -d "$work/handoff_logs/recorder_fix_20260913.XXXXXX")
printf '%s\n' "$logs"
```

确认 HEAD 是设计者交付的修复提交，相关源码无未提交改动。若 `static_v2` 已存在，
先审查该轮已执行内容；不得覆盖、删除或在不核对版本的情况下补跑。
`static_v1` 失败目录和原回传报告的字节/哈希记录在
[原证据清单](evidence/recorder_fix_20260913/preserved_static_v1.json)，可只读核对：

```bash
python3 - <<'VERIFY'
import hashlib, json
from pathlib import Path
root = Path.cwd()
manifest = json.loads((root/'docs/tdt_migration/evidence/recorder_fix_20260913/preserved_static_v1.json').read_text())
for name, expected in manifest['files'].items():
    data = (root/name).read_bytes()
    if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
        raise SystemExit('Old evidence changed: ' + name)
print('Original static_v1 evidence and returned report preserved')
VERIFY
```

如果哈希不符，保留差异并回传，不改 manifest 来获得通过。原始报告按当时认识保留，
后续纠正只写在新报告中。

## 2. 增量构建、39 项测试与配置核对

依赖已经存在且原构建成功，无新外部依赖；先复用现有固定前缀。逐条执行并记录
退出码；任一步失败即停止其依赖步骤。无需重下载求解器或重跑 P2A 3,200 次比较。

```bash
bash "$pkg/tools/p2b_validation.sh" build 2>&1 | tee "$logs/build.txt"
bash "$pkg/tools/p2b_validation.sh" check 2>&1 | tee "$logs/tests.txt"
bash "$pkg/tools/p2b_validation.sh" profiles 2>&1 | tee "$logs/profiles.txt"
```

预期构建成功，CTest **4 个入口、39 个用例**：19 core、3 plugin、13 工具、
4 ROS 消息记录器用例。必须看到 `simulation_ros_messages` 及其 `Ran 4 tests`；
仍只有旧 35 项说明新 CMake 注册未进入构建。新测试导入/序列化/回调失败，均按
共享记录器或依赖问题停止；不可跳过这组、修改断言或在主机伪造 ROS 模块通过。
39 是本修复版的预期值，不是设计者本轮实测值。

四 profile 应仍与生成器和基线一致，哈希与原回传一致。若不符，记录差异后停止；
不能覆盖旧 profile。所有日志在新 `$logs` 目录，不覆盖原验证日志。

## 3. 只先重验 Navfn 首例

```bash
bash "$pkg/tools/p2b_validation.sh" run navfn 1 2>&1 | tee "$logs/navfn_1.txt"
trial_dir="$work/runs/$P2B_SERIES/navfn_1"
cat "$trial_dir/observation/summary.json"
cat "$trial_dir/observer.log"
```

运行期间按 [完整交接第 4 节](p2b_terra_handoff.md#4-每组第一次静态试验) 在另一个
终端记录该容器的 node list、`map -> odom` TF 和必要资源观测。检查：

1. 不再出现整数与 bytes 的等级比较错误，启动告警能写入 `events.jsonl`。
2. 若新异常出现，回传 `error_type`、完整 traceback、失败回调和最早有效错误；
   没有调用栈时保留“位置未知”，不能只凭最后日志或静态行号给出历史定位。
3. costmap 流实际存在且内容有效，未知值仍为 -1；确认两条墙的 spawn 与实际
   感知、canonical TF 前提。只看到 spawn exit=0 不足以通过这项。
4. 分别报告预检查 action、导航 action、trajectory/plans/commands、停止及几何门。
   记录器不再崩溃不等于 Navfn 成功，更不等于其他组通过。

共享记录器/启动/输入错误出现时再次停止全部仿真。若只有该规划器的功能或安全
失败，停止该组重复，其他独立组仅按完整交接条件继续首例。明确的设备性能限制
只记录为待新设备验证，不修性能参数、不淘汰路线。

## 4. 按条件继续四组并回传

只有上一步证明共享链路可用后，逐条运行另外三组首例；人工复核后才给通过组补
编号 2–5。命令、停止条件、0.05 m 间隙门、SVG/costmap 审查及统计口径继续遵守
[完整交接](p2b_terra_handoff.md)。每次单独读日志，不先批量发起四组。

```bash
bash "$pkg/tools/p2b_validation.sh" run smac2d 1
bash "$pkg/tools/p2b_validation.sh" run tdt_astar 1
bash "$pkg/tools/p2b_validation.sh" run tdt_qp 1
# 首例有效且人工核对通过的组，才按完整交接补 2–5。
```

完整或中断均保留本轮汇总：

```bash
bash "$pkg/tools/p2b_validation.sh" summarize 2>&1 | tee "$logs/summarize.txt"
```

部分矩阵的汇总退出 1 是不完整状态，保留 JSON 并照实报告。新回传目录使用
`docs/tdt_migration/evidence/p2b_static_v2_20260913/`（若跨日，用实际日期）；复制
[结果模板](p2b_result_template.md) 填写，记录实际执行模型、提交、日志路径/SHA、
39 项测试计数、首例与后续矩阵、旧证据核对结果、新异常调用栈以及未覆盖项。
不得混入 b1c40e6/static_v1 的试次，也不能宣称其已有 35 项通过验证了当前修复。

设计者本轮只进行源码审阅、AST/脚本语法及改动范围核对；其记录见
[修复编写检查](evidence/recorder_fix_20260913/authoring_checks.json)。原
`p2b_authoring_checks.json` 属于 b1c40e6 的历史编写记录，不用它判定修复文件损坏。

## 可直接交给 Terra

> 请核对用户提供的修复提交，在 `/home/wpie/worktrees/rm2027_tdt_phase2` 执行
> `docs/tdt_migration/p2b_recorder_fix_20260913.md`，再按完整交接继续静态验证。
> 原报告的 costmap 回调定位已被设计者更正为 rosout 等级类型问题；保留原报告和
> static_v1，用 static_v2 重验。你负责执行和回传，不修改源码、断言或安全参数。
> 先增量构建、39 项测试、四 profile 核对，再单独 Navfn 首例，确认共享链路可用后
> 才按门继续其余组及重复。失败保留 traceback 和原始数据；不以设备性能否决方案。
> 新报告明确哪些实测通过、哪些失败或未覆盖。全程文件存 /home，不做移动障碍、
> 实车、push 或 merge。返回报告、aggregate、日志路径和实际 HEAD 给设计者复核。
