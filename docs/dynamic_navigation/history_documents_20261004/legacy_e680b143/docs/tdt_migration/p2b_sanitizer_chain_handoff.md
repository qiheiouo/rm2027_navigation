# P2B：sanitizer 构建链修复与 static_v4 交接

状态：**74f68e2 的 static_v3 已中断并复核；构建链修复及边界回归已编写，待 Terra 验证。**
本轮开发基线 `74f68e2d6e6957dfbdc64542165acbbb6f740f41`，分支
`experiment/tdt-planner-phase2`；实际交付提交与设计者最终回复核对。
全部工作继续保存在 `/home/wpie/worktrees/rm2027_tdt_phase2`。

## 已有结果与归因依据

[static_v3 真实回传](evidence/p2b_static_v3_20260914/validation_20260914.md)：
构建、45/45 常规测试、四 profile 核对通过；sanitizer 两次退出 8，在第一个
`Planner.OpenMapActuallyRunsCorridorBackend` 中止；没有任何 static_v3 仿真试次。
Navfn/Smac2D 在 static_v2 的 5/5 仍只是 e85b076 的历史结果。

| 已核对证据 | 含义 |
| --- | --- |
| ASan 在 `handmade_aligned_free` 读取已分配块前 8 字节 | 释放方正在查找手工对齐分配保存在指针前的原始地址 |
| 144 字节分配来自原 `/work/deps/lib/libOsqpEigen.so.0.8.1` 的 `getSolution()` | 分配在未插桩的 Release 库内；释放在调用方隐式 `Solver` 析构函数中 |
| 原库 flags 为 `-O3 -DNDEBUG -fPIC`，原 core 为 Debug + ASan/UBSan | 先前入口只给 core/vendor 插桩，复用了 Release 求解器 |
| 同一镜像中只做预处理：Release `EIGEN_MALLOC_ALREADY_ALIGNED=1`，ASan 为 `0` | 跨库分配/释放所采用的 Eigen 内存布局存在实际差异 |

镜像仍为 `rm2027_navigation:humble`，ID
`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。
镜像内 `Eigen/src/Core/util/Memory.h` 的 GLIBC 判断明确排除 `__SANITIZE_ADDRESS__`；
1 分支直接使用 malloc/free，0 分支使用带前置指针的手工对齐分配/释放。
固定上游说明见 [Eigen 3.4.0 Memory.h](https://gitlab.com/libeigen/eigen/-/raw/3.4.0/Eigen/src/Core/util/Memory.h)。
本地固定 OsqpEigen 的 `src/Solver.cpp:168` 在库内给成员 `m_solution` 赋值，
`Solver.hpp` 未提供库外定义的析构函数，正好形成此边界。

**先前由设计者提供的混合 sanitizer 构建入口需要修正。** 这是构建语义问题，
不属于当前设备性能问题。ASan 所报告的越界读取是真实失败证据；常规通过不能把它
判为误报。现有证据足以优先统一分配边界，但新构建链是否消除该失败、后续是否还有
错误，仍要靠 Terra 实测。没有修改 MinimumSnap 来绕开析构或放弃后端。

原始首次/重试 ASan 日志另存入
[首次](evidence/sanitizer_chain_20260914/static_v3_sanitizers.txt)、
[重试](evidence/sanitizer_chain_20260914/static_v3_sanitizers_retry.txt)。
[保护清单](evidence/sanitizer_chain_20260914/preserved_inputs.json) 含 238 项：原
static_v2 的 166 项、static_v3 日志/空 aggregate/报告、旧失败构建的关键文件与
原 Release 求解器安装前缀。保留这些文件与全部旧 series，不覆盖旧失败。

## 改动范围

- `build_dependencies.sh` 新增显式 `sanitizers` 模式；原三参数 Release 用法保持。
  校验 OSQP/OsqpEigen/QDLDL 固定 SHA 和 tracked diff 后，用离线 git archive 创建
  独立受控源码快照，避免 OSQP 生成头文件改变原 `deps_source`。不联网、不升级依赖。
- `p2b_validation.sh sanitizers` 每次生成独立
  `build/tdt_p2b/sanitizer_runs/chain_<UTC时间>.<随机值>/`，旧源码和 Release 前缀
  在此容器中只读。新路径记录 session.log、exit.txt、source_commit.txt、image_id.txt。
- `run_core_sanitizers.sh` 用同一镜像/GCC，在独立前缀中以 Debug、ASan/UBSan 构建
  OSQP（含 QDLDL/AMD）、OsqpEigen 和本地 core/vendor/test；保持 ASan 泄漏检查与
  UB 首错中止。不强制 Eigen 对齐宏、不关 SIMD、不屏蔽 sanitizer 或跳过首例。
- 新 `audit_sanitizer_chain.py` 读取三份 compile_commands，核对每个编译单元的插桩
  参数；按实际 C++ 编译参数预处理 Eigen，核对分配/对齐宏；检查 ldd 中两份求解器
  动态库来自新前缀，并含 ASan/UBSan 符号。失败写 `chain_audit.json`，停止测试。
- 新 `SolverBoundary.SolutionStorageSurvivesLibraryAndCallerLifetime` 用已知解析解的
  17/18/33 维对角凸 QP，覆盖库内取原/对偶解、调用方复制和 Solver 析构；18 维对应
  原 144 字节向量。它不涉及地图或 MinimumSnap，便于单独观察跨库生命周期。
- ROS、系统库及已安装 GTest 没有重新插桩；本入口是求解器/core 安全检查，不能作为
  ROS 全负载验收。普通仿真仍使用原 Release 依赖，不混用 sanitizer 前缀。
- 本轮 core、Nav2 插件、vendor 算法、costmap 语义、footprint/clearance、MPPI、场景、
  目标及超时均未改。保留 74f68e2 的代价语义修复，静态导航能否通过仍未知。

## Terra 操作步骤

先核对交付提交；若有源码偏离，停止依赖该版本的执行并回传。不要切换主实车工作区。

```bash
cd /home/wpie/worktrees/rm2027_tdt_phase2
git branch --show-current
git log -1 --oneline
git status --short
pkg="$PWD/experiments/tdt_planner/rm_tdt_planner"
work="$PWD/build/tdt_p2b"
export P2B_SERIES=static_v4
set -o pipefail
mkdir -p "$work/handoff_logs"
logs=$(mktemp -d "$work/handoff_logs/sanitizer_chain_$(date -u +%Y%m%dT%H%M%SZ).XXXXXX")
printf '%s\n' "$logs"
```

记录这个新日志路径；如 static_v4 已有试次，先核对其来源，禁止覆盖或换名掩盖失败。
先查保护清单，结束时再执行同一段核对：

```bash
python3 - <<'VERIFY'
import hashlib, json
from pathlib import Path
root=Path.cwd()
manifest=json.loads((root/'docs/tdt_migration/evidence/sanitizer_chain_20260914/preserved_inputs.json').read_text())
for name, expected in manifest['files'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Preserved input changed: '+name)
print('Preserved inputs:', len(manifest['files']))
VERIFY
```

逐条执行，核对每条退出码再进入依赖步骤；无需运行 deps 重新下载/覆盖 Release 依赖：

```bash
bash "$pkg/tools/p2b_validation.sh" build 2>&1 | tee "$logs/build.txt"
bash "$pkg/tools/p2b_validation.sh" check 2>&1 | tee "$logs/tests.txt"
bash "$pkg/tools/p2b_validation.sh" sanitizers 2>&1 | tee "$logs/sanitizers.txt"
bash "$pkg/tools/p2b_validation.sh" profiles 2>&1 | tee "$logs/profiles.txt"
```

| 检查 | 必须回传的实际证据 |
| --- | --- |
| 常规 CTest | 4 入口、预期 46/46：core 25、plugin 4、工具 13、ROS 消息 4 |
| 独立 sanitizer 构建 | 新目录、固定依赖 SHA、三个 CMakeCache/compile_commands、镜像/提交 |
| 链审计 | `chain_audit.json` 的 passed=true、各 Eigen 分配宏为 0 且对齐宏一致；ldd.txt 两份库均在新前缀 |
| 插桩与加载 | 两份库各自 SHA、ASan/UBSan 符号存在；不能加载 `/work/deps/lib` 旧库 |
| 边界回归 | `solver_boundary.log` 中 1 项通过，无 ASan/UBSan/LSan 报告 |
| 全套核心 sanitizer | `core_tests.log` 中 25/25 通过，无报告；其中已包含上述边界项，不计成 26 个不同用例 |
| 四 profile | 与 static_v2、static_v3 相同的四 SHA；verify 不覆盖原配置 |

若构建链核对失败，先归为构建/记录问题；若审计通过后仍出现 sanitizer 报告，保存
首个有效堆栈与加载库信息，停止全部仿真。不要只重试到成功，不改链接参数、断言、
内存对齐宏或屏蔽报告。原失败已有两次，无需再跑旧混合构建来补数量。

上述全部通过后，按 [完整静态规范](p2b_terra_handoff.md) 逐组跑首例，每次先读结果：

```bash
bash "$pkg/tools/p2b_validation.sh" run navfn 1
bash "$pkg/tools/p2b_validation.sh" run smac2d 1
bash "$pkg/tools/p2b_validation.sh" run tdt_astar 1
bash "$pkg/tools/p2b_validation.sh" run tdt_qp 1
```

首例与人工 TF/costmap/SVG 审查通过的组才补 2–5。共享基础设施失败停全部；组内功能
或安全失败停该组重复。终点、无恢复、停止命令、车体间隙至少 0.05 m 等门不变。
若 T-DT 失败，分别回传精确端点诊断与 snapshot 更新拒收；不移动目标或缩 footprint。
明确设备性能问题只归档待新设备，不专项优化或淘汰方案。LIO teardown 的 -11 另案记录。

无论通过或中断，都保存新汇总和报告：

```bash
bash "$pkg/tools/p2b_validation.sh" summarize 2>&1 | tee "$logs/summarize.txt"
```

按照 [回传模板](p2b_result_template.md) 写
`docs/tdt_migration/evidence/p2b_static_v4_<实际日期>/validation_<实际日期>.md`。
空矩阵/不完整矩阵退出 1 是未完成状态，不是规划算法失败。不要把静态历史通过填进
新矩阵，不宣布部署验收、不执行移动障碍/实车、不要 push/merge。

## 设计者实际完成与未执行

实际完成：源码/堆栈与构建参数审阅；同镜像只读头文件和预处理宏比较；脚本静态语法、
改动范围和旧证据哈希核对。编译宏比较没有生成或执行测试程序。
未执行：新构建、链审计工具的构建后运行、新 GTest、sanitizer、Gazebo 或实车。
详细静态记录见 [authoring_checks](evidence/sanitizer_chain_20260914/authoring_checks.json)。

## 可直接交给 Terra

> 在 `/home/wpie/worktrees/rm2027_tdt_phase2` 核对设计者交付提交后，执行
> `docs/tdt_migration/p2b_sanitizer_chain_handoff.md`。保留全部 static_v1/v2/v3，
> 用 static_v4。先增量构建、46 项常规测试，使用新 sanitizer 入口重建独立求解器链，
> 核对 chain_audit 和 ldd，确认边界 1 项及包含它的核心 25 项通过，再逐组静态首例。
> 失败保留原始栈/路径/退出码并按门停止；不修改源码、参数、断言、对齐或安全门，
> 不因设备性能淘汰算法，不做移动障碍、实车或 push/merge；最后按模板回传实际结果。
