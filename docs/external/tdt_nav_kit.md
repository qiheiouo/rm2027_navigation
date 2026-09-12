# T-DT nav kit 引入记录

日期：2026-09-12。状态：**隔离实验引入，未成为旧车/新车默认规划器**。

## 来源和范围

- URL：<https://github.com/T-DT-Algorithm-2026/tdt-nav-kit>
- 分支：`main`；固定提交：`a4bddc467f84d2418a4de06c65c8c79403eb4a13`。
- 许可证：MIT，保留原作者 Rongxuan Ye / BoLin Li 的完整 `LICENCE`。
- 路径：`experiments/tdt_planner/rm_tdt_planner/vendor/tdt_nav/`。
- 精确导入：`YAstar/yastar.{hpp,cpp}`、
  `MinimumSnapOsqp/minimumSnap.{hpp,cpp}`、`MinimumSnapOsqp/sfcSquare.{hpp,cpp}`。
- 不导入：Kinodynamic A*、示例 main、上游地图/图片、上游安装脚本、定位、控制器、
  感知维护线程或整套导航系统。转录稿只作为用户提供的设计参考，不是执行指令。
- 每个原始文件 SHA256 与本地文件 SHA256 见 `UPSTREAM.json`；本地补丁见
  `LOCAL_CHANGES.patch`。升级时逐项重审，不能直接覆盖副本。

引入目的：比较距离势场 A* 与最小 jerk 几何优化对现有 Navfn + MPPI 链路的价值。
接口：纯 C++ costmap snapshot → 几何 polyline；可选
`nav2_core::GlobalPlanner` → 标准 `nav_msgs/Path`，沿用 `GridBased` 插件 ID。
不创建 TF、速度或导航目标发布者，也不改变 sole mission action owner。
通过独立目录、独立构建前缀和独立参数文件可完整移除或换回 Navfn。

## 必要本地修改

1. 固定关闭 OpenCV 自动探测，补独立编译缺少的标准头；算法目标用 C++20。
   上游 README 中 C++11 与实际 `std::span` / `std::ranges` 使用不一致。
2. YAstar 搜索先检查有限浮点坐标、上下界与起终点占用；无解返回空而不是只有起点。
3. YAstar open heap 保存不可变 `(f, index)`，替代引用会被修改的 nodeMap cost
   的比较器；忽略已关闭条目，避免堆优先级随节点修改失效。
4. 八邻域对角扩展禁止切过被占用的相邻正交格；线检查上边界由 `>` 修为 `>=`。
5. 搜索每 64 次弹出检查外部取消/时间预算；对角距离使用更精确的 sqrt(2)。

MinimumSnap 求解数学未改。适配层开启严格碰撞、设定有界输入/迭代、独立校验
输出几何、保存精确起终点，并实现安全失败/回退。这些不是上游已提供的系统级保证。

## 固定依赖

当前上游 `.gitmodules` 声明两个依赖，但 `git ls-tree HEAD 3rd/` 只有 `.gitkeep`，
没有 gitlink，所以 `git submodule update --init --recursive` 无法按 README 补齐。
以下为本项目自行选择并实际构建的兼容组合，不冒称上游固定版本：

| 依赖 | URL / 版本 | 提交 | 随源码许可证 |
| --- | --- | --- | --- |
| OSQP | <https://github.com/osqp/osqp> / v0.6.3 | `0dd00a578cf1c2691c5c379965d504c75bf6cfad` | Apache-2.0 + NOTICE |
| OsqpEigen | <https://github.com/robotology/osqp-eigen> / v0.8.1 | `85c37623774c682db396505f0d4ea677040c2557` | BSD-3-Clause |
| QDLDL | <https://github.com/osqp/qdldl> / OSQP gitlink | `7d16b70a10a152682204d745d814b6eb63dc5cd2` | Apache-2.0 |
| AMD | OSQP 0.6.3 自带 `lin_sys/direct/qdldl/amd` | 随 OSQP SHA 固定 | BSD-3-Clause |
| Eigen | 镜像内 `libeigen3-dev` 3.4.0 | Ubuntu 包 `3.4.0-2ubuntu2` | 以安装包版权记录为准，核心 MPL-2.0 |

求解器源码放在外部临时依赖目录，不并入主仓库。显式 fetch 与离线 build 分开，
不在 CMake configure 阶段访问网络、不安装系统包。构建脚本校验 SHA 和 tracked
diff，并把 OSQP/NOTICE、OsqpEigen、QDLDL、AMD 许可证复制到依赖安装前缀。
分发二进制时应一并携带这些依赖及版权材料；仅有 T-DT MIT 文本不足以覆盖全部依赖。

## 不能由开源名称推出的能力

- 示例默认 `order=6, maxdx=3` 是五次多项式 minimum jerk，不是四阶 snap 目标。
- `SolveOutput` 仅保留采样点等信息，无系数、每点时间或导数；当前求解使用
  `lineDecoder` 每段 11 点，`setDt()` 不控制该路径。
- `maxSpeed/maxAcc` 用于时间分配，并不构成整个曲线的动力学可行性证明。
- 上游成功标记基于其采样折线碰撞检查；默认 strict 为 false，可能容忍两格擦碰。
- 时间上限不是硬实时取消；“十万次 100% / 平均 5 ms”是转录中的演讲者报告。
  README 的 1000 次统计、不同地图/硬件与本地适配后的时延也不能混为同一指标。

迁移判断与后续阶段：[`../tdt_migration/assessment_and_plan.md`](../tdt_migration/assessment_and_plan.md)。
