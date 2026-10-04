# T-DT 规划候选：2026-09-12 离线验证

结论：**规划前后端隔离集成与自动化回归通过；当前证据不足以替换 Navfn/MPPI
部署基线。** 未做 Gazebo 导航闭环、录包运行负载回放或实车验收。

## 可复核输入

- 项目基线 `46c521b`，实验分支 `experiment/tdt-planner-intake`。
- 上游 `a4bddc467f84d2418a4de06c65c8c79403eb4a13`，精确范围/依赖/补丁见
  [`../external/tdt_nav_kit.md`](../external/tdt_nav_kit.md)。
- 机器：Intel Core i7-10710U；镜像 `rm2027_navigation:humble`。
  GCC 11.4.0、Eigen 3.4.0、Nav2 Humble `1.1.20-1jammy.20260607.*`。
- 容器 `--network none`、UID 1000、工作区只读、无设备挂载；ROS_DOMAIN_ID=173，
  ROS_LOCALHOST_ONLY=1。仅依赖构建目录允许写入生成头文件。
- 当前实验文件 SHA256：[`evidence/implementation_sha256.json`](evidence/implementation_sha256.json)。
- 用户定位稿 SHA256：`1224d5defc4a776ee3676fbdee99fdc4f95a6c0d4abb159c8295b3c0d3afdb0b`；
  导航稿：`68abeba991a908f0e0ddc4b79737c2a2b11e75cd6948396621ec84629cb116ed`。
  稿件原文件未改、未复制进仓库。

## 构建和回归

| 检查 | 结果 |
| --- | --- |
| OSQP 0.6.3 / OsqpEigen 0.8.1 固定 SHA 校验与离线构建 | PASS；仓库脚本实际执行，5 份依赖版权/NOTICE 已复制 |
| 常规 `colcon list --base-paths /ws` | 23 个现有包，不含实验包 |
| 显式指定实验 package 的 colcon 构建 | PASS，1 个 package；不依赖日常 install |
| 算法/适配核心 | **17 tests passed** |
| pluginlib 实际加载、Humble API、生命周期与基准 | **3 tests passed** |
| ASan + UBSan + leak detection 核心测试 | **17 tests passed**；使用匹配 ASan OsqpEigen |
| 上游补丁可应用、逐文件原始/本地 SHA | PASS |
| 新完整 profile 生成 | PASS，仅 GridBased 配置变化；保留 controller/costmap/BT |
| 覆盖同名输出负例 | PASS，拒绝写入；包括不覆盖用户 baseline 的语义 |

证据：[ROS2 测试全文](evidence/tests_ros2_20260912.txt)、
[内存检查测试全文](evidence/tests_sanitized_core_20260912.txt)。CTest 显示 2 个
test executable，内部是 17 + 3 个 GTest case，不能把“2 tests”误报为只有 2 个场景。

核心场景包括：非零/负地图原点、精确起终点、同格/同点、完整墙无解、未知和
inscribed cost、窄口外接圆拒绝、NaN/非法尺寸/参数、边线终点、超时拒收、
稀疏输出跨障碍检查、地图变化后不复用旧路径、关闭后端、控制点超预算回退、
真实 QP/几何拒收回退、20 组确定性随机障碍及独立几何距离 oracle。
上游修补另用 10 组确定性场景与独立 Dijkstra 最短代价结果对照。

ASan 第一次混用了 release OsqpEigen 和插桩调用者。Eigen `Memory.h` 根据
`__SANITIZE_ADDRESS__` 选择不同的 aligned malloc/free 分支，导致跨库释放模式
不匹配。重新构建同模式 OsqpEigen 后通过；没有屏蔽该错误或关闭检测。
ASan 覆盖本地 core、导入 T-DT 与 OsqpEigen；OSQP C 库保持原 release 构建，
这不是对所有第三方依赖的完整 sanitizer 审计，也不是 ROS 插件的 sanitizer 验收。

## 同一静态 snapshot 的微基准

100×80、0.1 m 栅格；全向矩形 footprint，由 Costmap2DROS 提供 padded footprint；
测试没有运行局部控制器。两场景为开放区域与带软膨胀 cost 的墙体绕行。
每个方案每场景重复 20 次，共 120 次调用，所有调用均返回路径。
Navfn 为安装版本默认 Dijkstra，T-DT 分为 optimize=false / true。

| 场景 | 方案 | 成功 | P50 ms | P95 ms | 最大 ms | 平均路径长度 m |
| --- | --- | --- | --- | --- | --- | --- |
| 开放 | Navfn Dijkstra | 20/20 | 0.217 | 0.298 | 0.303 | 7.397 |
| 开放 | T-DT A* | 20/20 | 1.099 | 1.423 | 1.545 | 7.548 |
| 开放 | T-DT A* + QP | 20/20 | 1.283 | 1.548 | 2.540 | 7.456 |
| 绕墙 | Navfn Dijkstra | 20/20 | 0.295 | 0.422 | 0.817 | 11.009 |
| 绕墙 | T-DT A* | 20/20 | 1.734 | 2.115 | 2.563 | 13.027 |
| 绕墙 | T-DT A* + QP | 20/20 | 2.259 | 2.775 | 2.957 | 12.072 |

原始每次数据：[`evidence/benchmark_20260912.csv`](evidence/benchmark_20260912.csv)。
P95 使用 20 个样本中的第 19 个值，不用如此小的样本估计可靠 P99。
QP 模式具有安全回退；CSV 记录的是选择模式，不包含每次优化采用标志。
核心测试独立证明了至少开放场景确实采用后端输出，并证明回退场景有效。

解释：QP 在这两幅图上相对 T-DT A* 缩短了折线，仍比 Navfn 慢且路径更长。
这不能推出 Navfn 在所有任务更好，也不能宣称 T-DT 已胜过现有系统：T-DT 包括
保守 footprint 再膨胀、独立线段检查、额外 snapshot 检查，并且重建势场后没有保留
Navfn 输入中的所有软 cost；两者并非完全相同的碰撞/代价问题。
本轮没有测实际横向跟踪误差、加速度/jerk、最小执行间隙、全负载 CPU/RSS 或长期
规划成功率，也没有复制分享稿的“十万次”实验。

## 保留的限制和下一门

1. 仅验证几何折线，未恢复连续时间轨迹。没有多项式全程碰撞、速度/加速度硬约束
   或 TinyMPC/IPOPT 控制验收。
2. 外接圆与 253 区域再次膨胀可能拒绝窄通道；狗洞/坡道继续使用现有专项流程。
3. 时间预算只能拒收迟到结果；大图 EDT 和单次 QP 不可硬抢占。负例已观察到
   1 ms 预算后仍需等待大图计算退出，所以不宣称硬实时。
4. 源码实现了求解结束 snapshot/footprint 变化时拒收；已测试跨调用地图变化。
   持续并发 costmap 更新、TF/时钟异常和动态障碍闭环仍需仿真/回放压力验证。
5. 未执行完整仓库回归：改动是被默认排除的独立 package 与文档，现有 src 配置和
   launch 未修改。原工作区仍在 `fix/old-car-fast-lio-high-spin`，保留 3 个 tracked
   现场改动以及 bags/logs。

下一步按 [`assessment_and_plan.md`](assessment_and_plan.md) P2 做相同碰撞口径的
Nav2 仿真/匹配地图回放比较，再决定是否保留 QP 或改用成熟 Nav2 A*。本轮不把插件
写入部署默认，也不自动进入实车或开启其他冻结实验。
