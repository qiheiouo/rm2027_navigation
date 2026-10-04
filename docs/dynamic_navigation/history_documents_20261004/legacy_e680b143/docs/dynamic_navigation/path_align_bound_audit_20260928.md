# 冻结 MPPI 回放：PathAlign 路径末端越界前提审计

状态：**找到了当前镜像标准 critic 回放不一致的具体源码风险；旧实验镜像缺失，尚不能证明旧二进制与当前二进制在越界时执行了同一数值路径。** 本轮只读冻结数据与当前镜像的 Nav2 头文件，未改运行插件、T-DT、tracker、控制参数或安全门。

## 源码前提与审计口径

当前镜像 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6` 安装的 Nav2 1.1.20 头文件 `/opt/ros/humble/include/nav2_mppi_controller/tools/utils.hpp`，SHA256 为 `cd48e40c5e698eaf6568ef6f50671e1b90190c8210464d2b7dff54fdc5f2dbe7`。其中 `findClosestPathPt` 对路径累计距离数组调用 `std::lower_bound`，检查了 `iter == begin + init`，却没有检查 `iter == end`，之后直接读取 `*iter`。当轨迹累计路程大于数组最后一个路径累计距离时，`lower_bound` 返回 `end`，这一读取属于 C++ 未定义行为。PathAlign 的积分循环会调用这个函数；它使用的路径累计距离数组长度为 `furthest_reached_path_point`，末尾不包含完整 global path 的终点。

[审计脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/path_align_bound_audit.py)用原始 float32 路径与全部捕获轨迹，复算 furthest path point、路径累计距离及 PathAlign 每 4 步的轨迹累计路程；只判断是否到达 `lower_bound == end` 的前提，不执行越界读取。它同时比较旧记录的 V1 前累计标准 critic 分数与当前原生评分。输入 SHA、每条暴露样本索引、分数差异索引与 16 组新采样的暴露数存于[审计结果](evidence/path_align_bound_audit_20260928/summary.json)。

| 冻结周期 | PathAlign 实际非零得分 | 路径累计数组末值 | 达到 `end` 前提的 rollout | 当前原生分数与旧记录差异 >0.001 |
| --- | ---: | ---: | ---: | ---: |
| Navfn 碰撞前 162 | 300/300 | 0.8500 m | **207/300** | **22/300，22 条全在暴露组** |
| phase 4 的 31 | 300/300 | 1.4716 m | 0/300 | 0/300 |
| phase 4 目标附近 263 | 0/300 | 0.4250 m | 254/300 的轨迹若进入该循环会暴露 | 0/300 |

周期 263 的 PathAlign 在评分循环前返回：机器人距目标约 0.499 m，低于 0.5 m 的 `threshold_to_consider`，且 furthest index 18 小于 `offset_from_furthest=20`。因此它的 300/300 分数对齐**没有验证 PathAlign 数值计算**；此前文档称两个留出周期的 PathAlign 都非零，这是错误的，本次同步修正。真正执行 PathAlign 且完整对齐的留出周期 31 没有一条达到越界前提。

同周期四种子的新 batch 中，达到该前提的数量为：300 条时 **198、208、200、199**；600 条时 **395、406、378、410**；1000 条时 **647、692、654、683**；2000 条时 **1310、1366、1323、1353**。原生评分器当时没有做边界修正，因此[大 batch 排序与聚合结果](native_critic_batch_sensitivity_20260928.md)还受当前镜像 PathAlign 未定义行为影响；联合安全样本数、真值几何和独立 CostCritic mask 不依赖这个分数，仍成立。不能再把现有大 batch 原生得分当成稳定的算法排序基准。

## 判断与后续门

源码越界前提、22 条差异与暴露集合的包含关系，以及两个留出周期的对照，强烈指向 PathAlign 的路径末端处理是周期 162 分数不一致的来源。但旧实验镜像没有保存，无法还原旧进程的越界内存内容；“22 条差异完全由此导致”仍是**推断**，不能宣称已逐位复现旧二进制。修复方向是在隔离研究回放中为 `lower_bound == end` 定义确定的末路径点行为，再重新评分同一冻结输入；该修复会改变这类轨迹的 PathAlign 分数，因此必须重新计算 300/600/1000/2000 的排名、softmax、滤波后控制与动态间隙。原始数据和安全门不变。要进入比赛运行链，还需单独审核 Nav2 修复范围、完整控制周期负载及闭环行为；当前审计本身没有产生可部署改动。
