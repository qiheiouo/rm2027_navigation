# 移动机器人相位 2：冻结近面区间的侵入前交叉检查（2026-09-30）

按[预登记窗口和规则](moving_view_crosscheck_preregistration_20260930.md)，只读复核已有 Navfn+V1 相位 2 归档的 `36.0–39.99 s` 确认 source；每个 source 的第九步均早于 `40.893 s` 的首次本体动态间隙失守。机器人此时沿箱体北侧移动并靠近障碍。[冻结近面规则](visible_face_geometry_probe_preregistration_20260929.md)、过去四帧斜率与[噪声推导速度区间](face_velocity_uncertainty_preregistration_20260930.md)均未按该归档重调。

**纵向输入交叉门通过：**60 个不同确认 source 全有同时间戳扫描、可插值的在线 ROS odom 和动态近面点；source 时刻物理箱体纵向中心误差中位/P90/最大为 `0.0184/0.0252/0.0288 m`，60/60 小于 `0.05 m`。全部 source 都有四帧速度历史，未来九步的完整物理箱体纵向区间 **540/540** 覆盖，原 V1 宽框在此窗口也为 540/540。候选区间第 1 步中位宽度 `0.707 m`，原 V1 为 `1.363 m`；第 9 步为 `1.565/1.807 m`。

| 关键约束 | 结果 |
| --- | ---: |
| 固定窗口确认 source / 扫描精确匹配 / 在线位姿 / 近面点 / 四帧速度 | 60 / 60 / 60 / 60 / 60 |
| 在线观测侧与物理侧不一致 | 0 |
| 近面估计 `y` 绝对误差中位 / P90 / 最大 | `0.0184 / 0.0252 / 0.0288 m` |
| 近面估计 `|y误差|≤0.05 m` | 60 / 60 |
| 候选九步纵向覆盖 | 540 / 540 |
| tracker `x` 中心 `|误差|≤0.05 m` | **46 / 60**；最大 `0.1063 m` |

**二维几何门仍未定义/通过。** 上表的 `x` 误差是在 source 时刻把 tracker 可见中心与物理中心比较，14/60 超过 `0.05 m`；不能简单以本次 `y` 结果宣称完整物理箱体被框住。相位 2 的 observer 在线位姿订阅 `/simulation/ground_truth/odom`，是仿真可用的 ROS 话题，不代表比赛设备 LIO 的定位误差。当前窗口及该试次的真值早已用于诊断，因此这是移动视角交叉检查而非新的盲封试验。第九步限制在原碰撞间隙失守之前，也不证明临界周期 613 的未来安全。

[源时刻表](evidence/moving_view_crosscheck_20260930/sources.csv)、[九步表](evidence/moving_view_crosscheck_20260930/source_steps.csv)和[摘要/原始输入哈希](evidence/moving_view_crosscheck_20260930/summary.json)可复核。原相位 2 归档、profile、tracker、V1/MPPI、T-DT、footprint、padding 与原 `0.05 m` clearance 均未更改。下一门需用在线扫描建立 `x` 方向完整箱体占用及其未来不确定性，再在冻结 MPPI 周期检查候选排序和聚合；当前不能将这个输入规则放入运行链。
