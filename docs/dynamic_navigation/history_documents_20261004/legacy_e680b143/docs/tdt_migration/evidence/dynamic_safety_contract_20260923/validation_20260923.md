# P2B 动态安全：HWNav 先例核对与冻结首例制动反事实

状态（2026-09-23，Asia/Shanghai）：**浙江大学 Hello World 的 HWSentryNav26 含有动态目标跟踪、2 秒预测代价图和 Follow 轨迹命中静态硬图后的 STOP 切换，值得借鉴；它没有现成解决本轮“移动箱体扫入当前/停车轨迹”的安全合同。** 本轮只审查源码并重放已有 A*、QP 冻结首例，没有运行新的导航试次、改动 T-DT/Nav2 运行时、部署默认或安全门。动态验收仍失败，`accepted_for_deployment=false`。

[上游源码提交与哈希](hwnav_source_audit.json)、[逐周期假设制动结果](braking_replay.json)、[分析器](braking_replay.py)与[专项测试](tests.txt)供复核。上游当前只有 `main@f5f941288197e14c867d711a4c4cd85bfd7a3194`；本项目的 `feature/hwsentry-selective-migration-phase1` 是选择性迁移分支，不是上游控制器的直接移植。

## 上游究竟做了什么

| 上游实现 | 对本轮问题的作用与限制 |
| --- | --- |
| [map_server目标跟踪](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/map_server/src/object_tracker.cpp)与[动态地图输出](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/map_server/src/map_server_node.cpp) | 对动态点聚类、关联、Kalman速度估计，并将确认目标的足迹外推为20个0.1 s预测帧；未进入运动预测的障碍保留静态保底。依赖地图、点云和时序质量。 |
| [按时间步消费预测图](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/path_executor/mpc/follow_problem.cpp) | Follow MPC 每个stage取对应的动态软代价。理念上比仅给当前位置障碍打分更接近本轮移动箱体问题。 |
| [Follow命令硬拒收](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/path_executor/mpc/mpc_solver.cpp) | 对实际应用的Follow rollout再检查，命中就改发STOP；但检查函数在 `masked_global_map_` 上采样点代价，即[静态/路线硬图](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/common/environment/obstacle_semantics.cpp)。动态当前图与预测图走软代价，不被此硬拒收直接覆盖。 |
| [STOP问题](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/path_executor/mpc/stop_problem.cpp) | 有指令变化率硬界和障碍软代价；没有证明STOP轨迹自身对移动障碍扫掠始终满足polygon间隙。拒收Follow后“停车”不自动意味着停车位置安全。 |
| [地图接收](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/nav_executor_node.cpp) | 验证地图几何及prediction_dt；`CostMap(OccupancyGrid)`[只保留几何和代价字节](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/common/environment/nav_map.cpp)，源header stamp未进入地图对象。控制时间步按收到预测后的相对时刻索引，没有已见的观测年龄补偿/过期门。 |
| [通道route monitor](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/task_manager/route_monitor.cpp) | 按预计到达时间查预测帧，主要守护特殊地形通道；预测时域外样本跳过。不能直接当作一般移动障碍的可停车保证。 |

上游[设计文档](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/DESIGN.md)也把最终硬检查明确描述为“静态+台阶掩码”，把未来动态图列为 Follow 代价。其[DEMO](https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/DEMO.md)提供手工开启障碍的简化轮腿仿真，不是参考新车八边形、实际雷达滞后、连续polygon间隙和制动能力的对应验收。因而**不能把上游动态预测能力当作本例已解决的证据**；轮腿LPV/FDDP、专用地图和底盘命令模型也不适合直接替换当前Nav2 Omni/MPPI。

本项目已有选择性成果，无需重造tracker：[Phase1设计](../../../hwsentry_migration/phase1_design.md)与[MPPI集成评估](../../../hwsentry_migration/dynamic_mppi_integration_plan.md)将动态预测保持为默认关闭的shadow输入；[动态通道shadow准入](../../../hwsentry_migration/dynamic_clearance_shadow_validation.md)还把过期、未来时域不足等标为UNKNOWN。后者只为特殊通道报告状态，不消费/取消普通Nav2控制。旧车[实车tracker报告](../../../hwsentry_migration/hwsentrynav26_phase1_linux_validation_report_20260824.md)记录D01空场改善，但D05身份碎片、D07运动误检和在线延迟门仍未关闭，动态MPPI critic尚未接入。[旧车停后全局重规划](../../../validation/old_car_global_dynamic_replanning_20260806.md)解决的是**持续障碍堵住路径后的停—重规划**，不是本轮横穿障碍在约0.1 s观测年龄下扫入车辆的闭环间隙。

## 在冻结首例上做的有限反事实

复用上一轮`dynamic_map_age_pilot_v1`两例、同一参考八边形、Gazebo实际箱体位姿与0.05 m本体/padded不接触原门。假设控制周期当刻开始请求停车：响应延迟取0/0.1/0.2 s，延迟后按实验profile的`velocity_smoother.max_decel=[-1,-1,-2]`逐分量减速，数值积分0.01 s，并检查随后2 s实际观测箱体与车体polygon的间隙。采样间用速度与旋转上界扣减；该下界只适用于**假设模型和插值后的已知箱体轨迹**，数值积分误差、未来未知运动和真实底盘跟踪误差未得到安全认证。0.1/0.2 s只是诊断敏感性，不是已测响应延迟。

| 假设响应延迟 | A*最后一个满足假设门的决策周期 | QP最后一个满足假设门的决策周期 |
| --- | ---: | ---: |
| 0 s | #140，首次实际0.05 m违例前0.313 s | #132，前0.743 s |
| 0.1 s | #139，前0.376 s | #131，前0.808 s |
| 0.2 s | #136，前0.671 s | #130，前0.930 s |

这是检查范围内的**离散周期**，不是连续时间精确deadline，更不是控制器当时可获得的未来真值。零延迟、配置减速度这样偏乐观的模型里，临近违例才停车也已太晚；QP在前约0.63 s的下一周期已无法满足假设门。A*零延迟最后可行周期本体下界仅约0.054 m，0.1 s延迟最后可行周期约0.051 m，余量很薄。实际本轮两组均没有执行这种反事实停车，不能宣称在线拦截会成功。停止后若仍处于障碍未来扫掠区，单纯保持零速也可能失败；2 s检查窗口外未获保证。

## 对下一步动态安全合同的取舍

值得吸收的是**时间戳完整的动态目标预测、逐时刻障碍占用和最终输出后的独立检查**，而不是直接复制上游预测代价图/轮腿MPC。一个可放行的普通导航合同至少需：用同一时钟明确观测源时间、到达/处理时间和控制时刻；由新车真实polygon、地图/感知误差、障碍速度界构成未来占用包络；由实测命令链最坏延迟、最小制动能力和跟踪误差构成**包括停车后等待**的机器人可达包络；对最后实际下发命令及可执行停车/让行轨迹检查整个扫掠，始终保持原0.05 m本体和padded无接触门。输入缺失、过期、预测时域不足或停车本身不可行时不能输出“安全通过”；需要在更早的安全区域减速/让行或拒绝进入，不能等到近障碍才发送零速。

当前没有新车实测制动下界与最终整车包络，也没有证明tracker达到D05/D07门，因此该合同仍是设计/离线诊断，**未实现在线控制保护**。下一轮应先在独立仿真前测量命令链和仿真底盘的可保证响应界，用冻结回放验证包络对已知事故是否能及时拒收，然后才做单一实现候选及独立A/B；不得为了通过修改footprint、MPPI权重、BT或原安全门。本轮不开始多相位动态验收、terminal selector或实车执行。
