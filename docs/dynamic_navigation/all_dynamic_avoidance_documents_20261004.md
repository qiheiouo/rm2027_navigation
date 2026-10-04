# 动态避障全部方案：R1至Temporal MPC冻结与文档目录

2026-10-04，Asia/Shanghai。用户已明确暂时停止本方案；本次只整理记录，不恢复研究、修改正式导航或运行实验。


2026-10-04最新冻结补充：Temporal MPC也已按用户要求冻结，`research/temporal-mpc-frozen-20261004`→`04291a41`，未接受部署。完整历程见[总文档](dynamic_obstacle_prediction_full_history_20261004.md)，全部原文见[合订本](dynamic_obstacle_prediction_all_text_20261004.md)。下列三轮统计保留原归档口径，本次另追加25份MPC文档，合计372份Markdown；2份聊天单列参考，不是实验。

## 一、历史三轮与新增Temporal MPC的对应关系和状态

这里按本次对话中的三轮研究归类：旧V1及后续风险研究、重新建立的原生CV critic路线、最新HWS对照与限时净收益计划。可见几何、名义尺寸、observed-surface属于第二轮子实验；STVL基线、HWS前端迁移和T-DT早期guard单列前置记录，不漏记，也不另算第四个方案。

| 方案 | 做了什么 | 实际结果和当前状态 | 版本入口 |
|---|---|---|---|
| 一：旧V1 / 风险、采样、排序与输出研究 | HWS参考tracker →预测box →PredictionV1Critic →MPPI；STVL另供当前占用costmap。随后CV/CA、overlap、proposal、SG/输出与设备回放等限定实验 | 部分夹具净空改善；恢复、绕行、完整排序/输出/泛化未联合过门。已冻结，不部署，不恢复 | `codex/dynamic-differential-risk@e680b143`；冻结tag peeled SHA同 |
| 二：原生CV DynamicObstacleCritic + guard | 从main单独建路；静态剔除/KF →公共状态 →原生时间评分 →MPPI/smoother →pass/brake guard；后续机械、地图、可见/名义几何及surface证据 | 已做构建、DDS和隔离闭环/回放；visible接触、nominal未完成任务，两试次综合FAILED。surface只有离线原型；在线成员证据部分改动未提交，C闭环未执行。现暂停 | `feature/dynamic-obstacle-critic`及experiment族；审计前native `b5645eca` |
| 三：HWS重新对照 / 一个月净收益计划 | 只读迁移审计、逐模块机制对照、STVL基线与短时CV的最小矩阵、false_block及A/B/C退出门 | 审计和计划文档已完成；**计划实验没有启动**。当前部署判断C；2026-10-04用户暂停，原日期/预算仅保留为计划历史 | 文档提交 `652f14ce` |

历史三轮及新增MPC均不改变正式main；main仍为 `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。已有未提交实验改动和未知core保留。旧归档记录中的持续开发goal为PAUSED；本次冻结不新建续跑安排，不把历史文档中的“下一步”当恢复授权。

## 二、先读这些总结和结果

### 方案一

- [V1方案范围与收缩边界](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/v1_prediction_scope.md)
- [V1真正运行的固定相位A/B结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/validation_20260924.md)
- [V1冻结记录](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/v1_research_freeze_20260925.md)
- [中间阶段决策](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/dynamic_prediction_stage_decision_20260929.md)
- [旧路线最终阶段结论与暂停记录](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/research_stage_conclusion_20261001.md)
- [最后一项成本/输出对齐结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/standard_output_cost_alignment_20261001.md)
- [历史复现与交接检查点（当前不执行恢复）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/research_continuation_checkpoint_20260930.md)

### 方案二

- [新路线建立前仓库审查与复用边界](../dynamic_obstacle_critic/repository_audit.md)
- [原生CV critic与guard架构](../dynamic_obstacle_critic/architecture.md)
- [第一阶段验收和失败结果](../dynamic_obstacle_critic/validation.md)
- [近场失败分析](../dynamic_obstacle_critic/nearfield_failure_review.md)
- [完整第二阶段逐项进度、结果与失败记录](../dynamic_obstacle_critic/stage2_progress.md)
- [可见/名义几何配对试次与冻结结果](../dynamic_obstacle_critic/observation_diameter_consumption_experiment.md)
- [揭露距离、全三秒回放和停止/逃逸条件](../dynamic_obstacle_critic/dynamic_surface_reveal_experiment.md)
- [实际扫描成员/关联证据登记（含既有未提交文档扩展，非闭环接受）](../dynamic_obstacle_critic/observed_surface_members_experiment.md)

### 方案三（未运行实验）

- [HWS选择性迁移完整审计](hws_migration_audit_20261003.md)
- [HWS与当前路线逐模块对照及最小实验矩阵](hws_vs_current_prediction_analysis.md)
- [一个月退出计划与本次暂停更新](prediction_one_month_exit_plan.md)
- [迁移审计机器清单](hws_migration_audit_inventory_20261003.json)：分支/提交/路径与消费者检索证据。

## 三、完整清单如何阅读

共找到 **347份已有Markdown文档**：三轮核心及前置迁移目录304份，另外43份基线/契约/实车等关联文档。后者可能以定位、TF、建图或静态迁移为主，保留是因为其中包含动态避障/STVL/失败门引用，不把它们当额外避障方案。

检索覆盖整理开始时96个本地Git引用，包括heads、remote-cache、tags及Codex内部文档树快照，共86个不同Git快照（80个提交、6个无提交时间的tree对象）。核心目录历史新增/修改的原始Markdown路径也检查过，没有只存在已删除历史而遗漏的路径。每个原路径列一个可读版本；修改过程及其他版本保留在Git。远端缓存不代表远端实时状态。

旧分支中当前checkout不存在的文档已导出 **279份只读历史阅读副本**，合计1,451,162字节；保持原文逐字节，不重写旧结论，也不搬运源码或原始大数据。当前文档链接直接指向原位置。下表“来源”标明canonical读取提交；完整source blob、SHA256、引用版本及工作区差异见[机器清单](all_dynamic_avoidance_documents_20261004.json)。

历史副本保持原目录结构；文档之间的已收录相对链接可沿用。代码、图、raw trace、bag、manifest等资产仍在原版本/证据目录，**不因导出Markdown而成为已复制或重新验证的资产**。需要原上下文时，以机器清单的source_commit/original_path使用 `git show <commit>:<path>`。副本内历史继续/恢复指令没有当前授权。

目录包含evidence里的真正Markdown结果报告；归档中的整套重复源码/文档镜像不重复计数。文档存在不等于实验执行；“预登记/计划/模板”和“结果/验收/冻结”按原文状态区分。未提交的surface登记扩展有差异标记，不升级为完整实验通过。

## 方案一：旧 V1 与后续风险 / 采样 / 输出研究 — 216份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [冻结周期 162 的 MPPI 采样空间对照](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/batch_sampling_probe_20260926.md) | `docs/dynamic_navigation/batch_sampling_probe_20260926.md` | e680b143，历史副本 |
| [冻结CA候选的独立近场验证](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/ca_independent_holdout_20261001.md) | `docs/dynamic_navigation/ca_independent_holdout_20261001.md` | e680b143，历史副本 |
| [扫描外廓中心预测：历史拟合、留出验证与冻结 MPPI 重放](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/calibrated_scan_support_holdout_20260925.md) | `docs/dynamic_navigation/calibrated_scan_support_holdout_20260925.md` | e680b143，历史副本 |
| [在候选输出计划上评价冻结CA风险](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/candidate_output_risk_20261001.md) | `docs/dynamic_navigation/candidate_output_risk_20261001.md` | e680b143，历史副本 |
| [仅用过去近面观测估计速度：离线预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/causal_face_velocity_preregistration_20260930.md) | `docs/dynamic_navigation/causal_face_velocity_preregistration_20260930.md` | e680b143，历史副本 |
| [过去四帧近面速度：有明显增益，但九步门仍失败（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/causal_face_velocity_result_20260930.md) | `docs/dynamic_navigation/causal_face_velocity_result_20260930.md` | e680b143，历史副本 |
| [过去扫描推导的条件运动界：离线留出与冻结周期](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/causal_scan_bound_probe_20260925.md) | `docs/dynamic_navigation/causal_scan_bound_probe_20260925.md` | e680b143，历史副本 |
| [关键源时刻的因果减速信号：二次外推不足](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/causal_turn_signal_20260929.md) | `docs/dynamic_navigation/causal_turn_signal_20260929.md` | e680b143，历史副本 |
| [动态障碍中心分布的历史留出校验](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/center_marginal_calibration_20260926.md) | `docs/dynamic_navigation/center_marginal_calibration_20260926.md` | e680b143，历史副本 |
| [冻结混合采样库上的间隙风险形状对照](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/clearance_shape_risk_20261001.md) | `docs/dynamic_navigation/clearance_shape_risk_20261001.md` | e680b143，历史副本 |
| [两telemetry副本组合加载的累计评分门预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/combined_trace_fixture_preregistration_20261001.md) | `docs/dynamic_navigation/combined_trace_fixture_preregistration_20261001.md` | e680b143，历史副本 |
| [两telemetry副本组合加载：八critic离线结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/combined_trace_fixture_result_20261001.md) | `docs/dynamic_navigation/combined_trace_fixture_result_20261001.md` | e680b143，历史副本 |
| [V1 保守占用的相交时长差异：冻结窗口否定实验](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/conflict_exposure_probe_20260928.md) | `docs/dynamic_navigation/conflict_exposure_probe_20260928.md` | e680b143，历史副本 |
| [Consumer trace fixture固定时钟类型错误：失败记录](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/consumer_trace_fixture_clock_failure_20261001.md) | `docs/dynamic_navigation/consumer_trace_fixture_clock_failure_20261001.md` | e680b143，历史副本 |
| [现有shadow tracker的consumer trace离线转发门预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/consumer_trace_fixture_preregistration_20261001.md) | `docs/dynamic_navigation/consumer_trace_fixture_preregistration_20261001.md` | e680b143，历史副本 |
| [Consumer trace离线转发门结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/consumer_trace_fixture_result_20261001.md) | `docs/dynamic_navigation/consumer_trace_fixture_result_20261001.md` | e680b143，历史副本 |
| [冻结周期的原生 CostCritic 碰撞判据与 batch 可行数](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/costmap_mask_probe_20260926.md) | `docs/dynamic_navigation/costmap_mask_probe_20260926.md` | e680b143，历史副本 |
| [动态危险窗口中的 MPPI critic 排序分解](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/critic_rank_decomposition_20260926.md) | `docs/dynamic_navigation/critic_rank_decomposition_20260926.md` | e680b143，历史副本 |
| [147/263 冻结周期的 MPPI batch 敏感性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cross_cycle_batch_sensitivity_20260929.md) | `docs/dynamic_navigation/cross_cycle_batch_sensitivity_20260929.md` | e680b143，历史副本 |
| [147/263 冻结周期的原生首步输出复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cross_cycle_first_step_audit_20260928.md) | `docs/dynamic_navigation/cross_cycle_first_step_audit_20260928.md` | e680b143，历史副本 |
| [周期 613 物理占用连续重叠排序：固定离线判别（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_face_xy_overlap_preregistration_20260930.md) | `docs/dynamic_navigation/cycle613_face_xy_overlap_preregistration_20260930.md` | e680b143，历史副本 |
| [周期 613：连续重叠有信息，但固定评分未改变危险控制（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_face_xy_overlap_result_20260930.md) | `docs/dynamic_navigation/cycle613_face_xy_overlap_result_20260930.md` | e680b143，历史副本 |
| [周期 613 可见面二维预测评分：离线预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_face_xy_rank_preregistration_20260930.md) | `docs/dynamic_navigation/cycle613_face_xy_rank_preregistration_20260930.md` | e680b143，历史副本 |
| [周期 613 可见面二维占用：九步评分仍饱和（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_face_xy_rank_result_20260930.md) | `docs/dynamic_navigation/cycle613_face_xy_rank_result_20260930.md` | e680b143，历史副本 |
| [周期 613 七项原生 critic 归因：预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_native_term_attribution_preregistration_20260930.md) | `docs/dynamic_navigation/cycle613_native_term_attribution_preregistration_20260930.md` | e680b143，历史副本 |
| [周期 613：原生 Goal 与 PathAlign 压过动态重叠排序（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_native_term_attribution_result_20260930.md) | `docs/dynamic_navigation/cycle613_native_term_attribution_result_20260930.md` | e680b143，历史副本 |
| [周期 613 第 2 步预测相交：全控制域上界预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_step2_reachability_preregistration_20260930.md) | `docs/dynamic_navigation/cycle613_step2_reachability_preregistration_20260930.md` | e680b143，历史副本 |
| [周期 613：增大 batch 无法解除第 2 步预测硬碰撞（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/cycle613_step2_reachability_result_20260930.md) | `docs/dynamic_navigation/cycle613_step2_reachability_result_20260930.md` | e680b143，历史副本 |
| [周期 47 std4 聚合独立数值门差异预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_aggregation_attribution_preregistration_20261001.md) | `docs/dynamic_navigation/device_aggregation_attribution_preregistration_20261001.md` | e680b143，历史副本 |
| [周期 47 std4 离线聚合参考差异定位结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_aggregation_attribution_result_20261001.md) | `docs/dynamic_navigation/device_aggregation_attribution_result_20261001.md` | e680b143，历史副本 |
| [新设备磁盘清理记录](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_disk_cleanup_result_20260930.md) | `docs/dynamic_navigation/device_disk_cleanup_result_20260930.md` | e680b143，历史副本 |
| [新设备隔离有界反事实数值门结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_guarded_gate_result_20260930.md) | `docs/dynamic_navigation/device_guarded_gate_result_20260930.md` | e680b143，历史副本 |
| [动态预测研发设备迁移记录（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_migration_20260930.md) | `docs/dynamic_navigation/device_migration_20260930.md` | e680b143，历史副本 |
| [新设备冻结回放复现协议（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_replay_preregistration_20260930.md) | `docs/dynamic_navigation/device_replay_preregistration_20260930.md` | e680b143，历史副本 |
| [新设备环境与原冻结回放结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_replay_result_20260930.md) | `docs/dynamic_navigation/device_replay_result_20260930.md` | e680b143，历史副本 |
| [新设备周期 47 样本字节复现差异预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_sample_reproduction_preregistration_20260930.md) | `docs/dynamic_navigation/device_sample_reproduction_preregistration_20260930.md` | e680b143，历史副本 |
| [周期 47 新设备样本字节复现否定结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_sample_reproduction_result_20260930.md) | `docs/dynamic_navigation/device_sample_reproduction_result_20260930.md` | e680b143，历史副本 |
| [新设备周期 47 评分差异归因预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/device_score_discrepancy_preregistration_20260930.md) | `docs/dynamic_navigation/device_score_discrepancy_preregistration_20260930.md` | e680b143，历史副本 |
| [动态预测：可见面方向与占用可行性复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/directional_support_feasibility_20260924.md) | `docs/dynamic_navigation/directional_support_feasibility_20260924.md` | e680b143，历史副本 |
| [动态预测 + MPPI 本轮决策记录（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/dynamic_prediction_stage_decision_20260929.md) | `docs/dynamic_navigation/dynamic_prediction_stage_decision_20260929.md` | e680b143，历史副本 |
| [2027 TF Contract](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/near_x_graph_attribution_inputs_20261001/11_tf_contract_2027.md) | `docs/dynamic_navigation/evidence/near_x_graph_attribution_inputs_20261001/11_tf_contract_2027.md` | e680b143，历史副本 |
| [near-X v2图门失败只读归因预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/near_x_graph_attribution_inputs_20261001/13_near_x_graph_attribution_preregistration_20261001.md) | `docs/dynamic_navigation/evidence/near_x_graph_attribution_inputs_20261001/13_near_x_graph_attribution_preregistration_20261001.md` | e680b143，历史副本 |
| [动态预测 V1：物理外形界离线影子检查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_extent_shadow_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_extent_shadow_20260924/validation_20260924.md` | e680b143，历史副本 |
| [动态预测 V1：冻结 phase 0 重复的恢复时窗复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_failure_window_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_failure_window_20260924/validation_20260924.md` | e680b143，历史副本 |
| [动态预测 V1：独立 MPPI critic 固定相位 A/B](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/validation_20260924.md` | e680b143，历史副本 |
| [动态预测 V1：Navfn 首例与固定配置复现的分叉审查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_navfn_repeatability_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_navfn_repeatability_20260924/validation_20260924.md` | e680b143，历史副本 |
| [动态预测 V1：Navfn 交叉首例与一次固定配置复现](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_navfn_transfer_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_navfn_transfer_20260924/validation_20260924.md` | e680b143，历史副本 |
| [动态预测 V1 输入首例：跟踪可用，直接使用可见框不够](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/validation_20260924.md) | `docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/validation_20260924.md` | e680b143，历史副本 |
| [实验基础设施统一notes](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/experiment_infrastructure_notes_20261001.md) | `docs/dynamic_navigation/experiment_infrastructure_notes_20261001.md` | e680b143，历史副本 |
| [冻结连续风险单一输入支持替换预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/face_support_rank_preregistration_20261001.md) | `docs/dynamic_navigation/face_support_rank_preregistration_20261001.md` | e680b143，历史副本 |
| [固定源区间支持替换：排序比例改善，首选安全仍失败](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/face_support_rank_result_20261001.md) | `docs/dynamic_navigation/face_support_rank_result_20261001.md` | e680b143，历史副本 |
| [近面速度不确定区间探针预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/face_velocity_uncertainty_preregistration_20260930.md) | `docs/dynamic_navigation/face_velocity_uncertainty_preregistration_20260930.md` | e680b143，历史副本 |
| [近面速度区间：当前静止视角有效段全覆盖，仍待移动视角验收（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/face_velocity_uncertainty_result_20260930.md) | `docs/dynamic_navigation/face_velocity_uncertainty_result_20260930.md` | e680b143，历史副本 |
| [周期 47 固定允许控制原型可行性对照预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_control_prototype_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_control_prototype_preregistration_20261001.md` | e680b143，历史副本 |
| [周期 47 固定允许控制原型可行性结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_control_prototype_result_20261001.md) | `docs/dynamic_navigation/fast_x_control_prototype_result_20261001.md` | e680b143，历史副本 |
| [周期 47 新设备控制矩阵冻结离线诊断预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_device_frozen_control_preregistration_20260930.md) | `docs/dynamic_navigation/fast_x_device_frozen_control_preregistration_20260930.md` | e680b143，历史副本 |
| [快运动源时间条件化的既有二维占用迁移预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_face_coverage_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_face_coverage_preregistration_20261001.md` | e680b143，历史副本 |
| [快运动五周期条件化二维占用覆盖结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_face_coverage_result_20261001.md) | `docs/dynamic_navigation/fast_x_face_coverage_result_20261001.md` | e680b143，历史副本 |
| [周期 47 固定随机控制集合风险迁移预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_frozen_batch_risk_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_frozen_batch_risk_preregistration_20261001.md` | e680b143，历史副本 |
| [周期 47 固定四 seed 在线风险迁移否定结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_frozen_batch_risk_result_20261001.md) | `docs/dynamic_navigation/fast_x_frozen_batch_risk_result_20261001.md` | e680b143，历史副本 |
| [周期 47 新设备固定控制 std4 覆盖、排序与输出结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_frozen_device_result_20261001.md) | `docs/dynamic_navigation/fast_x_frozen_device_result_20261001.md` | e680b143，历史副本 |
| [固定原型风险与原生最终输出一致性预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_native_risk_consistency_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_native_risk_consistency_preregistration_20261001.md` | e680b143，历史副本 |
| [固定原型在线风险与原生输出一致性结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_native_risk_consistency_result_20261001.md) | `docs/dynamic_navigation/fast_x_native_risk_consistency_result_20261001.md` | e680b143，历史副本 |
| [快运动冻结周期在线输入可用性审计预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_online_input_audit_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_online_input_audit_preregistration_20261001.md` | e680b143，历史副本 |
| [快运动输入：源时间可构造，实际在线可用尚未证明](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_online_input_audit_result_20261001.md) | `docs/dynamic_navigation/fast_x_online_input_audit_result_20261001.md` | e680b143，历史副本 |
| [周期 47 固定原型在线风险区分预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_prototype_risk_preregistration_20261001.md) | `docs/dynamic_navigation/fast_x_prototype_risk_preregistration_20261001.md` | e680b143，历史副本 |
| [周期 47 固定原型在线连续风险区分结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_prototype_risk_result_20261001.md) | `docs/dynamic_navigation/fast_x_prototype_risk_result_20261001.md` | e680b143，历史副本 |
| [周期 47 新设备离线诊断执行补充（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/fast_x_scale4_device_protocol_20260930.md) | `docs/dynamic_navigation/fast_x_scale4_device_protocol_20260930.md` | e680b143，历史副本 |
| [MPPI滤波history快照：隔离副本离线结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filter_history_fixture_result_20261001.md) | `docs/dynamic_navigation/filter_history_fixture_result_20261001.md` | e680b143，历史副本 |
| [MPPI滤波history只读快照副本预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filter_history_trace_preregistration_20261001.md) | `docs/dynamic_navigation/filter_history_trace_preregistration_20261001.md` | e680b143，历史副本 |
| [滤波候选评分与控制聚合表述对照](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_aggregate_alignment_20260928.md) | `docs/dynamic_navigation/filtered_aggregate_alignment_20260928.md` | e680b143，历史副本 |
| [单条滤波候选的七项原生 critic 重评分](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_all_critic_replay_20260928.md) | `docs/dynamic_navigation/filtered_all_critic_replay_20260928.md` | e680b143，历史副本 |
| [固定周期 162 的滤波候选批量敏感性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_batch_candidate_rank_20260928.md) | `docs/dynamic_navigation/filtered_batch_candidate_rank_20260928.md` | e680b143，历史副本 |
| [滤波候选的预测排序与真实安全标签](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_candidate_rank_audit_20260928.md) | `docs/dynamic_navigation/filtered_candidate_rank_audit_20260928.md` | e680b143，历史副本 |
| [滤波候选跨周期七项原生评分复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_cross_cycle_native_20260928.md) | `docs/dynamic_navigation/filtered_cross_cycle_native_20260928.md` | e680b143，历史副本 |
| [单条滤波轨迹上的连续预测评分：固定大 batch 诊断](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_graded_batch_20260928.md) | `docs/dynamic_navigation/filtered_graded_batch_20260928.md` | e680b143，历史副本 |
| [滤波候选连续 V1 评分的三个预选周期筛查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/filtered_graded_cross_cycle_20260928.md) | `docs/dynamic_navigation/filtered_graded_cross_cycle_20260928.md` | e680b143，历史副本 |
| [动态预测 V1：同周期 MPPI 诊断](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/frozen_cycle_probe_20260924.md) | `docs/dynamic_navigation/frozen_cycle_probe_20260924.md` | e680b143，历史副本 |
| [动态障碍冻结周期的完整 MPPI 回放契约](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/full_frozen_mppi_replay_contract_20260926.md) | `docs/dynamic_navigation/full_frozen_mppi_replay_contract_20260926.md` | e680b143，历史副本 |
| [连续预测重叠在固定大 batch 下的离线上界](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/graded_overlap_batch_20260928.md) | `docs/dynamic_navigation/graded_overlap_batch_20260928.md` | e680b143，历史副本 |
| [动态预测 V1：保守占用内的连续排序实验](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/graded_overlap_candidate_20260924.md) | `docs/dynamic_navigation/graded_overlap_candidate_20260924.md` | e680b143，历史副本 |
| [冻结周期 PathAlign C++ 边界修正验证](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/guarded_path_align_cpp_validation_20260928.md) | `docs/dynamic_navigation/guarded_path_align_cpp_validation_20260928.md` | e680b143，历史副本 |
| [冻结周期的 PathAlign 有界回放](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/guarded_path_align_replay_20260928.md) | `docs/dynamic_navigation/guarded_path_align_replay_20260928.md` | e680b143，历史副本 |
| [冻结周期的 MPPI 温度敏感性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/guarded_temperature_sensitivity_20260928.md) | `docs/dynamic_navigation/guarded_temperature_sensitivity_20260928.md` | e680b143，历史副本 |
| [动态预测：方向区间的独立试次核查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/heldout_directional_audit_20260924.md) | `docs/dynamic_navigation/heldout_directional_audit_20260924.md` | e680b143，历史副本 |
| [历史扫描残差分布留出检查：现有数据不能支撑软排序](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/historical_scan_residual_calibration_20260929.md) | `docs/dynamic_navigation/historical_scan_residual_calibration_20260929.md` | e680b143，历史副本 |
| [同运行晚期固定时刻三层诊断](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/late_fixed_time_three_gate_20261001.md) | `docs/dynamic_navigation/late_fixed_time_three_gate_20261001.md` | e680b143，历史副本 |
| [晚期零安全批次的固定零控制见证](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/late_zero_control_witness_20261001.md) | `docs/dynamic_navigation/late_zero_control_witness_20261001.md` | e680b143，历史副本 |
| [原bounds/filter映射的非可逆反例预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/mapped_control_preimage_preregistration_20261001.md) | `docs/dynamic_navigation/mapped_control_preimage_preregistration_20261001.md` | e680b143，历史副本 |
| [原bounds/filter映射存在不同raw的相同完整像](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/mapped_control_preimage_result_20261001.md) | `docs/dynamic_navigation/mapped_control_preimage_result_20261001.md` | e680b143，历史副本 |
| [高算力 miniPC 算法验证交接（无雷达、串口）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/minipc_algorithm_handoff_20260924.md) | `docs/dynamic_navigation/minipc_algorithm_handoff_20260924.md` | e680b143，历史副本 |
| [已归档移动机器人试次交叉检查预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/moving_view_crosscheck_preregistration_20260930.md) | `docs/dynamic_navigation/moving_view_crosscheck_preregistration_20260930.md` | e680b143，历史副本 |
| [移动机器人相位 2：冻结近面区间的侵入前交叉检查（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/moving_view_crosscheck_result_20260930.md) | `docs/dynamic_navigation/moving_view_crosscheck_result_20260930.md` | e680b143，历史副本 |
| [新设备MPPI / V1动态链接身份前置门预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/mppi_module_link_preregistration_20261001.md) | `docs/dynamic_navigation/mppi_module_link_preregistration_20261001.md` | e680b143，历史副本 |
| [新设备MPPI / V1链接身份与capture接口核查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/mppi_module_link_result_20261001.md) | `docs/dynamic_navigation/mppi_module_link_result_20261001.md` | e680b143，历史副本 |
| [多帧扫描可行集：历史留出与两个冻结周期](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/multi_scan_bound_probe_20260925.md) | `docs/dynamic_navigation/multi_scan_bound_probe_20260925.md` | e680b143，历史副本 |
| [冻结 MPPI 原生计算核心的重复耗时与内存](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/native_core_benchmark_20260928.md) | `docs/dynamic_navigation/native_core_benchmark_20260928.md` | e680b143，历史副本 |
| [冻结周期原生 critic 与 batch 敏感性（2026-09-28）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/native_critic_batch_sensitivity_20260928.md) | `docs/dynamic_navigation/native_critic_batch_sensitivity_20260928.md` | e680b143，历史副本 |
| [冻结控制输入的原生 MPPI 核心回放](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/native_full_cycle_replay_20260928.md) | `docs/dynamic_navigation/native_full_cycle_replay_20260928.md` | e680b143，历史副本 |
| [固定连续风险：新输入排序与完整输出](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_field_fixed_continuous_risk_20261001.md) | `docs/dynamic_navigation/near_field_fixed_continuous_risk_20261001.md` | e680b143，历史副本 |
| [固定恒速/恒加速度消融：完整控制期的软风险](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_field_kinematic_risk_20261001.md) | `docs/dynamic_navigation/near_field_kinematic_risk_20261001.md` | e680b143，历史副本 |
| [因果近面运动估计与排序信号](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_field_surface_motion_20261001.md) | `docs/dynamic_navigation/near_field_surface_motion_20261001.md` | e680b143，历史副本 |
| [新设备近场holdout三层诊断](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_field_three_gate_holdout_20261001.md) | `docs/dynamic_navigation/near_field_three_gate_holdout_20261001.md` | e680b143，历史副本 |
| [周期 162 的近场候选排序归因与四步风险消融](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_term_rank_attribution_20260929.md) | `docs/dynamic_navigation/near_term_rank_attribution_20260929.md` | e680b143，历史副本 |
| [原控制图精确身份fixture输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_command_graph_fixture_inputs_20261001.md) | `docs/dynamic_navigation/near_x_command_graph_fixture_inputs_20261001.md` | e680b143，历史副本 |
| [原Humble控制图精确身份fixture预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_command_graph_fixture_preregistration_20261001.md) | `docs/dynamic_navigation/near_x_command_graph_fixture_preregistration_20261001.md` | e680b143，历史副本 |
| [原控制图精确身份fixture结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_command_graph_fixture_result_20261001.md) | `docs/dynamic_navigation/near_x_command_graph_fixture_result_20261001.md` | e680b143，历史副本 |
| [近场 `x` 穿越周期 73：冻结输入扩批预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_crossing_batch_preregistration_20260930.md) | `docs/dynamic_navigation/near_x_crossing_batch_preregistration_20260930.md` | e680b143，历史副本 |
| [`x` 轴移动箱体近场穿越：单次闭环预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_crossing_holdout_preregistration_20260930.md) | `docs/dynamic_navigation/near_x_crossing_holdout_preregistration_20260930.md` | e680b143，历史副本 |
| [`x` 轴移动箱体近场穿越与 MPPI 冻结周期 73（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_crossing_result_20260930.md) | `docs/dynamic_navigation/near_x_crossing_result_20260930.md` | e680b143，历史副本 |
| [`x` 轴箱体较快运动的单次近场留出试次预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_fast_motion_preregistration_20260930.md) | `docs/dynamic_navigation/near_x_fast_motion_preregistration_20260930.md` | e680b143，历史副本 |
| [较快 `x` 轴箱体近场留出试次：冻结周期结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_fast_motion_result_20260930.md) | `docs/dynamic_navigation/near_x_fast_motion_result_20260930.md` | e680b143，历史副本 |
| [原生goal UUID日志fixture输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_goal_uuid_fixture_inputs_20261001.md) | `docs/dynamic_navigation/near_x_goal_uuid_fixture_inputs_20261001.md` | e680b143，历史副本 |
| [near-X原生goal UUID记录fixture预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_goal_uuid_fixture_preregistration_20261001.md) | `docs/dynamic_navigation/near_x_goal_uuid_fixture_preregistration_20261001.md` | e680b143，历史副本 |
| [原生goal UUID日志fixture结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_goal_uuid_fixture_result_20261001.md) | `docs/dynamic_navigation/near_x_goal_uuid_fixture_result_20261001.md` | e680b143，历史副本 |
| [near-X图归因输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_graph_attribution_inputs_20261001.md) | `docs/dynamic_navigation/near_x_graph_attribution_inputs_20261001.md` | e680b143，历史副本 |
| [near-X v2图门失败只读归因预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_graph_attribution_preregistration_20261001.md) | `docs/dynamic_navigation/near_x_graph_attribution_preregistration_20261001.md` | e680b143，历史副本 |
| [near-X图门来源归因结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_graph_attribution_result_20261001.md) | `docs/dynamic_navigation/near_x_graph_attribution_result_20261001.md` | e680b143，历史副本 |
| [南北近场冻结周期安全支撑上界预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_safe_support_oracle_preregistration_20260930.md) | `docs/dynamic_navigation/near_x_safe_support_oracle_preregistration_20260930.md` | e680b143，历史副本 |
| [南北近场安全支撑上界结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_safe_support_oracle_result_20260930.md) | `docs/dynamic_navigation/near_x_safe_support_oracle_result_20260930.md` | e680b143，历史副本 |
| [南侧出发的独立 `x` 轴近场穿越预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_south_crossing_preregistration_20260930.md) | `docs/dynamic_navigation/near_x_south_crossing_preregistration_20260930.md` | e680b143，历史副本 |
| [南侧出发的独立 `x` 轴近场穿越结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/near_x_south_crossing_result_20260930.md) | `docs/dynamic_navigation/near_x_south_crossing_result_20260930.md` | e680b143，历史副本 |
| [新设备独立周期采集前置条件只读核查](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_capture_inventory_preregistration_20261001.md) | `docs/dynamic_navigation/new_device_capture_inventory_preregistration_20261001.md` | e680b143，历史副本 |
| [新设备独立采集前置核查结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_capture_inventory_result_20261001.md) | `docs/dynamic_navigation/new_device_capture_inventory_result_20261001.md` | e680b143，历史副本 |
| [新设备near-X首次完整采集工具失败](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_failure_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_failure_20261001.md` | e680b143，历史副本 |
| [near-X采集v2前置图门失败与原始数据冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_failure_v2_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_failure_v2_20261001.md` | e680b143，历史副本 |
| [near-X capture v3原生UUID记录失败](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_failure_v3_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_failure_v3_20261001.md` | e680b143，历史副本 |
| [新设备near-X完整采集输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_inputs_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_inputs_20261001.md` | e680b143，历史副本 |
| [near-X采集v2输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_inputs_v2_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_inputs_v2_20261001.md` | e680b143，历史副本 |
| [near-X capture v3输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_inputs_v3_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_inputs_v3_20261001.md` | e680b143，历史副本 |
| [near-X capture v4输入冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_inputs_v4_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_inputs_v4_20261001.md` | e680b143，历史副本 |
| [新设备near-X完整输入capture预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_preregistration_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_preregistration_20261001.md` | e680b143，历史副本 |
| [near-X采集工具属性修复v2预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_v2_preregistration_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_v2_preregistration_20261001.md` | e680b143，历史副本 |
| [near-X capture v3原控制图工具接入预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_v3_preregistration_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_v3_preregistration_20261001.md` | e680b143，历史副本 |
| [near-X capture v4 UUID记录接入预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/new_device_near_x_capture_v4_preregistration_20261001.md) | `docs/dynamic_navigation/new_device_near_x_capture_v4_preregistration_20261001.md` | e680b143，历史副本 |
| [动态预测输入：偏置视角完整记录预登记（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/offset_view_observation_preregistration_20260929.md) | `docs/dynamic_navigation/offset_view_observation_preregistration_20260929.md` | e680b143，历史副本 |
| [动态预测输入：偏置视角完整记录结果（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/offset_view_observation_result_20260929.md) | `docs/dynamic_navigation/offset_view_observation_result_20260929.md` | e680b143，历史副本 |
| [冻结周期安全样本的滤波与加权上界（2026-09-28）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/oracle_aggregation_probe_20260928.md) | `docs/dynamic_navigation/oracle_aggregation_probe_20260928.md` | e680b143，历史副本 |
| [冻结输出过滤前后控制约束审计预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/output_control_bounds_preregistration_20261001.md) | `docs/dynamic_navigation/output_control_bounds_preregistration_20261001.md` | e680b143，历史副本 |
| [冻结原生输出约束审计结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/output_control_bounds_result_20261001.md) | `docs/dynamic_navigation/output_control_bounds_result_20261001.md` | e680b143，历史副本 |
| [最终控制序列的原生首步动力学复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/output_first_step_audit_20260928.md) | `docs/dynamic_navigation/output_first_step_audit_20260928.md` | e680b143，历史副本 |
| [动态预测输入：南北成对可见面观测预登记（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/paired_view_observation_preregistration_20260929.md) | `docs/dynamic_navigation/paired_view_observation_preregistration_20260929.md` | e680b143，历史副本 |
| [动态预测输入：首组成对观测结果（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/paired_view_observation_result_20260929.md) | `docs/dynamic_navigation/paired_view_observation_result_20260929.md` | e680b143，历史副本 |
| [新设备被动仿真 raw receipt / TF 前置门预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/passive_sim_receipt_preregistration_20261001.md) | `docs/dynamic_navigation/passive_sim_receipt_preregistration_20261001.md` | e680b143，历史副本 |
| [被动仿真 receipt / TF 前置门：未通过](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/passive_sim_receipt_result_20261001.md) | `docs/dynamic_navigation/passive_sim_receipt_result_20261001.md` | e680b143，历史副本 |
| [真实被动capture的因果TF到达归因预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/passive_tf_arrival_preregistration_20261001.md) | `docs/dynamic_navigation/passive_tf_arrival_preregistration_20261001.md` | e680b143，历史副本 |
| [真实被动capture的因果TF到达归因结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/passive_tf_arrival_result_20261001.md) | `docs/dynamic_navigation/passive_tf_arrival_result_20261001.md` | e680b143，历史副本 |
| [冻结 MPPI 回放：PathAlign 路径末端越界前提审计](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/path_align_bound_audit_20260928.md) | `docs/dynamic_navigation/path_align_bound_audit_20260928.md` | e680b143，历史副本 |
| [预测重叠的九步峰值排序：固定输入否决](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/peak_temporal_rank_20260928.md) | `docs/dynamic_navigation/peak_temporal_rank_20260928.md` | e680b143，历史副本 |
| [独立 Navfn+V1 相位 2 采集预登记（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase2_holdout_preregistration_20260929.md) | `docs/dynamic_navigation/phase2_holdout_preregistration_20260929.md` | e680b143，历史副本 |
| [Navfn+V1 独立相位 2 留出试次：输入、300 条候选与排序（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase2_holdout_result_20260929.md) | `docs/dynamic_navigation/phase2_holdout_result_20260929.md` | e680b143，历史副本 |
| [相位 2 实际消费源帧的可见表面偏差复核（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase2_source_surface_result_20260929.md) | `docs/dynamic_navigation/phase2_source_surface_result_20260929.md` | e680b143，历史副本 |
| [Navfn+V1 相位 6 新闭环输入：预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase6_control_holdout_preregistration_20260930.md) | `docs/dynamic_navigation/phase6_control_holdout_preregistration_20260930.md` | e680b143，历史副本 |
| [Navfn+V1 相位 6 闭环与冻结周期 816（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase6_control_holdout_result_20260930.md) | `docs/dynamic_navigation/phase6_control_holdout_result_20260930.md` | e680b143，历史副本 |
| [相位 6 首次启动失败与固定环境修正（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase6_control_holdout_setup_failure_20260930.md) | `docs/dynamic_navigation/phase6_control_holdout_setup_failure_20260930.md` | e680b143，历史副本 |
| [相位 6 冻结周期的旧规则迁移门（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase6_fixed_transfer_preregistration_20260930.md) | `docs/dynamic_navigation/phase6_fixed_transfer_preregistration_20260930.md` | e680b143，历史副本 |
| [北侧相位匹配残差的冻结排序：仍未过门](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/phase_conditioned_rank_20260929.md) | `docs/dynamic_navigation/phase_conditioned_rank_20260929.md` | e680b143，历史副本 |
| [原V1完整物理执行与单周期输出一致性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/physical_execution_consistency_20261001.md) | `docs/dynamic_navigation/physical_execution_consistency_20261001.md` | e680b143，历史副本 |
| [新设备near-X可移植资产与只读launch门预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/portable_near_x_assets_preregistration_20261001.md) | `docs/dynamic_navigation/portable_near_x_assets_preregistration_20261001.md` | e680b143，历史副本 |
| [可移植near-X首次只读preflight失败](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/portable_near_x_preflight_failure_20261001.md) | `docs/dynamic_navigation/portable_near_x_preflight_failure_20261001.md` | e680b143，历史副本 |
| [可移植near-X只读preflight v2结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/portable_near_x_preflight_result_v2_20261001.md) | `docs/dynamic_navigation/portable_near_x_preflight_result_v2_20261001.md` | e680b143，历史副本 |
| [可移植near-X只读preflight v2预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/portable_near_x_preflight_v2_preregistration_20261001.md) | `docs/dynamic_navigation/portable_near_x_preflight_v2_preregistration_20261001.md` | e680b143，历史副本 |
| [冻结过滤输出bounds投影反事实预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/post_filter_projection_preregistration_20261001.md) | `docs/dynamic_navigation/post_filter_projection_preregistration_20261001.md` | e680b143，历史副本 |
| [冻结过滤输出原 bounds 投影结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/post_filter_projection_result_20261001.md) | `docs/dynamic_navigation/post_filter_projection_result_20261001.md` | e680b143，历史副本 |
| [V1实际prediction快照：隔离副本离线结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/prediction_snapshot_fixture_result_20261001.md) | `docs/dynamic_navigation/prediction_snapshot_fixture_result_20261001.md` | e680b143，历史副本 |
| [V1实际prediction快照telemetry隔离副本预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/prediction_snapshot_trace_preregistration_20261001.md) | `docs/dynamic_navigation/prediction_snapshot_trace_preregistration_20261001.md` | e680b143，历史副本 |
| [动态障碍：滤波轨迹的接近风险离线复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/proximity_margin_probe_20260926.md) | `docs/dynamic_navigation/proximity_margin_probe_20260926.md` | e680b143，历史副本 |
| [同N原始时间相关提案与原white目标](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/raw_temporal_proposal_20261001.md) | `docs/dynamic_navigation/raw_temporal_proposal_20261001.md` | e680b143，历史副本 |
| [历史扫描坐标来源统一：未修复冻结候选倒排](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/reflected_residual_odom_frame_20260929.md) | `docs/dynamic_navigation/reflected_residual_odom_frame_20260929.md` | e680b143，历史副本 |
| [观测侧镜像残差进入冻结 MPPI 软排序：否决](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/reflected_residual_rank_probe_20260929.md) | `docs/dynamic_navigation/reflected_residual_rank_probe_20260929.md` | e680b143，历史副本 |
| [冻结observer事件顺序的独立TF replay预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_event_tf_preregistration_20261001.md) | `docs/dynamic_navigation/replay_event_tf_preregistration_20261001.md` | e680b143，历史副本 |
| [冻结事件顺序的因果TF replay通过，预载未来TF改变三条记录](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_event_tf_result_20261001.md) | `docs/dynamic_navigation/replay_event_tf_result_20261001.md` | e680b143，历史副本 |
| [原始CDR observer协议预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_receipt_raw_cdr_preregistration_20261001.md) | `docs/dynamic_navigation/replay_receipt_raw_cdr_preregistration_20261001.md` | e680b143，历史副本 |
| [原始CDR只读observer记录协议通过隔离夹具](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_receipt_raw_result_20261001.md) | `docs/dynamic_navigation/replay_receipt_raw_result_20261001.md` | e680b143，历史副本 |
| [冻结replay只读消息身份与接收证据记录器预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_receipt_recorder_preregistration_20261001.md) | `docs/dynamic_navigation/replay_receipt_recorder_preregistration_20261001.md` | e680b143，历史副本 |
| [Typed消息重新序列化不能作为原始CDR身份](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/replay_receipt_typed_failure_result_20261001.md) | `docs/dynamic_navigation/replay_receipt_typed_failure_result_20261001.md` | e680b143，历史副本 |
| [动态避障研究持续工作检查点](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/research_continuation_checkpoint_20260930.md) | `docs/dynamic_navigation/research_continuation_checkpoint_20260930.md` | e680b143，历史副本 |
| [研发方向纠正：三层算法判定](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/research_direction_reset_20261001.md) | `docs/dynamic_navigation/research_direction_reset_20261001.md` | e680b143，历史副本 |
| [动态预测与 MPPI 阶段结论（2026-10-01）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/research_stage_conclusion_20261001.md) | `docs/dynamic_navigation/research_stage_conclusion_20261001.md` | e680b143，历史副本 |
| [扫描中点点估计用于软排序：固定输入否决](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/scan_midpoint_soft_probe_20260928.md) | `docs/dynamic_navigation/scan_midpoint_soft_probe_20260928.md` | e680b143，历史副本 |
| [关键周期的扫描相位与观测侧联合支持](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/scan_phase_view_support_20260929.md) | `docs/dynamic_navigation/scan_phase_view_support_20260929.md` | e680b143，历史副本 |
| [原shadow tracker实际consumer采集预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/shadow_consumer_capture_preregistration_20261001.md) | `docs/dynamic_navigation/shadow_consumer_capture_preregistration_20261001.md` | e680b143，历史副本 |
| [原shadow tracker实际consumer capture：数据冻结](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/shadow_consumer_capture_result_20261001.md) | `docs/dynamic_navigation/shadow_consumer_capture_result_20261001.md` | e680b143，历史副本 |
| [Shadow真实consumer/observer字段与TF结果对齐预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/shadow_receiver_alignment_preregistration_20261001.md) | `docs/dynamic_navigation/shadow_receiver_alignment_preregistration_20261001.md` | e680b143，历史副本 |
| [实际shadow consumer与raw observer对齐结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/shadow_receiver_alignment_result_20261001.md) | `docs/dynamic_navigation/shadow_receiver_alignment_result_20261001.md` | e680b143，历史副本 |
| [动态障碍：软风险时域与 MPPI 控制滤波离线复核](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/short_horizon_filter_probe_20260926.md) | `docs/dynamic_navigation/short_horizon_filter_probe_20260926.md` | e680b143，历史副本 |
| [原始扫描构造箱体 `x` 中心区间：预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/source_x_interval_preregistration_20260930.md) | `docs/dynamic_navigation/source_x_interval_preregistration_20260930.md` | e680b143，历史副本 |
| [扫描端点约束 `x` 中心：已观察夹具源时刻覆盖（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/source_x_interval_result_20260930.md) | `docs/dynamic_navigation/source_x_interval_result_20260930.md` | e680b143，历史副本 |
| [标准任务成本的候选输出对象一致性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/standard_output_cost_alignment_20261001.md) | `docs/dynamic_navigation/standard_output_cost_alignment_20261001.md` | e680b143，历史副本 |
| [时间相关提案协方差双线性项：仅代数必要条件审计](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_covariance_algebra_preregistration_20261001.md) | `docs/dynamic_navigation/temporal_covariance_algebra_preregistration_20261001.md` | e680b143，历史副本 |
| [相关提案协方差代数通过，mapped控制与latent提案需另证](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_covariance_algebra_result_20261001.md) | `docs/dynamic_navigation/temporal_covariance_algebra_result_20261001.md` | e680b143，历史副本 |
| [周期 47 固定扰动时间结构覆盖预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_coverage_preregistration_20261001.md) | `docs/dynamic_navigation/temporal_coverage_preregistration_20261001.md` | e680b143，历史副本 |
| [周期 47 固定时间结构覆盖结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_coverage_result_20261001.md) | `docs/dynamic_navigation/temporal_coverage_result_20261001.md` | e680b143，历史副本 |
| [固定候选30步失效时刻归因预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_failure_timing_preregistration_20261001.md) | `docs/dynamic_navigation/temporal_failure_timing_preregistration_20261001.md` | e680b143，历史副本 |
| [完整三秒失效归因：相关组固定首选均在九步域外破门](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_failure_timing_result_20261001.md) | `docs/dynamic_navigation/temporal_failure_timing_result_20261001.md` | e680b143，历史副本 |
| [冻结时间结构候选的在线风险排序预注册](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_risk_rank_preregistration_20261001.md) | `docs/dynamic_navigation/temporal_risk_rank_preregistration_20261001.md` | e680b143，历史副本 |
| [固定时间结构候选在线风险排序否定结果](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/temporal_risk_rank_result_20261001.md) | `docs/dynamic_navigation/temporal_risk_rank_result_20261001.md` | e680b143，历史副本 |
| [冻结 MPPI 周期的真实箱体评分上界](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/truth_geometry_score_probe_20260929.md) | `docs/dynamic_navigation/truth_geometry_score_probe_20260929.md` | e680b143，历史副本 |
| [冻结周期 162：理想接近信号与 MPPI batch 敏感性](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/truth_near_batch_sensitivity_20260929.md) | `docs/dynamic_navigation/truth_near_batch_sensitivity_20260929.md` | e680b143，历史副本 |
| [动态预测 V1：收缩后的独立实验边界](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/v1_prediction_scope.md) | `docs/dynamic_navigation/v1_prediction_scope.md` | e680b143，历史副本 |
| [动态预测导航 V1 研究冻结说明](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/v1_research_freeze_20260925.md) | `docs/dynamic_navigation/v1_research_freeze_20260925.md` | e680b143，历史副本 |
| [扫描中心残差的观测侧镜像：留出分布诊断](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/view_reflection_calibration_20260929.md) | `docs/dynamic_navigation/view_reflection_calibration_20260929.md` | e680b143，历史副本 |
| [扫描近面几何探针预登记（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_geometry_probe_preregistration_20260929.md) | `docs/dynamic_navigation/visible_face_geometry_probe_preregistration_20260929.md` | e680b143，历史副本 |
| [扫描近面规则：源时刻纵向几何结果（2026-09-29）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_geometry_probe_result_20260929.md) | `docs/dynamic_navigation/visible_face_geometry_probe_result_20260929.md` | e680b143，历史副本 |
| [近面输入九步预测覆盖预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_nine_step_preregistration_20260930.md) | `docs/dynamic_navigation/visible_face_nine_step_preregistration_20260930.md` | e680b143，历史副本 |
| [近面输入九步覆盖结果：源位置准确仍不足以预测运动（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_nine_step_result_20260930.md) | `docs/dynamic_navigation/visible_face_nine_step_result_20260930.md` | e680b143，历史副本 |
| [九步误差归因：物理源速度 oracle 预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_oracle_velocity_preregistration_20260930.md) | `docs/dynamic_navigation/visible_face_oracle_velocity_preregistration_20260930.md` | e680b143，历史副本 |
| [九步归因：物理源速度 oracle 结果（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/visible_face_oracle_velocity_result_20260930.md) | `docs/dynamic_navigation/visible_face_oracle_velocity_result_20260930.md` | e680b143，历史副本 |
| [箱体沿 `x` 轴运动的南北视角输入验证：预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_axis_motion_observation_preregistration_20260930.md) | `docs/dynamic_navigation/x_axis_motion_observation_preregistration_20260930.md` | e680b143，历史副本 |
| [`x` 轴观察九步标签边界说明（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_axis_motion_observation_protocol_note_20260930.md) | `docs/dynamic_navigation/x_axis_motion_observation_protocol_note_20260930.md` | e680b143，历史副本 |
| [箱体沿 `x` 轴运动：新观察输入的固定规则覆盖（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_axis_motion_observation_result_20260930.md) | `docs/dynamic_navigation/x_axis_motion_observation_result_20260930.md` | e680b143，历史副本 |
| [`x` 轴运动北侧近视角：完整九步窗口预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_axis_near_view_preregistration_20260930.md) | `docs/dynamic_navigation/x_axis_near_view_preregistration_20260930.md` | e680b143，历史副本 |
| [`x` 轴运动北侧近视角：完整九步输入门通过（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_axis_near_view_result_20260930.md) | `docs/dynamic_navigation/x_axis_near_view_result_20260930.md` | e680b143，历史副本 |
| [`x` 轴移动箱体 + 移动机器人：独立闭环预登记（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_motion_control_holdout_preregistration_20260930.md) | `docs/dynamic_navigation/x_motion_control_holdout_preregistration_20260930.md` | e680b143，历史副本 |
| [`x` 轴移动箱体闭环：近场输入门失败（2026-09-30）](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/x_motion_control_holdout_result_20260930.md) | `docs/dynamic_navigation/x_motion_control_holdout_result_20260930.md` | e680b143，历史副本 |
| [当前与零均值双Gaussian提案](history_documents_20261004/legacy_e680b143/docs/dynamic_navigation/zero_mean_proposal_20261001.md) | `docs/dynamic_navigation/zero_mean_proposal_20261001.md` | e680b143，历史副本 |

## 方案二：原生 CV DynamicObstacleCritic + guard — 36份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [动态障碍公共输入：几何语义与未知边界](../contracts/dynamic_obstacle_geometry_contract_2027.md) | `docs/contracts/dynamic_obstacle_geometry_contract_2027.md` | 652f14ce，当前文件 |
| [CV DynamicObstacleCritic 第一版](../dynamic_obstacle_critic/architecture.md) | `docs/dynamic_obstacle_critic/architecture.md` | 652f14ce，当前文件 |
| [Controller里程计输入契约实验](../dynamic_obstacle_critic/controller_odom_contract_experiment.md) | `docs/dynamic_obstacle_critic/controller_odom_contract_experiment.md` | 652f14ce，当前文件 |
| [显式条件下的CV支持界：离线开发登记](../dynamic_obstacle_critic/cv_support_bound_experiment.md) | `docs/dynamic_obstacle_critic/cv_support_bound_experiment.md` | 652f14ce，当前文件 |
| [当前几何与未来CV支持：离线因子审计登记](../dynamic_obstacle_critic/cv_support_factor_experiment.md) | `docs/dynamic_obstacle_critic/cv_support_factor_experiment.md` | 652f14ce，当前文件 |
| [DynamicObstacleCritic精确消费证据实验](../dynamic_obstacle_critic/dynamic_consumption_evidence_experiment.md) | `docs/dynamic_obstacle_critic/dynamic_consumption_evidence_experiment.md` | 652f14ce，当前文件 |
| [可见动态表面与 reveal / stopping 证据实验](../dynamic_obstacle_critic/dynamic_surface_reveal_experiment.md) | `docs/dynamic_obstacle_critic/dynamic_surface_reveal_experiment.md` | 652f14ce，当前文件 |
| [全部地面机器人尺寸先验：离线契约实验登记](../dynamic_obstacle_critic/ground_robot_extent_prior_experiment.md) | `docs/dynamic_obstacle_critic/ground_robot_extent_prior_experiment.md` | 652f14ce，当前文件 |
| [静态地图不确定性规划预留实验（预登记）](../dynamic_obstacle_critic/map_uncertainty_experiment.md) | `docs/dynamic_obstacle_critic/map_uncertainty_experiment.md` | 652f14ce，当前文件 |
| [仿真机械足迹契约实验](../dynamic_obstacle_critic/mechanical_footprint_experiment.md) | `docs/dynamic_obstacle_critic/mechanical_footprint_experiment.md` | 652f14ce，当前文件 |
| [原CostCritic累计成本变化：证据实验登记](../dynamic_obstacle_critic/native_cost_accumulator_evidence_experiment.md) | `docs/dynamic_obstacle_critic/native_cost_accumulator_evidence_experiment.md` | 652f14ce，当前文件 |
| [原 CostCritic 与 raw203 地图足迹语义：离线实验登记](../dynamic_obstacle_critic/native_cost_map_contract_experiment.md) | `docs/dynamic_obstacle_critic/native_cost_map_contract_experiment.md` | 652f14ce，当前文件 |
| [原生候选周期证据实验](../dynamic_obstacle_critic/native_cycle_evidence_experiment.md) | `docs/dynamic_obstacle_critic/native_cycle_evidence_experiment.md` | 652f14ce，当前文件 |
| [输入因子的原生聚合后果：离线实验登记](../dynamic_obstacle_critic/native_factor_aggregation_experiment.md) | `docs/dynamic_obstacle_critic/native_factor_aggregation_experiment.md` | 652f14ce，当前文件 |
| [四因子原生权重与个体安全见证：离线实验登记](../dynamic_obstacle_critic/native_factor_safe_mass_experiment.md) | `docs/dynamic_obstacle_critic/native_factor_safe_mass_experiment.md` | 652f14ce，当前文件 |
| [原个体控制约束与SG阶段：离线实验登记](../dynamic_obstacle_critic/native_individual_stage_experiment.md) | `docs/dynamic_obstacle_critic/native_individual_stage_experiment.md` | 652f14ce，当前文件 |
| [原300 raw轨迹地图查询：离线实验登记](../dynamic_obstacle_critic/native_raw_map_contract_experiment.md) | `docs/dynamic_obstacle_critic/native_raw_map_contract_experiment.md` | 652f14ce，当前文件 |
| [原生完整三秒安全控制见证实验](../dynamic_obstacle_critic/native_safe_control_witness_experiment.md) | `docs/dynamic_obstacle_critic/native_safe_control_witness_experiment.md` | 652f14ce，当前文件 |
| [原生聚合 SG 前后路径：离线实验登记](../dynamic_obstacle_critic/native_sg_stage_experiment.md) | `docs/dynamic_obstacle_critic/native_sg_stage_experiment.md` | 652f14ce，当前文件 |
| [原生成本、权重与聚合归因实验](../dynamic_obstacle_critic/native_weight_attribution_experiment.md) | `docs/dynamic_obstacle_critic/native_weight_attribution_experiment.md` | 652f14ce，当前文件 |
| [连续目标对照后的动态近场失败复盘](../dynamic_obstacle_critic/nearfield_failure_review.md) | `docs/dynamic_obstacle_critic/nearfield_failure_review.md` | 652f14ce，当前文件 |
| [最近真实观测锚点 CV：接口与条件支持实验登记](../dynamic_obstacle_critic/observation_anchor_cv_experiment.md) | `docs/dynamic_obstacle_critic/observation_anchor_cv_experiment.md` | 652f14ce，当前文件 |
| [观测锚点＋完整直径名义代理：实际消费实验登记](../dynamic_obstacle_critic/observation_diameter_consumption_experiment.md) | `docs/dynamic_obstacle_critic/observation_diameter_consumption_experiment.md` | 652f14ce，当前文件 |
| [实际扫描成员与跟踪关联证据登记](../dynamic_obstacle_critic/observed_surface_members_experiment.md) | `docs/dynamic_obstacle_critic/observed_surface_members_experiment.md` | 652f14ce，工作区文档有更新 |
| [聚合/SG路径的纯CV目标：离线实验登记](../dynamic_obstacle_critic/processed_cv_objective_experiment.md) | `docs/dynamic_obstacle_critic/processed_cv_objective_experiment.md` | 652f14ce，当前文件 |
| [原始rollout与实际Dynamic目标：离线审计登记](../dynamic_obstacle_critic/raw_rollout_objective_experiment.md) | `docs/dynamic_obstacle_critic/raw_rollout_objective_experiment.md` | 652f14ce，当前文件 |
| [路线切换前仓库审查（2026-10-01）](../dynamic_obstacle_critic/repository_audit.md) | `docs/dynamic_obstacle_critic/repository_audit.md` | 652f14ce，当前文件 |
| [动态障碍尺寸参考：2026制作规范](../dynamic_obstacle_critic/robot_extent_manual_review.md) | `docs/dynamic_obstacle_critic/robot_extent_manual_review.md` | 652f14ce，当前文件 |
| [连续静态目标的等价性能实验](../dynamic_obstacle_critic/soft_clearance_performance_experiment.md) | `docs/dynamic_obstacle_critic/soft_clearance_performance_experiment.md` | 652f14ce，当前文件 |
| [连续地图预留目标实验（预登记）](../dynamic_obstacle_critic/soft_map_clearance_experiment.md) | `docs/dynamic_obstacle_critic/soft_map_clearance_experiment.md` | 652f14ce，当前文件 |
| [CV 路线第二阶段：失败诊断与接口修复](../dynamic_obstacle_critic/stage2_plan.md) | `docs/dynamic_obstacle_critic/stage2_plan.md` | 652f14ce，当前文件 |
| [第二阶段进度记录](../dynamic_obstacle_critic/stage2_progress.md) | `docs/dynamic_obstacle_critic/stage2_progress.md` | 652f14ce，当前文件 |
| [原生静态刹停约束实验（预登记）](../dynamic_obstacle_critic/static_stopping_experiment.md) | `docs/dynamic_obstacle_critic/static_stopping_experiment.md` | 652f14ce，当前文件 |
| [第一版验收与最终结论（2026-10-01）](../dynamic_obstacle_critic/validation.md) | `docs/dynamic_obstacle_critic/validation.md` | 652f14ce，当前文件 |
| [可见双面矩形观测实验](../dynamic_obstacle_critic/visible_box_geometry_experiment.md) | `docs/dynamic_obstacle_critic/visible_box_geometry_experiment.md` | 652f14ce，当前文件 |
| [rm_dynamic_obstacle_critic](../../src/rm_dynamic_obstacle_critic/README.md) | `src/rm_dynamic_obstacle_critic/README.md` | 652f14ce，当前文件 |

## 方案三：HWS 重新对照与限时净收益计划（实验未启动） — 3份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [HWSentryNav26 选择性迁移审计 — 2026-10-03](hws_migration_audit_20261003.md) | `docs/dynamic_navigation/hws_migration_audit_20261003.md` | 652f14ce，当前文件 |
| [HWS 与当前动态预测路线逐模块对照](hws_vs_current_prediction_analysis.md) | `docs/dynamic_navigation/hws_vs_current_prediction_analysis.md` | 652f14ce，当前文件 |
| [显式动态预测：一个月退出计划](prediction_one_month_exit_plan.md) | `docs/dynamic_navigation/prediction_one_month_exit_plan.md` | 652f14ce，工作区文档有更新 |

## 前置：HWS 选择性迁移、shadow tracker 与通道准入 — 12份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [动态通道准入 Shadow 验证](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/dynamic_clearance_shadow_validation.md) | `docs/hwsentry_migration/dynamic_clearance_shadow_validation.md` | e680b143，历史副本 |
| [动态目标与 Nav2 Humble MPPI 集成评估](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/dynamic_mppi_integration_plan.md) | `docs/hwsentry_migration/dynamic_mppi_integration_plan.md` | e680b143，历史副本 |
| [Execution Contract 与 Annotated Path 审计](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/execution_and_annotated_path_review.md) | `docs/hwsentry_migration/execution_and_annotated_path_review.md` | e680b143，历史副本 |
| [Factor-Graph LIO 实验计划](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/factor_graph_lio_experiment_plan.md) | `docs/hwsentry_migration/factor_graph_lio_experiment_plan.md` | e680b143，历史副本 |
| [开源选择性迁移冻结与验证交接](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/frozen_validation_handoff.md) | `docs/hwsentry_migration/frozen_validation_handoff.md` | e680b143，历史副本 |
| [HWSentryNav26 Phase1 Linux Validation Report](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/hwsentrynav26_phase1_linux_validation_report_20260824.md) | `docs/hwsentry_migration/hwsentrynav26_phase1_linux_validation_report_20260824.md` | e680b143，历史副本 |
| [离线射线证据地图清理原型](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/offline_map_cleanup.md) | `docs/hwsentry_migration/offline_map_cleanup.md` | e680b143，历史副本 |
| [HWSentryNav26 选择性迁移 Phase 1](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/phase1_design.md) | `docs/hwsentry_migration/phase1_design.md` | e680b143，历史副本 |
| [Phase 1 Linux 与旧车验证流程](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/phase1_linux_validation.md) | `docs/hwsentry_migration/phase1_linux_validation.md` | e680b143，历史副本 |
| [HWSentryNav26 选择性迁移状态](history_documents_20261004/legacy_e680b143/docs/hwsentry_migration/selective_migration_status.md) | `docs/hwsentry_migration/selective_migration_status.md` | e680b143，历史副本 |
| [rm_dynamic_clearance](history_documents_20261004/legacy_e680b143/src/rm_dynamic_clearance/README.md) | `src/rm_dynamic_clearance/README.md` | e680b143，历史副本 |
| [rm_dynamic_obstacle_tracking](history_documents_20261004/legacy_e680b143/src/rm_dynamic_obstacle_tracking/README.md) | `src/rm_dynamic_obstacle_tracking/README.md` | e680b143，历史副本 |

## 前置：T-DT / STVL / MPPI 及早期动态 guard 记录 — 38份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [东北大学 T-DT 导航吸收：项目判断与分阶段实施](history_documents_20261004/legacy_e680b143/docs/tdt_migration/assessment_and_plan.md) | `docs/tdt_migration/assessment_and_plan.md` | e680b143，历史副本 |
| [P2B 动态失败时间窗：观测、地图与停令的离线审查](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/dynamic_failure_analysis_20260922/validation_20260922.md) | `docs/tdt_migration/evidence/dynamic_failure_analysis_20260922/validation_20260922.md` | e680b143，历史副本 |
| [P2B 动态安全：已知扫掠区命令过滤首例](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/validation_20260923.md) | `docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/validation_20260923.md` | e680b143，历史副本 |
| [P2B 移动障碍首例验证：到点成功，动态安全未通过](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/dynamic_reference_20260922/validation_20260922.md) | `docs/tdt_migration/evidence/dynamic_reference_20260922/validation_20260922.md` | e680b143，历史副本 |
| [P2B 动态安全：HWNav 先例核对与冻结首例制动反事实](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/dynamic_safety_contract_20260923/validation_20260923.md) | `docs/tdt_migration/evidence/dynamic_safety_contract_20260923/validation_20260923.md` | e680b143，历史副本 |
| [P2B 移动障碍：扫掠路由与命令停车候选首例](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/dynamic_viability_guard_20260924/validation_20260924.md) | `docs/tdt_migration/evidence/dynamic_viability_guard_20260924/validation_20260924.md` | e680b143，历史副本 |
| [P2B endpoint_witness_v1：精确端点证据与实际验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/endpoint_witness_20260915/validation_20260916.md) | `docs/tdt_migration/evidence/endpoint_witness_20260915/validation_20260916.md` | e680b143，历史副本 |
| [Chassis Heading Motion Policy](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/heading_follow_20260916/reused_chassis_heading_policy.md) | `docs/tdt_migration/evidence/heading_follow_20260916/reused_chassis_heading_policy.md` | e680b143，历史副本 |
| [P2B 航向跟随路径 A/B：2026-09-16 实际回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/heading_follow_20260916/validation_20260916.md) | `docs/tdt_migration/evidence/heading_follow_20260916/validation_20260916.md` | e680b143，历史副本 |
| [P2B 动态失败：局部地图内部更新时间与观测年龄诊断](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/map_age_diagnostic_20260923/validation_20260923.md) | `docs/tdt_migration/evidence/map_age_diagnostic_20260923/validation_20260923.md` | e680b143，历史副本 |
| [P2B 移动障碍：MPPI 同周期诊断首例](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922/validation_20260923.md) | `docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922/validation_20260923.md` | e680b143，历史副本 |
| [P2B 动态闭环：MPPI odom 输入与冻结周期反事实重放](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/mppi_replay_20260923/validation_20260923.md) | `docs/tdt_migration/evidence/mppi_replay_20260923/validation_20260923.md` | e680b143，历史副本 |
| [P2B 静态仿真验证回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/p2b_static_20260913/validation_20260913.md) | `docs/tdt_migration/evidence/p2b_static_20260913/validation_20260913.md` | e680b143，历史副本 |
| [P2B static_v2 静态仿真验证回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/p2b_static_v2_20260913/validation_20260913.md) | `docs/tdt_migration/evidence/p2b_static_v2_20260913/validation_20260913.md` | e680b143，历史副本 |
| [P2B static_v3 代价语义修复验证回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/p2b_static_v3_20260914/validation_20260914.md) | `docs/tdt_migration/evidence/p2b_static_v3_20260914/validation_20260914.md` | e680b143，历史副本 |
| [P2B static_v4 静态仿真验证回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/p2b_static_v4_20260914/validation_20260914.md) | `docs/tdt_migration/evidence/p2b_static_v4_20260914/validation_20260914.md` | e680b143，历史副本 |
| [P2B static_v5：端点连接修复后的实际验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/p2b_static_v5_20260915/validation_20260915.md) | `docs/tdt_migration/evidence/p2b_static_v5_20260915/validation_20260915.md` | e680b143，历史副本 |
| [P2A 复现命令](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/phase2_20260913/commands.md) | `docs/tdt_migration/evidence/phase2_20260913/commands.md` | e680b143，历史副本 |
| [P2B 动态安全：仿真停车响应与完整障碍扫掠影子检查](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/sim_stop_probe_20260923/validation_20260923.md) | `docs/tdt_migration/evidence/sim_stop_probe_20260923/validation_20260923.md` | e680b143，历史副本 |
| [P2B 最新快照复查：隔离实验验证回传](history_documents_20261004/legacy_e680b143/docs/tdt_migration/evidence/snapshot_revalidation_20260922/validation_20260922.md) | `docs/tdt_migration/evidence/snapshot_revalidation_20260922/validation_20260922.md` | e680b143，历史副本 |
| [P2B：Nav2 代价语义修复与 static_v3 交接](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_costmap_semantics_handoff.md) | `docs/tdt_migration/p2b_costmap_semantics_handoff.md` | e680b143，历史副本 |
| [P2B 精确端点连接修复与验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_endpoint_connections.md) | `docs/tdt_migration/p2b_endpoint_connections.md` | e680b143，历史副本 |
| [P2B 精确端点拒收证据](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_endpoint_witness.md) | `docs/tdt_migration/p2b_endpoint_witness.md` | e680b143，历史副本 |
| [P2B 航向跟随路径：独立 A/B](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_heading_follow_experiment.md) | `docs/tdt_migration/p2b_heading_follow_experiment.md` | e680b143，历史副本 |
| [P2B 新车 footprint 审查与参考几何验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_new_car_geometry_review.md) | `docs/tdt_migration/p2b_new_car_geometry_review.md` | e680b143，历史副本 |
| [离线姿态几何检查器开发检查点](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_pose_geometry_checkpoint.md) | `docs/tdt_migration/p2b_pose_geometry_checkpoint.md` | e680b143，历史副本 |
| [P2B 记录器修复复核与 static_v2 重验交接](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_recorder_fix_20260913.md) | `docs/tdt_migration/p2b_recorder_fix_20260913.md` | e680b143，历史副本 |
| [P2B 静态仿真验证回传（复制为新报告后填写）](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_result_template.md) | `docs/tdt_migration/p2b_result_template.md` | e680b143，历史副本 |
| [P2B：sanitizer 构建链修复与 static_v4 交接](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_sanitizer_chain_handoff.md) | `docs/tdt_migration/p2b_sanitizer_chain_handoff.md` | e680b143，历史副本 |
| [P2B 滚动快照拒收诊断、修复与验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_snapshot_revalidation_review.md) | `docs/tdt_migration/p2b_snapshot_revalidation_review.md` | e680b143，历史副本 |
| [P2B 终点合同：姿态相关连接的下一阶段设计](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_terminal_contract_design.md) | `docs/tdt_migration/p2b_terminal_contract_design.md` | e680b143，历史副本 |
| [P2B：由后续动作定义终端可行性与自动选点](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_terminal_selection_design.md) | `docs/tdt_migration/p2b_terminal_selection_design.md` | e680b143，历史副本 |
| [Terminal selection：离线到执行的验收计划](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_terminal_selection_validation_plan.md) | `docs/tdt_migration/p2b_terminal_selection_validation_plan.md` | e680b143，历史副本 |
| [P2B 静态仿真：Terra 验证交接](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_terra_handoff.md) | `docs/tdt_migration/p2b_terra_handoff.md` | e680b143，历史副本 |
| [P2B 无实车期间的推进边界](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_without_hardware_plan.md) | `docs/tdt_migration/p2b_without_hardware_plan.md` | e680b143，历史副本 |
| [P2B 持久工作记录](history_documents_20261004/legacy_e680b143/docs/tdt_migration/p2b_work_status.md) | `docs/tdt_migration/p2b_work_status.md` | e680b143，历史副本 |
| [T-DT 规划候选：2026-09-12 离线验证](history_documents_20261004/legacy_e680b143/docs/tdt_migration/validation_20260912.md) | `docs/tdt_migration/validation_20260912.md` | e680b143，历史副本 |
| [T-DT P2A：2026-09-13 统一几何的离线比较](history_documents_20261004/legacy_e680b143/docs/tdt_migration/validation_20260913.md) | `docs/tdt_migration/validation_20260913.md` | e680b143，历史副本 |

## 基线、契约、实车与关联背景文档 — 42份

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [2027 Architecture Decision](../2027_architecture_decision.md) | `docs/2027_architecture_decision.md` | 652f14ce，当前文件 |
| [2027 Phase 1 Plan](../2027_phase1_plan.md) | `docs/2027_phase1_plan.md` | 652f14ce，当前文件 |
| [Chassis Heading Motion Policy](history_documents_20261004/other_8598b8c0/docs/chassis_heading_motion_policy.md) | `docs/chassis_heading_motion_policy.md` | 8598b8c0，历史副本 |
| [Competition Capability Status](../competition_capability_status.md) | `docs/competition_capability_status.md` | d735ee12，当前文件 |
| [2027 TF Contract](../contracts/tf_contract_2027.md) | `docs/contracts/tf_contract_2027.md` | 652f14ce，当前文件 |
| [2027 Topic Contract](../contracts/topic_contract_2027.md) | `docs/contracts/topic_contract_2027.md` | 652f14ce，当前文件 |
| [导航小型遗留项冻结记录（2026-09-03）](history_documents_20261004/other_2c0620c1/docs/deferred_navigation_issues_20260903.md) | `docs/deferred_navigation_issues_20260903.md` | 2c0620c1，历史副本 |
| [Docker Usage](../docker_usage.md) | `docs/docker_usage.md` | d735ee12，当前文件 |
| [Gazebo Fortress Systems Record](../external/gazebo_fortress_systems.md) | `docs/external/gazebo_fortress_systems.md` | d735ee12，当前文件 |
| [Nav2 MPPI PathAlign 隔离边界修正来源记录](history_documents_20261004/legacy_e680b143/docs/external/nav2_mppi_path_align_guard.md) | `docs/external/nav2_mppi_path_align_guard.md` | e680b143，历史副本 |
| [Old-Car Field Debug Competition Tutorial](../field_debug_competition_tutorial.md) | `docs/field_debug_competition_tutorial.md` | d735ee12，当前文件 |
| [Old-Car Navigation Integrity Field Validation Plan](history_documents_20261004/legacy_e680b143/docs/navigation_integrity/field_validation_plan.md) | `docs/navigation_integrity/field_validation_plan.md` | e680b143，历史副本 |
| [Navigation Integrity 第一版实现报告](history_documents_20261004/legacy_e680b143/docs/navigation_integrity/implementation_report.md) | `docs/navigation_integrity/implementation_report.md` | e680b143，历史副本 |
| [Shadow Localization Verifier 设计](history_documents_20261004/legacy_e680b143/docs/navigation_integrity/localization_verifier_design.md) | `docs/navigation_integrity/localization_verifier_design.md` | e680b143，历史副本 |
| [Mission / Nav2 Stale Result 审计](history_documents_20261004/legacy_e680b143/docs/navigation_integrity/mission_stress_audit.md) | `docs/navigation_integrity/mission_stress_audit.md` | e680b143，历史副本 |
| [新场地比赛部署与狗洞穿越作业手册](history_documents_20261004/other_8598b8c0/docs/new_field_competition_deployment_runbook.md) | `docs/new_field_competition_deployment_runbook.md` | 8598b8c0，历史副本 |
| [Old-Car Debug Knowledge Base](../old_car_debug_knowledge_base.md) | `docs/old_car_debug_knowledge_base.md` | d735ee12，当前文件 |
| [Old-Car Self-Test Guide](../old_car_self_test_guide.md) | `docs/old_car_self_test_guide.md` | d735ee12，当前文件 |
| [Phase 1.5D Multi-Obstacle Course Validation](../phase1_5d_course_validation.md) | `docs/phase1_5d_course_validation.md` | d735ee12，当前文件 |
| [Phase 2I Managed Mapping Pipeline](../phase2i_managed_mapping.md) | `docs/phase2i_managed_mapping.md` | d735ee12，当前文件 |
| [Phase 2L Dual-LiDAR Obstacle Fusion](../phase2l_dual_lidar_obstacle_fusion.md) | `docs/phase2l_dual_lidar_obstacle_fusion.md` | d735ee12，当前文件 |
| [Phase 3 Competition Software Validation](../phase3_software_validation.md) | `docs/phase3_software_validation.md` | d735ee12，当前文件 |
| [Phase 3D Competition Bringup And Readiness](../phase3d_competition_bringup.md) | `docs/phase3d_competition_bringup.md` | d735ee12，当前文件 |
| [Project Structure And Documentation Index](../project_structure_and_documentation_index.md) | `docs/project_structure_and_documentation_index.md` | 652f14ce，工作区文档有更新 |
| [New-Car Ramp Perception Shadow Validation](history_documents_20261004/other_8598b8c0/docs/ramp_perception_shadow_validation.md) | `docs/ramp_perception_shadow_validation.md` | 8598b8c0，历史副本 |
| [Runtime Profiles](../runtime_profiles.md) | `docs/runtime_profiles.md` | 652f14ce，当前文件 |
| [Old-Car Field Debug Competition 更新报告](../validation/old_car_field_debug_20260717.md) | `docs/validation/old_car_field_debug_20260717.md` | d735ee12，当前文件 |
| [fresh03 候选有限现场验收](../validation/old_car_fresh03_field_validation_20260720.md) | `docs/validation/old_car_fresh03_field_validation_20260720.md` | d735ee12，当前文件 |
| [旧车动态障碍全局重规划验证记录（2026-08-06）](history_documents_20261004/legacy_e680b143/docs/validation/old_car_global_dynamic_replanning_20260806.md) | `docs/validation/old_car_global_dynamic_replanning_20260806.md` | e680b143，历史副本 |
| [老车高速自转全局定位漂移：根因分析、工程治理与实车闭环报告](history_documents_20261004/legacy_e680b143/docs/validation/old_car_high_spin_localization_engineering_report_20260825.md) | `docs/validation/old_car_high_spin_localization_engineering_report_20260825.md` | e680b143，历史副本 |
| [旧车导航当前状态与后续收敛报告](../validation/old_car_navigation_status_and_roadmap_20260720.md) | `docs/validation/old_car_navigation_status_and_roadmap_20260720.md` | d735ee12，当前文件 |
| [老车无标注坡道实车录包分析与共享跟踪修复报告](history_documents_20261004/legacy_e680b143/docs/validation/old_car_ramp_live_tracking_report_20260830.md) | `docs/validation/old_car_ramp_live_tracking_report_20260830.md` | e680b143，历史副本 |
| [旧车无标注坡道选择性验证](history_documents_20261004/legacy_e680b143/docs/validation/old_car_ramp_selective_validation.md) | `docs/validation/old_car_ramp_selective_validation.md` | e680b143，历史副本 |
| [PCD/PGM 地图脏点诊断、收敛与交付总结报告](../validation/pcd_pgm_dirty_map_end_to_end_report_20260720.md) | `docs/validation/pcd_pgm_dirty_map_end_to_end_report_20260720.md` | d735ee12，当前文件 |
| [PCD/PGM 地图脏点诊断工具实施记录](../validation/pcd_pgm_map_quality_implementation_20260717.md) | `docs/validation/pcd_pgm_map_quality_implementation_20260717.md` | d735ee12，当前文件 |
| [rm_competition_interfaces](../../src/rm_competition_interfaces/README.md) | `src/rm_competition_interfaces/README.md` | 652f14ce，当前文件 |
| [rm_competition_mission](../../src/rm_competition_mission/README.md) | `src/rm_competition_mission/README.md` | d735ee12，当前文件 |
| [rm_map_tools](../../src/rm_map_tools/README.md) | `src/rm_map_tools/README.md` | d735ee12，当前文件 |
| [rm_mid360_driver_bridge](../../src/rm_mid360_driver_bridge/README.md) | `src/rm_mid360_driver_bridge/README.md` | d735ee12，当前文件 |
| [rm_nav_config](../../src/rm_nav_config/README.md) | `src/rm_nav_config/README.md` | d735ee12，当前文件 |
| [rm_navigation_launch](../../src/rm_navigation_launch/README.md) | `src/rm_navigation_launch/README.md` | d735ee12，当前文件 |
| [rm_simulation](../../src/rm_simulation/README.md) | `src/rm_simulation/README.md` | d735ee12，当前文件 |

## 四、保存与本次验证

本目录与新历史阅读副本、来源清单、计划暂停说明和项目文档索引一起保存在 `experiment/dynamic-documents-archive-20261004`。本次提交只涉及文档，不提交既有tracker/core/member证据工具改动，不移动或删除core，不合并/push，不运行仿真/实车。

校验：清单的每个读取文件存在；历史副本与对应Git blob逐字节相同；本目录的文件链接存在；既有dirty文件摘要/大小、main、旧research与冻结tag保持。没有重新跑旧实验，故暂停记录和文档整理不构成新增安全验收。

## 新增独立路线：Temporal MPC — 25份（已冻结）

直接main派生，最后实现2d95b026，研究冻结04291a41；不恢复原三轮或一个月计划。R1占用层与timed-reference动态闭环未找到已完成证据，旧V1不与R1混称。

| 文档 | 原始路径 | 来源 |
|---|---|---|
| [Temporal MPC：独立最小研究架构](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_architecture.md) | `docs/dynamic_navigation/temporal_mpc_architecture.md` | 04291a41，冻结tag逐字节副本 |
| [目标可开放场景与有限候选登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_candidates_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_candidates_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [目标开放场景：单候选与有限候选对照](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_candidates_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_candidates_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 执行模型与回退保护登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_execution_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_execution_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 执行重验、静态走廊与回退保护结果](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_execution_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_execution_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 离线第一轮预登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_experiment_registration.md) | `docs/dynamic_navigation/temporal_mpc_experiment_registration.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 研究冻结记录](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_freeze_20261004.md) | `docs/dynamic_navigation/temporal_mpc_freeze_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 实际仿真输入与配对实验预登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_gazebo_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_gazebo_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 实际输入、Gazebo 配对与阶段结论](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_gazebo_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_gazebo_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 侧向参考与保护节拍登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_lateral_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_lateral_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [侧向参考、生产节拍与速度重锚结果](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_lateral_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_lateral_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [新聊天记录中的论文：核验与当前实验对照](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_literature_review_20261004.md) | `docs/dynamic_navigation/temporal_mpc_literature_review_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC 进度与结果](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_progress.md) | `docs/dynamic_navigation/temporal_mpc_progress.md` | 04291a41，冻结tag逐字节副本 |
| [v2速度重锚投影：物理复验登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_projection_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_projection_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [v2速度重锚投影物理复验与模型占用诊断](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_projection_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_projection_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Worker时序与提案老化诊断登记](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_timing_registration_20261004.md) | `docs/dynamic_navigation/temporal_mpc_timing_registration_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Worker时序与提案老化诊断结果](history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_timing_results_20261004.md) | `docs/dynamic_navigation/temporal_mpc_timing_results_20261004.md` | 04291a41，冻结tag逐字节副本 |
| [Temporal MPC offline prototype](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/README.md) | `experiments/temporal_mpc/README.md` | 04291a41，冻结tag逐字节副本 |
| [T-DT前端桥接](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/frontend/README.md) | `experiments/temporal_mpc/frontend/README.md` | 04291a41，冻结tag逐字节副本 |
| [Isolated actual Gazebo research](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/gazebo/README.md) | `experiments/temporal_mpc/gazebo/README.md` | 04291a41，冻结tag逐字节副本 |
| [Humble 双控制器实验](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/integration/README.md) | `experiments/temporal_mpc/integration/README.md` | 04291a41，冻结tag逐字节副本 |
| [Frozen offline project references](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/reference_inputs/README.md) | `experiments/temporal_mpc/reference_inputs/README.md` | 04291a41，冻结tag逐字节副本 |
| [动态障碍公共输入：几何语义与未知边界](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/reference_inputs/dynamic_obstacle_geometry_contract_2027.md) | `experiments/temporal_mpc/reference_inputs/dynamic_obstacle_geometry_contract_2027.md` | 04291a41，冻结tag逐字节副本 |
| [rm_competition_interfaces](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/ros2/rm_competition_interfaces/README.md) | `experiments/temporal_mpc/ros2/rm_competition_interfaces/README.md` | 04291a41，冻结tag逐字节副本 |
| [rm_dynamic_obstacle_tracking](history_documents_20261004/temporal_mpc_04291a41/experiments/temporal_mpc/ros2/rm_dynamic_obstacle_tracking/README.md) | `experiments/temporal_mpc/ros2/rm_dynamic_obstacle_tracking/README.md` | 04291a41，冻结tag逐字节副本 |

### 原始聊天参考（非指令、非实验接受）

- [mpc_proposal](history_documents_20261004/reference_chats/mpc_proposal.txt).
- [mpc_literature_chat](history_documents_20261004/reference_chats/mpc_literature_chat.txt).

MPC完整源码/二进制/JSONL/失败保存在冻结tag；阅读副本只含文档及选定引用资产。原347份资料的reading hash保持，逐项原文和来源可在合订本/机器清单查阅。
