# 路线切换前仓库审查（2026-10-01）

审查发生在任何实现修改之前。原 HEAD 为 `codex/dynamic-differential-risk` 的 `e680b1430db6efd8dc601d5585ee207e5f9b42a4`（docs: close cost alignment and pause dynamic research）。已跟踪工作区干净，未知未跟踪 `core` 保留。原分支比缓存 origin 同名分支 `e067c37` 超前 230 提交；历史冻结分支 `origin/experiment/dynamic-prediction-v1=f85149e` 和 tag `dynamic-prediction-v1-frozen-20260925` 已存在。

稳定 `main` 与缓存 `origin/main` 均为 `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。网络限制解除后的只读 Gitee 查询仍需凭据，因此没有确认远端实时状态，没有 fetch/push。全分支、tag、提交图、main..research diff 和研究源码已检查；差异共 4,338 文件、1,265,859 新增行、215 删除行，包含许多无关迁移和历史证据，整体合并不合适。完整路径差异保存在 `research_inventory.json`。

原研究头新增 annotated tag `research/dynamic-differential-risk-frozen-20261001`，原分支保留。新 `feature/dynamic-obstacle-critic` 直接从 main 创建，没有 merge 研究分支，没有 cherry-pick 整个研究提交；没有修改原研究证据。当前工作区切换至新分支，旧 ignored build/install/log 仍在原位置，新构建仅使用隔离 /tmp 输出。

## 实际代码归类及边界

| 能力 | 审阅源码/历史 | 判断与迁移 |
|---|---|---|
| 感知/tracker | `src/rm_dynamic_obstacle_tracking/.../core.py`, `dynamic_obstacle_tracker_node.py`; 历史 `22117ce`, `5209607`, `cc254b4` | 静态 OccupancyMap 剔除、质心静态距离过滤、聚类、门控关联、逐轴位置速度 Kalman、tentative/confirmed/coasting/lost 状态及过期删除。整个独立包按 e680b14 复制，原 shadow 配置不改 |
| API/预测 | `DynamicObstaclePrediction{,Array}.msg`; `_snapshot()` / `_publish_predictions()` | 已有 id、xy、vxy、size、源时间、最近观测时间、完整性、frame、版本。没有输出 covariance/confidence；内部 Kalman 方差没有外露。旧默认显示轨迹衰减且速度可限幅，而消息 velocity 是估计速度。只复制两消息并登记生成；新独立配置设置 tau=0、30×0.1，critic 根据原状态重算 CV |
| 原动态 critic | `experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp`, `geometry.hpp`; `462896d`, `70ec7bd` | pluginlib 原生 CriticFunction 扩展。已按时间评分，不是二维预测 costmap。只接受 confirmed，要求消息 frame 与 costmap 完全相同；过期时跳过动态评分；约 1s 时域、已知夹具0.45×0.55尺寸、加速度包络、硬碰撞1e6，后增加 uniform-center overlap。在旧分支冻结，不整包迁移 |
| sampler | `frozen_cycle/batch_sampling_probe.py`, `raw_temporal_proposal.py`, `zero_mean_proposal.py`, temporal covariance/preimage 工具 | std/batch 诊断、AR(1) 和双 Gaussian proposal 为离线生成/冻结/密度修正实验；不是正式 MPPI noise_generator 的运行补丁。新分支完全不迁移 |
| CA/future-risk/candidate ranking | `near_field_kinematic_risk.py`, `candidate_output_risk.py`, `temporal_risk_rank_probe.py`, `clearance_shape_risk.py`, 原生评分 replay C++ | 候选输入支持、CV/CA对照、返回控制域评分与重排；隔离实验，不迁移 |
| footprint/碰撞几何 | 旧 `geometry.hpp`、`simulation_geometry.py`, `audit_geometry.py`, `dynamic_metrics.py` | 仅迁移无 ROS 的 Point/Box、polygon transform、point-segment distance、convex polygon-box distance。包络增长、概率 overlap/排名删除于新副本；原文件留存。新增 polygon-circle clearance 供 critic/guard 共享 |
| safety guard | `docs/tdt_migration/evidence/dynamic_guard_{pilot,horizon,scaled,viability}_*/`, `guard_core.py`, `guard_node.py` | 旧 guard 依赖夹具 sweep、特殊响应模型、缩放/可行性逻辑。审阅后决定不直接迁移；新独立节点只模拟当前命令、测量速度和刹停尾段，公共 CV/几何复用 |
| 仿真/真值 | main已有 `rm_simulation/models/moving_obstacle.sdf`, `moving_obstacle_controller.cpp`, `course_dynamic_bridge.yaml`; 旧近场南/北/x-fast launch 与物理真值审计 | 主线已有0.45×0.55箱体、slider、实际物理碰撞和 odometry ground truth，直接引用，不复制。旧 /work 绝对路径场景冻结，不整目录迁移 |
| 测试/验收 | tracker tests、旧 three-gate/replay/hash/native probes | tracker测试原样迁移；旧复杂自动验收依赖研究树，不移入产品包。新模型、原生插件DDS、guard运行测试及闭环报告单独提供，原严格几何/控制/推进门不放宽 |
| zero witness | `late_zero_control_witness.py` 和对应报告、manifest | 旧工具依赖大量冻结库、SG历史、原生binary身份，不直接搬入运行包。保留旧三秒实测状态零控制见证引用；新 guard 提供零命令及测量动量检查，不冒称理想静止安全或复制旧 native witness 的等价证明 |
| logging/diagnostics | tracker DiagnosticArray/MarkerArray；旧 `cycle_trace.{hpp,cpp}`, `prepare_trace.py`, raw receipt capture | tracker日志/marker迁移；旧 observer/大规模dump冻结。新 critic 记录状态、CV marker、动态成本范围、最低动态成本候选的间隙/TTC；guard记录最终命令的间隙/TTC和pass/brake理由 |

## Nav2 MPPI 内部到底改过什么

正式工程没有 vendored `nav2_mppi_controller` 包，也不是控制器 wrapper。旧运行动态评分是原生 plugin 扩展；调试/回放阶段确实构建过隔离 upstream 源码副本，不能笼统说“从未改过 MPPI”：

1. `prepare_trace.py` 向 `controller.cpp` 添加周期锁定/输出记录；`optimizer.cpp` 添加 initial、采样控制/状态、评分、softmax概率、聚合、滤波前后控制和设置遥测；`critic_manager.cpp` 记录逐 critic 成本与 fail；`cost_critic.cpp` 记录碰撞 mask。
2. 诊断 CMake 添加 trace writer，`mppi_critics` 链接 `mppi_controller` 保证同一 observer 状态。旧 `PredictionV1Critic` 副本也添加输入 snapshot 遥测。
3. `prepare_guarded_nav2.py` 在 `tools/utils.hpp` 的 `findClosestPathPt`/lower_bound helper 添加 `iter==end` 检查，避免 PathAlign 末尾解引用。这是隔离有界评分反事实，不是正式部署通过。
4. `prepare_filter_history_trace.py` 只向隔离 `optimizer.cpp` 两处追加实际四项 Savitzky-Golay 历史 dump，并逐字节证明去掉遥测后恢复原文；prediction snapshot/goal UUID/graph fixture 也是身份记录副本。
5. 离线 replay/probe 能提供冻结噪声、additive costs、聚合/限幅/滤波反事实；它们是测试 worker，不是正式 sampler 路线。AR、mixture 的 raw density 修正没有部署到 MPPI optimizer。

这些副本和所有 sampler/CA/ranking 代码均留在旧 research，当前新插件链接镜像中原装 Nav2 Humble 1.1.20，不使用研究 observer/guarded core。

## 复用的实际限制

tracker 的 size 是可见点簇范围，不保证完整障碍 footprint；新第一版使用 circumscribed radius + YAML minimum radius，仍需实车尺寸/遮挡验收。原 shadow tracker 的最小位移确认门意味着静止对象可能一直 tentative，这类对象继续由当前 costmap 负责。主线移动夹具为正弦往复，长 CV 外推无法保证换向预测正确。上述限制作为感知/预测问题保留，不用恢复 CA、调 sampler 或降低安全门掩盖。
