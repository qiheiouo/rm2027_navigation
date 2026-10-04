# 动态预测 V1：独立 MPPI critic 固定相位 A/B

状态（2026-09-24，Asia/Shanghai）：**V1 在四个候选首例中均避免了参考八边形与移动箱体的采样重叠，并取得大于 0.05 m 的本体连续插值间隙下界；但预定 phase 0 重复中 QP/A* 分别恢复 5/1 次，未通过原零恢复门。** 因此停止两组 phase 2/4/6，矩阵不完整，`accepted_for_deployment=false`。这是动态导航独立实验，不修改 T-DT 迁移静态结论。[完整审计](review.json)和四个[首例审计](ab_audit_v2.json)是数值入口。

## 输入、唯一候选与检查

在 `experiment/dynamic-prediction-v1` 隔离工作树构建独立 `rm_dynamic_prediction_critic` 插件。T-DT、MPPI 原 critic、STVL、BT、goal `(5.6,0,0)`、参考 382/126 mm 八边形、padding 0.03 m、0.05 m 本体间隙与其他原门均未改；两组 baseline/候选均沿用已修正 `controller_server.odom_topic=/odometry/lio` 的实验 profile。候选 YAML 解析后仅在 MPPI `critics` 列表追加 `PredictionV1Critic` 及其夹具参数；[汇总器](summarize.py)逐组断言删除这一个 critic 后与 baseline 全量 YAML 相同。baseline A*/QP profile SHA256 分别为 `c3a3ce2c6f5493d3b538866f94f173409fb4b6edfe5ca371fdf5f2d0c8bf95a8`、`1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505`；候选分别为 `00b3feb7e1dded57ea0f475cebc6d0028827d215f560af47ae13835aba0d7ad1`、`4dd877afd4d85a88c702e3cdb8a6d38534524a4ef7b04ea246f3ed1c2bdaccbe`。

插件只消费已有 HWSentry shadow tracker 的 `DynamicObstaclePredictionArray`，不重写聚类、关联或 KF。tracker 的静态夹具地图在局部 costmap 的 `odom` frame 发布；插件拒绝错误 frame、schema、authority、不完整消息、无已确认目标及超过 0.4 s 的源年龄，缺输入时原 STVL/CostCritic 仍工作。第 `j` 个 MPPI 轨迹点使用 `age+(j+1)·model_dt` 的恒速预测。物理占用按上轮[离线外形检查](../v1_extent_shadow_20260924/validation_20260924.md)的同一规则：可见簇半宽＋每侧完整已知箱体边长 0.45×0.55 m＋夹具正弦目标导出的 `0.5·0.555165·t²` 扩张。机器人使用运行时 padded polygon 和轨迹 yaw；动态代价沿用原 CostCritic 的碰撞/临近代价尺度，不设置新的停止监督器或覆盖原 `fail_flag`。这些尺寸与加速度参考只来自已知仿真夹具，**不是未知目标真实尺寸/加速度上界或绝对安全证书**。

隔离 Docker 镜像 ID `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；插件 `.so` SHA256 `91dee01fa110e693b187bab044b7d0f9104302dc69a3b8f18e27ca3816f5a064`。构建通过，插件专项 CTest 入口 1/1、其中 GTest 用例 3/3 通过。第一次直接 CTest 因未 source ROS 环境缺 `ament_cmake_test`，加载 `/opt/ros/humble/setup.bash` 后重跑通过；最初 `colcon test` 未指定 base paths、发现0包，不计为测试。候选 launch 均记录 `Critic loaded` 和实际 `consumed source age`；四例分别记录 600、777、602、760 条 tracker 预测消息。按每秒节流的消费日志所见源年龄为 0.029–0.109 s，且在部分周期报告 300/300 预测碰撞候选。**消费日志是采样日志，不等于全部周期逐项导出；未量化未插桩 MPPI 的完整周期资源与最坏延迟。** 插件动态评分、载入和消息链已在真实首例执行，不仅是编译成功。

## 固定相位首例与预定重复

所有下表 action 状态均为 4（成功）；`B` 是 baseline，`V1` 是仅增加预测 critic。间隙为独立 Gazebo model/link 实际位姿与参考新车 polygon 的线性位姿插值下界，单位 m；`停滞`为导航中距目标超过 0.3 m 且 odom 平移速度不大于 0.03 m/s 的累计时长，仅作离线查询，不是新安全门。同相位运行并非逐帧确定性配对。

| 组与阶段 | 恢复 | 本体 / padded 移动间隙下界 | 行程 m | 导航仿真时长 s | 停滞 s | 原动态门 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| QP 首例 B | 11 | -0.0036 / -0.0117；本体采样重叠 | 7.36 | 22.19 | 6.16 | 失败 |
| QP 首例 V1 | 0 | 0.1368 / 0.0958 | 9.16 | 23.41 | 0.16 | **通过此首例** |
| A* 首例 B | 8 | 0.0391 / 0.0062 | 8.40 | 25.12 | 6.20 | 失败 |
| A* 首例 V1 | 0 | 0.1963 / 0.1559 | 11.41 | 27.00 | 0.16 | **通过此首例** |
| QP phase 0 重复 B | 6 | -0.0114 / -0.0125；本体采样重叠 | 7.86 | 25.11 | 6.16 | 失败 |
| QP phase 0 重复 V1 | **5** | 0.0773 / 0.0378 | 9.00 | 23.48 | 2.84 | **失败：恢复** |
| A* phase 0 重复 B | 6 | 0.0461 / 0.0094 | 8.34 | 25.74 | 6.32 | 失败 |
| A* phase 0 重复 V1 | **1** | 0.2070 / 0.1653 | 11.35 | 25.87 | 0.16 | **失败：恢复** |

八例 preflight 成功、action 成功，原始 observer summary 与事件相符。四条 V1 均未发生参考本体/padded 的采样重叠且连续下界为正；四条 baseline 均不满足本体 0.05 m 或零恢复门。V1 路线在 QP 两对中比对应 baseline 长约 1.8/1.1 m，在 A* 两对中长约 3.0 m；不能用四条轨迹计算成功率或因果效果大小。终点 XY/yaw、cross-track 与原始完整几何见[审计 JSON](review.json)。

重复试次的失败时窗是具体的：QP 候选于约 30.19–30.59 仿真秒多次因移动障碍附近 `start blocked` 触发全局规划端点拒收，随后有约 2.68 s 连续低速段；其本体连续间隙下界仍为 0.077 m。A* 候选在 28.451 s 一次 `TDT latest snapshot rejected: unsafe_latest_path`，恢复总数 1；本体下界 0.207 m。不能把这些恢复解释成预测层碰撞，也不能据此归因唯一根因。部分周期预测框覆盖 300/300 条轨迹，说明外形界可能过保守；这也是 V1 的通行代价，而不是调低安全代价的理由。

## 失败记录、边界与判断

最初 `dynamic_prediction_mppi_ab_v1/candidate_tdt_qp_1` 因静态地图脚本导入路径错误没有发布地图、没有预测消息；插件只记录 `missing`，容器主动停止退出137。原始目录保留，**不算有效候选**。修正导入并单独烟测地图后首次创建 `dynamic_prediction_mppi_ab_v2`，没有覆盖 v1。下一轮 `dynamic_prediction_mppi_multiphase_v1/phase_0/baseline_tdt_qp_1` 的严格二维 oracle 因 36.522 s 的非平面 Gazebo quaternion 分量超出原 1e-3 容差而拒绝复算；导航成功不等于可审计的平面几何结果。保留原目录与 traceback，不放宽容差。修正矩阵审计对这种情况标记无效并新建 `dynamic_prediction_mppi_multiphase_v2`，其四条 phase 0 试次均完整通过原二维审计。预先写定的相位顺序是 `0/2/4/6 s`，两组候选均在**首个重复 phase 0**违反零恢复门，按停止规则未创建 phase 2/4/6；不能宣称完成多相位矩阵。

**结论：保留 V1 为独立研究候选，暂不增加 V2 停车 guard、连续扫掠或形式化 supervisor。** 当前能说的是：已知箱体夹具中，时间对齐的 tracker 预测在本轮四次候选均改善了几何间隙，但零恢复未稳定满足，绕行有明显代价。下一步应先用这些冻结的 `start blocked`、`unsafe_latest_path` 和 300/300 预测碰撞时窗判断是占用外形过宽、局部绕行与全局重规划协同，还是 tracker 输入问题；只有观察到 V1 可复现的实际动态碰撞且输入/物体尺寸校正不足时，才按[范围文档](../../v1_prediction_scope.md)考虑薄层 V2。**不得为通过测试缩小 footprint、放松原门、调取有利相位或反复改经验权重。**

当前 SDF 载体、参考新车八边形、已有 shadow tracker、单夹具机械运动均不等于最终新车 CAD/附件包络、真实传感器年龄、制动能力或竞赛环境。未运行完整 ROS/插件全栈 sanitizer、未做目标设备负载、实车、狗洞、完整 TF/动态多目标验收。没有改变部署默认、T-DT 核心、heading、terminal selector、MPPI 原 critic、安全门；没有 push/merge。旧 T-DT 静态和早期动态证据保持原范围。

[插件源码](../../../../experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp)、[首例运行器](run_ab.py) / [A* 运行器](run_astar.py)、[重复运行器](run_multiphase_v2.py)、[审计器](audit.py)及[最终清单](manifest.json)提供复核入口；所有 trial 目录首次创建，清单自身不包含自身哈希。
