# P2B 动态闭环：MPPI odom 输入与冻结周期反事实重放

状态（2026-09-23，Asia/Shanghai）：**确认 controller_server 的 odom 接线缺失；独立实验 profile 修正后，MPPI 已收到真实非零速度，但 A* 与 QP 的固定相位首例仍未通过动态安全门。** 本轮没有修改运行时源码、部署默认、footprint、BT、控制参数或安全门；没有实车、push 或 merge。新的首例仅用于定位，不纳入动态验收矩阵。

本报告以[冻结周期的离线重放](replay.json)、[候选首例独立审计](candidate_audit.json)和两组原始 trial 为依据。上一阶段的[逐周期报告](../mppi_cycle_diagnostic_20260922/validation_20260923.md)保持原有证据范围。

## 1. odom 合同确实未接通

冻结的 T-DT 动态 profile 在 `bt_navigator` 和 `velocity_smoother` 中指定 `/odometry/lio`，但 `controller_server.ros__parameters` 没有 `odom_topic`。本地保存的 Nav2 1.1.20 源码中，`ControllerServer` 构造 `nav_2d_utils::OdomSubscriber(node)`；该类在无参数时默认订阅相对话题 `odom`。同版 `navigation_launch.py` 只重映射 TF 和 `cmd_vel`，没有重映射 odom。夹具由 `lio_adapter` 发布 `/odometry/lio`。这些源码与配置共同确定了接线不一致；没有在旧试次运行中取得 `/odom` 的逐时刻 publisher 全图，不能仅凭图谱快照证明整个 ROS 域绝无其他发布者。

实际周期证据与此一致：A* 的 198/198、QP 的 172/172 个 MPPI 速度输入均为 `(0,0,0)`，而邻近的 canonical odom 分别有 196/198、164/172 个周期平移速度大于 0.1 m/s；最近 odom 时间差最大 0.02 s。评分轨迹的**全部 370 个周期、每条 rollout 的第一步位置和朝向**均仍在记录的初始 pose。Nav2 Omni 运动模型第一个预测速度取 `robot_speed`，后续取候选控制序列；以同期 odom 替代零输入做第一步纯运动学敏感性计算，中位位移分别约 0.05147/0.05162 m，最大约 0.06860/0.07063 m。这不是对原运行的重算结果或碰撞因果证明；它量化了该输入错误可影响的第一预测步。

## 2. 冻结周期的有限反事实

[重放脚本](replay.py)读取上一阶段已冻结的五周期窗口、同周期 raw 局部图、pose、滤波前后控制序列和后来**实际观测**的移动箱体位姿。它按所装 MPPI 的 Omni 积分顺序，以 `shift_control_sequence=1` 从第 1 项作为首个输出，理想化地逐 0.1 s 施加后续命令；用已有凸 polygon oracle 计算参考新车本体到 raw 254/255 闭 cell 面积和未来实际箱体的采样距离。第 1 项与十个窗口周期各自的真实返回命令逐值相符。重放器的移位、坐标变换、闭 cell 接触专项测试[3/3通过](tests.txt)。

| 冻结窗口 | CostCritic 已标记碰撞 rollout | 滤波后理想轨迹到当前 raw 254/255 的最小本体间隙 | 到未来实际移动箱体的采样间隙 |
| --- | ---: | ---: | ---: |
| A* 周期145–149 | 每周期 5–105 / 300 | 0.0554–0.0989 m | 五周期均为 0，发生几何相交 |
| QP 周期132–136 | 每周期 95–244 / 300 | 0.0309–0.0435 m | 五周期均为 0，发生几何相交 |

这些数值分别回答两个有限问题：评分时已经有一部分 rollout 被 CostCritic 标记；当前冻结 raw 图上的独立连续 polygon 检查与后来移动障碍的实际轨迹不是同一时刻的事实。**不能用后验的未来箱体位姿指责当时控制器应当预知其精确运动**。重放没有实现速度平滑器、底盘响应、Nav2 的 raster footprintCostAtPose 或实时地图刷新，也没有建立可执行制动距离；它不是“只因 odom 接线导致碰撞”的证据。尤其原 CostCritic 对采样 rollout 评分在加权更新和滤波之前，离线最终序列未必曾按相同判据重新评分。

## 3. 单变量接线候选的固定相位首例

新建 `build/tdt_p2b/runs/dynamic_odom_routing_pilot_v1/`，首次创建且不覆盖旧 trial。[候选脚本](run_candidate.py)从旧 `dynamic_cycle_diagnostic_v2` 两份 profile 复制，只插入 `controller_server.ros__parameters.odom_topic: /odometry/lio`，解析后的其余 YAML 完全相等。A* 原/新哈希为 `bd1aaba64d16dee9b1e1a93520fd2cf384980f499b65d7af0ade1bb192a035e9` / `c3a3ce2c6f5493d3b538866f94f173409fb4b6edfe5ca371fdf5f2d0c8bf95a8`；QP 为 `624f6bd1c9f60ea6faff7c4a14a0800dfb2179ea9d15f3fdf3a8405936cac018` / `1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505`。输入清单与独立审计确认 fixture、运行库、目标 `(5.6,0,0)`、相位0、13 m 全局窗口、MPPI/BT、八边形、padding 及全部安全门未变。T-DT 运行时源码仍与 `a419654a` 的 `src/`、`experiments/` 一致；插桩 Nav2 库与上一阶段相同。插桩影响调度，且同相位仿真不是逐帧确定性配对，结果只能比较**这两次实际首例**，不能计算修正的效果大小或成功率。

| 指标 | A* 原首例 | A* odom候选 | QP 原首例 | QP odom候选 |
| --- | ---: | ---: | ---: | ---: |
| MPPI 非零速度周期 | 0/198 | 209/211 | 0/172 | 194/197 |
| action / recovery | 4 / 0 | 4 / 7 | 4 / 10 | 4 / 19 |
| 终点XY误差 m | 0.1207 | 0.1148 | 0.0270 | 0.0888 |
| 终点yaw误差 rad | 0.1771 | 0.1707 | 0.1709 | 0.1622 |
| cross-track RMS m | 0.0415 | 0.0457 | 0.0942 | 0.0390 |
| 新车本体对移动障碍采样最小间隙 m | 0，重叠 | 0.05294 | 0.01936 | 0.01979 |
| 本体线性位姿插值间隙下界 m | -0.01016 | **0.04794** | 0.01220 | **0.01116** |
| padded footprint 采样重叠 | 是 | 否 | 是 | 是 |
| 现有动态几何与目标门 | 失败 | **失败** | 失败 | **失败** |

A* 候选采样本体间隙虽略高于 0.05 m，连续插值下界低于原 0.05 m 门，不能宣布通过；同时有 7 次恢复。QP 候选仍有 padded footprint 采样相交和 19 次恢复。两组导航 action 均成功，但不覆盖这些失败。A*、QP 候选分别有 2 个记录到异常展开且无返回命令的 MPPI 周期，审计如实计为 209/211 和 195/197 输出周期；没有将其当成丢包或补造输出。`docker_exit=1` 来自首例安全门失败，不是构建或容器启动失败。

[独立审计结果](candidate_audit.json)从原始 observer 事件、轨迹、Gazebo model/link 实际位姿重算；两例 raw summary、运行前和导航后 footprint 查询均齐全。上一阶段 991 项冻结清单逐项哈希仍一致。原有参考八边形用于目标平台算法判断；旧车物理 footprint 与仿真载体几何仍单列，不能用新车较小 polygon 授权旧车运动。

## 4. 判断及后续边界

**应修正 controller_server 的 odom 输入合同，但这项修正自身不足以使本场景通过动态安全门。** 本轮只在独立实验 profile 验证该接线，不把它写入部署默认；不要因 A* 消除采样重叠就放宽 0.05 m 门、padding 或零恢复门，也不要为此开发 terminal selector、调 path heading 或挑选成功相位补跑。

下一项应沿当前动态失败时间窗，在独立 series 精确记录 raw costmap 的**内部最后更新时刻**以及局部图与实际箱体的位置滞后，再建立包含移动障碍扫掠和可执行制动响应的最小动态安全合同。当前仅有地图复制时刻、后验障碍运动和理想开环轨迹，不足以在 MPPI、地图更新与底盘响应之间分配唯一根因。完成针对性的拒收/让行验证后，才能开展预先规定的多相位动态矩阵；当前 `accepted_for_deployment=false`。

本轮没有重新运行先前的构建72/72 Nav2测试或全链 sanitizer；这些是未改动二进制的历史证据。本轮实际新增检查为离线重放测试3/3、候选 profile 语义差异断言、两例原始数据审计与 991 项清单复核。新[证据清单](manifest.json)冻结本报告、脚本、结果、原始候选 series 与复核日志，清单自身不包含自身哈希。
