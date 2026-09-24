# 动态预测 V1 输入首例：跟踪可用，直接使用可见框不够

状态（2026-09-24，Asia/Shanghai）：**现有 shadow tracker 已在原移动障碍夹具中 live 发布 761 条完整预测；碰撞前持续确认同一移动目标。直接把其可见簇中心/尺寸当作完整预测占用，仍会漏掉箱体。尚未接 MPPI，也没有完成 V1 A/B 或动态安全验收。** 本轮以[live 审计](live_audit_v2.json)、[0.2–1.0 s 预测复算](live_prediction_eval_v2.json)与[冻结扫描回放](probe_v2.json)为依据。`accepted_for_deployment=false`。

## 范围与输入

隔离工作树从 `e374894a7a713ea84b82de9e92479a37041f39ff` 开出 `experiment/dynamic-prediction-v1`；未修改 `src/`、`experiments/`、部署默认、T-DT、MPPI、BT、footprint 或原安全门，没有实车、push 或 merge。沿用已修正 controller odom 的实验 QP profile，SHA256 `1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505`。goal `(5.6,0,0)`、相位 0、13 m 全局窗口和原动态夹具均未改变。镜像 ID `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。

只将已有 `rm_competition_interfaces`、`rm_dynamic_obstacle_tracking` 编译到独立 prefix，构建2包通过；既有 tracker core 测试18/18、两项 Humble 消息序列化回调2/2、离线几何专项3/3通过。live 候选为**只读影子采集**：tracker 用原 `/scan`、仿真时钟和实验专用 `/prediction_v1/static_map`；该地图按 SDF 中心块和两段 course wall 生成，不包含移动箱体。除 `map_topic`、`scan_topic`、`use_sim_time` 外，仅将影子预测的速度衰减设为0以测试 V1 恒速模型；没有让预测进入 costmap、planner、controller 或底盘。实验地图不冒充赛场 `/map`。实际代码和输入哈希见[trial metadata](../../../../build/tdt_p2b/runs/dynamic_prediction_tracker_live_v2/tdt_qp_1/metadata.json)。

## 原始试次和失误保留

第一份 `dynamic_prediction_tracker_live_v1/tdt_qp_1` 在录制器诊断回调处报错：Humble 的 `DiagnosticStatus.level` 是 bytes，初版 JSON 序列化失败。检测到录制器退出后停止容器，保留 traceback、原始流、`docker_exit=137`；此试次没有可用 live 结论，不计为规划器失败或 A/B。修正录制器并通过消息级2/2检查后，首次创建 `dynamic_prediction_tracker_live_v2/tdt_qp_1`，没有覆盖 v1。

v2 记录预测761条、诊断762条；761条诊断为 `ok`，启动时有1条 TF 暂不可用，预测数组761/761均标记完整，761条 `ok` 诊断与预测成对记录；无法据此证明DDS层绝无丢包。source→processing 中位/P95/最大为0.019/0.028/0.057仿真秒；source→录制器接收为0.031/0.045/0.062仿真秒。这是**tracker 到本录制器**的观测年龄，不是 MPPI 消费年龄、DDS单段延迟或原 STVL 的70–200 ms观测年龄。预测 source stamp 相邻间隔中位约0.066 s。

v2 原导航 preflight=4、action=4，但恢复21次；终点 XY/yaw 误差0.0703 m/0.1734 rad。参考新车本体对真实移动箱体采样最小间隙0.01936 m、线性位姿插值下界0.01298 m，均低于原0.05 m门；padded footprint 有采样平面重叠，插值下界为-0.00883 m。`docker_exit=1` 来自现有安全门失败。影子节点没有改变导航决策，因此不能把这次与历史无影子试次的恢复差异归因于 tracker；不进行成功率或控制收益比较。

## 关键输入缺口：可见簇不是完整障碍

首次采样本体间隙低于0.05 m发生于仿真秒28.451。此前导航窗口的189条预测中，122条含已确认目标；违例前最后两秒，31/31条有同一个已确认 ID 1 靠近真实箱体。用 Gazebo 实际箱体中心及 SDF 的0.45×0.55 m真实尺寸作**离线评估真值**，31个可见簇轴向框中0个完全包含箱体。其正 Y 隐藏侧缺口中位0.1586 m、最大0.2680 m；正 X 缺口中位0.0647 m。这不是漏看移动目标，而是把激光可见表面误认为全物体的几何偏差。

先用原冻结、无影子的 QP 失败扫描做独立 core 回放：242条扫描中175条确认箱体附近目标；碰撞前最后两秒32/32条确认，但可见框完整覆盖0/32，隐藏侧正Y缺口中位0.3763 m。这个离线回放用 SDF 静态地图和 Gazebo机器人位姿代替实时 ROS TF，只用于核查 core 和几何假设；初始[`probe.json`](probe.json)曾把碰撞后的箱体位姿当成未来预测评估，结果不采用，以[`probe_v2.json`](probe_v2.json)在首次本体重叠前截断的结果为准。

live 发布的中心预测与**真实箱体几何中心**比较，0.2 s/1.0 s 原 source 时刻预测误差中位分别为0.227/0.401 m（118/106个未越过违例时间的样本）。用实际录制器接收年龄做恒速传播后，中位分别为0.227/0.416 m；此例短年龄补偿没有消除可见面中心偏差，较长时间还受目标转向影响。可见尺寸随轨迹平移后，五个0.2–1.0 s时域的完整箱体包含数全为0。误差把目标中心偏差与运动预测误差合在一起，不能单独判定 KF 速度估计优劣；Gazebo未来位姿只用于离线评分，未送给 tracker。

## 判断和下一项

这份证据支持复用现有 tracker，但否定“直接将当前 `position + size + CV` 画成 MPPI 预测占用”这一最省事的接法。在 V1 内先用**已知物理尺寸界**把可见回波转为保守物体占用，并对 source age 和预测时间步位移；在本夹具中尺寸来自现成 SDF，不从失败轨迹反调经验半径。先在冻结窗口核查目标覆盖与通行余量，再实现一个独立 MPPI critic，做[预定的最小 A/B](../../v1_prediction_scope.md)。未知对象没有可信尺寸界时，不把夹具模型外推为通用安全保证。

当前没有控制器消费、预测代价、候选导航收益或多相位结果，不能宣布“V1 已解决动态碰撞”；也没有理由继续开发扫掠 guard、terminal selector、航向参数或毫秒级全链追查。最终新车 CAD、真实动力学和制动仍待硬件验证。所有新目录首次创建；原静态与动态冻结证据未改写。
