# 周期 613 七项原生 critic 归因：预登记（2026-09-30）

[固定二维面积评分](cycle613_face_xy_overlap_result_20260930.md)将真实安全 #176 的重叠单项排第 1，但含七项原生 critic 与 MPPI 正则后，最高权重为危险 #190。为定位**当前评分相互作用**，本项只在原逐条滤波候选、原 raw local costmap 和原 profile 上读取七项原生 critic 的逐项输出。绝不改 profile 权重、path、T-DT、tracker/KF、预测框、batch、footprint、padding 或原 0.05m 门。

使用此前为周期 613 单线程构建的固定 `frozen_critic_score` 二进制，运行前核对其 SHA256 `1046705d...`、当前镜像 ID `sha256:0aa16ce3...`、已封存的 `filtered_map_and_poses.bin`、`filtered_controls.bin`、`filtered_meta.json` 与 `filtered_native_scores.bin` 的哈希。按原 profile 顺序 `Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle` 生成七份**仅缩短 critics 列表前缀**的元数据，依次在同一固定镜像、无网络及有限资源条件下运行原生评分器；第七份须逐条复现先前封存的全项分数，最大绝对差超过 `0.001` 即停止结论。相邻前缀累计分之差就是该固定顺序下每项的操作性贡献。前缀运行不修改运行控制器；若原生 scorer 对截断前缀有其它状态依赖，则这种差分不得解释成一般因果贡献。

以早已固定的真实联合安全 `#5/#176/#194`、新面积项总分首名危险 `#190` 作逐项对照：报告每项安全 #176 减危险 #190 的分差、面积项分差、正则分差和总分差；另对全部 300 条报告各项跨度及真实联合安全候选相对“静态无碰撞但动态不安全”候选的两两排序比例。Gazebo 真值只用于已封存标签，不参与原生评分。若七项里某项明显偏好 #190，只作诊断，**不按这一个已看过的周期调其 weight**。后续任何算法修改仍需新的冻结控制输入、聚合输出、原静态/动态门及目标推进共同验证。
