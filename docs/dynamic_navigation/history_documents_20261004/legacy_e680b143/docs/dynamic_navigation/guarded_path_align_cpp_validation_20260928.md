# 冻结周期 PathAlign C++ 边界修正验证

状态：**隔离插件与冻结评分核对完成，未切换运行控制器。** 上轮[离线有界回放](guarded_path_align_replay_20260928.md)在路径累计长度被轨迹超过时，把 PathAlign 的 `lower_bound(end)` 映射到最后一个有效路径点。本轮将同一条规则写入隔离 Nav2 1.1.20 C++ 插件，并在原始捕获的 300 条及四个固定种子的 300/600/1000/2000 条样本上核对评分。没有生成新的仿真相位或采样控制。

## 构建和来源

[来源记录](../external/nav2_mppi_path_align_guard.md)固定了上游标记、提交、归档、MIT 许可、引入范围和移除方法。[准备脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/prepare_guarded_nav2.py)在复制诊断包之前检查原始头文件 SHA256；仅在匹配的帮助函数中插入 `iter == vec.end()` 分支。隔离插件和[冻结评分器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_probe_cpp/src/frozen_critic_score.cpp)分别在 `rm2027_navigation:dynamic-prediction-20260924` 镜像中重新构建，镜像 ID `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6`。两次构建均使用 `--parallel-workers 1`、`MAKEFLAGS=-j1`、`CMAKE_BUILD_PARALLEL_LEVEL=1`，未使用多线程编译。隔离 overlay 的 `ros2 pkg prefix nav2_mppi_controller` 指向本工作树的构建前缀；默认运行链未加载此插件。

评分器顺序运行七个标准 critic（不含 PredictionV1Critic），输入为冻结的 robot pose/velocity、raw local costmap、global path、采样控制及轨迹。V1 在此周期对共同第一步给所有样本相同的硬碰撞项；本轮仅验证标准 critic 中的 PathAlign 边界修正，不把七项得分冒充完整线上 MPPI 得分。[验证脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/validate_guarded_nav2.py)逐例执行并比对每条 `float32` 分数，拒绝非有限值、数量不符或误差大于 0.001。

## 结果

17 组共 **15,900 条**评分，C++ 与离线有界参考的最大绝对差 **`3.052×10⁻⁵`**，**0 条**超过 0.001；原始捕获 300 条最大差 `4.768×10⁻⁷`。每组输入、输出和构建物 SHA256、逐组差值及 C++ 分数二进制文件保存在[证据目录](evidence/guarded_path_align_cpp_20260928/summary.json)。这确认先前的 Python 路径末端处理能够由当前 Nav2 C++ critic 实现，而非仅依赖解释器的近似。原生 C++ `score_eval_ms` 在 300 条时约 2.5 ms、2000 条时约 14.6–16.2 ms；它只包围七个标准 critic 的一次评分，不是完整 MPPI 控制周期，也不是部署可达频率。

结论仍限于冻结周期：已知有安全采样，但先前的有界评分加原 MPPI 权重/滤波聚合在 17/17 组都未越过 0.05 m 联合安全门。C++ 一致性没有改变这个诊断，不能据此宣称动态碰撞已解决。下一步应使用线上可得的 tracker 预测，对已经采到的安全候选及**聚合后的控制序列**提供可区分的排序信号，再在早期、关键、目标附近及留出周期验证；真值仅用于事后标注。
