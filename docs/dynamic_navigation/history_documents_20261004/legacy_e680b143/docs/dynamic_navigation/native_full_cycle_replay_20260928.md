# 冻结控制输入的原生 MPPI 核心回放

状态：**17 组原生动力学、评分及控制聚合核对通过；完整 ROS 控制周期性能门未通过。** 本轮接续[有界 PathAlign C++ 校验](guarded_path_align_cpp_validation_20260928.md)，把原始 300 条与四种子 300/600/1000/2000 条冻结控制送入隔离 Nav2 `Optimizer`，调用原生 Omni 运动传播、七项标准 critic、控制正则、softmax、限速和 Savitzky–Golay 滤波。没有改变 T-DT、tracker、footprint、padding、0.05 m 间隙门、默认控制器或仿真相位。

## 输入及 V1 边界

[准备/验证脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/native_full_cycle_probe.py)从周期 162 提取初始控制与前四周期命令，将原始采样控制或已固定的新采样直接注入回放器。这替代噪声生成以保持**逐条同输入**；不是随机生成器性能测试。原始 raw local costmap、路径、位姿、速度和预先捕获的控制均经输入哈希固定。[C++ 回放器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_probe_cpp/src/frozen_cycle_replay.cpp)调用隔离构建的 Nav2 核心，使用[已核对的路径末端分支](../external/nav2_mppi_path_align_guard.md)。编译沿用镜像 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6`，`MAKEFLAGS=-j1`、`CMAKE_BUILD_PARALLEL_LEVEL=1`、`--parallel-workers 1`。

原 `PredictionV1Critic` 的 ROS 订阅与 `node->now()` 未在此回放器启动；其**已接受消息、source age 和原 profile**由冻结周期注入。回放器调用项目现有的 [`geometry.hpp`](../../experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp)，照原 [`prediction_critic.cpp`](../../experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp) 的 legacy 几何、九个有效预测步与分数缩放计算 V1 项。这验证固定消息下的算术与计算成本，**不等于直接执行运行插件**。原捕获的 V1 分项相减含大数 `float32` 舍入，回放逐条最大差 `0.0001221`；17 组 V1 分数跨度均为 **0**，每条约 `1666.6666`。共同第一步几何及源数据与既有冻结分析一致。

## 逐条核对与安全结果

17 组共 15,900 条：原生重新积分轨迹相对冻结 Python/原始轨迹最大误差 `4.77×10⁻⁷` m/rad；七项标准 critic 相对独立 C++ 评分器最大差 `3.05×10⁻⁵`。完整 V1 常数加入后，原生控制聚合/滤波与离线复算的最大差分别为 `3.84×10⁻⁵`、`1.43×10⁻⁵` m/s 或 rad/s。大罚分在 float32 中使少量低位被量化；不含 V1 常数的原生基准聚合误差低于 `3×10⁻⁷`。17 条完整原生输出的真实动态间隙仍 **0/17** 达 0.05 m，符合前轮有界回放。这里的未来真实箱体只在输出计算结束后用于评价。

| Batch | 组数 | 原生核心单次计算范围（ms） | 其中 V1 几何（ms） |
| ---: | ---: | ---: | ---: |
| 300 | 5 | 1.53–1.74 | 0.44–0.46 |
| 600 | 4 | 2.63–2.77 | 0.86–0.90 |
| 1000 | 4 | 4.14–4.28 | 1.43–1.45 |
| 2000 | 4 | 6.43–8.11 | 2.09–3.02 |

计时器只包围原生运动积分、七项标准 critic、V1 几何镜像、控制聚合和滤波。每组仅一次进程内测量；未包括噪声生成、输入装载、ROS 订阅/TF/costmap 更新、控制器外层及系统满载。此表说明在这台设备上值得继续测完整周期，**不是**已达到比赛设备 10 Hz 全负载门。单根 12 GB 内存要求只约束编译并发；运行峰值内存尚未记录。

[证据清单](evidence/native_full_cycle_20260928/summary.json)保存逐组输入、隔离源码及构建物哈希、逐条误差上界、控制、几何与耗时；[输入清单](evidence/native_full_cycle_20260928/inputs.json)和每组滤波后控制、V1 分数二进制允许独立检查。总计 17 组仍未形成动态过门控制，且 2000 条已有 79–93 条真实/静态图联合安全原采样；这进一步排除“本例仅因 300 条没有采到安全轨迹”，将后续工作指向**在线预测差异排序及最终控制聚合**。完整控制器性能、其它关键周期和独立留出闭环仍是下一门。
