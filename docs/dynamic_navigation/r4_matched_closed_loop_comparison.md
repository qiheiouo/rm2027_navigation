# A24 速度与圆形支持对齐的最小闭环对照

Research，承接 [A23 Modify](r4_finite_closed_loop_comparison.md)。假设：A23 的安全收益在消除空场速度与包络差异后仍存在。main、A22 consumption/solver/prediction/dynamic cost/free-s 不改，不建生产输出设施，不 push。

两组复用 A23 二进制、地图、起终点、物理模型、S1/S2 障碍运动、STVL 与原 Nav2 输出链。共同 local/global footprint 使用包住 `r=hypot(.32,.27)+.02` 圆的 32 边形、padding=0；最大支持比 R4 精确圆多约 2.12 mm（0.484%）。共同 local inflation radius=.50 m，确保超过圆支持。此配套改动有必要：原 .30 m 小于圆支持，native CostCritic 跳过中心 cost<1 的点，不能只扩大 footprint 而把膨胀范围留在圆内。依据 [Nav2 1.1.20 CostCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp) 与 [footprint 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint.cpp)。不修改正式配置，也不新增 MPPI。

只在 S0 校准 baseline `vx_max`，最多三个候选，空场到达时间相对 R4 误差≤5%后冻结，动态结果不用于调参。R4 所读 limits/cruise/free-s 维持 A23；其 native angular 委托共用新几何，速度上限维持原 .5。baseline `vy_max`、smoother 等其他设置保持。对齐到达时间并不代表逐拍速度/加速度曲线完全一致；同时报告空场进度差异。

有限样本：S0 每组3次，S1/S2每组5次，按序号交替先后。首次失败保留、分类，不救结果；启动失败单列后才允许补样本。原 A23 数据保留。最多约100 MB新证据，不录大 bag/core。

判定预期：重复碰撞减少且无新增失效支持 Go；仅净空收益但时间/振荡变差则 Modify；成功/碰撞/净空均无明显改善且效率或稳定性更差则 Stop 当前配置的生产化。小样本不代表统计优越性或实车可靠性。动态净空中位改善≥.10 m视为有意义的安全余量信号，但必须结合最小值、成功率、耗时与振荡判断。

## 运行与当前结果

入口为 [matched.py](../../experiments/r4_gazebo_comparison/matched.py)，依次 `prepare`、`reference`、最多3次 `calibrate`、`freeze`、`batch`；分析复用 `analyze.py/fairness.py`。原始输出在 `build/r4_matched_comparison_20261006`。结果待完成后填写。

磁盘：已从六个旧 R4 验证目录删除49个可再生成的 `CMakeFiles/gtest` 中间目录，实际释放652.6 MiB。安装依赖、源码、测试记录、A23全部原始/失败证据和其他工作树保留。清理列表在 `build/r4_disk_cleanup_20261006.json`；这些旧目录再次编译前需重新执行 CMake 配置。
