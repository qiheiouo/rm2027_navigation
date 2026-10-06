# A24 速度与圆形支持对齐的最小闭环对照

Research，承接 [A23 Modify](r4_finite_closed_loop_comparison.md)。假设：A23 的安全收益在消除空场速度与包络差异后仍存在。main、A22 consumption/solver/prediction/dynamic cost/free-s 不改，不建生产输出设施，不 push。

**判决：Modify，保持Research，暂停生产化接线。** 对齐后成功率与真实contact优势消失；S2仍有约12cm的净空中位收益，但付出约33%的到达时间代价和小幅回退。有限结果不足以支持Go，不宣称R4整体优于native MPPI。

两组复用 A23 二进制、地图、起终点、物理模型、S1/S2 障碍运动、STVL 与原 Nav2 输出链。共同 local/global footprint 使用包住 `r=hypot(.32,.27)+.02` 圆的 32 边形、padding=0；最大支持比 R4 精确圆多约 2.12 mm（0.484%）。共同 local inflation radius=.50 m，确保超过圆支持。此配套改动有必要：原 .30 m 小于圆支持，native CostCritic 跳过中心 cost<1 的点，不能只扩大 footprint 而把膨胀范围留在圆内。依据 [Nav2 1.1.20 CostCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp) 与 [footprint 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint.cpp)。不修改正式配置，也不新增 MPPI。

只在 S0 校准 baseline `vx_max`，最多三个候选，空场到达时间相对 R4 误差≤5%后冻结，动态结果不用于调参。R4 所读 limits/cruise/free-s 维持 A23；其 native angular 委托共用新几何，速度上限维持原 .5。baseline `vy_max`、smoother 等其他设置保持。对齐到达时间并不代表逐拍速度/加速度曲线完全一致；同时报告空场进度差异。

有限样本：S0 每组3次，S1/S2每组5次，按序号交替先后。首次失败保留、分类，不救结果；启动失败单列后才允许补样本。原 A23 数据保留。最多约100 MB新证据，不录大 bag/core。

判定预期：重复碰撞减少且无新增失效支持 Go；仅净空收益但时间/振荡变差则 Modify；成功/碰撞/净空均无明显改善且效率或稳定性更差则 Stop 当前配置的生产化。小样本不代表统计优越性或实车可靠性。动态净空中位改善≥.10 m视为有意义的安全余量信号，但必须结合最小值、成功率、耗时与振荡判断。

## 运行与当前结果

入口为 [matched.py](../../experiments/r4_gazebo_comparison/matched.py)，依次 `prepare`、`reference`、最多3次 `calibrate`、`freeze`、`batch`；分析复用 `analyze.py/fairness.py`，空场进度检查为 `matched_audit.py`。原始输出在 `build/r4_matched_comparison_20261006`。协议先提交为 `35d212c0`，完成动态结果后不改判定条件。

空场校准：R4参考12.579 s；baseline候选 `.34` 为11.720 s（−6.83%，拒绝），候选 `.32` 为12.897 s（+2.53%，接受）。在任何动态trial之前冻结 `.32`。到达 x=1/1.5/2/3 m，baseline为3.498/5.098/6.678/9.738 s，R4为3.539/5.099/6.659/9.799 s。单数字校准目录读取少补零的问题修正后重读原结果，没有重跑或替换候选。

26个有效有限trial完成，另2个空场校准保留；没有启动失败、controller failure或真实actor contact。时间为成功样本中位数；净空为独立真值机械投影的逐次最小值，再取中位数，S0无动态净空。

| 场景 | Baseline成功/contact | R4成功/contact | Baseline到达s | R4到达s | Baseline净空中位m | R4净空中位m |
| --- | --- | --- | --- | --- | --- | --- |
| S0 | 3/3，0 | 3/3，0 | 12.881 | 12.579 | — | — |
| S1 | 5/5，0 | 5/5，0 | 11.017 | 14.108 | .29449 | .29746 |
| S2 | 5/5，0 | 5/5，0 | 13.478 | 17.922 | .18054 | .30045 |

- 空场最终到达差+2.40%，满足预设≤5%；2–8s世界系前进速度中位 baseline .31347、R4 .32005 m/s。到达x=2m的中位时间6.658/6.659s，x=3m为9.798/9.799s。单次R4有约.24s起步变化，不能声称逐拍速度完全相同。
- S1净空中位差仅2.97mm，低于5cm costmap分辨率；R4耗时增加28.06%，没有明确附加价值。
- S2净空中位差11.99cm，最坏逐次最小值baseline .14726m、R4 .28322m；这是真实保守机械余量信号，达到预设.10m中位门槛。R4耗时增加32.97%，WAIT中位3.64s（2.30–3.70），最长连续停滞最大3.60s；baseline WAIT中位.06s、最长停滞最大.10s。2/5 R4发生两次world-forward正负切换，所有R4有小幅回退，最大4.674cm；baseline均零切换、零回退。图中baseline绕行，R4主要等待，因此不能把更大净空直接等价为预测更准或整体能力更强。
- 两组S2在独立定义的实际clear时均已前进，`actual_clear_resume_s=0`；该值不表示零感知延迟，也未显示额外恢复优势。目标t=9s与实际actor clearance的时刻区分保留在逐次metrics。
- 3974/3974个R4消费调用有效；solver P95 S0/S1/S2为1.306/1.564/1.545ms，最大2.663ms。完整consumption P95≤2.548ms，最大8.249ms；native angular委托加consumption P95≤34.171ms、最大46.114ms。仍保留native耗时，不能宣称CPU替代收益或硬实时保证。
- 13对实际actor轨迹位置差最大.675mm，100ms中央速度差最大.002432m/s，机器人起点差0；所有trial参数语义检查无额外变化，唯一实际Gazebo输出publisher仍为原chassis stub。两个组别native `vx_max`不同是空场匹配手段，R4原world bounds保持.5；native angular内部采样因此不能宣称逐样本相同。

这轮**联合**调整速度、circle与local inflation，不能分别归因三个变量；但足以否定“A23四次baseline contact就是prediction-consumption独立价值”的判断。原A23证据保留，不撤回其配置下的真实contact事实。

16/28个实例在trial结束后的Nav2 cleanup仍记录exit −11（包括校准）；目标/真值/contact窗口在结束前已保存，日志保留且core禁用。此历史shutdown问题未在Research轮中修工程，也不据此声称生产稳定。没有新运行中失效。

本阶段停在判决。后续只有新的、能改变决策的Research假设才继续：例如在baseline同等目标净空代价下验证prediction consumption是否减少延误，或在明确更困难但公平的动态时序下是否避免baseline失败。不是默认继续调参或ROS化，不建设owner/lease/fallback/旋转模型。当前未证明值得把R4升级为正式controller；main保持 `2849cbe4`，库实现保持A22。

必要汇总、冻结参数、校准、轨迹对齐、清理列表与两张图见[紧凑证据](../../experiments/r4_gazebo_comparison/evidence_matched/)。全部28次原输入/控制/预测/真值/日志保留在独立build目录，约62MiB，没有新的大规模实验或全仓回归。共同实际输出receipt gap逐次P95最高54ms、最大62ms；动态真值最大采样间隔20ms。receipt不等于物理apply ACK。

磁盘：已从六个旧 R4 验证目录删除49个可再生成的 `CMakeFiles/gtest` 中间目录，实际释放652.6 MiB。安装依赖、源码、测试记录、A23全部原始/失败证据和其他工作树保留。清理列表在 `build/r4_disk_cleanup_20261006.json`；这些旧目录再次编译前需重新执行 CMake 配置。
