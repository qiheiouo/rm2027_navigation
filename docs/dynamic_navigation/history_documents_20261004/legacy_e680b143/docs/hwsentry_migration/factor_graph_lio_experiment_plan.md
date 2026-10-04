# Factor-Graph LIO 实验计划

## 源码审计结论

HWSentryNav26 的 `small_glim` 不是 FAST-LIO 的轻量替换件。它包含 IMU
预积分、位姿/速度/偏置状态、GICP/VGICP 因子、iSAM2 fixed-lag smoother、
IMU 去畸变以及饱和和数据缺口处理。这些能力对碰撞、高速自转和 IMU 偏置
鲁棒性有研究价值，但依赖和运行成本也显著更高。

当前 `FAST-LIO + lio_adapter` 已在旧车上完成核心运动验证，因此不替换默认
后端。本阶段状态为 `DESIGN ONLY`。

审计范围包括 `small_glim` 的 IMU preintegration、fixed-lag smoother、
GICP/VGICP factors、deskew 和 odometry publication 路径，并对照当前
`fast_lio_multi` 与 `rm_localization_adapters`，不是只比较算法名称。

## 不破坏现有 TF 的 A/B 结构

同一份 MID360/IMU bag 同时离线输入两个后端：

```text
                         +-> FAST-LIO -> /odometry/fast_lio_raw
MID360 + internal IMU ---|
                         +-> factor graph -> /experiment/factor_graph_lio_raw
```

实验后端必须满足：

1. 不发布 `/tf` 或 `/tf_static`；
2. 不发布 `/odometry/lio`；
3. 不接 Nav2、mission 或真实串口；
4. 输出 frame 和 child 语义写入独立 adapter；
5. 允许整套实验后端从 launch 一键移除。

## 同包指标

至少比较：

- 输出连续性、NaN/Inf、时间回退和最大间隔；
- 静止漂移、闭环首尾误差和可用外部参考误差；
- 高速自转前后 yaw、一致性和恢复时间；
- 碰撞或剧烈振动段的跳变、失锁与恢复；
- IMU 饱和、丢帧和时间缺口行为；
- CPU、RSS、P95/P99 callback 延迟和端到端延迟；
- 离线重放确定性和退出稳定性。

没有真实 ground truth 时，不伪造绝对精度；使用闭环、人工地标和重复轨迹作为
受限证据。

## 决策门

只有在相同录包上同时证明稳定性有明确收益、计算资源可接受、接口可隔离，才
进入可选 experimental backend。即便通过，也先保持 FAST-LIO 为默认，并由
显式 launch 参数选择。新车动态云台的逐点运动补偿仍需独立实车确认，不能靠
更换 LIO 后端回避外参和时序合同。
