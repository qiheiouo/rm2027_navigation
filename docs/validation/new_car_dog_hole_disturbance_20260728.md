# 新车狗洞扰动鲁棒性验证（2026-07-28）

## 范围

- 分支：`feature/new-car-dog-hole-sim`
- 验证基线：`8b8f675`
- 隧道标称宽度：`0.80 m`
- 机器人临时外形：`0.60 m × 0.50 m`
- 全部为 Gazebo 无硬件仿真；没有启动串口、雷达或真实底盘。
- 定位和底盘扰动只存在于 `rm_simulation`，不修改正式定位和下位机接口。

## 新增仿真能力

定位扰动链：

```text
/simulation/ground_truth/odom
-> localization_disturbance
-> /simulation/localization/odom
-> lio_adapter
-> odom->base_link
```

支持横向/yaw 白噪声、延迟、低频漂移和固定随机种子。节点不发布 TF，
`map_odom_stub` 与 `lio_adapter` 的 TF 所有权保持不变。

底盘扰动链：

```text
/cmd_vel
-> chassis_interface_stub
-> /simulation/chassis/cmd_vel
-> chassis_command_disturbance
-> /simulation/chassis/cmd_vel_applied
-> Gazebo bridge
```

支持前进、横移正负方向和角速度增益，以及横移/角速度一阶响应时常。
单位增益和零时常是透明默认值。

实验 CSV 同时记录：

- Gazebo 二维真值轨迹、旋转 footprint 间隙；
- 估计位姿相对当前真值的有效误差；
- commanded 与 applied 底盘速度误差；
- 独立 3D ground-truth 的最大高度和俯仰。

## 定位扰动结果

| 工况 | 样本 | 结果 | 主要峰值 | 最小间隙 |
| --- | ---: | --- | --- | ---: |
| 横向 `1 cm`、yaw `0.5°` 白噪声 | 3 | `3/3` 成功 | 有效误差 `3.19 cm/1.56°`；真实洞内 `5.4 mm/0.31°` | `0.1441 m` |
| `100 ms` 延迟，`+10 cm/+10°` | 3 | `3/3` 成功 | 对正滞后 `1.70 cm/2.39°`；洞内近零 | `0.14998 m` |
| 横向 `2 cm`、yaw `1°`、`0.08 Hz` 漂移 | 3 | `3/3` 成功 | 真实洞内 `2.18 cm/0.99°` | `0.1232 m` |
| 上述噪声、延迟、漂移组合，四个边界角点 | 4 | `4/4` 成功 | 有效误差 `6.25 cm/4.25°`；真实洞内 `2.85 cm/1.23°` | `0.1158 m` |

低频漂移是本轮影响最大的单项，但组合工况仍高于当前 `0.03 m` 最小
间隙门。

## 底盘执行误差结果

| 工况 | 样本 | 结果 | 主要峰值 | 最小间隙 |
| --- | ---: | --- | --- | ---: |
| 横移正负方向仅 `60%` | 4 | `4/4` 成功 | 横移命令缺口 `0.09 m/s`；真实洞内偏差 `2.55 mm` | `0.1475 m` |
| 角速度 `85%`，响应时常 `0.25 s` | 4 | `4/4` 成功 | 角速度缺口 `0.320 rad/s`；真实洞内 yaw `0.007°` | `0.14996 m` |
| 横移 `+55%/-85%`、横移时常 `0.15 s`、角速度 `85%`、角速度时常 `0.20 s`，四角 | 4 | `4/4` 成功 | 横移/角速度缺口 `0.155 m/s`、`0.305 rad/s` | `0.1480 m` |

## 几何变体与明确阻塞

使用以下临时变体：

```text
隧道深度：1.50 m
台面高度：0.06 m
入口坡面：11°
出口坡面：15°
```

二维任务多次到达 `FINISHED`，穿越段约 `10.2–10.4 s`。但新增独立
3D OdometryPublisher 后仍测得 `z≈0、pitch≈0`；把模型请求抬高到
`1 m` 后，Mecanum 模型也立即保持在平面附近。

因此当前仿真只能证明：

- 深度参数能正确改变场景与二维穿越距离；
- 二维中心线控制在 `1.50 m` 通道中仍能完成任务。

当前不能证明：

- 机器人能沿 `11°/15°` 坡面产生真实 z/pitch 运动；
- `250 mm` 洞顶具有物理碰撞约束；
- 临时机器人高度等于最终新车 CAD。

坡面/洞顶被当前 planar Mecanum 物理模型阻塞，不能计为通过。下一步应先
换成可产生三维车体姿态的轮地接触模型，再加入最终 CAD 碰撞体和实体洞顶。

## 结论

当前中心线方案在本轮二维定位与执行扰动范围内通过，没有出现需要立即开发
MPPI 狗洞 Critic 的证据。该结论仍是仿真软件候选，不是坡面、洞顶或新车
实车验收。

最终定向验证：

```text
rm_dog_hole、rm_simulation 构建：通过
rm_dog_hole geometry gtest：5/5 通过
colcon test-result：6 tests, 0 errors, 0 failures, 0 skipped
Python 脚本与 launch 语法：通过
新增可执行安装：localization_disturbance、chassis_command_disturbance
```

证据目录：

```text
/tmp/rm2027_dog_hole_robustness_phase2_20260728/
```

关键文件 SHA-256：

```text
228cc9b99be9c9fbddd37351cfa3ab1d7a99173123a7bb9d0ab931910a6c4b10  localization/moderate_noise_R3/capture_matrix.csv
87846ef6b439d2dd8220e3affc2e724f976d36fde873736050ffdd16fcc62164  localization/delay_100ms_boundary_R3/capture_matrix.csv
e5c5b50d78744f8f0172c718e85d5f024c5409e61cc44500991dda6628b68fdf  localization/slow_drift_R3/capture_matrix.csv
19b9095099448b41ace065f25c0e9c528c6700ce9febe1da82b4a4f3e12e0edb  localization/combined_boundary_4/capture_matrix.csv
0bb0d15dbdbb3296d076074fa3c51aa1ae4d514a9d20ae56dd989ec23c42ed8c  execution/lateral_60_R2/capture_matrix.csv
a612663cd709b1c37de553563aff01374dc3dcd1071396338f78a4476d4a0b82  execution/angular_lag_R2/capture_matrix.csv
c0005013987ded9a5b322546710a061903dc6c48c9fcf8fb857cebd40db32d8e  execution/asymmetric_combined_boundary_4/capture_matrix.csv
a5bdbeb6dfbfb6511a4e1dfc934f8e56e101c35858e951fc2ee612004c0f47aa  geometry/deep_slopes_3d_validated/capture_matrix.csv
dc452379231ed461c706e837c32d5f62beb949a323db9c87ee28edd33211f719  final/build.log
86cf30b7bc388dac603c85e0f56b7039e658e7212070c051e2799357946cb0b0  final/test_result.log
```
