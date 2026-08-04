# 新车狗洞三维几何验证（2026-08-04）

## 范围

- 分支：`feature/new-car-dog-hole-sim`
- 基线：`4102e4f`
- 全部为 Gazebo Fortress 无硬件仿真；没有串口、MID360、裁判系统或实车运动。
- 临时隧道：宽 `0.80 m`、长 `1.50 m`、台高 `0.06 m`、入口坡 `11°`、
  出口坡 `15°`、净高 `0.25 m`。
- 临时机器人包络：`0.60 m × 0.50 m × 0.22 m`，不是最终 CAD。

## 确定缺陷与修复

1. 生成的狗洞 SDF 含 `(-3, 0, π)`，但 `ros_gz_sim create` 的默认
   `x/y/yaw=0` 覆盖了模型内 pose。控制器此前在 `x≈-3` 运行，物理墙和坡面
   却在世界原点，因此旧统计只证明虚拟中心线控制，没有证明物理穿洞。
   修复后 create 显式传入配置的 `-x/-y/-Y`，生成 SDF 保持局部零位姿。
2. 原 `approach_offset=0.15 m` 会让车体在入口坡面上同时对正和横移；两组
   `±10 cm/±10°` 对角工况出现单侧轮接坡、横移饱和和对正超时。现在根据
   车长、坡高、坡角和安全余量自动计算完整车体位于坡外的最小对正距离。
3. 开放仿真世界允许 Navfn 绕洞。显式狗洞任务增加
   `task.require_traversal=true`，保留原始 `path_crosses` 诊断，但任务选择过洞
   后不能被普通全局路径偶然绕过。
4. 实验脚本原先把绕洞后 `FINISHED` 计为成功。现在必须实际进入
   `CROSSING`，并分别记录墙体重叠、洞顶重叠和旋转包络最小洞顶净空。

## 三维物理结果

修复场景 pose 后，中心工况首次测得：

```text
maximum_ground_truth_z_m       0.06986 m
max_abs_ground_truth_pitch_deg 9.16°
```

这直接证明当前 Mecanum 模型并未锁死平面；此前 `z≈0/pitch≈0` 的原因是
机器人根本没有经过实际坡面。

加入实体 `250 mm` 洞顶、`0.22 m` 临时车体包络和安全对正距离后，四个
边界角点结果如下：

| 横向 / 航向初值 | 结果 | 最小墙间隙 | 最小洞顶净空 | 最大 z | 最大俯仰 | 穿越时间 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `-10 cm / -10°` | 通过 | `7.89 cm` | `4.27 mm` | `6.99 cm` | `9.16°` | `10.77 s` |
| `-10 cm / +10°` | 通过 | `8.03 cm` | `4.28 mm` | `6.98 cm` | `9.16°` | `10.73 s` |
| `+10 cm / -10°` | 通过 | `8.00 cm` | `4.29 mm` | `6.99 cm` | `9.16°` | `11.57 s` |
| `+10 cm / +10°` | 通过 | `7.88 cm` | `4.32 mm` | `6.99 cm` | `9.16°` | `11.07 s` |

四项均实际经过 `CROSSING`，没有墙体或洞顶包络重叠。全局规划路径在这些
工况中均报告 `path_crosses=false`，由显式狗洞任务策略选择专用过洞流程，
没有把绕洞当成成功。

## 边界

- `4.27 mm` 最小洞顶净空非常小，只能说明临时模型在当前数值条件下可过；
  不能吸收 CAD、轮胎压缩、质心、坡角、地面起伏或装配误差。
- 新车最终长宽高、雷达/云台最高点、轮径、轴距、质心和隧道精确纵剖面
  尚未提供，必须替换临时值后重跑。
- 当前平面 GPU lidar、方向摩擦球轮和仿真 gimbal 是算法验证工具，不是
  MID360、麦克纳姆/全向轮细节或真实底盘动力学验收。
- 本轮支持“仿真软件候选通过”和“三维几何边界通过”；不支持实车通过结论。

## 证据

```text
/tmp/rm2027_dog_hole_3d_physics_20260804/
  required_traversal_center/
  required_traversal_roof_boundary_4/
  required_traversal_launch.log
  final_state_evidence.log
```

最终定向验证：

```text
rm_description、rm_dog_hole、rm_simulation build：通过
rm_dog_hole geometry gtest：7/7 通过
colcon test-result：8 tests，0 errors，0 failures，0 skipped
Python launch/runner 语法：通过
生成的 dog_hole_scene.sdf：Valid
robot.height == roof_clearance：启动前按预期拒绝
git diff --check：通过
```

关键证据 SHA-256：

```text
c6c456e371d8c581ec523d0dc54e6e60aeb5661e850823e1801965bb25498fa8  required_traversal_center/capture_matrix.csv
256cb79267e901b9d55e0c3bdbfda40885cc539255a2fbedd30eb8e03ce7e820  required_traversal_roof_boundary_4/capture_matrix.csv
99838834f31ee907ca9e31e3c604c6ca36d3b3d8e2c44166919bda7dad3c363b  final/build.log
9707cd379b0cd2d0fdd2f12c7063ea6ba531930355273113f1e6140fc26a6d5b  final/test_result_rm_dog_hole.log
```
