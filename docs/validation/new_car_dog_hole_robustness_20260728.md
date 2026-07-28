# 新车狗洞入口接管鲁棒性验证（2026-07-28）

## 范围与边界

- 分支：`feature/new-car-dog-hole-sim`
- 验证基线：`29b7107`
- 隧道临时模型：宽 `0.80 m`、长 `1.00 m`
- 机器人临时外形：长 `0.60 m`、宽 `0.50 m`
- 本轮只验证二维平面入口接管，不代表最终 CAD、坡面、俯仰和
  `250 mm` 洞顶验收。
- 未启动真实串口、雷达、底盘或旧车链路。

## 实现调整

- Nav2 只导航到入口外的开阔准备点，并保持当前底盘航向。
- 专用控制器接管 `ALIGNING`：先原地旋转，再横向对中并前进到洞口。
- 狗洞仿真专用 MPPI profile 使用 `batch_size=1000` 和
  `retry_attempt_limit=3`，不修改旧车正式配置。
- 最小墙体间隙按旋转矩形 footprint 的角点投影计算。
- 新增参数化矩阵工具，可在入口交接点注入横向/航向真值偏差，记录
  状态、误差、间隙和耗时；支持中断后 `--resume`。
- Ignition `set_pose` 增加最多三次重试，避免单次服务瞬态污染控制统计。

## 实验结果

修正偏差符号后的 49 组矩阵覆盖：

```text
横向：0、±2、±5、±10 cm
航向：0、±2、±5、±10 deg
```

- 49 组已调度，48 组成功进入任务，均到达 `FINISHED`，无碰撞。
- 唯一未执行项 `+10 cm/+2 deg` 在任务前发生一次
  `Gazebo set_pose failed`；同一工况复测 `3/3` 成功。
- 因此控制器实际执行样本为 `51/51` 成功，但原始矩阵仍保留该基础设施
  失败行，不篡改原始证据。
- 矩阵内注入误差最大偏差：横向 `0.000151 m`，航向 `0.0105 deg`。
- 洞内最大横向误差 `0.000189 m`，最大航向误差 `0.000626 deg`。
- 最小旋转 footprint 墙体间隙 `0.149808 m`。
- `-10 cm/-5 deg` 有一次穿越段耗时 `16.13 s`；复测 `3/3`
  为 `8.77–9.00 s`，未形成稳定失败。
- 四个边界角点 `±10 cm × ±10 deg` 各重复三次，`12/12` 成功，
  最大穿越段耗时 `9.40 s`，最小间隙 `0.149822 m`。

结论：当前二维仿真中心线控制器已覆盖本轮定义的
`±10 cm/±10 deg` 入口接管范围。本轮没有证据要求立即开发 MPPI
狗洞 Critic，也没有修改正式定位或旧车导航配置。

## 构建与测试

```text
rm_dog_hole、rm_simulation 定向构建：通过
rm_dog_hole geometry gtest：5/5 通过
colcon test-result：6 tests, 0 errors, 0 failures, 0 skipped
Python 脚本与 launch 语法：通过
安装可执行：dog_hole_manager、dog_hole_capture_matrix
```

## 证据

宿主机目录：

```text
/tmp/rm2027_dog_hole_robustness_20260728/
```

关键文件 SHA-256：

```text
ed30c40ee33560a603d7192d2e6c509181dfd52d7baf6aa6c0360bfb32f0b72d  full_matrix_49_v2/capture_matrix.csv
e8c6f06ced3a0e27e9c26c49b3bb4b16e6e99301091f6053028008bee71b684f  retry_L10_Y2_R3/capture_matrix.csv
a26acf14c2d4068c20cb77d134d861f705b63a465426b8ed585bffdd731184bc  boundary_corners_R3/capture_matrix.csv
fad96b367b2a45bd73c9f63d57ec244337e96c1ddb1c738ca58364850fa19db8  retry_Lm10_Ym5_R3/capture_matrix.csv
96e85113db74dc988d2df3e9969165476a17ca3e9ea856eed66b705bf69d1d6b  final_build.log
86cf30b7bc388dac603c85e0f56b7039e658e7212070c051e2799357946cb0b0  final_test_result.log
```

## 后续

下一轮按优先级增加：

1. `map -> base_link` 横向/yaw 噪声、延迟和低频漂移；
2. 横移响应不足、角速度延迟和方向不对称；
3. 最终车体尺寸、不同洞深、规则坡面与洞顶三维碰撞。

在这些验证完成前，结论仅为二维仿真候选通过，不等于新车实车可用。
