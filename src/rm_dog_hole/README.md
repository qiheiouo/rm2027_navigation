# rm_dog_hole

新车隧道（队内俗称“狗洞”）穿越候选实现。旧车不加载本包。

第一版采用混合职责：

- Nav2/MPPI 生成原始路径，只负责到达入口外的开阔准备点和出洞后的目标恢复；
- `dog_hole_manager` 检查路径是否同时穿过隧道两半；
- 到达准备点后由专用控制器先原地对齐底盘，再校正中心线并推进到洞口；
- 对正和洞内控制均根据 `map -> base_link` 计算横向误差与底盘航向误差；
- 专用速度发布到 `/cmd_vel_nav`，继续经过现有 velocity smoother 和最终
  `/cmd_vel` owner；
- 雷达或云台朝向不参与底盘航向误差计算。
- 狗洞仿真 profile 使用 1000 条 MPPI 候选轨迹和 3 次软重试，避免基础
  300 条采样在入口接近阶段反复中止；该设置不修改旧车正式配置。

任务状态为：

```text
IDLE -> PLANNING -> APPROACHING -> ALIGNING
     -> CROSSING -> EXITING -> FINISHED
```

失败会进入 `FAILED` 并发布零速度。可用以下服务触发或撤销：

```text
/dog_hole/start
/dog_hole/cancel
```

仿真尺寸、机器人尺寸、控制参数和最终目标统一位于
`config/dog_hole_sim.yaml`。当前 `0.80 m x 0.25 m` 来自 2026 规则手册
V2.1.0 图 4-35；`1.0 m` 深度和 `0.60 m x 0.50 m` 机器人外形均为临时值。

诊断话题：

```text
/dog_hole/state
/dog_hole/control_mode
/dog_hole/path_crosses
/dog_hole/lateral_error
/dog_hole/heading_error
/dog_hole/minimum_wall_clearance
/dog_hole/command
/dog_hole/succeeded
```

同一份信息会写入 `/tmp/rm2027_dog_hole_sim/dog_hole_run.csv`。

## 接管范围实验

先以 `auto_start:=false` 启动仿真，再运行参数化接管实验：

```bash
ros2 launch rm_simulation dog_hole_sim.launch.py \
  headless:=true use_rviz:=false auto_start:=false

ros2 run rm_dog_hole dog_hole_capture_matrix --mode full
```

`full` 默认运行横向 `0、±2、±5、±10 cm` 与航向
`0、±2、±5、±10 deg` 的 49 个组合；`single` 只运行 13 个单轴工况。
实验在入口准备点直接重置 Gazebo 真值位姿，因此偏差不会先被 Nav2 消除。
结果默认保存到 `/tmp/rm2027_dog_hole_robustness/<timestamp>/`。
实验中断后，可对同一个 `--output-dir` 增加 `--resume`，只补跑 CSV 中
尚未出现的 case。

最小墙体间隙按旋转矩形 footprint 计算：

```text
width / 2
- abs(lateral_error)
- robot_width / 2 * abs(cos(yaw_error))
- robot_length / 2 * abs(sin(yaw_error))
```

这部分仍是二维平面接管范围验证；坡面、俯仰和 250 mm 洞顶需要使用最终
三维车体模型另行验证。
