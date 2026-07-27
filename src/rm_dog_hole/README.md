# rm_dog_hole

新车隧道（队内俗称“狗洞”）穿越候选实现。旧车不加载本包。

第一版采用混合职责：

- Nav2/MPPI 生成原始路径并完成入口接近、入口对正和出洞后的目标恢复；
- `dog_hole_manager` 检查路径是否同时穿过隧道两半；
- 洞内由中心线闭环控制器根据 `map -> base_link` 计算横向误差和底盘航向误差；
- 洞内速度发布到 `/cmd_vel_nav`，继续经过现有 velocity smoother 和最终
  `/cmd_vel` owner；
- 雷达或云台朝向不参与底盘航向误差计算。

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
