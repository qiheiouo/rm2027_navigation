# 新车狗洞穿越仿真候选

## 规则与临时假设

《RoboMaster 2026 机甲大师超级对抗赛比赛规则手册 V2.1.0》图 4-35
给出的隧道截面尺寸为宽 `800 mm`、高 `250 mm`。5.5.3 节还建议机器人以
低于 `0.8 m/s` 的速度通过隧道，以稳定触发场地交互模块。

手册图中没有给出可独立确认的完整隧道深度。本候选暂用 `1.0 m`，并使用
现有 Phase 1.5 占位机器人尺寸 `0.60 m x 0.50 m`。所有尺寸集中在：

```text
src/rm_dog_hole/config/dog_hole_sim.yaml
```

入口和出口坡度已参数化。第一轮自动穿越使用平面 profile，即坡度和台面
高度为零；这不是对正式场地无坡的判断。顶部只做半透明可视模型，未作为
碰撞体参与第一轮二维控制验证。最终 CAD、离地间隙和真实坡道验收仍然是
独立机械门槛。

## 方案决策

候选方案包括普通 MPPI、切换 MPPI 参数、专用 MPPI Critic、洞内专用控制
和下位机动作。第一版选择：

```text
Nav2/MPPI 接近入口
-> Nav2 对正底盘
-> 中心线专用控制器穿洞
-> Nav2 恢复原始目标
```

普通 MPPI保留为路径生成和对照基线。仅切换参数不能清楚表达底盘航向与
中心线约束；首版自定义 Critic 会增加 Nav2 Humble 插件开发成本；交给
下位机不利于当前仿真记录和方案比较。

洞内控制使用 `map -> base_link`，不使用雷达方向。控制器把速度发到
`/cmd_vel_nav`，继续复用 Nav2 `velocity_smoother -> /cmd_vel` 和现有
底盘仿真边界。行为树或比赛任务后续只需要调用开始/撤销接口，不应承载
每周期速度计算。

## 状态与接口

```text
IDLE
-> PLANNING
-> APPROACHING
-> ALIGNING
-> CROSSING
-> EXITING
-> FINISHED
```

错误进入 `FAILED` 并停止洞内命令。`/dog_hole/start` 和
`/dog_hole/cancel` 为第一版任务接口。路径必须同时经过配置区域的入口半
区和出口半区，才会触发专用流程。原始最终目标保存在任务节点中，穿越后
重新交还 Nav2。

## 动态云台 TF

仿真链为：

```text
map -> odom -> base_link -> gimbal_yaw_link -> sim_lidar_link
```

`sim_gimbal_state_publisher` 同时发布 `/gimbal/state` 和 Gazebo 关节命令。
`gimbal_state_adapter + robot_state_publisher` 仍是 ROS 动态 TF owner。
第一轮使用 `0.65 rad`，实测 TF 为：

```text
base_link -> gimbal_yaw_link
xyz = [0.0, 0.0, 0.35]
yaw = 0.65 rad
```

真实新车仍需要下位机提供有时间戳的云台角度和最终外参，本轮没有新增
串口字段。

## 启动

```bash
source install/setup.bash
ros2 launch rm_simulation dog_hole_sim.launch.py \
  headless:=true use_rviz:=false auto_start:=true gimbal_yaw:=0.65
```

也可以从统一仿真入口启动：

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=dog_hole headless:=true use_rviz:=false
```

生成的场景和派生 Nav2 profile 位于
`/tmp/rm2027_dog_hole_sim/`。运行 CSV 默认写到同一路径。

## 第一轮结果

Humble 定向构建通过，狗洞几何测试 `4/4` 通过，URDF 和 SDF 校验通过。
一次自动运行生成 `201` 个路径点并识别出路径穿越隧道，完整进入
`APPROACHING -> ALIGNING -> CROSSING -> EXITING -> FINISHED`。

洞内记录：

```text
穿越控制时间       5.171 s
中心线控制样本       163
最大横向误差       0.0204 m
最大底盘航向误差   0.0290 rad
最小几何侧向间隙   0.1296 m
最终状态           FINISHED
```

入口 Nav2 阶段曾出现两次 MPPI optimizer failure，恢复流程随后成功，未
阻止任务完成。该现象保留为后续对照指标，不在首轮通过后立即大规模调参。

## 尚未验收

- 非零入口/出口坡度和顶部物理碰撞；
- 最终新车 CAD、真实轮地接触和底盘尺寸；
- 真实 MID360、双雷达、旋转云台 LIO 和时间同步；
- 真实狗洞材质、边缘、坡道与低矮环境感知；
- 比赛行为树正式接线、失败重试策略和多次统计；
- 真实车辆穿越。

因此当前结论仅为“第一版软件与平面 Gazebo 自动穿越候选通过”，不是实车
或比赛场地准入。
