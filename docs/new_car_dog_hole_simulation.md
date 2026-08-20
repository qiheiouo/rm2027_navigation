# 新车狗洞穿越仿真候选

## 规则与临时假设

《RoboMaster 2026 机甲大师超级对抗赛比赛规则手册 V2.1.0》图 4-35
给出的隧道截面尺寸为宽 `800 mm`、高 `250 mm`。5.5.3 节还建议机器人以
低于 `0.8 m/s` 的速度通过隧道，以稳定触发场地交互模块。

当前实验按用户提供的近似值使用 `0.30 m` 隧道深度。新车俯视轮廓暂解释为
四条 `382 mm` 直边和四条 `126 mm`、45 度倒角边交替组成的对称八边形，
由此推导轴向外接尺寸为 `560.19 mm x 560.19 mm`。所有尺寸集中在：

```text
src/rm_dog_hole/config/dog_hole_sim.yaml
```

入口和出口坡度已参数化。当前自动穿越使用平面 profile，即坡度和台面高度
为零；这不是对正式场地无坡的判断。侧壁、顶板、八边形底盘、云台圆盘和
雷达占位体均参与 Gazebo 碰撞。最终 CAD、离地间隙和真实坡道验收仍然是
独立机械门槛。

当前变形机构尚未定型，因此只提供启动时几何 profile，不发送下位机标志：

- `deformed`：包络高暂定 `220 mm`，默认用于自动穿越；
- `undeformed`：包络高暂定 `320 mm`，用于确认物理上会被 `250 mm` 顶板阻挡；
- 底盘—云台接口离地 `115 mm`；
- 云台半径暂定 `200 mm`、厚 `40 mm`；
- 仿真雷达暂放在云台轴前方 `120 mm`，最终安装位置必须按 CAD 更新。

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

## Chassis-heading LIO 与动态云台 TF

仿真链为：

```text
map -> odom -> base_link -> gimbal_yaw_link -> sim_lidar_link
```

默认狗洞仿真不再把已知云台角直接送入 ROS TF，而是明确启用
`lio_adapter.pose_conversion_mode=chassis_heading_fusion`：

```text
Gazebo base truth + configured gimbal truth
  -> synthetic /odometry/fast_lio_raw[sensor pose]
  -> synthetic /chassis/heading
  -> lio_adapter
  -> /odometry/lio + odom -> base_link
  -> /gimbal/state_derived
  -> gimbal_state_adapter -> /joint_states
  -> robot_state_publisher -> dynamic sensor TF
```

`/simulation/gimbal/state_truth` 只供 A/B 诊断，不连接
`gimbal_state_adapter`；默认运行中 `/gimbal/state` 没有发布者。因此派生角是
`gimbal_yaw_joint` 的唯一输入，不存在两个角度源争抢 TF。

仿真源支持 `fixed`、`sine` 和 `continuous` 三种运动。默认从 `0.65 rad`
开始，以 `0.60 rad/s` 连续旋转。当前变形 profile 下的标定占位值为：

```text
base_link -> gimbal_yaw_link: xyz = [0.0, 0.0, 0.115]
gimbal_yaw_link -> sim_lidar_link: xyz = [0.12, 0.0, 0.065]
```

仿真还故意给下位机 world yaw 加 `1.10 rad` 零偏，验证算法只使用 yaw
差值，不要求下位机世界零点与 ROS `odom` 对齐。默认时间偏移为零；
`heading_timestamp_offset_sec` 可生成带正确时间戳和 yaw rate 的异步样本，
`heading_publish_divider` 可把 heading 降采样以测试 `30 ms` 匹配门槛。

真实新车需要下位机提供已有 `/chassis/heading` 契约，并测量云台轴心、启动
home 和 `base_link -> lio_imu_link` 外参；不再要求下位机直接提供机械云台
角。本轮没有修改实车串口协议或 FAST-LIO 实现。

## 启动

```bash
source install/setup.bash
ros2 launch rm_simulation dog_hole_sim.launch.py \
  headless:=true use_rviz:=false auto_start:=true \
  gimbal_motion_mode:=continuous gimbal_angular_velocity:=0.60
```

往复扫动可改为：

```bash
ros2 launch rm_simulation dog_hole_sim.launch.py \
  gimbal_motion_mode:=sine gimbal_yaw:=0.0 \
  gimbal_amplitude:=0.8 gimbal_frequency:=0.10
```

未变形物理干涉检查使用：

```bash
ros2 launch rm_simulation dog_hole_sim.launch.py \
  robot_geometry_profile:=undeformed auto_start:=false \
  gimbal_motion_mode:=fixed
```

若机械要求车体以相对洞轴的特殊方向通过，只修改
`control.traversal_yaw_offset`。该角度同时用于专用控制器对正和八边形净空
计算，零值表示底盘 x 正方向沿洞轴。

也可以从统一仿真入口启动：

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=dog_hole headless:=true use_rviz:=false
```

生成的场景和派生 Nav2 profile 位于
`/tmp/rm2027_dog_hole_sim/`。运行 CSV 默认写到同一路径。

## 2026-08-20 新车占位模型结果

ROS 2 Humble 定向构建、URDF/SDF 校验和狗洞几何测试均通过。几何测试现为
`10/10`，其中包含八边形在洞轴方向、45 度方向和特殊通过方向偏置的净空
计算。默认 `deformed + continuous gimbal` 无界面运行完整经过：

```text
PLANNING -> APPROACHING -> ALIGNING -> CROSSING -> EXITING -> FINISHED
```

启用 `chassis_heading_fusion` 后，带 `20 ms` 正确时间偏移的完整运行仍到达
`FINISHED`。本次洞内记录：

```text
穿越控制时间       3.573 s
中心线控制样本       117
最大横向误差       0.000021 m
最大底盘航向误差   0.000009 rad
最小几何侧向间隙   0.119882 m
最终状态           FINISHED
```

真值 A/B 诊断累计 `2506` 组底盘和云台匹配样本，丢配为零，最大底盘 yaw
与云台角误差均为 `0.0008 rad`，平移误差在九位小数输出下为零，诊断为
`OK`。名义时间对齐运行另累计 `3504` 组，同样通过。

负向测试把 heading 降为每 5 个里程计样本发布一次。最近 heading 超过
`30 ms` 窗口时，`lio_adapter` 记录等待超时并丢弃 raw odometry；融合覆盖率
降至 `0.398075`，A/B 诊断为 `ERROR`。这验证了稀疏/失步输入不会使用无界
旧值继续输出。系统无法仅靠最近邻算法发现“时间戳本身被错误标定但仍连续”
的时钟偏差，因此实车 MCU—主机时钟映射仍是接入门槛。

`undeformed` profile 报告垂直余量 `-70 mm`。生成模型的雷达碰撞体顶部为
`320 mm`，与 `250--290 mm` 的顶板碰撞层重叠；把模型放到顶板正下方后，
Gazebo 解算出约 `6 mm` 平面位移及小角度倾斜，符合物理干涉预期。该 profile
只用于证明“不变形不能通过”，未执行自动穿越。

## 历史基线结果

以下数据来自旧的 `0.60 m x 0.50 m` 矩形占位模型，仅保留作回归基线，
不能当作当前新车几何验收。Humble 定向构建通过，狗洞几何测试 `4/4`
通过，URDF 和 SDF 校验通过。
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

- 最终新车 CAD、真实轮地接触和底盘尺寸；
- 变形后的真实高度、外伸部件和强制通过方向；
- 非零入口/出口坡度及实际 `0.30 m` 深度复测；
- 真实 MID360、双雷达、旋转云台 LIO 和时间同步；
- 真实狗洞材质、边缘、坡道与低矮环境感知；
- 比赛行为树正式接线、失败重试策略和多次统计；
- 真实车辆穿越。

因此当前结论为“新车占位几何 + 连续旋转云台的平面 Gazebo 自动穿越候选
通过”，不是实车或比赛场地准入。
