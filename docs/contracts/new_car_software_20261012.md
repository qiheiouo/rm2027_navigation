# 新车第一版软件集成合同

这是独立Integration软件基线，不是Deployment或比赛profile。`main-new-car@bd6cf689`上保留FAST-LIO `3911af9c`与AUD-001 `753131e6`的完整有效代码；历史来源为 `64d0a2d4`，本分支只保存必要资产；AUD-003 `a7778c67/a50a4baf/b6407f4d`保留在原独立分支，**未迁入**本基线。V2原transport仍有AUD-003，不得用于硬件导航。新入口只有清单/软件测试模式，没有hardware选项或授权接口。

## 可复现入口与环境

`ros2 launch rm_navigation_bringup new_car_software.launch.py`默认inventory，启动零节点。其他模式必须明确`enable_test:=true`。

| 模式 | 环境/实际节点 | 输入、TF、速度及边界 |
|---|---|---|
| `bundle_no_input` | 固定Humble镜像、Livox消息overlay、修复FAST-LIO、bringup；仅启动laserMapping_bundle | driver未启动；`/livox/left/lidar`及`/livox/lio_imu`等待真实输入；raw输出`/odometry/fast_lio_raw`，backend TF隔离且publish_tf=false；无adapter/地图TF/Nav2/serial，缺输入不伪造odom |
| `v2_protocol` | Humble；原dry-run V2 + mock lower + gimbal adapter | 固定dry_run=true，不打开device，无Nav2/Mission/Goal；默认无cmd publisher时zero/disabled。专用probe可以验证有限/限幅/0.2s watchdog编码，正编码只是codec fixture，不能将此模式与导航组合或称任务授权 |
| `s2` | **另一个Jazzy/Gazebo8环境**；复用S2平地、oracle、RSP、map/Navfn/MPPI/behavior/smoother/lifecycle | 明示仿真真值及provisional geometry，无driver/FAST/全局定位backend/serial/Mission；static+inflation。controller+四个behavior publisher→`/cmd_vel_nav`→唯一smoother→`/cmd_vel`→ros_gz_bridge→模拟车；不是硬件动作授权 |

两种ROS环境分别构建/运行，不相互source overlay，不同domain/partition。不能把S2的Jazzy Goal成功写成Humble全栈或真实定位通过。S2可选包不在Humble目标包集合；Humble默认全仓colcon会发现该包并缺Gazebo8，必须使用文档的目标路径，不冒充全仓构建。没有自动升级共享镜像/库。

## 第一版真实新车候选链（尚不执行）

| 边界/owner | 源码实际候选 | 未闭合条件 |
|---|---|---|
| MID360/Livox | single driver left契约：`/livox/left/lidar` CustomMsg、`/livox/left/pointcloud`；raw IMU `/livox/lio_imu_raw`；SDK frame_id livox_frame由imu_frame_adapter改为lio_imu_link，输出`/livox/lio_imu` | 现有IP .50/.3和time_sync模板需按实机确认。selected_side=left是软件topic名，S2 A/B不能直接等同实装雷达。没有导入生产外参 |
| FAST-LIO | gitlink ae5bef29，默认bundle，raw Odometry child=body；backend TF只进quarantine；实时原始点时间/IMU用于ESKF | AUD-038/046通过；async/adaptive同型越界仍未修，禁止本候选使用；真实初始化、退化/精度/时间同步待验 |
| canonical odom | `lio_adapter`发布`/odometry/lio`并独占odom→base_link；body只alias为lio_imu_link，sensor_tf用真实时刻T_base_sensor反变换，并估计canonical twist | backend sensor-initial/reference语义、IMU原点、动态外参/6DoF和真实运动学尚未证明；不能靠改child name闭合 |
| 动态云台/heading | V2 `/gimbal/state`、`/chassis/heading`；RSP根据joint_states发布base→gimbal→sensor；fusion候选会生成`/gimbal/state_derived` | Full-base/heading融合仍非验收能力，alignment=false保持。Phase2A没有转发gimbal_use_input而localization_adapters最终覆盖false，derived配置不能单凭路径启用（已有AUD-060）；本轮未启用、不绕过校验、不修复未选融合路径 |
| global定位第一候选 | AMCL_2D（需真实scan和地图），AMCL tf_broadcast=false；amcl_pose_gate→`/localization/amcl_backend_valid/status`→map_odom_from_global_pose | bridge是唯一dynamic map→odom owner；hardware禁止map_odom_stub/仿真oracle并存。源预算1.0s及1.1s截止、bridge接收1.0s；真实静止accepted-pose节奏需实测 |
| global备选 | GICP_3D（需经过验证PCD/seed），gicp_pose_gate→backend status→同一bridge | 1.5/1.6s源预算；generic registered-cloud发布/配置缺口AUD-061仍未闭合；须显式enable scan_publish和正确坐标，当前bundle_no_input不是GICP profile |
| Nav2 | Navfn + MPPI + behavior + velocity_smoother，速度必须controller/behavior→raw→smoother | 本轮实际S2完成这一软件路由；generic默认behavior bypass仍存在，old-car competition不是新车配置。硬件新car专用参数需真实footprint、动力学和授权再制作 |
| V2 | 保留原competition_v2 codec/driver，独立dry-run可验；新车候选不采用HPM/legacy | `SerialConnectionState.online/compatible/ready`不等于autonomous许可。无真实chassis_mode_raw生产者；AUD-003未闭合，没有Goal→硬件串口运行组合 |
| 地图/障碍/任务 | S2使用生成test occupancy/包络footprint、static+inflation；实车首版候选普通VoxelLayer（Phase2G接口参考），实时PointCloud2和硬件footprint待验 | S2未验实时障碍避让。旧STVL参数/旧车地图不迁入；STVL、Mission、裁判、自动巡逻暂不启用。人工单目标Task owner须在AUD-003合同通过后才接硬件 |

S2TF：oracle独占map→odom及odom→base_link，RSP只发机器人joint/sensor树，`/odometry/simulation`显式真值。真实候选：lio_adapter独占odom→base_link，bridge独占map→odom。两者不能同图混合；map server只提供地图，无定位TF。AMCL/GICP各只能选择一个，现有定位状态失效会关闭新TF并保留校正诊断，不能据此认定车辆停止。

## 配置和已知能力边界

本轮没有新车生产标定常数。既有S2 CAD和mass/inertia/support/传感器原点配置仅仿真假设，仍标provisional；S2 footprint是模型包络，不是实物认可footprint。Phase2G/Nav2/driver模板也不是生产文件。Full-base、covariance、T-DT、Temporal MPC和动态预测研究没有迁入。

`LocalizationStatus`的instance/started_at/sequence/emission/valid_until/source pose保持AUD-001完整合同；变化Bool、缓存true、有限odom、TF刷新和心跳均不能替代新测量或授权。probe中的Mission/readiness只是继承修复的消费者回归，第一版profile不启用Mission。

来源：S2包从`d5673e3a1cfe6937e4126e1df894ee6d66e63e07`精确选取；仅补smoother路由、实际owner诊断与入口测试，不改physics/规划算法。外部场地mesh和Apache-2.0 LICENSE原样保留，来源记录在`docs/external/polarbear_rmuc2025_field.md`；默认flat不使用参考比赛mesh。旧研究结论、58k行历史证据不合并。

实测与版本见[集成报告](../new_car_software_saved_20261012.md)，复现见[入口说明](../../tests/new_car_integration/README.md)。实车步骤见[联调验收](../new_car_hardware_acceptance_20261012.md)。
