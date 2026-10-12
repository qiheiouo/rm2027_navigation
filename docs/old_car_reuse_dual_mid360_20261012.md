# 旧车复用与共刚体双MID360候选

本轮为局部配置/接口Integration核对，不是新算法或整车Deployment。公开保存分支与实验候选分开；正式main/main-new-car/main-old-car、所有冻结研究不修改。AUD-003仍Modify，禁止真实底盘导航正授权。

## 固定版本与发布

| 对象 | 现场完整身份 |
|---|---|
| main | fcd9a05b03c9539b4b5ea44de0f09d30c4468455 |
| main-new-car | bd6cf689d3d5aca5c8edd430ff760ca5ceb27496 |
| main-old-car（远端ref） | 820179457b70789b0f46697758dbe3d947ec1d4a |
| 原集成 / 原精简 | 64d0a2d427ba50ba2858fbbcb0abbf9e770f63f3 / b2739535b8951b1fcc9c3a9fbbef0fdf395b2ac5 |
| 新公开保存版本 | b5177b9b818b6cbeb3dac4a014e915f9e6d69ec1，integration/new-car-software-remote-20261012 |
| 本候选基线/分支 | b5177b9b；integration/new-car-dual-candidate-20261012 |
| AUD-001完整来源 | 753131e6b09b0440b0ef0c9fd6155aba8e798687 |
| FAST / 原pin | ae5bef29c89ba7a1004d75657db482bb277e603a / e7864a63a7e2a1ad62ec8fd75fcb9149db08e321 |
| ikd-Tree / Livox | 0438b0daa6ccfaaad9f65f45a3addc318b19ae7a / 2a2029a6e62a2196b280be6ec00bb2418065b8e0 |

保存版直接从bd6选取精简有效内容；与b273的src/tests仅差CAD YAML三项个人元数据（历史聊天标题、聊天ID、附件ID），经用户授权删除，全部数值不变；.gitmodules仅FAST URL改为用户fork，gitlink不变。原提交/历史保留。重新执行S2几何3测试，3200包络样本、最小裕度.0248262344m；其他源码/测试逐字节同一性核对通过，不能算新保存版全栈重跑。

superproject原origin为https://gitee.com/qiheiovo/rm2027_navigation.git，SSH git@gitee.com:qiheiovo/rm2027_navigation.git，用户本轮允许该公开仓库。FAST fork为https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git；独立维护ref为integration/rm2027-bundle-fixes，不覆盖main。fork main仍e786，父仓库Draxran。Gitee SSH认证/读取及指定保存分支push --dry-run通过（未实际写入）；GitHub通过官方API公钥及官方指纹核验host key后仍Permission denied(publickey)，HTTPS无认证。**未push任何分支。空对象库从用户fork fetch ae5实际not our ref，纯远端递归恢复未通过。** 先解决GitHub认证并保存ae5，再保存superproject，否则后者不算完整备份；没有自动创建其他远端。

原完整FAST bundle 144394828B，SHA256 a762e1ecae688137c4c6ee7324948c1dddf37c40928344e06dbccf33473abc5e，工作区外备份/home/qihei/rm2027_git_backups/20261012与原文件均保留。此为同盘备份，异机冷备未完成。新保存bundle、候选提交/对象体积和外部清单见交付记录。大型日志、实验JSON/PCD、build/install/log、旧车只读导出均ignored，不进新历史。

## 旧车实际链与实车证据

以下源码路径均按旧车82017945解释，不以新车研究反向推测。可在新只读checkout该SHA查看原报告，不复制其全部材料。

实际入口是src/rm_navigation_launch/launch/old_car_full_navigation.launch.py，委托rm_navigation_bringup/launch/old_car_2026_competition.launch.py和old_car_2026_validation.launch.py。当前wrapper启用right_lidar/dual_fusion、nav2_old_car_2026_dual_stvl.yaml、AMCL spin robust candidate、hpm_crc_v1；Mission startup=false，原始操作员Goal RX关闭，现场依赖受管服务和人工SOP。

一SDK双MID360→两侧PointCloud2/CustomMsg；**FAST验证入口仍sensor_mode=single、左LiDAR、一条选定IMU**。右雷达用于双障碍融合。lio_adapter独占odom→base_link，AMCL gate→map_odom_from_global_pose独占map→odom。local STVL合并两路marking、两侧独立clearing；global static+ObstacleLayer仍左/localization/scan，因此仅右侧看到障碍不保证全局换路。Navfn+MPPI；old validation对Nav2组设置cmd_vel→cmd_vel_nav，使controller与behavior经Humble velocity_smoother输出/cmd_vel，再进入HPM serial_transport。需要使用这个组级remap，generic Nav2直接include不能自动继承它。

旧车rm_description/urdf/rm_old_car_2026.urdf.xacro明确“no navigation gimbal”，base_link→gimbal_yaw_link为固定零变换；有这个frame名称不代表旧车实测动态云台。left/IMU是旧车标定，right历史占位后2026-08-23操作员确认标定并允许provisional override，原始标定产物未归档，不能用于2027新车。

| 已有证据（旧车docs/validation） | 能证明的范围 | 不能继承的结论 |
|---|---|---|
| old_car_navigation_status_and_roadmap_20260720.md | 单左LIO、canonical TF、AMCL、真实HPM、基础目标/部分Mission链 | GICP当时仅fake seeded；当时无右/双验收 |
| old_car_fresh03_field_validation_20260720.md | 人工初始位姿、AMCL对齐、.15m/s短Goal SUCCEEDED、recovery=0，操作者确认停止 | 新地图正式批准、MCU/物理定量停止、双LIO |
| old_car_three_point_spin_field_test_20260722.md | 现场流程操作者认为效果可用、mock referee | 未量化10rad/s或定位漂移；不证明最终授权 |
| old_car_global_dynamic_replanning_20260806.md | 遮挡后FollowPath失败→Navfn绕路，旧BT/进度检查实测 | 只右障碍全局换路、完整新车障碍覆盖 |

## 旧车→新车复用矩阵

A直接复用；B参数/标定/launch；C已确认局部软件缺口；D真实硬件；E研究暂不迁入。源码路径在两版相同，除特别说明。

| 模块 / 旧车入口 | 新车真实差异与类别 | 证据 / 最小行动 |
|---|---|---|
| Livox与rm_mid360_driver_bridge/launch/dual_mid360_driver.launch.py | B/C/D：一SDK两设备现成；原硬编码旧IP及JSON无法正确切换新设备 | 此轮仅增加config_file、left/right_device_ip，核对两身份与JSON一致，派生真实SDK topic；legacy默认不变。新IP仍未知，不填新生产值。已有CustomMsg adapter保留点内时间；右IMU仅诊断 |
| rm_lio_bringup/config/fast_lio_multi_old_car_2026.yaml；旧validation | A/B/D及C待决：bundle和单IMU直接复用；双模式已有但旧实车FAST未用双 | 原算法/pin不改。新backend-only实验模板从已有dual派生，固定IMU←左右LiDAR、dual=true；AUD-038/046未覆盖回旧代码。双时序/断流边界见下 |
| rm_description/urdf/rm_old_car_2026.urdf.xacro；新rm_sentry_2027.urdf.xacro | B/D：旧固定底座，新continuous yaw+两侧固定枝已存在 | 直接复用RSP为唯一机械TF owner、gimbal_state_adapter仅JointState。须实测轴/符号/home、杆臂、左右/主IMU刚体外参；不把新placeholder当标定 |
| rm_localization_adapters/src/lio_adapter.cpp、canonical_odometry.cpp；lio_adapter_old_car_2026.yaml | A/C/D：canonical-parent源时刻反算与有限差分已有；sensor_initial动态初始参考处理不完整 | 复用现成正确模式；没有把FAST初始系假称canonical。不改算法来掩盖反例。B0与raw epoch合同需先明确，见下；Phase2A未正确传入动态输入AUD-060仍在 |
| gimbal_state_adapter.cpp；V2 competition_v2_transport_node.cpp | A/B/D：有效、有限、非零/新鲜源时刻检查已有；V2用estimateMcuStamp(mcu_time_ms)，heading带boot/reset | 无需第二TF owner或消息框架。MCU/ROS映射由已有clock sync估计，但其与Livox时钟真实误差尚未验收；HPM不能提供相同独立heading语义 |
| AMCL gate/bridge、old competition与generic入口 | A/B/D：算法与阈值直接复用，新保存版完整AUD-001改状态合同 | 初始位姿、地图、scan投影/footprint重配。新/旧gate不能混用；有效状态为/localization/status。AMCL稳定静止消息节奏待真机，不用TF刷新替代质量 |
| GICP现有rm_gicp_relocalization及competition flags | A/B/C/D：已有独立backend，无需NEU替换；generic registered cloud接线AUD-061仍有限制 | 要用实际对齐的registered cloud/map接口；旧车未实测GICP，原batch软件probe不等于新双实车。此次不扩展该算法/接线研究 |
| Nav2/Navfn/MPPI、nav2_old_car_2026*.yaml、旧validation组级remap | A/B/D：算法已有；新车frame/footprint、速度/加速度、路由与Goal owner必须选定 | 不改Nav2。S2路由smoke为原基线结论，不能代替新Humble硬件；generic include与旧competition行为输出接线有差异，尚无新车比赛profile |
| dual_pointcloud_obstacle_fusion.launch.py；dual STVL YAML、旧replan BT | A/B/D：已有过滤/融合/STVL，不需新障碍前端 | 重标自体滤除、双清除/投影范围；右-only全局路由边界保留。此轮未启用或部署STVL |
| HPM与V2 transport、Mission/operator authority | B/C/D：旧真实HPM不可直接等同新V2；最终运动授权仍AUD-003 Modify | 保留既有协议限幅/watchdog能力；不导入a7778c67/a50a4baf/b6407f4d正授权。Mission及自主启用推迟，HPM现场停机不能继承为V2硬件停止 |
| 最新旧车readiness、map revision UTC、mapping退出处理 | C可选差异，本轮不迁入 | 820较新接口不全在新基线；非此轮双输入阻塞，不整体merge旧分支 |
| Full-base / covariance / NEU / Temporal MPC与动态预测 | E | Full-base部分sensor→base/杆臂/源时刻功能已有；真正额外能力为非平面heading/初始锚点/epoch失效、协方差传播。NEU与现成全局定位职责重叠，但候选算法不等价。MPC/预测不是基础双输入必需；全部保持冻结，不复制第二owner |

## 共刚体与backend真实合同

机械已确定：两MID360同云台刚体，gimbal→L1/L2及L1→L2恒定，不随yaw重标。主IMU首候选左MID360内置IMU；同云台固定IMU→L1/L2。一SDK选择lio_imu_source=left将/livox/imu_<left-IP>送/livox/lio_imu_raw，右送/livox/right/imu_raw；沿用imu_frame_adapter得到/livox/lio_imu。backend只订阅此一条IMU；不是双IMU融合。

ae5 bundle在extrinsic_imu_to_lidars=true下计算L1←L2=inv(IMU←L1)*(IMU←L2)，变换右云至左，统一一次ESKF更新。扫描begin取两路最小、end/输出stamp取最大，IMU需覆盖两end。点内offset_time经CustomMsg→curvature为扫描局部时间；driver接收/到达顺序不能代替source同步。

源码sync_packages将各侧排队扫描直接+=，不重基准后续扫描的局部点时间；缺某雷达不自动降级，另一侧队列无年龄上限；恢复会合并积压云。缺IMU会等待覆盖，当lidar_pushed已锁定一对扫描时恢复先处理旧对，再清空后来积压帧。UndistortPclMulti的IMU pose offset以min(begin)为零，第二云仍直接使用本地curvature，未显式加入begin2-min(begin)。所以源扫描错开下“有限odom”不证明运动去畸变正确。本轮按停止条件记录，不新增时序融合框架或改FAST数学。

## lio_adapter判断

旧/新sensor_tf都在raw源时间查base→sensor（use_latest_transform=false），FIFO等TF后发布；先转换pose再finite_difference，能取消合法canonical-parent下云台运动。零stamp/latest fallback存在，有限差分零stamp不发布；旧frame alias body→lio_imu_link不是坐标转换。有限差分跨坏时间/大gap会重建基线。

canonical_parent已有正确关系：O←B(t)=(O←S(t))*inv(B(t)←S(t))。sensor_initial helper则用B(t)*raw(t)*inv(B(t))；对于固定旧车B0=B(t)，此为合法共轭。动态新车需要B0*raw(t)*inv(B(t))，前提是知道raw初始epoch和实际参考t0。不能把“第一条收到的TF”或第40扫描zero_start重基准直接当FAST真实t0；后端同frame单调stamp隐藏rebase也不能由现接口识别。

本轮实际RSP/gimbal adapter/lio adapter探针对canonical-parent三种运动保持正确；sensor_initial纯云台/反向运动保留明确反例（结果见下）。**因此本轮不改lio_adapter，不把配置切成虚假的canonical_odom以求绿色。** 最小后续前置是选定raw origin/rebase合同、源时刻初始锚点与失效边界；若需Full-base整体迁入，应另决策。现成chassis_heading_fusion为独立heading下平面候选，不能未标定开启或替代该证明。

sensor_tf pose covariance目前沿用raw covariance，有限差分twist改为固定配置对角值；没有完整SE(3)协方差传播证明。暂不迁covariance研究。实车先核对其下游使用范围，不能称已完成不确定性正确变换。

## 本轮改动与默认边界

- dual_mid360_driver增加网络配置参数，启用时校验IPv4、两身份互异、JSON精确两设备；仍一SDK+两adapter，默认use_driver=false，legacy IP不变。
- new_car_dual_mid360_pending.yaml：沿用已有dual阈值、topics、滤波/噪声；改固定双IMU外参选择，全部8项外参null，不生成生产IP/几何/补偿。
- new_car_dual_mid360_experimental.launch.py：默认零节点；显式enable_experiment、calibration_revision、完整config_file才可进入现有bundle launch。有限double数组、proper rotation与固定变换组合检查。backend TF仍quarantine，无driver、canonical adapter、global定位、Nav2、串口或Motion owner，不能当整车双定位profile。
- 现有S2 C++ GID观察器新增可选edges参数，默认仍只观察两条canonical边；本轮复用它观察机械边，诊断不构成DDS身份认证或TF发布。
- 新 tests/new_car_dual仅覆盖现有资产缺的双输入和动态初始系反例；原FAST native与driver/adapter单测复用。python3-yaml声明运行依赖。无新驻留节点或框架。

## 新运行验证

所有新输入均合成、容器Humble禁网/无设备/独立DDS域；与旧车实车、原基线软件结论分别处理。

| 本轮重新执行 | 实际结果 / 边界 |
|---|---|
| 固定Livox、bundle debug断言、7目标包及2消费者包 | 构建通过；仅bundle执行。初次colcon发现手动backend未安装、ignored旧车导出重复包，分别显式排除FAST/限定src后通过，非生产修复 |
| 既有adapter/driver测试 | 36 tests，0 errors/failures/skips；包括实际点云adapter点内时间/身份和融合路径、canonical/有限差分单测 |
| 既有FAST native | 初始化/复位3模式各5更新（含双）；state误差1.135e-15、cov误差6.537e-13。测量36次，Jacobian差8.428e-8、reference/posterior差0、分配复用；无效特征负控制保持 |
| 新启动/标定检查 | 4测试通过，含reflection、NaN、矛盾外参、错误IMU、重复/不匹配IP拒绝；CLI默认零节点退出成功，显式启用缺标定拒绝。完整active候选launch没有运行验收，backend正控制使用真实可执行文件直启 |
| 实际dual ROS | 正常、40ms反向到达差、25ms源begin差各6次finite raw odom、无崩溃/正常退出；外参右Rz(pi)、t=(0,-.24,0)为**test fixture**。只静止平面，不能验证移动deskew |
| 右LiDAR gap | 3扫描期间odom保持6；恢复升7，max end stamp=1001.699677749；源码仍合并左积压且未重基准点时间，不能称无损恢复 |
| 主IMU gap | 3扫描期间odom保持6；恢复升7，但stamp=1001.399677749，而最新输入云结束约1001.699677749；证实先处理旧对、未证明新鲜质量 |
| 实际RSP/gimbal/lio_adapter | 源时刻TF（故意推进latest yaw）和output stamp保留；canonical-parent纯云台/纯底盘/反向三例角误差0、位置误差≤3.104e-17m，持稳后twist=0。sensor_initial固定云台纯底盘正确；纯云台角误差.6rad、反向.4rad，动态反例仍存在 |
| 实际per-message TF GID | 两模式中base→gimbal、odom→base各1 owner，map→odom不存在；无第二canonical/机械owner。不是DDS密码学身份认证 |
| AUD-001 AMCL/GICP | 原样AMCL/GICP完整probe均通过：12s稳定923/906状态、0 false；原始输入停1.031/1.477s失效，gate SIGKILL .966/.983s失效；bridge退出时消费者约1.0–1.1s失效，restart/旧消息/重复、未来、过期、负、零时间及无效协方差断言均通过。边界合成pose/odom，不能算原生算法或新车实测 |

动态探针首次遇镜像缺SciPy、URDF CLI解析、Humble Python缺MessageInfo及numpy bool序列化问题，修正测试输入/观察方式后最终统一脚本通过；未修改生产数学、阈值或删除负例。初次尝试日志保留在ignored build，未批量进入Git。机械试验使用现有明确placeholder URDF，是软件合成证据而非生产标定。pose covariance观察到原值照搬，不声称其物理正确。

固定环境镜像id 81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3；命令入口[tests/new_car_dual/README.md](../tests/new_car_dual/README.md)。新增IP/rigid校验仅在launch做一次，driver/后端每帧路径未改；没有新增常驻融合/授权进程。未重跑S2 Nav2 smoke、完整Humble所有包、V2原29场景、FAST Release对照或任何硬件；对应生产源码/pin同一性保留，原版本结论不升级。本轮已通过的测试合同限于声明软件范围，双LIO/动态初始系整体仍Modify。

## 最小行动与回退

可冻结：已通过修复的代码保存版本、旧车实际单左LIO/双障碍证据边界、现有driver/adapter/AMCL/Nav2复用判断。远端保存还需GitHub公钥权限，不能宣称已完成。

等硬件：左右IP/序列号、主IMU识别、两个LiDAR与主IMU/云台转轴刚体标定、home符号、Livox/MCU/ROS真实时钟、姿态/速度/停止、双真实deskew/断流恢复质量。不要重做两雷达相对随yaw变化的研究，它们已共刚体。

硬件前软件的明确最小阻塞：raw sensor_initial初始锚点/epoch选择；源时间偏移与积压扫描的实际bundle时间缺口；AUD-003受管正常管线/behavior/恢复合同。前两涉及数学/时序，按本轮停止条件仅给缺口，不自动开发。无必要恢复T-DT、MPC、预测、双IMU或整套Full-base。

在新工作区选b517可回到已通过single软件保存内容；撤销此候选只revert本轮候选提交（或新worktree直接选b517），gitlink保持ae5。若撤销全部已通过修复则revert保存提交并同步自身submodule，不能把撤回安全修复版本称有效。原64、b273与冷备不动。候选未取得完整双LIO/动态云台Integration Go，保持Modify且默认关闭；完成本轮后暂停。
