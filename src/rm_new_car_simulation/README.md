# rm_new_car_simulation

ROS 2 Jazzy / Gazebo Harmonic 的独立新车Research候选入口。

```bash
ros2 launch rm_new_car_simulation baseline.launch.py
ros2 launch rm_new_car_simulation baseline.launch.py field:=competition2025_reference
ros2 launch rm_new_car_simulation baseline.launch.py lidar:=B sensors:=true use_nav2:=true
```

四轮顺序FL/FR/RL/RR；实际wheel joint速度和理想方向接触驱动车体。CAD8点polygon、轮轴和A/B原点来自保存的用户接受基线；mass/inertia/轮安装方向/摩擦/完整外参仍为假设。

同系解释下CAD8点polygon未包含所有轮心，完整机械包络仍unknown。S2分别记录CAD车体、物理collision proxy及Nav2 footprint；后者包络实际代理、旋转雷达、roll±3°/pitch±18°和support行程，再加20 mm导航margin，Nav2不另加padding。

`rules2026_course`是规则维度测试场，`competition2025_reference`是北极熊2025native mesh参考，`dog_slope`是带坡局部洞fixture，`banked_ground`是1°–2°外倾地面fixture。`tunnel_section:=A/B`区分700×260的保守A throat与800×250的B，`field_tolerance:=lower_clearance`选−5%边界。完整2026场地没有认证。

S2 **限定Research Go**：默认采用四个被动数值柔顺support，11°/15°坡顶侧滑降至约11.6/13.3 mm。mass、I、CoM、roller、k/d均非实车标定；`physics_config:=<yaml>`可显式复验rigid与敏感性负例。base_link为名义地面投影参考，压缩后平地z约−3.26 mm，保留实际位姿。默认实体碰撞不包含导航margin。真实机构/动力学、全场拓扑和多层自主过坡/洞仍未验证。

每次生成同revision的world、URDF、PCD、occupancy和manifest；TF由显式physics oracle和robot_state_publisher唯一拥有。不得同时启用其他odom/base或map/odom定位TF源。`/odometry/simulation`为真值；regular GPU lidar为几何扫描代理，非实际Livox/IMU时序。

`publish_truth_tf:=false`只关闭oracle的map→odom与odom→base两edge，仍保留truth odometry；其他owner未接替时不能运行依赖这些edge的Nav2。当前Nav2只含static+inflation，controller明确订阅`/odometry/simulation`；STVL尚未安装/接入此Jazzy入口。occupancy为全部结构的保守cell相交投影，不声称高架/洞口的多层通行性。

本轮仅迁入d5673e3a的S2软件载体，冻结研究证据仍留原分支。新增 controller/behavior → cmd_vel_nav → velocity_smoother → cmd_vel 的实际仿真路由；不改变物理模型或Nav2算法。Humble构建需明确排除此可选Jazzy包；两种ROS环境不可同图混用。

来源及边界见 [软件集成合同](../../docs/contracts/new_car_software_20261012.md)。原研究说明可在 `d5673e3a1cfe6937e4126e1df894ee6d66e63e07:docs/research/new_car_simulation_baseline/`恢复；没有迁入58k行历史研究或批量证据。
