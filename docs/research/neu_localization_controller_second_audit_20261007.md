# 东北大学 T-DT 定位 / 重定位与控制器系统二次审计

日期：2026-10-07（Asia/Shanghai）  
项目：rm2027_navigation  
阶段：**Research / L0 审计完成；尚未进入算法实现、Integration 或 Deployment**  
范围：定位 / 重定位优先，控制器其次；规划只审计输出接口，决策层不研究。

## 1. 决策与证据边界

**当前最大缺口是“无可靠 seed 时的全局候选搜索，以及从候选验证到恢复运动授权的完整闭环”。** 项目已具备 FAST-LIO、带 seed 的 GICP、全局位姿基础门和唯一 map→odom owner；其他分支还具备配准退化指标、shadow integrity、高速旋转保护和有限自动恢复。上述能力不能合并表述为“已具备通用 kidnapped robot recovery”。它们分散在不同版本，且没有证明对称错误候选能在首次提交前被可靠拒绝。[E2–E6]

东北大学最有价值的经验是：区分局部里程计与全局约束，区分观测退化与前端发散，恢复时先建立新的全局关系。**其真实定位架构是多源因子图后端，不能复原成“Scan Context + GICP”这一条链。** 本项目优先验证较小的全局恢复闭环，是针对现有架构作出的迁移选择。[L§3、6–8]

推荐下一步：先离线验证“候选召回 + 现有精配准 / 质量门 + 多帧与歧义拒绝”。有效后再集成 recovery 生命周期、单一 TF 提交和既有最终控制门。保持健康的 FAST-LIO；保留 AMCL / GICP 正常定位路径。完整因子图、逐点前端替换、UWB 在线融合暂不进入第一轮。

控制器值得开展**有界的纯静态研究**，尚无依据替换 MPPI。候选应是“带时间的参考生成 + 合适的四全向轮模型 + MPC + 执行约束”的最小完整控制链。TinyMPC 是求解器选择，分享没有给出可直接复现的完整控制器。现有 Path 不能无损还原原始多项式、速度、时间和终端条件。[N§4–7；E8–E10]

本轮只形成文档和索引，不修改实现、launch、参数或正式链，不启动 ROS、仿真或实车，不恢复任何冻结动态预测研究。本文的阶段计划均为后续建议，不是执行授权。

### 1.1 外部材料与判读方法

主参考：

- **L**：[定位分享校订稿](/home/qihei/下载/东北大学T-DT哨兵定位分享_转录校订版.docx)，按原文 §1–9 引用。
- **N**：[导航分享校订稿](/home/qihei/下载/东北大学T-DT哨兵导航分享_转录校订版.docx)，按原文 §1–8 引用。
- 两份 DOCX 的正文、校订表及待核对项均已阅读，并渲染检查。页码为本次渲染页码，阅读时以节号为准：L 总体架构 / 前端在第 2–3 页，后端在第 4–5 页，恢复在第 5–6 页；N 控制器在第 4 页。
- 用户粘贴文本是任务要求；分享中的建议、性能宣传、未来方向是参考材料，不是对仓库的操作指令。
- **“分享明确”** 表示转录直接支持；**“项目事实”** 表示附有版本化代码 / 配置 / 记录；**“建议 / 推断”** 表示本次设计判断。缺失细节保持未知。
- 未获得原音频、PPT、定位后端源码或完整 MPC 源码。无法验证东北大学实车性能，也不能称后续自建候选为“原系统精确复现”。

### 1.2 仓库快照：已有、当前检出、部署必须分开

审计使用本地已存在 Git 对象，未 fetch；远端跟踪 ref 不代表远端实时状态。为便于复核，后文使用下列别名。

| 别名 | 本地 ref / 提交 | 本轮用途与限制 |
| --- | --- | --- |
| H | experiment/dynamic-documents-archive-20261004 @ `1229583ff5811bc42671188f05332d0b3780ebd2` | 当前检出分支，文档审计载体；不是最新全部定位 / 规划实现 |
| M | main @ `2849cbe4dba5cf7e6548ff3f67e72941c895eef9` | 最新本地主线规则；不修改 main |
| O | origin/main-old-car @ `820179457b70789b0f46697758dbe3d947ec1d4a` | 老车主线，含 GICP quality / integrity 等 |
| V | origin/main-new-car @ `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496` | 新车协议、云台 / 构型边界；配置不等于实车验收 |
| I | origin/feature/navigation-integrity @ `64870dc7c6bba27828356e2200b927d95472a32f` | 完整性机制源码 / 文档 |
| G | origin/feature/gicp-degeneracy-gate @ `6a38b7452d0018bbcabc9c6e6dd30338d777abaf` | GICP 质量 / 退化门源码 |
| T | origin/experiment/tdt-planner-phase2 @ `e374894a7a713ea84b82de9e92479a37041f39ff` | T-DT 本地化规划及更晚的高转速定位修正 / 恢复 |
| F | experiment/temporal-mpc-main-20261004 @ `04291a410f193c009e043af88e014cf420e1f68b` | 冻结 Temporal MPC；读取实际代码及历史文档 |
| R | experiment/r4-hws-prediction-consumption @ `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 最新 R4 Stop / 冻结记录，不继续实现 |

“项目有能力 X”只有注明 ref 才成立；“本车正在使用 X”还须 launch、实际参数回读、ROS graph 和新运行证据。本轮没有运行时核验。没有在所查生产 / 对应实验源码中找到 Scan Context 全局恢复链，不扩展为“所有历史文件都不存在同名算法”的绝对结论。

## 2. 项目规范与本轮调整

| 权威来源 | 实际约束 |
| --- | --- |
| H 的 README、docs/development_workflow.md、docs/2027_architecture_decision.md | 保护可工作的主链；沿用 ROS2 包、命名、参数及开源引入规则；禁止无差别复制外部整套仓库 |
| M 的根 AGENTS.md、docs/research_integration_deployment_principles.md | 仓库级长期强制 Research → Integration → Deployment；先假设、baseline、最小可判定实验，再 Go / Modify / Stop |
| H 的 docs/project_structure_and_documentation_index.md | 冲突时依次采用公共 contracts、runtime profiles、phase 文档、包 README、日期验证报告；历史 2026 说明不能覆盖 2027 契约 |
| docs/contracts/tf_contract_2027.md、topic_contract_2027.md、chassis_contract_2027.md | TF / odometry / cmd_vel 唯一 owner；串口、底盘、规划和控制模块不得抢定位 TF |
| docs/runtime_profiles.md 与定位 phase2a/b/c/j/k 文档 | 显式 profile、默认关闭硬件 / 运动和候选入口；地图、传感器、launch 支持范围必须对应 |
| T 的 docs/branch_mainlines.md | old-car / new-car 部署主线有不同硬件约束；不得把新车修改直接混入老车。文内曾称 main 与 old-car 相同，属于历史快照，当前提交已不同 |
| F 的冻结记录、R 的最终 Stop、H 的动态研究历史总文档 | 未通过部署验收；历史下一步命令、期限和测试通过不能解冻或证明部署有效 |

当前 H 没有根 AGENTS.md 或新的分级原则文件；它们存在于 M。本轮读取 M 的确切 Git 版本并遵循其长期原则，不将其他分支的文件复制进 H。未发现当前作用域另有 CLAUDE.md / CONTRIBUTING.md / 多级 AGENTS 指令。[E1]

本轮允许修改范围仅为本报告及文档索引。工作区原有动态观测成员文档、tracker 源码和未跟踪工具 / 测试不属于本任务，保持原样；不暂存、提交、切换分支或修改 main。下一轮代码研究应从明确选定的硬件 / 源码基点建立隔离实验分支，不能直接把 H 当作最新定位基点。

对阶段建议的调整：

1. L2 与 L3 合成一个离线可判定核心；仅召回率提高不能证明重定位有价值。
2. L4 的正式接口 / 提交 / 运动授权工程化以离线 Go 为前提。Research 首版只输出候选和判定，不接 TF / 底盘。
3. C1 + C2 + C3 在 C0 精确复现之前共同确认；原模型和参考未知时直接“复现 TinyMPC”没有可检验含义。
4. 实际修改 TF、localization、最终 cmd_vel 或进入实车高速运行时，按项目规则提升验证等级；这不要求离线首版先跑全仓回归。
5. 本轮正式长报告是用户明确要求的系统阶段审计；下一轮 Research 记录应简短，聚焦效果和限制，不以测试 / 文档数量代替进展。

## 3. 东北大学方案忠实复原

### 3.1 定位总体结构

分享明确将先验地图放到后端，作为测量约束；前端自行维护局部点云并输出里程计。后端优化每帧 SE(3) 位姿，含里程计增量、scan-to-map、UWB 及回环约束，并处理地图坐标与 UWB 坐标的对齐。[L§3、6]

~~~mermaid
flowchart LR
  S["LiDAR + IMU"] --> LIO["逐点紧耦合前端<br/>自维护局部地图"]
  LIO --> O["里程计增量 / 局部位姿"]
  O --> FG["SE(3) 因子图后端"]
  P["先验点云地图"] --> REG["scan-to-map 匹配与信息"]
  LIO --> REG
  REG --> FG
  U["UWB 位置与坐标对齐变量"] --> FG
  LC["回环约束"] --> FG
  REG --> HEALTH["可信性判断 / 异常恢复"]
  HEALTH --> RESET["前端发散时重置滤波器"]
  RESET --> LIO
  INIT["UWB 位置 + yaw 搜索<br/>或 Scan Context 召回 + 粗 yaw + 匹配"] --> FG
  FG --> GLOBAL["全局定位关系 / map-to-odom"]
~~~

这是概念数据流，分享没有披露具体 topic、frame 命名、TF publisher、ROS1 / ROS2、求解库、线程所有权或消息接口。不能从图推导其符合本项目 TF 契约。回环如何生成、因子边缘化和信息标定也未给出。

前端逐点更新、自适应点云累积窗口根据传播 / 更新前后协方差的信息变化，在延迟与估计精度间权衡；地图使用校订稿所称 iVox / 哈希加类八叉树、多尺度平面残差及点 / 面不确定性的概率建模。[L§4–5] 这些不是普通 FAST-LIO 参数开关。分享没有足够公式复现其窗口策略、噪声模型或多尺度融合。

后端把里程计因子用于保持增量连续性；scan-to-map 的 Hessian / 信息用于描述各维约束；UWB 给予较大协方差，在 LiDAR 好时权重较低、遮挡 / 狗洞有效点少时补充位置观测。演讲者称他们测试的 UWB 较准，同时明确融合 UWB 可能降低建图精度，纯建图更推荐里程计加回环后校正原点。[L§6] 这不证明我们裁判链路 UWB 也有该精度。

### 3.2 初始化、异常判断与重定位

| 场景 | 分享明确做法 | 未披露 / 不应补全 |
| --- | --- | --- |
| 有 UWB 初始化 / 恢复 | 用位置作初始信息，yaw 不可信，搜索 yaw 后匹配 | yaw 步长、搜索半径、精配准算法名、候选排序 / 拒绝阈值 |
| 无 UWB | Scan Context 召回关键帧、粗估 yaw、再匹配 | Top-K 数量、数据库构造、全库查询范围、多帧提交 |
| 配准变差 | 用 Hessian 秩 / 特征值区分观测约束不足与可能前端发散 | 严格故障判定公式、统计校准、对称多解的处理保证 |
| 观测与前后状态不一致 | 使用转录中的 “as” 统计判断是否纳入因子图 | “as” 的全称 / 公式，不能写成已使用 NIS 或 NEES |
| 前端漂移 | 正常外部约束逐渐校正 | 校正限幅、具体延迟 |
| 前端发散 | 重置前端滤波器，以当前处为局部原点，再求 map-to-odom | reset API、缓存清理、odom epoch、控制暂停 / 重新授权 |

[L§7–8] 的合理启发是“有信息”与“状态一致”要分别验证。满秩且残差很差也不能单独证明前端必然发散：地图变化、时间错位、外参错误、错误 basin 都可能导致此结果。这是本次对其经验规则的适用性判断。

演讲者声称 Scan Context 在其赛场和合适参数下召回可达 100%；没有样本分母 / 留出数据，不能当作我们 MID360、云台姿态和对称赛场的保证。对称特征增强、第二描述子重排明确说“最后没有用上”；应保留为未采用的探索，不能复原成其实际成功闭环。关于 UWB 运动后 yaw / 对齐变得可观测是特定模型讨论，不构成 LOST 后自主移动的安全依据。

### 3.3 导航与控制链

分享的数据流为：多源感知 → 前后台异步障碍地图 → A* / ESDF 代价搜索 → Douglas–Peucker 化简 → 所称 Minimum Snap QP、方形区域约束和迭代碰撞修正 → MPC 运动控制。[N§1–7]

感知包含 360° LiDAR、雷达站处理后的机器人位置、深度相机、自瞄视觉。雷达站位置在此用于感知，转录没有证明它一定是“本机全局定位源”。前后台异步地图用于降低等待地图维护的规划延迟；本轮只记录边界，不研究新的感知 / 动态预测系统。

后端放宽严格控制点为方形区域约束；采样轨迹发现碰撞后插控制点，或按转录所说减少段分配时间。声称约六次迭代、十万次测试求解稳定性 100%、平均 5 ms。这是其测试口述，不等于连续时间无碰撞证书、完整任务成功率或本机求解性能。

控制部分仅明确：全向底盘各方向建模近似相同，简化为线性 MPC，用 TinyMPC；其他所提构型用 IPOPT 优化。未给状态矩阵、输入是速度还是力 / 加速度、周期、horizon、权重、前馈、heading、轮速、加速度 / jerk、terminal 或 corridor 在 MPC 中的定义。也未披露 planner→controller 消息是否保留多项式及时间。因此，原方案存在轨迹生成与控制的**潜在耦合**，尚未证明整套搜索算法必须跟 MPC 一起搬迁。

### 3.4 必须保留的转录歧义

| 原文 / 校订词 | 本轮处理 |
| --- | --- |
| “6系列里程计”“potato”“IQ外推” | [转录可能有误] 不指定为 FAST-LIO、Point-LIO 或任何具体实现 |
| “pv6”“as”“碘伏匹配”“唇釉RV” | [转录可能有误] 不补成论文、NIS / NEES、ICP / GICP / NDT 或建图库 |
| UWB 与地图对齐的“纵向变量”等描述 | [转录可能有误] 只确认有额外对齐变量，维度 / 参数化未知 |
| iVox 名称 | 校订表仍保留不确定性；结构思路明确，具体代码未知 |
| “脉轮”“串腿” | [转录可能有误] 不擅自改成麦克纳姆 / 腿式，也不由此推出某构型必然非线性 |
| “方形而非矩形避免多解” | [转录可能有误] 无公式，不作 QP 唯一性结论 |
| “降低这段路径分配的时间” | [转录可能有误] 保留原意；时间缩短可能提高导数 / 执行需求，不能自动认作避碰有效规则 |
| 决策部分 | 转录在开头结束，不补写、不纳入研究 |

## 4. 当前项目定位审计

### 4.1 FAST-LIO、传感器与 TF

2027 公共链为 `map → odom → base_link → gimbal_yaw_link → 传感器`。`lio_adapter` 是 canonical `odom→base_link` / `/odometry/lio` owner；外部位姿模式的 `map_odom_from_global_pose` 是唯一动态 `map→odom` owner。上游 LIO 私有 TF 隔离，backend 的 body / lio_imu_link 不能直接重命名为 base_link。[E2]

选定 MID360 与其内部 IMU 的刚性关系用于主 LIO；底盘 IMU 不能在云台转动时直接当成雷达固连 IMU。云台角与点云 / IMU 源时刻对齐、测量外参、传感器到基座补偿都属于已有 adapter 边界。双 MID360 障碍融合不等于双雷达 LIO。新车倾斜安装 / 变形 / 狗洞配置仍需对应硬件确认，不能用老车通过记录替代。[E2、E7]

因此第一轮应使用 adapter 输出及已有点云处理，不能为了描述子方便旁路云台时序。主 LIO 健康与全局定位健康要分开：AMCL 失效时健康局部 odom 可继续供停车和配准；LIO 自身发散时，增加一个 map→odom 修正不会使错误运动估计恢复可信。

### 4.2 GICP 到底已经具备什么

| 能力 | H / M | O / G / T 中可见的增强 | 本次判断 |
| --- | --- | --- | --- |
| prior PCD 精配准 | PCL GICP；odom 系点云对 map 系 PCD | 同类后端，输入点云 topic / 基准随 profile 有差异 | A：复用，不再迁移另一份 GICP |
| seed | `/initialpose` 转为 map→odom 初值 | 仍以已有 guess / seed 起步 | 不是无 seed 全局定位 |
| coarse / place / 多 basin 搜索 | 未发现 | 所查对应源码仍未形成 SC→多候选闭环 | 真实缺口 |
| 基础拒绝 | 收敛、有效点数、fitness、相对 guess 位移 / yaw | 保留上述门 | 已有，但只能保护局部 basin |
| 几何质量 | 无完整 quality 实现 | overlap、法向点面近似 Hessian 特征值 / condition | A：分支内存在；阈值未等于实车标定 |
| 连续帧全局候选确认 | 单次通过即发布 raw pose | T 的桥有局部恢复一致性计数 | 不是全局多候选独立帧确认 |
| TF | 不发布 TF；发布 raw pose / valid / score / map_id | 仍由桥发布 TF | A：保持唯一 owner |
| 首次错误配准隔离 | 基础门可拒绝；单次通过可进入桥 | T 对已有基线加强；首次基线 / 手动 seed 仍需额外验证 | 仍有污染 active correction 风险 |

H 的默认示例：配准 2 Hz，地图 / source voxel 0.25 / 0.20 m，最大 correspondence 1 m，30 iterations，最低 100 点，fitness≤0.50，guess 邻域跳变≤2 m / 1 rad。它们来自当前 YAML / 源码，不是本报告推荐全局恢复阈值。[E3]

已有 GICP 按新 source generation 配准，基础输出 covariance 由 fitness clamp 到 0.01–0.5 构造，xy / yaw 等量、z / roll / pitch 放大；这是启发式数值，**不是校准后验协方差**。不要把它直接用作因子图信息权重或统计创新门的真实噪声。

O / G / T 的质量实现以目标法向、最近邻、有效 correspondence 计算 source 一侧 overlap；其近似 H 为平均 `JᵀJ`，J 含法向及以点集中心为原点的转动项。它不是 GICP 求解器完整的 covariance-weighted Hessian；overlap 也不是唯一双向覆盖率。特征值 / condition 随尺度、米与弧度参数化和地图几何变化。默认 overlap≥0.30、min eigenvalue≥0.001、condition≤1e5，以及 toy fixture 的通过值，都不构成赛场泛化证据。[E3]

PCL 的 fitness 是最近邻距离统计，不是正确匹配概率；版本 / cutoff 语义要以实际安装 PCL 为准。[PCL 官方实现](https://pointclouds.org/documentation/registration_8hpp_source.html) 支持这一指标含义，不支持任意通用接受阈值。

### 4.3 global pose gate 与 map→odom 桥

基础 `global_pose_gate` 检查 frame、时间、新鲜度、finite、四元数、平面约束及 xy / yaw covariance 上界；检查所有 covariance 元素有限不等于验证完整正定性，更不等于地图匹配正确。门可拒绝 malformed / stale，但对称场地中合法而错误的位置仍可通过。[E4]

H / O 的基础桥按全局测量时刻在缓存中匹配**最近** odom（默认 0.05 s 容差），不是已实现精确插值。它有 age、bounded pending、时间非单调失效和 reset 边界，但没有一般的全局候选事务或独立 odom epoch。接受匹配后直接计算修正。高转速下时间容差重要：7.35 rad/s × 0.05 s ≈ 0.37 rad；这是量级推算，不是桥当前实测误差。不能给旧测量改成 now 来绕过时间门。

T 中更晚的桥 / 老车配置已有：

- 常规创新门（generic 默认关闭），老车对连续三次拒绝才锁定失效，前两次保留可信修正。
- 高角速度 hold：老车在约 3 rad/s 以上且 LIO 健康时保持最后可信修正，暂不消费 AMCL 更新。
- 显式 opt-in 自动恢复：等待停车、odom 稳定窗、hold / settle、reseed cooldown、最多尝试次数和若干一致性确认。
- bounded rebase 分支：围绕最后可信 XY / settled LIO yaw 的限定局部重基准；不是跨场地无 seed kidnapped 搜索。
- `/localization/correction_recovery_state` 输出健康、hold、待停、reseed、一致性确认和 exhausted 等原因。[E4–E5]

generic 与 old-car 参数不同；回调计数也不能自动解释为独立点云帧的证据。首个基线接受和人工 initialpose 是需要重点补门的入口。桥 reset 不等于 FAST-LIO 滤波器 reset。

`Bool valid` 本身不是带时间、epoch 和理由的完整健康合同；上游 valid 禁用时，旧修正可能持续广播。**停止 TF 发布也不保证车辆即时停车**：TF buffer 仍有缓存，控制链可能继续使用旧状态。安全提交必须与最终控制许可联动，不能将“TF 断了”当成停车证明。

### 4.4 navigation_integrity 与恢复 owner

I / O 的 navigation_integrity 是 shadow 诊断模块，输出 GOOD / SUSPECT / REJECT / UNKNOWN，不写 TF、pose、goal 或 cmd_vel。它使用扫描端点对 2D 地图 distance field 的一致性、pose / odom 时刻、jump / rate / freshness；已有规则在证据齐全且 hard jump 与 hard scan 同时成立时才 REJECT，单项异常多为 SUSPECT，缺证据为 UNKNOWN。[E6]

| 对比维度 | 当前 integrity | 东北大学后端状态管理 | 推荐职责 |
| --- | --- | --- | --- |
| 语义 | 观测是否可疑 / 被拒绝 / 无证据 | 漂移、退化、前端发散与因子纳入 / reset | 诊断与运行阶段分开 |
| owner | 只读 shadow monitor | 位姿估计后端内部 | 复用 monitor；恢复协调与提交由唯一 localization owner 管 |
| 输入 | 2D map、scan、pose、odom、TF 时间证据 | 前端状态、匹配 Hessian、一致性、多源因子 | 不用 monitor 输出冒充因子图后验 |
| 输出权限 | 无运动 / TF 权限 | 涉及滤波器重置、全局关系 | 正式桥与既有最终控制 owner 合作，不新增第二 TF / cmd owner |

因此建议先复用其纯评价逻辑与诊断数据，通过明确 adapter 提供恢复证据；不直接将 GOOD / SUSPECT / REJECT 改名成完整 recovery 状态机，也不把整个因子图压进该 Python monitor。具体代码落点在 L4 Integration 决定，首轮离线不新建生产包。

### 4.5 历史高转速结论必须更新到正确版本

7 月记录确有高转速、单次约 0.84 rad 更新及 recovery catastrophic failure；Omni alpha4 旋转噪声与扫描失真是重要原因。调小 alpha4 与 strict SE(3) deskew 的组合已有单 bag / 多 seed 改善，但不能证明任意恢复可靠。[E5]

T 的 9 月 3 日实车记录进一步发现原链还发生 LIO 点云处理 backlog；保留 native CustomMsg 时间基准和 LIO age / backlog 门后改善。之后用三次拒绝确认、高转速 hold 和停车后恢复避免瞬态 AMCL 直接污染修正。记录包含约 52.75 s 的高转速段、峰值 7.273 rad/s，期间 validity 保持 true；这是指定老车 / profile 的有界证据。

所以不能将 7 月失败率当作所有当前版本仍有相同故障的证据，也不能把 9 月 hold 成功升级为无 seed 全局恢复成功。下一轮优先复用这组故障归因和保护，不重复同一 bag 的大量验证；仍需解决 hold 后不确定全局位置如何正确重建。

### 4.6 地图与外部位置

有效三维恢复需 `occupancy_with_pcd` 等有一致坐标 / origin / yaw / revision 的地图 bundle。occupancy-only fresh03 候选不能自动作为已批准 PCD 地图。Scan Context 库还需关键帧位置、局部 submap、预处理和传感器基准；对整幅合并 PCD 做一个描述子不足以支持地点召回。[E7]

所查接口未发现已接通且获验收的“雷达站 / UWB 本机全局位置”producer。V 的 OperatorNavigationTarget 或目标 track 是导航目标 / 目标位置，不是 ego pose；现有 referee 状态也不能被当成 UWB 位置。缺 producer 不妨碍离线评估先验价值，但禁止伪造真实可用率。[E7]

## 5. 控制与规划接口审计

### 5.1 正式配置、实验规划器和底盘链

H / O 所查配置的 GridBased 仍为 NavfnPlanner，MPPI 为 Omni。例：old-car dual STVL profile 10 Hz、dt=0.1 s、30 steps（3 s）、batch=300、vx / vy≤0.5 m/s、wz≤1.2 rad/s；显式 spin test 速度上限明显更高。不同配置不能混为统一控制能力 / 上限。[E8]

已本地化 T-DT 规划器在 T 的 `experiments/tdt_planner/rm_tdt_planner`，有 COLCON_IGNORE、显式实验入口及大量研究记录。**“已经本地化”成立；“当前 H / 所有比赛默认都使用它”不成立。** 本轮不重新研究搜索，只分析其保留 / 丢失的数据。[E9]

cmd_vel 路径依 profile 经 controller_server、velocity smoother、可能的实验 guard、最终底盘 / 串口 owner。串口 transport 有限幅及约 0.2 s command watchdog，stub 约 0.5 s，但所查源码没有因此自动消费 localization valid。Temporal MPC 的共同 execution guard 属于明确实验链；不能据此认定每个正式配置都已有同一停车门。[E8、E10]

用户要求的 STVL + MPPI + 近场减速 / 停车应作为保持的基线。进入 L4 / C4 时须查清**选中实际 profile**的唯一最终 owner 和可用停车许可；若该链缺定位 inhibit，要在集成阶段扩展既有 owner 的合同，不能临时加第二个 cmd publisher。

### 5.2 不能忽略 controller odom 输入

部分 H YAML 的 `odom_topic: /odometry/lio` 属于 bt_navigator / smoother，controller_server 内没有显式设置同参数。已有实际 Humble 实验确认默认 /odom 与 canonical 输入不一致会让 MPPI 使用错误 / 零速度；后来显式实验 profile 与 F 的 dual-controller 配置做了修复。[E8、E10]

这是一项**已有发现，应复用**。不能说所有正式配置都已修复，也不能只看 YAML 出现相同字符串就认定 controller 已接通。下一次 A/B 前回读 controller 参数和订阅、确认 canonical publisher / freshness；不能靠新增 /odom 别名掩盖 owner 问题。基线输入错误会使 MPC 比较失真。

### 5.3 T-DT 规划输出保留了什么

| 层次 | 实际数据 | 对控制器的影响 |
| --- | --- | --- |
| upstream MinimumSnapOsqp Output | 采样 path、corridor、iteration、success、time | time 是求解耗时，不是每个轨迹点执行时刻 |
| 本地 planner Result | 二维 path、success、optimized、reason、elapsed | 没有输出多项式系数、段时长、导数或完整 corridor |
| Nav2 createPlan | nav_msgs/Path；Pose header 统一生成时刻，内部切向 heading / 首末姿态处理 | stamp 不代表轨迹时间；没有 desired velocity、curvature、terminal dynamics |
| 实验 heading 连续策略 | 显式默认关闭选项 | 不能把几何切线默认当成四全向轮底盘朝向合同 |
| F 的 StaticRoute frontend | 静态 path / center region、局部参考时序、plan ID / epoch | 是冻结研究中的特殊构造，语义不等同原控制点方形区 |

T 源码内部确有段时间和初始速度参与优化，但未通过当前公开结果传给 controller。当前默认 order=6 / maxdx=3 对应五次多项式、三阶导目标，更接近 minimum jerk；“MinimumSnap”类名与分享标签不能证明当前默认四阶 snap 目标。[E9；固定版本上游](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit/tree/a4bddc467f84d2418a4de06c65c8c79403eb4a13)

控制点方形约束也不是对整条连续轨迹及实体机器人 footprint 的完整安全走廊认证。若 MPC 需要 corridor，须确认它约束的是控制点、轨迹中心、whole body 还是采样点，不能把同名数组直接当安全证书。

**Path→TimedTrajectory adapter 可以定义新的时间分配 / 速度曲线，但不能无损重建已丢失的信息。** 可选择：只用几何路径并明确自建参考；或在现有优化后端加可选 rich output，保留段时间、系数 / 可评估导数、最终有效 corridor 和终端定义。是否需要后一种由控制假设 / 原方案资料决定，不重新迁移搜索算法。

### 5.4 Temporal MPC 的可复用范围

F 已实现 bounded QP、warm start、固定 horizon / 20 Hz 研究节拍、Nav2 双插件、plan generation / epoch、源时刻状态请求、worker deadline、proposal、重锚重验、受限减速 / MPPI fallback 和共同执行保护。插件本身不发布最终 cmd，controller_server / guard 保留 owner。[E10]

但其模型域是近似固定 yaw 的全向平移：yaw 偏差约≤0.001、wz 约≤1e-6；15 nodes、1.5 s horizon、50 ms ZOH、约 15 ms solver / 40 ms worker 预算是那个实验的约束。它不是覆盖任意 yaw / 自旋 / 四轮耦合的东北大学控制器。

可选择性复用接口知识、时间 / epoch 规则、插件骨架和观测工具；**不复用被冻结动态预测问题的接受结论、障碍模型和固定 yaw 假设**。F 的任务停滞 / 净空 / 时序失败不能推出所有 MPC 都无效，也不能因大量测试通过就把其低层原样上线。R 最新 Stop 同样维持：限定五对试次中候选没有形成稳定完整任务收益，不恢复该路线。[E11]

## 6. A / B / C / D 差距矩阵

A：项目某指定版本已有，应复用；B：边界明确、值得独立借鉴 / 研究；C：需作为语义完整簇评估；D：当前不建议迁移。C 表示值得联合验证，**不是已经证明强耦合必要或已授权迁移**。

| 模块 / 模块簇 | 东北大学依据 | 当前项目 / 差距 | 耦合度 | 分类与建议 |
| --- | --- | --- | --- | --- |
| 局部 LiDAR-IMU odometry | 自维护局部地图 [L§3] | FAST-LIO + adapters 已有 [E2] | 传感器时序高 | A；保持 |
| 逐点自适应窗口前端 | 信息变化调窗口 [L§4] | 没有等同实现证据，当前无需重写 LIO | 前端噪声 / 时间高 | D 当前；仅条件 C 研究 |
| 多尺度概率平面地图 | iVox / octree [L§5] | GICP map / normal 不是该表达 | 地图、残差、信息高 | D 当前；不能孤立换容器 |
| Scan Context 地点召回 | 无 UWB 初始化 [L§8] | 所查链缺无 seed 召回 | 单独组件中 | B 离线；接受属于 C 恢复闭环 |
| 多候选 / yaw 搜索 | 粗 yaw、UWB yaw 搜索 [L§7–8] | 已有 guess 不能覆盖多 basin；Top-K 是本次建议 | 与 fine / 歧义高 | C 最小恢复簇 |
| GICP 精配准 | 原文精算法不明 | 项目 seeded GICP 已有 [E3] | 局部 basin 中 | A；不另迁移 |
| overlap / fitness 基础门 | 原文泛称匹配质量 | 分支已有 [E3] | 单指标低 | A；先校验适用性 |
| Hessian 退化判断 | 秩 / 特征值 [L§6–7] | 近似 H gate 已有，非因子信息 | 信息模型高 | A gate；B 借鉴解释，不能冒充原 H |
| 观测创新一致性 | “as” 不明确 [L§7] | jump / innovation 和 shadow 已有 | 状态 / 时刻高 | A 部分；C 候选验证，不复制未知统计 |
| 全局确认 / 安全提交 | 恢复全局关系 [L§7] | 通用多候选多帧 / 原子提交仍缺 | epoch / owner 高 | C 优先 |
| 原 SE(3) 多源后端 | odom / map / UWB / loop [L§3、6] | 现有门不是因子图 | 数学高 | 条件 C；本轮不实施 |
| frontend divergence reset | 发散重置 [L§6–7] | 桥 reset 不等于 LIO reset | estimator / epoch 高 | C 条件；先证明故障、不能默认 reset |
| map→odom TF 管理 | 原文仅概念关系 | 已有唯一桥 [E4] | 系统权限高 | A owner；C 扩展受控提交 |
| external prior adapter | UWB 位置 [L§6–8] | 缺已验证本机 producer [E7] | 输入边界中 | B 可独立验证；不直接融合 |
| integrity monitor | 后端小可信状态机 | 现有 shadow 诊断无权限 [E6] | 证据可拆，执行高 | A 监测；C 恢复协调 |
| 对称增强第二描述子 | 明确最后未用 [L§8] | 先缺基本全局召回 | 不确定 | D 首轮；真实多解保留 LOST |
| 无图探索 / 决策 | 展望 / 内容截断 | 不属本任务 | 不评估 | D |
| A* / DP / QP / 方形区 | [N§3–6] | T 已本地化 [E9] | 输出可能与控制高 | A；只补必要接口 |
| 线性 MPC / TinyMPC | 全向近似线性 [N§7] | MPPI baseline；F QP 仅受限模型 | model / reference 高 | C 静态研究候选 |
| 独立 TinyMPC solver | 求解器名字明确 | 不是控制器完整定义 | 计算组件低 | B 评估库；D 单换 solver 当收益 |
| IPOPT / 特定构型 | 构型词不明 [N§7] | 我方四全向轮未证明需要 NLP | 未知 | D 首轮 |
| Timed reference / feedforward | 分享未披露接口 | Path 丢失时域、导数 [E9] | 时域 / 导数高 | C；B adapter 仅在新参考假设成立时 |
| terminal / heading / wheel constraints | 分享未披露 | F 固定 yaw，不能覆盖自旋 | chassis / execution 高 | C 候选必须定义 |
| MPC 工程接入 | 分享未披露 ROS 接入 | F 插件 / 双控制器 / deadline 已有 [E10] | 生命周期中高 | A 选择性复用，冻结本体保持 |
| slowdown / stop / fallback | 分享没有足够内容 | 既有链需按 profile 查 owner [E8] | 实际安全高 | A 保持；C 验证恢复许可 |
| 动态障碍预测再加入 | 不是静态收益前提 | 多轮 frozen / Stop [E11] | 不纳入 | D |

## 7. 高耦合模块簇：必要边界与回退

“高耦合”须由共享数学 / 状态 / 时间 / 约束说明。可以分进程、通过足够丰富且有版本的接口传递；**普通 Pose / Path / Bool 丢失语义**才是问题，不是 topic 或 plugin 本身不能使用。

### 7.1 C-L：全局候选验证到恢复授权

| 必须回答的问题 | 审计判断 |
| --- | --- |
| 为什么耦合 | 候选只能在生成它的地图版本、点云时刻、odom epoch 和搜索 basin 内解释；质量、跨帧跟踪及接受决定必须针对同一假设 |
| 算法耦合还是代码耦合 | 候选相关性、多解拒绝是算法耦合；TF / 运动授权是系统正确性耦合。无需复制东北大学整套前端 |
| 普通接口丢失什么共享状态 | candidate ID、runner-up、map revision、source scan ID / time、odom epoch、先验来源 / age、变换线性化基准和门理由；普通 initialpose 丢失这些 |
| 强行拆开损失什么 | 单个低 fitness 假候选可能被默认广播；重复回调可能被当多帧；odom reset 后旧候选可能仍被提交；不同节点各自“通过”无法组合成接受证据 |
| 最小完整集合 | 关键帧库 / coarse generator、seeded fine registration、候选质量 / 歧义验证、独立帧确认、recovery 生命周期、单一 commit / resume 协调；优先复用已有部件 |
| Nav2 是否容纳 | 可以，放在 Nav2 外部定位层；Nav2 消费 canonical TF。原桥保留 TF owner，最终控制门保留 cmd owner |
| 是否值得调整接口 | 是，Research 先用结构化离线记录；Go 后再增加明确 candidate / commit / permit 合同。不能把所有候选写到 active raw pose topic |
| 失败如何回退 | 研究禁用后正常基线继续；若基线全局定位已 LOST，则停车 / 人工 seed 后验证。不能把已失效 AMCL 当恢复期间可直接运动的 fallback |

这组 C 是本项目必须完整验证的恢复语义，分享只支持其中若干原则，**不声称东北大学已实现此处所有事务 / 多帧机制**。

### 7.2 条件 C-FG：原多源因子图估计闭环

| 问题 | 审计判断 |
| --- | --- |
| 为什么耦合 | SE(3) 状态、odom 增量、scan-to-map 信息、UWB-map 对齐变量、回环与异常纳入共同决定后验 |
| 耦合层次 | 数学 / 估计状态耦合，不能用现有 covariance heuristic 假装等同因子信息 |
| 共享状态 | 因子 residual / Jacobian / noise、linearization、frame / time、对齐变量、图窗口 / 边缘化先验、reset 后 epoch；公式多数未披露 |
| 拆开损失 | 只输出 pose 再与 UWB 二次融合可能重复计算相关信息；忽略对齐变量引入系统偏差；独立 reset 破坏连续因子关系 |
| 最小完整集合 | 合适的前端增量 / 不确定性接口、scan-to-map residual + 信息、SE(3) 图后端、外部源对齐与鲁棒质量纳入、epoch/reset 协议；UWB / loop 是否必需由对应假设决定 |
| 当前架构能否容纳 | 可以作为隔离 backend，输出受验证的全局结果给唯一桥；不能自行新增 canonical TF。现有 FAST-LIO 是否供足够信息尚待确认 |
| 是否值得改接口 | 只有先证实外部位置真实可用或当前恢复簇无法处理关键退化，且获得模型 / 源码后才值得 |
| 失败如何回退 | 关闭候选 backend，恢复选定 AMCL / GICP profile；原图状态不混入 baseline。失效定位保持停车 |

完整因子图可能优于单次配准门，当前缺模型、外部源标定和效果证据，因此不作为首轮。不能因后端叫“因子图”就自动要求换掉 FAST-LIO；也不能未经验证把其原前端排除为一定可替代。

### 7.3 条件 C-LIO：自适应累积与多尺度概率前端

| 问题 | 审计判断 |
| --- | --- |
| 为什么耦合 | 点时间窗改变 IMU 传播、deskew、观测密度和协方差；多尺度平面概率直接参与残差及信息 |
| 耦合层次 | 算法 / 噪声模型耦合，不是更换 hash / octree 容器这一代码动作 |
| 共享状态 | 每点时间、传播 / 更新协方差、面统计、点不确定性、窗口准则、filter reset epoch |
| 拆开损失 | 独立调点数但不重算不确定性，或仅复制地图容器，都不能复现其精度 / 延迟权衡 |
| 最小完整集合 | 前端估计更新、窗口准则、相应 map / residual / noise，及完整 sensor-time adapter 边界 |
| 当前架构能否容纳 | 可作为 quarantine 下的实验 LIO backend，由 canonical adapter 接出；不是本轮实现 |
| 是否值得调整接口 | 当前没有缺口证据足以证明收益；先排除云台时序 / 原始点时间错误，再考虑 |
| 失败如何回退 | 保留 FAST-LIO / adapter 的原配置；独立 epoch 重新初始化，不在运行中无缝混用两个局部坐标系 |

当前分类为 D；仅在 L1 证明主要问题是前端估计且有可复现资料时，将这一整簇作为条件 C。不会把它偷偷加入全局召回研究。

### 7.4 C-C：静态轨迹参考、模型与 MPC 执行

| 问题 | 审计判断 |
| --- | --- |
| 为什么耦合 | dt / phase、位置导数、heading、自旋、输入物理含义、速度 / 加速度 / 轮速边界及 terminal 状态必须对应同一个模型 |
| 耦合层次 | 模型与时域约束是算法耦合；deadline、plan epoch 和最终输出权限是执行正确性耦合。分享未证明搜索算法与 solver 必须整体迁移 |
| 共享状态 | 段时间 / coefficients、reference phase、body↔world yaw、v / a / wz、terminal、map / plan revision、执行内环模型、warm start / previous command |
| 拆开损失 | Path 丢导数，按切线补 yaw 错失横移 / 自旋；solver 以为输出加速度而底盘消费速度；重规划后旧 reference / proposal 仍执行 |
| 最小完整集合 | 已有静态几何 / 可选 corridor、明确 timed reference frontend、四全向轮适用模型、MPC、限制 / 下发语义、周期与 epoch 校验；不含动态预测、不重做搜索 |
| Nav2 是否容纳 | 可以用 controller plugin 或完整候选控制链；保留上层目标接口、唯一 cmd owner 和独立 baseline。不得为了只接 Path 丢失必要 rich output |
| 是否值得调整接口 | 条件成立时值得为现有轨迹后端增加可选丰富输出；若几何 + 新 timing 足以验证假设，先用明确有损 / 新定义的 adapter |
| 失败如何回退 | Research 在隔离环境停止；Integration 仅在定位、地图、命令权限和 MPPI 输入都可信时回 MPPI，否则停车。不回退到超时或不可信旧提案 |

这组是**建议的研究边界**；原 NEU 的确切轨迹 / MPC 契约未知，不能写成已证实“必须迁移整个 Search→Minimum Snap→MPC”。

## 8. 推荐定位结构与运行语义

### 8.1 目标结构

~~~mermaid
flowchart TD
  LIO["健康 FAST-LIO / canonical odom<br/>源时刻云台补偿"] --> OBS["静态可用观测 / map revision / odom epoch"]
  DB["关键帧 + submap / 描述子库"] --> COARSE["Scan Context 或同类候选召回"]
  PRIOR["可选雷达 / UWB 粗先验<br/>无先验时仍能运行"] --> COARSE
  OBS --> COARSE
  COARSE --> FINE["多个 coarse basin / yaw<br/>复用 GICP 精配准"]
  FINE --> GATE["残差 / overlap / 可观测性<br/>歧义 / 时序 / odom 一致性"]
  GATE --> CONFIRM["独立帧、同一候选假设确认"]
  CONFIRM --> COMMIT["唯一定位提交协调<br/>map / epoch / age 再核验"]
  COMMIT --> TF["既有桥：map→odom"]
  COMMIT --> PERMIT["既有最终控制门<br/>停止 / 恢复导航许可"]
  MON["复用 integrity + LIO 健康证据"] --> STATE["恢复运行阶段"]
  STATE --> COARSE
  STATE --> PERMIT
~~~

图为本项目建议；所有新组件第一轮离线，不发布 active pose / TF / cmd。正常 AMCL / GICP 如何共存取决于选定 profile：不能让两个全局源未经仲裁同时驱动桥。

### 8.2 初始定位与运行中恢复分开

| 要求 | Initial Localization | Runtime Relocalization |
| --- | --- | --- |
| 入口 | 开机、换图、人工 seed；默认尚无运动许可 | 明确全局 LOST 或 odom epoch 变化 |
| 搜索 | 可以慢，全库 / 多 yaw / Top-K；有粗 prior 可加速 | 先停车，再有限预算搜索；不能只沿失效旧 guess |
| 确认 | 多个独立 scan，正确性优先 | 独立 scan + 当前 odom / stop / map 一致性；旧导航不继续 |
| 接受 | 人工 seed 也只作为候选，不绕过质量 | 大 correction 仅在 recovery 通道、停稳并验证后提交 |
| 失败 | 保持无许可，提示人工 / 补地图 | 保持 LOST，冷却 / 次数预算后人工介入 |
| 时间目标 | 秒级甚至更久可接受 | 记录耗时，但错误接受优先判失败 |

地图对称且传感器视域无法区分时，不存在只靠调阈值一定解决的几何方案。保留多个 basin；runner-up 足够接近时拒绝或请求可信外部 / 人工信息。停车后反复观察同一完全对称景象不等于新增可辨识信息。

Scan Context 作者实现把检索与成对相似度 / coarse yaw 分开，适合独立离线召回组件；这不保证倾斜、云台转动的 MID360 输入天然适配。[Scan Context 作者仓库](https://github.com/gisbi-kim/scancontext_tro) 第一轮使用真实 MID360 点型 / 视域；必要时统一 descriptor 坐标基准和积累方式，不先实现演讲中未用的增强版。

### 8.3 必需质量证据：复用优先，避免指标堆积

| 证据 | 作用与最小实现 | 不能证明什么 |
| --- | --- | --- |
| 时间 / frame / map / epoch | 每个 candidate 绑定生成输入，任何不匹配拒绝；reuse 基础 pose / odom 门 | 数据合法不等于位置正确 |
| 收敛、residual / fitness、有效点数 | 先使用既有 GICP；保留失败原因 | 低残差不能排除对称错匹配 |
| overlap | 已有近似实现可作第一版；记录定义与动态遮挡比例 | source 一侧高覆盖不等于全局唯一性 |
| 几何可观测性 | 复用近似 H；按可通行任务关注 xy / yaw，保留 6DoF 诊断 | 满秩不是正确匹配；H 不是校准 posterior |
| 多 basin / runner-up | 竞争候选几何分离、分数差；不能直接平均对称位姿 | 分数差阈值未标定时无置信概率 |
| odom 增量一致性 | 每个 candidate 的 map pose 随健康 odom 预测；跟踪同一假设 | 多帧稳定也可能稳定在对称错解 |
| 外部 prior 一致性 | 仅可信、及时、frame 对齐时补充 / 限定区域 | 雷达 / UWB 共链路不算两份独立证据 |

Research 先选上述直接改变接受结论的少数证据；若 overlap / residual / H 已能区分某失败，不另建重复评分。若需要 NIS 类统计，必须先建立测量 / 预测噪声与相关性假设，不能用启发式 covariance 或未知 “as” 名称套公式。

正常 tracking 的 correction jump gate 保护连续更新；全局 recovery 每个 seed 有自己的局部 basin。将新全局候选与失效旧 map→odom 比较并强限 2 m / 1 rad，可能拒绝真正 kidnapped 恢复；直接放开 normal gate 又可能污染运行链。两种接受通道需要不同权限与状态，而不是改大同一个阈值。

6DoF 匹配可使用地图几何，但投向 Nav2 平面位姿须满足既有 planar 合同。斜坡 / 狗洞 / 构型变化若超出当前支持 profile，应作为场景限制或单独合同研究，不能将 roll / pitch 门一概关闭。

### 8.4 LOST 与 recovery 状态：由故障驱动

监测质量维度仍可为 GOOD / SUSPECT / REJECT / UNKNOWN；下表是运行阶段，不替代监测 enum。连续计数、hysteresis、hold 时长先离线确定，不能给出未经标定的通用秒数。

| 阶段 | 进入依据 | 输出 / 行为 | 离开条件 |
| --- | --- | --- | --- |
| GOOD | 全局修正和 LIO 均有新鲜可信证据 | 使用选定正常定位路径与既有控制许可 | 短暂异常进 DEGRADED；硬失效进 LOST |
| DEGRADED | 短暂遮挡 / 退化 / 孤立创新异常，LIO 健康、旧 correction 有限可保留 | 不接受可疑 correction；按已验证 profile 限速 / 停车；高自旋 hold 需有可信边界 | 足够新鲜好证据回 GOOD；过期 / 连续异常进 LOST |
| LOST | 持续全局不一致、证据长期缺失、LIO hard invalid / reset、epoch 改变等 | 撤销导航运动许可，既有 owner 执行经确认的停车行为；保留诊断 | 停稳且最低观测条件满足，进 RECOVERING |
| RECOVERING | 停车、有效地图及可用观测，开始候选搜索 | candidate 始终私有；验证、独立帧确认、预算 / cooldown | 通过 commit 后重新确认并回 GOOD；失败回 LOST |

UNKNOWN 不能默认为 GOOD；对已有稳定系统短暂缺证据可先 DEGRADED，但必须有限时，不能无限保持运动。若 LIO 硬失效，停车不能继续依赖它给出的错误速度做“精确闭环制动”；应使用现有底盘可执行的停止 / inhibit 与 watchdog 边界，并保留人工急停。

不要默认 LOST 后原地旋转扫描、沿路径低速探索或为 UWB 可观测性主动平移。只有先证明局部运动估计、近场感知和停车余量都可信，且另有明确受限测试，才可研究该动作；它不是第一版恢复所需。

**前端故障处理与全局故障处理不同。** AMCL / 全局匹配坏但 LIO 好：不重置 FAST-LIO。LIO 真发散：现有接口未证明能完成滤波器 reset；重启 / reset 必须建立新 epoch、清缓存、让所有旧候选失效，并重新确认局部 odom 健康。东北大学此能力不能通过一个 bridge reset 服务复现。

### 8.5 何时允许更新 map→odom

后续 Integration 的提交顺序建议：

1. 保存 active correction；候选只进入私有恢复通道，禁止查询阶段写 active raw pose / initialpose 链。
2. 对候选输入绑定 `map_revision、odom_epoch、candidate_id、source_scan_id、source_stamp`；同一 scan 重复计算不增加独立帧数。
3. 在同一 epoch 内沿健康 odom 增量跟踪候选；要求跨独立扫描一致，同时检查竞争 basin / prior。不能只要求连续 pose 很近。
4. 提交前再核验：地图未切换、epoch 未重置、源时刻可配对、观测仍新鲜、LIO 健康、停车成立、其他 global source 不会交叉覆盖。
5. 用源时刻对应的 `T_map_base(t) × inverse(T_odom_base(t))` 构造 correction；在候选验证期间只做有界重锚，不以 now 伪造旧测量。
6. 由原桥唯一提交新 correction，关联 commit ID / epoch。提交和授权应有一致协议：可以先更 TF 再放行，但任何中断都维持无许可，绝不能先放行后等待 TF。
7. 新鲜 TF、costmap / robot pose、旧计划 / controller reference 失效重建及当前 localization 证据均确认后，才恢复导航许可。全局跳变后不能立即复用旧局部轨迹。
8. 任何失败保持停车；不回滚至已知失效位姿然后继续导航。保存旧 correction 便于审计 / 人工恢复，不代表旧位置重新可信。

这里的“安全提交”是协议保证，不要求所有模块合并进一个进程。离线 Research 只验证接受 / 拒绝；实现上述 runtime 协议在离线 Go 之后。

### 8.6 雷达站与 UWB 的正确用法

先建明确“本机位置先验”的输入合同，最少含本机 ID / 队伍坐标约定、source time 与 receipt time、时钟 / 延迟不确定性、frame / 单位、covariance 或保守范围、sequence、source quality。UWB tag 到 base_link 的杆臂、地图与裁判坐标的旋转 / 镜像 / 原点都要实测；不能默认已经对齐。

- **雷达站**：及时、标定有效时作为粗区域 / seed / candidate re-ranking；没有精度证据时只软加权，不能覆盖当前 pose。
- **UWB**：用户预期米级，先作更宽位置区域；只提供 xy 不提供可信 yaw，候选仍搜索 yaw。
- **延迟**：有可信速度界时可按测量 age 扩张搜索范围；无界 / 时钟未知时不得硬排除其他候选。
- **冲突 / dropout**：明确来源不健康，退回无先验搜索或保持 LOST；不能选择性相信更支持当前候选的源。
- **相关性**：两种源走相同裁判链可能共用延迟、坐标 / ID 错误；不能视为两个独立投票。
- **在线融合**：本轮不推荐直接进因子图。先用录制 / mock 分档误差与延迟验证 prior 对召回及错误接受的影响；mock 不能证明真实链路可用率。

## 9. 控制器结论：先证明静态控制链价值

### 9.1 与 MPPI 的差别及潜在收益

MPPI 本身属于预测控制思路：在动力学 rollout 上采样控制，用 critics / 代价形成更新。TinyMPC 类候选通常解明确线性模型、二次目标和状态 / 输入约束的有限时域优化；**不是“MPPI 不预测、MPC 才预测”**。候选的模型 / 约束若错误，快速求解也会快速输出错误参考。[E8、E10；TinyMPC 官方 solver](https://tinympc.org/solver/solver/)

最可能的收益假设：明确 timed reference 及导数前馈能降低弯道 / 横移跟踪误差、速度震荡和终端姿态误差；显式约束和可复现优化有助于分析响应。这些是待测假设，没有 NEU 或本项目数据证明收益。MPPI 当前正常导航可工作，须用输入正确、速度限制相同的 baseline 比较。

最大风险是**参考、模型和真实底盘执行不一致**，其次是 heading / 自旋耦合、重新规划时间基准、期限和 fallback。只看 solver latency 或平滑曲线不能判断实车能否执行。

TinyMPC 官方标准模型是线性离散系统；非线性系统通常需在线性化点附近使用线性近似。[官方模型说明](https://tinympc.org/get-started/model/) 是否选择 TinyMPC、现有 QP 或其他 solver，先由模型与约束决定，不因分享点名就新增依赖。

### 9.2 四全向轮“线性”必须说清坐标与输入

以底盘速度输入为例，世界系运动满足：

~~~text
dx/dt = cos(ψ) vx_body - sin(ψ) vy_body
dy/dt = sin(ψ) vx_body + cos(ψ) vy_body
dψ/dt = wz
~~~

因此全向轮允许横移，不等于任意 yaw / 自旋下世界系状态为恒定 A/B 的线性系统。若在世界系采用 double integrator / 速度参考，可在固定 yaw、已知 yaw 变换或局部线性化下得到适合 QP 的模型；但应标明模型域和执行内环抽象。

同时平移与自旋时，body / world 加速度有旋转耦合；轮速通过底盘几何 Jacobian 同时受 vx、vy、wz 影响，三个独立 box 限幅不自动满足每个轮子的上限。需要确认实车消费的是 body velocity、acceleration 还是力 / 轮速，及通信延迟、死区、饱和、加减速和内环跟踪误差。[E2、E8、E10 支持当前输入 / 坐标边界；上式是模型分析]

不能使用差速 / Ackermann 模型忽略 vy，也不能直接搬 F 的固定 yaw 模型作为全向哨兵完成姿态控制的证明。新车底盘 heading 与云台 yaw 的独立关系亦须保留；几何 Path tangent 不能强制覆盖底盘自旋任务。

### 9.3 候选最小合同及 adapter 决策

Research 候选应明确定义以下数据，无需先创建完整 production 消息包：

| 合同项 | 最低语义 |
| --- | --- |
| reference | 位置 / xy 导数、底盘 heading / wz、时间或 phase；是否需要 acceleration / jerk 由模型假设决定 |
| terminal | 目标位置与姿态、允许终端速度 / 停车；不能只到 XY 后让 yaw 永远不完成 |
| corridor | 可选；明确约束对象、footprint / margin、static map revision、连续或采样认证范围 |
| state | 源时刻 canonical pose / body twist、yaw、输入 fresh / valid、frame 与 plan epoch |
| model | 状态 / 输入单位、dt、速度 / acceleration / wheel coupling、内环与 ZOH 假设 |
| proposal | 生成基准、适用时域、plan / odom epoch、deadline、下一命令；超时不继续执行 |
| execution | smoother / serial 实际改变控制的方式、同一物理限幅、唯一 cmd owner |

两条可比较候选边界：

1. **几何 Path + 自定义 timing frontend + MPC**：若研究假设只需几何路径，此 adapter 足够；明确它生成新的速度、导数和朝向策略，不能称无损复原 NEU。
2. **现有轨迹后端 rich output + reference frontend + MPC**：若收益依赖原段时间 / 多项式 / 终端 / corridor，就增加可选输出，不把数据截成 Path 再猜回来。保持现有 Path 输出与 baseline 并存。

无证据要求改搜索时，选现有规划几何。若实际资料表明某搜索 / trajectory feasibility 和控制模型共同优化，才扩展 C-C 边界并解释不可拆原因。当前资料不支持整套规划器重迁移。

### 9.4 复用与重设计清单

| 选择性复用已有基础 | 必须重新确认 / 设计 |
| --- | --- |
| F 的 Nav2 plugin / dual-controller 接入知识、controller odom 修正方法 | 我方四全向轮 yaw / vy / 自旋、真实内环和轮速耦合 |
| bounded solver、warm start、deadline 观测、generation / epoch 失效机制 | NEU 未披露的 state / input、Q/R、terminal、horizon、参考导数 / 时间 |
| proposal 重锚与单 owner 的经验 | 新模型下旧解是否仍可执行，超时 / 制动边界 |
| 既有测试场景 / 轨迹观测工具中适用于纯静态的部分 | corridor 对象 / 认证、重规划 reference phase、smoother 一致性 |
| baseline MPPI profile 与统一物理 / 任务指标 | static 候选收益和真实底盘可行性；不能继承 F 的接受状态 |

复用采用隔离新实验、明确来源和范围；冻结 F / R 本体及原证据保持。新静态研究不构成对冻结动态路线的恢复。

## 10. 下一阶段顺序与最小可判定实验

以下是建议路线，**只有 L0 在本轮完成**。每阶段先定义假设、baseline、少数决定性指标，再 Go / Modify / Stop；没有 Go 不补生产级工程。L 与 C 不并行争用实车 / 修改主链；先定位，后控制。

### 10.1 定位 L0–L6

| 阶段 / 分级 | 做什么及最小实验 | 决策门 / 产出 |
| --- | --- | --- |
| L0 / Audit，已完成 | 分享复原、规范、跨 ref 源码与冻结证据审计 | 本报告；确认真实缺口与候选簇，不写实现 |
| L1 / Research | 选定硬件 / map bundle / 来源 ref；复用既有 spin bag 与质量代码。建立 seeded GICP、无 seed / 错 seed、已有 T 局部恢复的能力 baseline。补真实独立数据与真值方案 | 输出 baseline failure modes、能做 / 不能做范围；时间 / 地图 / 前端错误先修正实验输入，不能归因 global retrieval |
| L2+L3 / Research | 一个离线完整核心：关键帧 / submap 库、SC 或简单检索、Top-K / yaw、已有 GICP / gates；所有候选私有。先不做 factor graph / UWB 实融合 | 同一输入下错误接受、正确召回、可确认恢复收益；仅算法 Go 才进 L4 |
| L4 / Integration | 多帧、map / odom epoch、首次 baseline 门、唯一桥提交、选中 profile 的 stop / permit / replan 协议。先 shadow，再隔离仿真 live commit | 拒绝错误候选时 active correction 不被污染；LOST 不执行导航；新 TF 与旧 plan 交接正确。聚焦核心回归 |
| L5 / Integration | Gazebo：真实位姿搬移而估计未知的 kidnapped、错误 seed / 对称 basin、遮挡 / 自旋后恢复、odom reset / stale 输入。区分全局丢失与 LIO 发散 | end-to-end 正确恢复或安全保持 LOST；记录错误 commit / 运动许可 / 停车行为。仿真不足以证明实车 MID360 召回 |
| L6 / Deployment | 后续实车授权后，先停稳 / 低速受控，再扩遮挡、旋转 / 构型与跨次记录；进入正式配置前执行相关 TF / localization / safety 部署门 | 对选定车、map / profile 建立实车接受与回退证据；不以“回来了”忽略中途错误接受 |

L1–L3 首轮最小数据建议为三类、每类约五个**独立 episode**：正常 / 不同朝向；对称或重复结构 + 错 seed；低点数 / 动态遮挡 / 高转速观测。数量是初始成本上限建议，不是安全可靠性的统计证明。数据不足时先证明不足，不能靠复制相邻帧充数。每 episode 取有限 query window，map 构建帧与留出查询分开。

baseline 至少分为：
- B-seeded：有真值附近人工 seed 的现有 GICP，只衡量精配准上限，不给 candidate 召回喂真值。
- B-operational：选定当前正常定位 / T 局部恢复，展示已知 guess 有效范围。
- Candidate-global：无正确 seed 查询、可选 noisy prior；它补能力，不能只挑 baseline 本来完全无法初始化的场景而省略正常退化对照。

首轮决定性指标：

| 指标 | 精确定义 / 注意 |
| --- | --- |
| false ACCEPT | 在预先定义正确位姿误差范围外仍标接受；离线即可视为失败。范围由地图对称分离、footprint / 通道余量及任务容差定义，不临时按结果改 |
| recall@K | coarse 集合含正确 basin 的 episode 比例；记 K、搜索次数和无先验 / prior 条件 |
| verified recovery | 正确 basin 经 fine / 多帧后可接受；拒绝和无候选均计恢复未完成，不从分母删除 |
| age / confirmation latency | 来源测量至确认时间与计算耗时分开；初始定位与 runtime 停车时长分开 |
| false LOST / false reject | 正常数据被拒比例，体现可用性；不能为减少此值放行错位置 |

真值缺失时只能比较内部一致性 / 人工场景标签，不能宣称绝对定位误差或零 false ACCEPT。GT 仅供独立评分，不进入 candidate、prior 或控制算法。

- **Go**：能在无 seed 场景完成正确恢复，所测对称 / 错 seed episode 无错误接受，正常数据不明显倒退；报告每类分母及限制，再做 L4。
- **Modify**：可召回正确 basin 但精配准 / 拒绝门导致可归因失效，只改一个影响结论的因素后重测；每种失败首轮最多一轮定向修改，再重新决策。
- **Stop / 改路线**：关键对称场景出现稳定错误接受、依赖真值 / 不现实 prior、无法形成比已有 seed 方案更有价值的恢复。记录原因，不继续堆生产接口。
- 有限样本“零错误接受”是继续研究条件，不是 Deployment 概率保证；校准阈值和留出集必须分开。

L5 不能仅修改初始 seed 就声称模拟真正 kidnapped；观测、估计与真实运动的关系需成立。仿真点云视域 / 密度与 MID360 不同，应单列限制，禁止用它替代真实描述子验证。

### 10.2 控制 C0–C6：先确认模型与接口，再谈复现

| 调整后的阶段 / 分级 | 做什么 | Go / Stop 依据 |
| --- | --- | --- |
| C0a / Audit 补充，定位阶段通过后 | 尽量取得 NEU state / input / dt / reference / terminal 或源码；梳理当前实车四轮几何与内环 | 若没有资料，明确名称为“按分享思路构建的本地候选”，不称精确 NEU 复现 |
| C1+C2+C3 / Research 边界合并 | 同时确定模型域、heading / 自旋、timed reference、terminal、执行限幅、adapter 或 rich output。先一条横移 + 弯曲参考检验一致性 | 无法构造物理可执行 / 同时满足终端姿态的链则 Stop；不先大规模接 plugin |
| C0b / Research 核心运行 | 最小 model + frontend + MPC 离线 / 受控 harness；可复用 F bounded QP 框架，也可评估 TinyMPC | 数值能运行且参考 / 模型匹配只是进入 A/B 条件，不算收益 |
| C4 / Research 纯静态对照 | 定位可靠的隔离 harness / 低速 Gazebo，与输入正确 MPPI 配对；相同几何、目标、footprint、物理限制和安全层。先少量场景 | 有预先定义的跟踪收益、无任务 / 接触 / terminal 倒退，且 deadline 可接受，才工程化 |
| C5 / Integration | 复用 dual-controller 接口，补正式时间 / epoch / fallback / sole owner；Gazebo 扩高速、弯道与平移自旋组合 | 证明执行链与参考模型一致，切换 / 超时可控；不加入动态预测 |
| C6 / Deployment | 后续实车授权后低速开始，对真实底盘做配对 A/B，再逐级扩速度 | 实车收益 / 任务 / 周期与保护门都通过，才考虑进入正式配置；MPPI 可回退 |

C4 首轮建议三类静态轨迹，每类约五次配对：
1. 横移 + S 弯 / 直角转弯：检查 vy、弯道速度和横向误差。
2. 到达目标 XY 并完成独立 heading / 停车：检查 terminal 与底盘朝向解耦。
3. 平移 + 不同 yaw / 自旋：揭露固定 yaw 模型局限；若候选声明固定 yaw 域，应将此项明确列为不支持，不能称通用哨兵控制替代。

主要决策看任务完成 / 静态碰撞 / 终端位置姿态、横向误差、速度或加速度震荡、deadline miss；同时保存 heading error、弯道响应、命令变化 / jerk、solver / 控制接收周期 P95 / max、CPU 及安全层触发原因作为归因信息。连续性指标要区分规划导数、下发命令差分和实际运动加速度，不能互相代替。

公平性分两层：
- **完整控制链 A/B**：共享几何、终点、速度 / 轮速 / 加速度限值、同定位 / 地图 / 底盘环境。候选可利用丰富时间 / 前馈，但必须说明这些是它的组成部分；不能只把收益归给 solver。
- **归因实验**：确有收益后再用同模型 / 参考消融 timing、feedforward 或求解器。MPPI 不消费 TimedTrajectory 时，不能称两者使用完全同一时域合同。几何 cross-track 与任务终端可共同比较，time-tracking 误差只在共同参考时钟有定义时报告。

预先设一个主要改善门，例如配对横向误差 P95 或终端姿态显著改善；具体幅度应超过重复噪声，由 baseline 确定后冻结，本文不凭空指定百分比。其余任务成功、接触和物理限制为不得倒退项。无明显收益 / 高频安全停滞 / 模型不适配则 Stop；只有可归因的单个模型 / 参考问题允许一次定向 Modify。

在实际计划使用的计算机上记录运行环境和期限，避免将旧实验主机偶发迟延直接当成 solver 本质失败；同时不能因新机器更快就忽略真实接收 / 执行链的 deadline。研究价值由控制效果决定，部署实时性仍需证据。

## 11. 二十项适配风险核对

| 用户要求检查项 | 结论 / 下一阶段处理 |
| --- | --- |
| 1. 特定底盘模型 | NEU 模型缺失；我方四全向轮 vy / yaw / 轮速耦合必须验证，不能套差速 |
| 2. 雷达是否固定底盘 | NEU 未披露；我方云台固连使用源时刻补偿，描述子亦需一致基准 |
| 3. TF 假设 | 保持 canonical owner；原 NEU map-to-odom 概念不是 ROS TF 合同 |
| 4. 点云密度 | MID360 非重复点型 / 积累窗 / 遮挡影响 descriptor 和 H；真实数据验证 |
| 5. IMU 安装 | 选定 MID360 内部 IMU 与雷达刚性；底盘 IMU 不可无条件替代 |
| 6. 地图格式 | occupancy / PCD / descriptor 库对应同 map revision、origin / yaw；仅 PGM 不够 |
| 7. ROS 实现 | 分享未知；本项目 ROS2 / Nav2 接口和生命周期为适配依据，不移植 ROS1 假定 |
| 8. 控制频率 | NEU 未给；MPPI profile / F 20 Hz 为各自条件，不能直接继承 NEU 实时性 |
| 9. 速度范围 | profile 各异；四轮 / 加速度 / 自旋组合与 deadline 联合确认 |
| 10. 动态机器人假匹配 | 静态特征可用率、遮挡、map mismatch 和对称 basin 在离线留出集检验；不引入预测 |
| 11. 默认可通行区域 | map mask 只在其正确且 task 支持时作 prior；不得 hard clip 合法斜坡 / 构型位置 |
| 12. 变形 / 狗洞 | 有效点、sensor extrinsics / footprint / body height 变化必须有 profile；不能拿普通姿态结果覆盖 |
| 13. 错误 TF 污染 | 首次候选门、独立帧、epoch、唯一 commit 与停车许可构成闭环 |
| 14. MPC 四全向轮适配 | 任意 yaw 非固定 LTI；横移 / 自旋 / 终端朝向纳入静态测试 |
| 15. planner 输出修改 | 当前 Path 丢高级信息；只在候选需要时加 rich output，baseline 保留 |
| 16. 隐式共享状态 | 图的噪声 / 对齐、恢复 candidate epoch、控制 reference phase 都须显式化 |
| 17. 单模块迁移破坏闭环 | seeded GICP ≠ recovery，TinyMPC ≠控制链；每个候选需技术完整边界 |
| 18. Nav2 适配丢时域 / corridor | plugin 能承载 rich reference；不为 Path 统一牺牲算法所需数据 |
| 19. 共状态估计 / 时间基准 | source stamp、odom epoch、plan epoch 与实际 receipt / execution 区分；latest TF 不替代源时刻 |
| 20. 联合安全保证 | 质量门、TF 提交、计划失效和最终运动许可联合验证；serial watchdog 不自动保证定位丢失停车 |

## 12. 当前不值得做与待确认信息

现在不做：整套前端 / 因子图替换、未经标定 UWB 在线高权重融合、未验证雷达 ego pose 覆盖 TF、SC 未用对称增强、无图探索、重新迁移路径搜索、decision 层、直接 MPPI / AMCL 替换，以及任何动态预测路线续跑。也不为一个尚未证明召回 / 拒绝有效的候选先建全仓测试、完整生产 fallback 或大规模抽象。

下一轮真正影响选择的信息：

1. 选定目标是老车还是新车、实际部署 commit / profile、地图 bundle、云台 / 构型与传感器外参。现有跨 ref 事实不能自动合并为该车 baseline。
2. 可独立评分的真实定位数据、初始 / runtime 故障窗口；自旋旧 bag 可复用，但对称 / kidnapped 真值需另有证据。
3. 雷达站 / UWB 的本机 producer、坐标 / ID、误差和时钟 / 延迟标定；缺少时保留无 prior 方案。
4. NEU 定位后端的 noise / 信息 / reset 公式和控制模型 / reference / terminal 资料。没有资料可进行本地假设研究，但不能宣称原算法复现。
5. 真实底盘速度内环、wheel geometry / limits、控制接收间隔，以及当前最终 stop / inhibit consumer。

这些是下一阶段的输入条件，不妨碍本轮审计结论。首选最小技术完整单元仍是 C-L 恢复闭环；原因子图是条件升级选项。控制进入 C-C 静态研究前，先完成定位可靠条件与 C1+C2+C3 边界。保持“宁可暂时 LOST，不错误 ACCEPT”和“静态收益成立再考虑 MPPI 替代”。

## 13. 可复核证据索引

引用方式：`别名:仓库相对路径` 指第 1.2 节提交的 Git 对象，**不是当前检出文件**。可用 `git show <提交>:<路径>` 复核；当前存在文件另有阅读链接。代码 / YAML 证明存在和配置，日期报告只证明其中条件下的历史观测。

### E1 — 规则、分支与阶段

- H:`README.md`、`docs/development_workflow.md`、`docs/project_structure_and_documentation_index.md`、`docs/2027_architecture_decision.md`、`docs/runtime_profiles.md`。
- M:`AGENTS.md`、`docs/research_integration_deployment_principles.md`：实际读取根规则和完整分级原则；§3 / 8 / 16 / 19 分别对应 Research、最小实验、核心风险提升验证、代理执行规则。
- T:`docs/branch_mainlines.md`：硬件双主线和历史快照限制。
- 当前阅读：[开发流程](../development/git_workflow.md)、[结构索引](../README.md)、[runtime profiles](../operations/runtime_profiles.md)。

### E2 — LIO、TF、twist 与传感器时序

- H:`docs/contracts/tf_contract_2027.md`、`docs/contracts/topic_contract_2027.md`、`docs/phase2a_lio_validation.md`、`docs/phase2b_twist_validation.md`、`docs/external/fast_lio_multi_ros2.md`。
- H / T:`src/rm_localization_adapters/src/lio_adapter.cpp`、相关 gimbal / imu adapter 源码及 `config/lio_adapter.yaml`；读取 canonical 输出、frame compensation 和有效性边界。
- T 的 LIO 更晚增强及 E5 记录用于 age / backlog / native CustomMsg 归因，不能默认 H 同步拥有全部实现。
- [TF 契约](../contracts/tf_contract_2027.md)、[twist 验证](../archive/phases/phase2/phase2b_twist_validation.md)。

### E3 — Seeded GICP 与 quality 的真实定义

- H / O / G / T:`src/rm_gicp_relocalization/src/gicp_relocalization_node.cpp`、`config/gicp_3d.yaml`、`README.md`；审计 seed、timer、source generation、GICP、jump / fitness、输出 covariance 与无 TF。
- O / G / T:`src/rm_gicp_relocalization/src/registration_quality.cpp`、`include/rm_gicp_relocalization/registration_quality.hpp`；审计 overlap 分母、normal / nearest correspondence、中心化 J、平均 H、eigen / condition。
- H:`docs/phase2j_3d_relocalization.md`、`docs/phase2k_old_car_gicp_integration.md`。
- [当前 3D 重定位边界](../operations/relocalization_3d.md)。

### E4 — Global pose gate、唯一 TF 桥与局部恢复

- H / T:`src/rm_relocalization_bridge/src/global_pose_gate.cpp`、`amcl_pose_gate_node.cpp`、`map_odom_from_global_pose.cpp`、`relocalization_math.cpp`。
- H / T:`src/rm_relocalization_bridge/config/map_odom_from_global_pose.yaml`；T 的 `map_odom_from_global_pose_old_car_2026.yaml` 和老车 AMCL launch，区分 generic / 老车开关。
- T 桥中的 innovation rejection confirmation、high angular rate hold、autonomous recovery / bounded rebase、baseline reset、recovery state 输出是本报告局部恢复结论来源；不等同 place recognition。
- H:`docs/phase2c_relocalization_boundary.md`、`docs/phase2j_2d_relocalization.md`。
- [重定位 owner 边界](../archive/phases/phase2/phase2c_relocalization_boundary.md)。

### E5 — 高速旋转：历史失败与更晚修正

- H:`docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`：历史更新角度 / Omni noise / recovery 失败和限定候选。
- T:`docs/validation/old_car_fast_lio_high_spin_validation_20260903.md`：LIO backlog / native 点时间修正、拒绝确认、hold 与停车后恢复的实车条件、成功与限制。
- T:`src/rm_relocalization_bridge/config/amcl_2d_spin_robust_candidate.yaml`、相关 pointcloud-to-scan / LIO 配置与 E4 源码。
- [7 月原始归因](../validation/old_car/amcl_high_spin_root_cause_and_candidate_20260720.md)。最新 9 月记录在 T Git 对象，不把缺失 H 文件伪装为当前文件链接。

### E6 — Integrity 的证据、状态与无权限边界

- I / O / T:`src/rm_navigation_integrity/README.md`、`rm_navigation_integrity/core.py`、`config/localization_integrity_shadow.yaml`。
- 对应 `docs/navigation_integrity/design_audit.md`、`implementation_report.md`、`localization_verifier_design.md`；源码中的 quality state 与 hard scan / jump / evidence completeness。
- 这些 ref 中的实现为 shadow；没有在本轮提升其权限或状态定义。

### E7 — 地图与外部位置接口

- H:`docs/phase2e_map_bundle.md`、`docs/phase2f_map_deployment.md`、`docs/phase2i_managed_mapping.md`、`docs/validation/pcd_pgm_dirty_map_end_to_end_report_20260720.md`；bundle / candidate / approved 边界。
- H / V:`src/rm_competition_interfaces/msg/`、`src/rm_referee_interface/`、`src/rm_serial_driver/`；V 的 `OperatorNavigationTarget.msg` 对应目标语义，不能作为 ego position。
- V 的传感器 / 云台 / 构型契约与配置仅支持“有此边界 / 配置”，不支持“硬件已验收”。

### E8 — MPPI、controller odom 与最终执行

- H / O:`src/rm_nav_config/config/nav2_old_car_2026_dual_stvl.yaml`、`nav2_old_car_2026_left_stvl.yaml`、显式 spin / simulation profiles；检查 controller_server、bt_navigator、planner_server、velocity_smoother 各参数层级。
- H:`docs/dynamic_obstacle_critic/controller_odom_contract_experiment.md`、`docs/runtime_profiles.md`：已有实际默认 /odom 消费问题及显式实验修复。
- H / T:`src/rm_serial_driver/src/serial_transport_node.cpp`、`src/rm_chassis_interface/`；command freshness / watchdog，不自动构成 localization permit。
- H:`docs/contracts/chassis_contract_2027.md`、`docs/contracts/serial_protocol_2027.md`。
- [controller odom 已有实验](../dynamic_obstacle_critic/controller_odom_contract_experiment.md)、[底盘 owner 契约](../contracts/chassis_contract_2027.md)。

### E9 — T-DT 本地化规划输出

- T:`experiments/tdt_planner/COLCON_IGNORE`、`rm_tdt_planner/README.md`、`include/rm_tdt_planner/planner.hpp`、`src/nav2_plugin.cpp`。
- T:`experiments/tdt_planner/rm_tdt_planner/vendor/tdt_nav/MinimumSnapOsqp/minimumSnap.hpp`、`minimumSnap.cpp`、相关 sfcSquare；审计 Output、内部段时间、默认阶数 / 目标、采样输出。
- T:`docs/tdt_migration/assessment_and_plan.md`、`docs/external/tdt_nav_kit.md` 及对应迁移 / 接口记录。
- 上游固定来源：`https://github.com/T-DT-Algorithm-2026/tdt-nav-kit` @ `a4bddc467f84d2418a4de06c65c8c79403eb4a13`，MIT；所引内容为 YAstar / MinimumSnapOsqp 等规划组件，未获得配套定位 / 控制源码。相关 OSQP 0.6.3 / OsqpEigen 0.8.1 的来源 / 许可按该实验既有引入记录保留，不新增依赖。

### E10 — 冻结 Temporal MPC 实际代码与工程知识

读取 F Git 对象及 `build/temporal_mpc_main_20261004` 中对应工作树的实际代码：

- `experiments/temporal_mpc/temporal_mpc/frontend.py`、`frontend/README.md`：StaticRoute / center region / timing 与 plan 状态。
- `experiments/temporal_mpc/ros2/rm_temporal_mpc_controller/src/controller.cpp`：setPlan、状态请求、generation、worker / proposal、canonical TF、fixed yaw 域 / deadline；无独立最终 cmd publisher。
- `experiments/temporal_mpc/integration/README.md`、`dual_controller_humble.yaml`、`execution_guard_node.py`、`temporal_mpc/execution_guard.py`：双控制器、odom、fallback、shared final guard 和已知失败条件。
- solver / runtime 参数用于解释 F 的受限范围，不作为 NEU 型号或新候选推荐默认值。

### E11 — 冻结与 Stop，不重新实验

- H:`docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md`、`all_dynamic_avoidance_documents_20261004.md`。
- F:`docs/dynamic_navigation/temporal_mpc_freeze_20261004.md`、`temporal_mpc_architecture.md`、`temporal_mpc_timing_results_20261004.md`。
- R:`docs/dynamic_navigation/r4_final_corridor_research.md` 的最新阶段决定；`r4_native_stop_audit.md` 作为同路线补充入口。R 是 10 月 6 日更新，不用 10 月 4 日总文档取代最新结论。
- [动态研究历史](../dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md)、[Temporal MPC 冻结阅读副本](../dynamic_navigation/history_documents_20261004/temporal_mpc_04291a41/docs/dynamic_navigation/temporal_mpc_freeze_20261004.md)。阅读副本不代表重新执行。

### 本轮验证范围

已完成材料阅读 / 渲染、源码 / 参数 / 日期证据交叉核对和文档链接 / 差异检查。没有运行算法效果实验、构建或全仓测试；文档改动不触发 runtime gate，且分级原则不要求为审计重复已冻结实验。本报告不新增任何“性能更好”“实车可部署”结论。
