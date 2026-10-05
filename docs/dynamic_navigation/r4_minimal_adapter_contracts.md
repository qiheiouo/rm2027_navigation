# R4 A04：最小适配接口与首个 producer 切片

2026-10-04 接入与验证，2026-10-05 收尾核对。承接 [A03复用审计](r4_repository_reuse_audit.md)。用户回复“继续吧”后，继续在从main建立的R4分支工作。本阶段先落实具体接口与同次关联members适配；不把A02 tracker/frontend/execution转成生产导航链。

## 1. 接入来源与范围（先于代码接入记录）

canonical tracker与public v2来自本仓库已提交 `experiment/dynamic-surface-reveal@b5645eca`，Apache-2.0。只取 `src/rm_dynamic_obstacle_tracking` 的既有core/node、包元数据、shadow launch/config、README，以及core/v2回归测试；不取未提交member evidence、visible-box/surface原型、R2 critic/guard或其他研究包。两个public prediction `.msg` 原样接入现有 `rm_competition_interfaces`；更新其生成依赖但不改变wire字段。

此分支基于main，原先没有该producer；在其canonical包路径接入一份既有实现，是复用资产，不是在A02目录再起tracker。唯一tracker调用仍由既有node拥有，默认shadow配置不变。原checkout dirty代码、冻结R3与main不写入。逐文件source/destination SHA256和修改分类写入 `r4_minimal_adapter_sources.json`。

新增生产代码仅为无法由v2表达的private observed-members接口、原association/member最小导出及薄序列化。首个切片尚不部署R4控制器，也不修改默认Nav2/速度/串口接线。移除方式：关闭新增opt-in参数即恢复仅public prediction输出；移除该可选producer包不影响main默认Nav2链。

## 2. 同一producer的原子 observed-members envelope

最小private消息包为 `rm_r4_interfaces`，不与已有public v2共用schema。`ObservedPredictionEnvelope` 包含原样 `DynamicObstaclePredictionArray prediction`、producer实例ID、reset generation、每次成功tracker更新的sequence、members completeness/reason和 `ObservedTrackMembers[]`。**一个envelope携带同次public值与members**，控制消费端不用异步拼接两个到达顺序不定的topic。原public topic仍发布同一构造结果；没有第二次预测、第二次tracker update或公共sequence字段。

每个 `ObservedTrackMembers` 包含 `track_id`、整数 `last_observation_stamp`、`association_sequence`、本次/最后一次detection index、原始LaserScan beam indices、观测centroid及相对该centroid的观测端点。几何仅表示被关联的2D测量支持；不表示hidden/full-body、future occupancy或安全证书。raster化与stage平移属于后续R4消费，不在producer建立另一套prediction pipeline。

| 情况 | 合同 |
|---|---|
| 正常关联/new track | 从既有assignment/new-track分支导出ID；同一cluster的端点和beam IDs一起保存，不再匹配质心 |
| centroid静态过滤 | 过滤既有detection对象时携带相同members，不能过滤后按位置重建关联 |
| coasting / tentative miss | 保存最后一次真实关联端点及其整数观测epoch，不伪造新观测；public v2 anchor仍按已有CV推进 |
| consumer stage | 使用public数组源时刻anchor，按 `stage-source` 推进一次；端点保持centroid-local；不再从last observation重复补偿anchor |
| reset/restart | node实例UUID区分restart；tracker `time_reset`清members并递增generation；sequence标记更新身份，不填入public v2 |
| 丢成员、公开不完整、预算超限 | private `complete=false`、明确reason、无部分shape冒称完整；清除不可用缓存，public输出遵循原合同 |
| 同源重用 | consumer只可重用未过TTL且身份/摘要不变的envelope；同epoch热改或代际改变使warm失效 |

新增参数 `observed_members.enabled=false`；只有显式启用且producer选择已有 `last_observation_cv`、无decay/clip时才创建private publisher。默认private topic `/perception/dynamic_obstacles_shadow/observed_predictions`。private tracks≤已有prediction max_tracks，全部端点合计≤4096；只限制adapter，不更改tracker/公共预测算法或静默截去障碍。该producer始终不发布cmd/Path/TF/action。

## 3. 静态 corridor：直接使用既有API，不复制frontend

固定迁移来源 `e680b143:experiments/tdt_planner/rm_tdt_planner/vendor/tdt_nav/MinimumSnapOsqp/sfcSquare.hpp` 已有 `SfcSquare::getCorridor(path, maxRange, shrink)`，返回 `CorridorOutput{corridor,index}`；因此无需先扩展T-DT `Result`，更无需移植A02/F的Python frontend。后续薄适配直接绑定同一迁移库的公开Sfc API，复用Nav2 `setPlan(Path)`，而不是再次运行YAstar。

`PreparedStaticCorridor` 内部只读值约定：path generation、path digest、map frame、raw-static revision/content digest、footprint/padding/clearance policy digest、anchors/arc length、centre bounds及对应path indices。生成在plan/map变化时进行；控制拍只读。raw-static unknown/边界与连续机械支持证书必须独立验证，Sfc方框本身不自动构成证书。实际local costmap的odom frame不能因原型而改成map；frame转换和map revision撤销在adapter完成。

本切片仅固定调用接口，尚未导入T-DT库、改动其vendor或新增frontend实现。静态库的来源/许可/OSQP ABI继续使用既有迁移记录，不能把A02 Python OSQP1.0.5视为C++部署依赖。

## 4. 执行租约：必须在原发送链保留来源

针对项目Humble版本读取上游 **1.1.20**：`ControllerServer::publishVelocity`取 `TwistStamped.twist` 发布，丢弃stamp；smoother订阅/发布Twist，按接收时刻和配置timeout判断旧命令。这支持A03判断：单独设置timeout不能保存原提案期限，也不能提供generation撤销。[controller source](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)、[smoother source](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_velocity_smoother/src/velocity_smoother.cpp)。本次只读，不导入或patch上游代码；tag版本不代替本地部署库验收。

拟议唯一发送合同是 **原owner消费与速度原子绑定的来源信息**：selected producer/controller、instance/generation、command sequence、控制输入epoch、有效期限、速度、明确revoke。R4不能向独立topic发一个“健康心跳”来续期另一条Twist，不能以相同速度值/hash猜哪一份命令获得了lease。native MPPI/behavior/zero/cancel也必须使用同一发送规则；否则R4私有校验不是共用保护。

具体窄扩展点：controller的既有 `publishVelocity` 和behavior的既有publish边界保留来源到smoother输入；smoother的原 `inputCommandCallback/smootherTimer` 消费期限、generation，在限幅/平滑后的**实际待发送命令**上执行共用current admission，并保留唯一 `smoothed_cmd_pub_`。使用原timer、limits和lifecycle撤销责任，不建立新的节点/publisher/MPPI线程。此接口尚未实现；标准Twist模式保持，不能假称已获得75ms保证。

剩余期限的ROS epoch检查与steady receipt倒计时必须同时使用：pause/回拨/重启撤销旧generation，未知时间基准拒收，不用跨进程裸steady timestamp假设时钟可比。75ms原始提案期限、20Hz发送采样的反应界限、串口receive timeout和物理减速是四个不同指标，分别登记与验收。deadline过期时执行有界退化不等于MPPI新提案；卡住compute恢复后由既有selector选择原生MPPI。

current admission输入为最后实际输出/本次平滑后命令、measured state、current costmap revision/age、实际footprint与执行区间；输出accept/degrade及原因。它只查当前图/首个执行区间，不重算未来1.5s动态预测或stop-tail硬否决。不能把R4 proposal通过检查写成平滑后的实际命令已通过，也不能直接加载R2/R3 guard。此切片不新增安全FSM或接实车。

## 5. 首个切片的验证与剩余工作

验证目标是：canonical单tracker关联/预测回归等价；public v2消息字节定义不变；source beam identity跨静态过滤/重排/coasting/reset正确；members缺失/预算超限使private消息明确失效；同一次构造的public值与private envelope一致；opt-in关闭时不增加privatepublisher。用小范围单元/消息生成/ROS回环检查，不启动Nav2/Gazebo/底盘或大规模实验。

不以本切片声明完整R4 Nav2接线、实际MPPI回退、lease/admission或物理验收完成。后续按以上具体接口继续：先绑定既有Sfc API与R4消费核心，再在原发送链实现必要的lease/current-admission窄扩展并验证publisher所有权。


实际结果：复用本机已有 `rm2027_navigation:humble` 镜像，三个相关包构建成功；45项核心/固定来源回归通过，4项Humble消息与节点回环通过。核心回归包含4种既有tracker配置、每种32帧，与固定来源逐值比较；ROS检查包含public与envelope内嵌prediction的序列化字节一致、每scan单次tracker更新、coasting/reset身份以及默认无private/速度publisher。

第一次Humble全套运行中47项通过、2项失败，原因是测试使用了Jazzy订阅查询API；改用两版本共用的 `Node.count_publishers` 后单独重跑4项ROS检查全部通过。45项核心与4项ROS为分批结果，不冒称一次49项无失败运行。主机Jazzy仅为先期smoke，不代替Humble验证。

容器无网络、无设备，源码只读，仅R4隔离build目录可写。记录与结果文件摘要见 [A04来源及验证记录](r4_minimal_adapter_sources.json)。收尾核对确认main、冻结R3、原checkout的10处dirty文件及A02非Markdown源码保持原状态；未启动Nav2/Gazebo/串口/底盘或新大规模实验。上述结果仅覆盖本producer切片，不构成控制接线、实时或实车验收。

后续用户授权继续后的Sfc绑定与消费值库见 [A05](r4_consumer_library.md)。本A04记录仍仅覆盖producer切片；command lease/controller/MPPI回退未因此完成。

A07 的 [原输出链接口设计](r4_owner_adapter_design.md) 补充了本节拟议边界的具体限制：behavior 的原速度调用点还在 TimedBehavior/Spin/DriveOnHeading 中；若要保留源 lease 至现有 transport，plain Twist 输出不足，原 publisher/consumer 应以互斥模式选择原子来源接口。active motion grant、共用旋转/连续 current admission 以及时间上界仍未实现/验证，不将 A04 的边界描述当成已完成的执行合同。
