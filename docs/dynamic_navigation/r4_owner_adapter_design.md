# R4 A07：现有输出链的最小接口设计

2026-10-05，Asia/Shanghai；工作分支 `experiment/r4-hws-prediction-consumption`，设计基点 `0cc2f068`，main 基点保持 `d735ee12`。承接 [十二项复用矩阵](r4_reuse_audit_checkpoint.md)。本轮仅完成接口设计与只读源码核对，不创建消息定义、插件、节点、运行配置或求解器，不进行闭环/故障实验。

**设计决定：R4 通过标准 Controller 返回 proposal，现有 controller/behavior 发布边界携带其来源，现有 smoother 继续拥有所选末级 ROS 输出，现有 serial/stub 继续消费。** 最小适配必须同时解决原子来源、撤销、实际待发送命令和下游持有期限；仅在 R4 plugin 内检查 lease 不能覆盖 native MPPI、behavior 或串口。没有新增独立 owner 的必要性证明。

## 1. 核对版本与实际变更点

只读检查既有固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3` 的十个接口头文件；controller/smoother/behaviors/costmap/BT 的包均为 Humble 1.1.20。官方 [1.1.20 release](https://github.com/ros-navigation/navigation2/releases/tag/1.1.20) 解析到 `a097086719c88f781aa59788eca29ac6ca5e56db`；从该 commit 只读九个相关文件，在临时缓存核对 controller/smoother/TimedBehavior/OdomSubscriber 四个头文件与已安装文件 SHA256 相同。源码没有导入或执行。

逐文件 URL、license、摘要、安装版本、关键行与保留状态见 [设计证据](r4_owner_adapter_design_sources.json)。源码流程事实与下文的拟议接口分别记录；标准版本没有这些新合同。

| 现有模块 | 核对到的边界 | 后续最小变更；本轮未实施 |
|---|---|---|
| controller_server | `computeAndPublishVelocity` 调用所选标准插件；`publishVelocity` 只发布 Twist；cancel/goal completion/deactivate 各有既有分支 | 在原调用前记录控制上下文，在原发布分支绑定来源；异常、取消、切 controller/path/限值时撤销相应上下文，不重写 action/lifecycle 循环。[源码](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_controller/src/controller_server.cpp) |
| behavior plugins | `TimedBehavior` 创建 `vel_pub_` 并在 `stopRobot` 发布；Spin 和 DriveOnHeading 各直接发布 | 同一个薄发送 helper 覆盖这些原调用点，包括 stop/cancel/lifecycle；接线检查还须覆盖 profile 选用的 BackUp 等插件。仅改 behavior_server 不覆盖派生插件，不重写行为算法。[基类](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp)、[Spin](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_behaviors/plugins/spin.cpp)、[DriveOnHeading](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp) |
| velocity_smoother | `inputCommandCallback` 用接收时刻更新 command；原 timer 限幅/rate/deadband 后由 `smoothed_cmd_pub_` 发布，`last_cmd_` 在 deadband 前更新 | 输入必须原子带来源；原 timer 生成实际候选后 admission，再由原 publisher 发送。新增“实际已发值”的记录须在最终变换之后，不能照搬原 `last_cmd_` 作通用执行证据。[源码](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_velocity_smoother/src/velocity_smoother.cpp) |
| odometry | 已有 `OdomSubscriber::getTwistStamped`，callback 写入 header/velocity 时持 mutex，但两个 inline getter 未持同一锁 | 复用已有 subscription，加一个在原 mutex 下复制的只读 getter；不再订阅一份 odom 来替代原输入，不把不加锁的 getter 宣称为原子 snapshot。[源码](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp) |
| current costmap | `Costmap2DROS::getCostmap/getRobotFootprint/isCurrent`、`Costmap2D::getMutex`、既有 `CostmapSubscriber::getCostmap`；FootprintCollisionChecker 沿 polygon 边缘评分 | 复用原 grid/footprint 流；补只读 snapshot 的版本/时间/完整性边界，并补 filled footprint/连续执行区间适配。不得在 smoother 再起 Costmap2DROS/STVL；周边评分单独不能证明内部 cell/连续扫掠。[评分实现](https://github.com/ros-navigation/navigation2/blob/a097086719c88f781aa59788eca29ac6ca5e56db/nav2_costmap_2d/src/footprint_collision_checker.cpp) |
| serial/stub | main `SerialTransportNode::handleCmdVel/writeFrame` 与 stub handle/watchdog；当前 input Twist 无来源；serial 每 10ms 按 steady 接收年龄写帧 | 若严格保留原 proposal deadline，现有消费者需读取同一原子来源并在原 write/watchdog 分支判断，不能把重新接收的 Twist 续为原提案。编码/端口/输出权限继续原实现 |

Nav2 controller 的 SimpleActionServer 使用现有执行机制；安装头文件有 `std::async`，controller 的 costmap 已有独立 NodeThread。它们支持复用原调度责任，但不证明 owner timer、锁、串口 I/O 的最坏响应时间，本轮没有测实时性。

## 2. 三个值合同，分清身份与责任

下列字段是内部/private 接口设计，没有新增 `.msg`。public v2 prediction、A04 observed envelope 及其 wire 保持不变；perception generation 与 command generation 分开。

| 值合同 | 最少字段 | 产生/消费位置 |
|---|---|---|
| `ControlCycleContext` | host instance、execution/authority epoch、所选 controller/behavior ID、cycle sequence、控制 ROS epoch、源端本地 steady acquire time、state/TF stamp、plan/raw-static generation、limits/body digest、R4 receipt digest（仅 R4）、原始有效期限 | 原 host 在 acquire→compute 边界创建；R4 core 只读。MPPI 仍调用 native 算法，host 为其绑定同一来源规则 |
| `CommandProposal` | 上述身份/版本、base frame、Twist、source epoch/deadline、发送时剩余预算、明确 command/revoke 类型、reason | 标准插件返回结果与 host 本拍上下文绑定后，由原上游发布责任生成。behavior 的原 helper 同样绑定其本拍结果；不通过速度 hash 或独立 health topic 拼接 |
| `AppliedCommand` | owner instance/output sequence、完整原 proposal 身份和原始期限、最终待发送 Twist、平滑/limits/body policy、admission verdict/epoch/map revision/证据有效终点、normal/degraded/revoked 标记 | 原 smoother 在最终变换与 admission 后生成；原 serial/stub 消费。记录 ROS 发布不等于串口写入成功或物理执行；transport write 结果单独记录 |

源端 steady deadline 不作为跨进程可比较的裸 timestamp。期限同时携带 ROS clock domain/epoch 与源端剩余预算；接收端使用本地 steady 倒计时，后续 relay 只减去已耗时与已验证传输上界，不重置为新的 75ms。没有可验证 clock domain/延迟上界时，不能声明严格端到端 lease。pause/回拨/restart 撤销旧上下文；读取相同 prediction receipt 本身不授权续 command。

proposal identity 精确绑定本拍 compute，不能用“取当前最新预测”给旧结果补 provenance。R4 如需传递标准 Controller 返回类型之外的 cycle/result 身份，仅加私有 typed context/result adapter，与本次调用 token 对应；不得滥用 `frame_id` 编码身份或在 `computeVelocityCommands` 内另发速度。

### 2.1 原 publisher 的两种互斥接口模式

当前 main 继续是 legacy Twist，本轮不改配置。拟议实验 profile 可使用原 publisher/consumer 的 **leased 模式**：

- 上游 controller/behavior 的原发送责任选择 atomic proposal transport，legacy `cmd_vel_nav` command 入口不同时接受。
- smoother 的原 `smoothed_cmd_pub_` 选择 atomic applied transport；原最终 Twist publisher 不同时启用。拟议私有 topic 可分别为 `/navigation/command_proposal` 与 `/navigation/applied_command`，名称尚未写入运行配置。
- serial/stub 的原唯一 command subscription 选择相应类型/topic；不能同时从 Twist 与 private applied 接口接命令。默认 legacy 模式保持原链，leased 模式仍只有同一个 smoother 输出责任与同一个所选 transport。
- 已选地形 gate 如参与该 profile，必须在其原输入/输出转接中保留原期限/身份，hold/zero 不能续 normal proposal；不支持该合同就不把这个 profile 宣称为 strict lease 验收。本轮不普遍启用或迁入地形 gate。

这不是新增并行最终 publisher：只选择原责任的一种发送接口。若保留 plain Twist 到 serial，则只能证明 smoother 的发布 admission，不能证明原 lease 保留到 transport；不得同时发布 private metadata 和另一条裸速度，让下游靠到达顺序猜配对。

### 2.2 来源许可与撤销

来源身份不等于输出权限。controller/behavior 重叠时，envelope 仅能告诉 owner“谁产生”，不能自动决定“谁应被执行”。继续由所选 profile 的既有 Nav2 BT/mission/action 切换结果提供唯一 active motion grant；owner 只消费 grant/fence，不另建 mission 或安全 FSM。

该共用 grant 接口当前不存在，必须作为明确剩余适配：绑定 action execution ID、所选 producer/instance 和 authority epoch；切换先撤销旧 grant，再接受新 producer 的命令。旧队列中的 normal/zero/revoke 都不能覆盖新 execution；独立序号不能代替跨 producer 的 grant。仅有同 topic remap 或 selector health 不能关闭此项。任一时刻 grant 不明确则既有 owner 走退化责任，不任取最新 arrival 当权限证明。

| 事件 | 原责任中的处理 |
|---|---|
| 新 plan / raw-static revision / body 或限值改变 | 撤销旧相关 proposal/warm；新上下文明确新版本。动态 prediction 更新只影响下一拍，不对旧 1.5s rollout 新增 hard veto |
| 输入不完整、solver failure/overrun | 本拍无 normal proposal；清 warm，host 通知既有 selector health；不能把未认证 brake 当 Follow 成功 |
| controller ID / behavior action 切换 | 使用既有 action/BT 次序并换 grant/fence；只接收新 active execution，拒绝旧排队包 |
| cancel / inactive / cleanup / clock reset | 原 callback 触发 revoke；owner 独立期限兜住未及时到达的撤销。不得承诺在阻塞 compute 内即时完成 cancel |
| source/owner restart | 新 instance 与显式注册/active grant；旧 instance 包不因 sequence 重新从零就恢复权限 |
| proposal 重发 / owner timer 重发 | 身份/原始期限不变，不能当新 acquisition；新 admission 只更新当前执行证据，不续原 normal lease |

## 3. 共用 current admission 的最小边界

调用位置固定为 **原 smoother 最终候选变换后、发布前**。如 serial 再限幅/量化，要保证入库 policy 与最终编码一致；否则必须重新验证实际会被编码的值。输入和 verdict 原子绑定，不能跨 timer 复用上一拍 verdict。

| 输入/输出 | 要求 |
|---|---|
| 输入 command | deadband/limits/rate 后的实际候选、最后成功发送值及其证据层级；不是 optimizer 的未平滑 proposal |
| 输入 state | 最新有效 measured pose/velocity 与原 stamp，source-time TF/frame、当前执行 epoch；R4 corridor map frame 与 local costmap odom frame不混用 |
| 输入 grid | 原当前 costmap 的完整只读 copy、origin/resolution/frame/revision、publication/update/receipt 语义及 age、unknown/边界策略；raw-static corridor 与当前 STVL 分开 |
| 输入 body/interval | 实际 padded footprint、limit/execution policy、允许执行区间及期限；R4 fixed-yaw 不能被当作 MPPI/Spin 的通用假设 |
| 输出 | accept/degrade/revoke、reason、精确候选身份、检查 epoch/map/body/limit revision、证据有效终点；无 future prediction 输入 |

必须覆盖 filled polygon 内部 cell、地图边界、unknown、连续平移以及 native MPPI/behavior 的旋转。A02 CurrentGrid 只支持固定 yaw 和原型尺寸/速度，因此仍只作 harness；不能把它变成共用生产 gate，或为复用它禁止既有 Spin 的合法角速度。连续几何保守界限、成本阈值和实际执行模型必须独立形成有限证据；没有通过证据时 verdict 为 unavailable，不把边缘采样或零速度标为物理制动证书。

数据获取优先复用原 costmap/footprint 的只读流/对象；若现有 API 丢弃 revision/stamp，仅在原 subscriber/producer 上补只读 metadata。不得启动第二 costmap/STVL pipeline。main local grid 更新 10Hz、发布 5Hz；receipt age 不是每个 sensor 的观测年龄，也不能写成每 50ms 有全新地图。snapshot 有效期必须按这些真实语义验证。

原 owner 的发送路径不得等待 solver 持有的 mutex、无限 TF lookup 或 map 更新。读取采用有界 try/copy/失败处理；持 map lock 时不调用 solver、TF、发送或串口 I/O。独立进程/既有 executor 隔离优先于在同一阻塞路径加新线程；具体锁顺序、copy 时间与 timer delay 仍需检查。本轮没有测得上界。

期限/撤销/当前图失败的退化沿原 smoother/transport 分支。正常 brake 仍受 actual limits 和 current admission；不可认证时明确报告 degraded/uncertified，不承诺瞬停或碰撞安全，也不恢复未来动态/stop-tail 硬 veto。

## 4. 75ms：保持源起点，给出可复核时间反例

A02 `execution.py::offer_result` 把 deadline 设为 `snapshot.acquired_steady_ns + 75ms`，不是 solver 返回后再给 75ms。该文件仍是 harness；这里只用它核对原实验合同。

| 设计计算（非实测） | 时间 ms |
|---|---:|
| 第 0 拍 acquire / deadline | 0 / 75 |
| 本拍按原 40ms 总预算完成 | 40 |
| 20Hz owner 下一次发送（一个允许相位） | 50 |
| 第 1 拍按原 50ms 周期 acquire / deadline | 50 / 125 |
| 第 1 拍按 40ms 总预算完成 | 90 |
| 20Hz owner 下一次发送 | 100 |
| 第 0 拍源期限至新实际发送的间隔 | 25 |

这是参数允许的反例，足以反驳“50ms 周期 + 40ms budget + 75ms 源期限即可保证连续 normal 输出”。t=50 发出的 command 不能借首区间 50ms 证据把 normal lease 延长到 t=100；有效终点还须受 t=75 截断。plain Twist 的 serial 不识别该截断；识别 deadline 的 transport 则必须在间隔内停止使用 normal 来源，继续既有退化责任。是否有物理净空不由此算术决定。

保守的连续新提案可用条件包含 `control period + next-cycle compute/IO bound + owner phase wait + transport bound + jitter < source lease`；完整 50ms 执行证据还需计入允许持有区间。main 10Hz 控制也已不满足 75ms 连续提案前提。改成 20Hz 本身不能关闭这些条件。数值清单见证据 JSON；本轮不改频率/预算、延长 lease 或以重复发布续期。

serial 原 100Hz polling、UART 写帧延迟与底盘持有/物理减速各自形成反应界限。即使在 `writeFrame` 拒绝过期 normal command，也不能把下一帧到达或实物停止时间称为精确 75ms。若要声明相应观察门，应测量并预留这些界限；未证明的项目保持未通过。

## 5. native MPPI fallback 与有限检查

继续使用冻结 F 的 ControllerSelector/FollowPath 模式及 main 的 native MPPI。R4 proposal 失效时，既有 health 映射选 MPPI；正在执行的 Controller 调用不被 selector 抢占。owner 到期退化与恢复后 MPPI 新结果是两条证据，不增加 MPPI worker 或独立 publisher。

后续有限检查必须覆盖：

| 检查 | 必须看到的结果；当前均未运行 |
|---|---|
| 默认兼容 / leased 互斥 | legacy 默认图保持；实验模式只有一个所选末级 publisher/command consumer，behavior 与 controller 无旁路 |
| R4/MPPI/Spin/BackUp/DriveOnHeading/stop | 全部通过同一来源与最终 admission；旋转、filled footprint、deadband 后值和编码限幅均覆盖 |
| late/duplicate/restart/cancel/switch | 原始期限不刷新，旧 execution 不能覆盖新 grant；input receipt 与 command lease 分别失效 |
| compute 阻塞 / owner 调度 / transport 阻塞 | 实测 owner timer、lock/copy、串口写入和过期反应；未获上界就不宣布 deadline 或无间断 fallback |
| map/TF/odom 缺失、陈旧、回拨/pause | 不复用缓存 verdict/旧 warm；ROS/steady 撤销、地图/frame 与实际 body 正确；记录退化未认证情况 |
| 配对 comparison 边界 | 若以后比较 R4/native MPPI，两者共用同一 owner/admission/limits/route；本轮不运行比较或扩大实验 |

## 6. 当前状态与停止点

本轮交付的是具体接口/变更点、来源字节核对和时间反例。A04/A05 代码、public v2、A02 harness、main、冻结 R3、原 dirty 内容保持。未导入或 patch Nav2，没有构建/启动 R4 plugin，亦无新的运行测试或部署结论。

标准 Twist 返回值无法独自表达这些来源/共同执行合同；缺口已具体化为原 host/行为发布 helper、原 smoother 输入/最终发送、原 transport 的小范围适配，而非另一套导航链。active motion grant、连续 current geometry 与实际时间上界尚未证明。因此仍不能把“可供链接的 A05 值库”宣布为可部署 R4，也不能在未经这些有限证据的情况下恢复“独立输出 owner + MPPI fallback”方案。
