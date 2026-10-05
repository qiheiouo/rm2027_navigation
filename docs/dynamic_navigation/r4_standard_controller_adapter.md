# R4 A11：标准 Controller 的最小 typed-cycle 适配

2026-10-05，Asia/Shanghai。继续用户授权的隔离开发/有限检查；沿用 main 基点，不接实车。本阶段只创建可选 R4 `nav2_core::Controller` 插件，复用 A08 Follow。没有新 controller_server、lifecycle manager、action server、tracker/prediction/frontend、MPPI、current costmap、subscription、worker 或 cmd publisher。

## 先登记接口与边界

固定 Humble1.1.20 的标准 Controller/GoalChecker 安装头文件只读核对，使用原 configure/activate/deactivate/cleanup/setPlan/computeVelocityCommands/setSpeedLimit 方法。头文件由已有 Nav2 安装提供，不拷贝或修改上游实现。新插件 Apache-2.0；默认 `RM_R4_BUILD_NAV2_CONTROLLER=OFF`，开启必须已经显式构建 A08 Follow 和固定 OSQP 前缀，配置期间不联网。

标准 compute 的 Pose/Twist 不含 atomic prediction、static revision、actual last-applied、source acquisition、active grant 或 provenance。因此 private typed adapter 仅接受原 host 本拍提供的 owned FollowInput 与明确 CycleToken；standard compute 检查该 token 的 plan/speed revision 及输入 Pose/Twist，消费一次并返回原 TwistStamped 类型。原 host 必须用同一 token取回 FollowResult 后才构造 A10 atomic proposal；不使用 frame_id 编码身份，不另发速度或用两个 topic猜配对。

setPlan 绑定完整原 path digest；PreparedCorridor 只增加共享既有 digest 实现的 fingerprint 方法，不复制或重跑 frontend。路径改变、lifecycle、错误token、状态不一致、solver失败、源期限过期均清 context/result/warm。原 host 的 action/BT/owner 责任继续独立；plugin activation 与 token 本身不授予输出权限。

setSpeedLimit 先撤销旧 context/warm并换 revision。此最小切片只接受 native NO_SPEED_LIMIT=0；非零/无效限制给 unavailable，必须由原 host回退到已有能处理该限制的native MPPI。尚未复用实际profile的限值变化来源前，不自行实现另一套限速策略，不忽略限值继续输出。该限制作为明确未完成项保留。

## 预定有限验收

固定 Humble C++17 插件构建/安装/pluginlib加载；一个无 executor/无设备的测试 LifecycleNode提供原父节点接口，不启动 Nav2 graph或输出。验证标准lifecycle、exact plan/token/pose/body-Twist、无context拒绝、single-use/result配对、错误路径/frame/stamp/yaw、speed-change撤销、过期输入及与已有Follow的提案一致性。运行同范围sanitizer与默认OFF配置检查；不运行paired、大规模Gazebo或冻结R3。

原 host 的 bind/take调用、active fence来源与撤销确认、clock/transport witness、actual map与限值采集、原 smoother/serial atomic发送接口仍待接线。没有这些条件时此插件不能作为可部署R4，不能宣称native fallback或75ms物理输出通过。

## 已完成的有限核对

Humble Release 构建与安装成功；11 个 Controller GTest 通过，包括 pluginlib 标准类加载和 private adapter 动态转换、标准返回值与原 Follow 提案一致、无context拒绝、一次消费/取回、exact path与plan/speed/lifecycle revision、pose/stamp/frame/quaternion/body Twist负例、非零speed-limit、expired acquisition与取回前lease到期。没有启动controller_server/behavior_server/smoother或底盘。父LifecycleNode仅提供标准configure接口，无executor或motion IO；原canonical CDR producer的已有小范围fixture回归单独运行。

原35个值库GTest与五个CTest组通过。共享path fingerprint提取前后，以原A08已安装库和新库对照五个固定path（原path、header stamp、反向、orientation、重复vertex），path/map/body digest逐字节相同。安装后C++17下游标准/typed接口调用通过；默认OFF配置没有插件resource注册或OSQP查找。

首次CDR回归未设置容器内可写ROS日志目录而失败，后续值检查依赖项未运行；改用隔离ROS_LOG_DIR后同范围全部通过。失败日志保留；不是算法失败，也没有放宽门限。同范围 ASan/UBSan 的六组 CTest全部通过，本地插件/值库/Sfc与既有固定OSQP kernel插桩；ROS系统库未重新插桩，leak detection关闭。首次sanitizer链接未设置固定OSQP动态库搜索前缀而失败；补上与Release相同的明确依赖前缀后通过，未改代码/门限。实际接口常量核对为安装版 nav2_costmap_2d::NO_SPEED_LIMIT=0.0。源码、失败和保留状态见 [A11来源清单](r4_standard_controller_adapter_sources.json)。

插件只证明“标准调用薄适配可以加载并调用”，尚未由正式host bind/take。正常返回的TwistStamped不能独自承载grant/lease；原host必须检查同token的typed结果并走A10/A09与原输出链。native MPPI选择/阻塞恢复、actual costmap/TF/last-applied采集、非零speed-limit映射、原smoother/transport atomic接口和物理时间上界仍未完成。
