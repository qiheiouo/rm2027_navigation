# R4 A09：复用既有连续几何的当前命令适配

2026-10-05，Asia/Shanghai。用户授权离开期间持续开发和有限实验；沿用 main 基点的 R4 隔离分支，承接 A07/A08。不增加 tracker、prediction、costmap pipeline、controller、publisher、MPPI 或输出 owner。本阶段只落实未来原 smoother admission 可调用的几何值适配，不声明动作授权、lease 或物理安全已通过。

## 先登记复用来源

从既有 T-DT 迁移 `e680b1430db6efd8dc601d5585ee207e5f9b42a4` 接入原路径的四个源码/许可资产：`include/rm_tdt_planner/{planner.hpp,pose_geometry.hpp}`、`src/pose_geometry.cpp`、`LICENSE`。均为 RM Navigation 2026 的 MIT 文件；`planner.hpp` 只提供现有 Point/声明，不编译或引入 planner/frontend 算法。既有实现 `pose_geometry::RawSnapshot/certify` 已覆盖填充 polygon、closed cell/contact、unknown、map boundary、连续平移/显式旋转、有限区间/检查数/时间预算，以及 Nav2 253 的 centre-only 语义。

来源为本仓库固定迁移提交，不从网络重新引入几何库。kernel 在其既有路径保持唯一 provider，原始 SHA 和必要修改在来源清单记录。原 19 个固定 geometry 测试只用于有限 provider 回归，包含既有独立 long-double oracle；不是重跑 R3。新 wrapper 为 Apache-2.0，包为被动值库，没有 ROS IO 或运行 launch。

## 现有接口的确切缺口与最小扩展

kernel 只认证线性 centre + 线性 yaw。实际 held body Twist 在非零 yaw rate 下 centre 沿弧线运动。wrapper 计算精确恒定 body Twist 终点，沿 chord 调用原 kernel，并使用 `|v| |w| h² / 8` 的 centre interpolation error 上界。旋转仍用原显式 yaw interval。

原 `Limits.clearance` 只作用于 lethal/unknown/boundary footprint，253 的 centre constraint 没有 error-reserve 参数。仅给 clearance 加 chord reserve 不能保守覆盖 centre 的 253 判断。因此在既有 kernel 最小增加 `centre_reserve`（默认 0 保持原行为），同时用于 253 distance 的 required reserve；不重写几何算法。

当前 pose 的初始 position/yaw error、状态年龄、实际 velocity/yaw bounds 的保守位姿位移以及显式 actuator tracking tube 同样进入 required reserve。这些是调用端必须声明的模型假设，不从 public prediction/covariance 或 smoother 的 command rate 推断物理 actuator bounds。分别检查 measured-held 与 candidate-held 两种模型；两条轨迹的并集单独不认证实际响应，实际响应必须包含在所声明的 tracking tube 内。

当前 map 只接受 Nav2 raw master bytes、原 frame/revision/update/receipt/current 元数据；不接受经过 footprint 再膨胀的 configuration-space mask，也不消费未来 prediction。当前主线 local costmap 的 odom frame 与 A08 corridor map frame分别处理。snapshot 预处理在原数据更新边界完成，发送检查只读；不启动第二 costmap/STVL，kernel 时间预算不是 lock/timer/IO 上界。

几何输入明确使用实际已 padded 的 convex footprint，positive padding 元数据和至少 .05m 独立 clearance；实际 candidate 应由原 smoother deadband/限幅及原编码适配之后提供。R4 fixed yaw、MPPI/Spin 的非零旋转、BackUp 的反向速度均走同一值接口。结果只给 Certified/Collision/Unavailable 的模型证据及原 candidate 值回显，不选择或修改 velocity，不生成 brake，不更新 lease或授权。

## 有限验收与结论

Humble 1.1.20 固定镜像下，13 个 adapter GTest 与原 provider 19 个 GTest 全部通过；同范围 ASan/UBSan 通过，安装后独立 C++17 下游调用通过。没有 ROS/Nav2 graph、串口或实车运行。初次构建发现 `finite({x,y})` 的 Point/Twist 重载歧义，显式写 Point 修复；初次失败日志保留，未放宽预算/几何/错误门限。

- 已在安装版 Nav2 `FootprintCollisionChecker` 上复现 perimeter-only 漏过 footprint 内部 lethal cell；本适配以原 provider 的 filled polygon 检出。
- 已用实际 padded body、1.2rad/s 和 50ms 区间复现短 Spin：两个端点安全但中途碰撞，连续 provider 拒绝。没有用不符合实际限值的大角度运动制造反例。
- 四条 body-Twist 轨迹以独立 long-double midpoint 积分核对终点，并检查 404 个曲线查询在 chord-error 上界内。这些采样只核对公式；最终区间证据来自 provider 的连续 lower bound。
- 接收端 freshness 必须覆盖几何预算与完整 held interval；原始当前图的 update/current 元数据、TF source epoch、actual body/limits、253 centre reserve、work/time exhaustion 和 immutable copy 的负例均拒绝证书。

chord reserve 的依据：恒定 body Twist 的平面 centre 满足 `|p''(t)|=|v||w|`；线性插值误差不超过 `|v||w|t(h-t)/2 ≤ |v||w|h²/8`。yaw 按实际显式 `w*h` 交给原 provider，不以 wrap 后两个 yaw 端点替代。位姿 age/processing 与 tracking tube 的位置界加到 centre，角度界乘最大 footprint radius 加到 filled-footprint clearance；因此 253 与 lethal 的 reserve 分别覆盖自身几何语义。

从 main 固定配置解析并核对：local grid 为 odom、8m×8m、.05m 分辨率（160×160）；实际 footprint ±.32/±.27、padding .02；原 smoother 速度界 x[-.3,.5]、y±.5、yaw±1.2。对应四个有限空图调用耗时 1.769–2.740ms，每次 3816 cell checks、两条区间。短样本既不提供 WCET，也不覆盖 executor/lock/IO 调度延迟；10ms 仍为晚到拒收预算。

新增被动包的接口见 [current_geometry.hpp](../../src/rm_navigation_execution_adapters/include/rm_navigation_execution_adapters/current_geometry.hpp)；固定来源、默认兼容的最小 kernel 补丁、失败/通过日志摘要及保留核对见 [来源清单](r4_current_geometry_adapter_sources.json)。此阶段源码可审阅、可链接，尚未接入原 owner。

active grant/fence、原始 lease transport、实际 actuator error bounds、current-map sensor age、原 owner 调度/IO 和实际 fallback 仍独立待验收。没有这些证据，Certified 只指固定 current map 与明确模型假设的几何区间。
