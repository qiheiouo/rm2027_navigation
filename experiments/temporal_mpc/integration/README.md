# 双控制器运行时边界

本目录是M1完成的设计/监督政策，TemporalMPC的Nav2插件尚未实现和加载。
两个example文件不能直接作为完整Nav2启动配置。没有改项目正式launch、BT、
Mission、Planner、NavigateToPose或串口。

M3目标在同一个Controller Server同时注册`FollowPathMPPI`与
`FollowPathTemporalMPC`，保留完整MPPI参数，默认选择MPPI。实际plugin只返回
速度提案，Controller Server保持唯一命令权威。

`selector_node.py`是独立ROS选择监督器，只有String selector发布者，没有
cmd_vel发布者。输入`temporal_mpc/request_controller`显式选择ID；
`temporal_mpc/health`由未来插件/执行边界提供`ready`及`fallback_requested`
两个布尔字段。ready必须包括fresh tracker/prediction、有效map/TF/state、
wall输出期限、全量几何约束、目标模型子域及已取得阶段接受。没有这个生产者
就拒绝MPC选择。health接收失流100ms回到MPPI；恢复健康不会自动弹回。
插件故障应立即请求回退，同时输出重新验收的上一条序列或有界制动，直至
action preemption确认。该监督器自身也需由上层wall watchdog监控，不能把
transient_local最后一条选择永久当作存活证明。

Selector输出使用可靠+transient_local QoS，BT中必须每tick重读，例如示例的
ReactiveSequence。Humble 1.1.20原生FollowPath在RUNNING时读取controller_id
变化并更新动作目标，server的updateGlobalPath接受pending goal后setPlan。
因此运行时切换可以通过内部BT/action实现，不需要上层任务重新发NavigateToPose。
若直接使用FollowPath action，则发送保留当前path/goal_checker的更新目标，
不能把“取消旧动作后停住”冒充连续交接。

已核对源码：

- [Controller Server 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)
- [FollowPath action 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/plugins/action/follow_path_action.cpp)
- [ControllerSelector 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/plugins/action/controller_selector_node.cpp)

政策测试覆盖MPPI→MPC→MPPI、拒绝未知ID/未就绪、故障保持及健康失流；不是
ROS行为树/action/插件运行测试。M3进入门是固定Humble环境真实加载两个
插件，运行中的双向切换、setPlan/goal checker/speed limit/生命周期、
timeout/infeasible/预测退化注入、命令持续性/唯一publisher，以及同场景B0
物理oracle。当前主机ROS为Jazzy，未据此冒充项目Humble运行验证。
