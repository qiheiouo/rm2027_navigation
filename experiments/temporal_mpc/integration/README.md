# Humble 双控制器实验

本目录现在包含可构建的 `rm_temporal_mpc::Controller` 原生 Nav2 插件、异步
OSQP worker、选择监督器、完整隔离参数和 BT。正式 `src/`、Mission、Planner、
NavigateToPose、启动与硬件接口未改动；父目录 `COLCON_IGNORE` 保持隔离。
`*.example.*` 保留为 M1 历史设计片段，运行请用 `dual_controller_humble.yaml`
和 `runtime_tree.xml`，不要把片段直接当配置。

同一个 Controller Server 同时加载 `FollowPathMPPI` 和
`FollowPathTemporalMPC`；默认 MPPI。实验 BT 每 tick 重新执行 ControllerSelector，
运行中向 `temporal_mpc/request_controller` 发布 ID 即可双向切换，既有
NavigateToPose 目标继续执行。上层动作签名没有改变。若直接使用 FollowPath，
调用者发送保留 path/goal checker 的更新目标；选择 topic 自身不改变该直接动作。

## 控制与故障边界

worker 只发布类型明确、长度最多 30 的加速度提案；不发布速度命令。原生插件
订阅测量里程计和源预测，按当前状态重积分，检查源/观测年龄、plan/map generation、
全部 tracks、终态、整足迹静态证书与当前 raw costmap 执行区间。未知/足迹内部
occupied cell 均阻塞。输出由 Controller Server 唯一发布。costmap 锁采用 try-lock，
shadow 使用零等待 TF；原生重验限 10ms，失败返回有界制动并发布退化健康。

时间域为 map、固定 yaw/wz=0；不支持把 odom 帧消息直接改名接入。
默认几何是近面锚点加完整名义 D 圆；可见尺寸代理仅供隔离实验。两者都缺少
实际完整物体/未来运动支持证书。`ready` 表示本周期工程验收可用，**不表示
比赛部署通过**。制动没有有效碰撞证书时只叫有界制动，不能叫安全停止。

QP 仍为 20Hz、1.5s/15节点、最多4相关障碍/64全量重验、400迭代、solver15ms /
core40ms。原序列在新状态/预测重验后才可复用。ROS 抖动用真实 elapsed 时间
移位 warm inputs；4帧有界历史只选择不晚于状态请求的源帧，异常 receipt 清空
缓存。数值残差先投影到执行器界限/停止终态，再过原几何门，不放宽门。

Nav2 保持速度指令，worker 使用加速度模型。插件对当次保持速度区间另验当前
costmap，并对新测量速度预留时域内停止能力，全部修正位置重新验收。当前
0.02m动态 reserve 覆盖当次 ramp/ZOH 的最大0.00177m差异；**不覆盖整段真实
未来响应误差**。下位机响应、全负载和独立输出 watchdog 仍待 M2 验证。

选择监督器只发可靠 transient_local ID，健康失流100ms/退化请求 MPPI；恢复
健康不自动跳回。插件失去 worker 后自身继续制动，不依赖监督器或 BT 活着。
监督器进程失效时，原生制动仍工作，但 stock ControllerSelector 会保留最后 ID，
因此自动 MPPI 交接需要存活的监督器；部署还需要监督器和 Controller Server 的
独立 wall watchdog。现阶段 speedLimit<默认值会拒绝不符合新限制的 MPC 提案，
有界制动/交给已同时接收 SpeedLimit 的 MPPI；QP尚未自适应该限制。
Nav2 自身仍持有 goal checker；固定航向 MPC 不能完成任意最终转向，应使用 MPPI。

## 复现（隔离容器）

使用现有 `rm2027_navigation:humble`，镜像 digest、Nav2 1.1.20 和依赖身份见证据。
不使用 host network、设备或串口。运行时依赖装入 worktree 的
`build/temporal_mpc_ros2/python_deps`，不装入系统；源码/构建输出可删除回退。

```bash
# 在挂载实验 worktree 为 /workspace 的 Humble 容器中
bash /workspace/experiments/temporal_mpc/integration/build_humble.sh
bash /workspace/experiments/temporal_mpc/integration/run_humble.sh /workspace/build/temporal_mpc_ros2/new-run
```

run 脚本设置独立 ROS_DOMAIN_ID=87、localhost DDS、单线程 BLAS，只启动 Nav2 /
worker / supervisor / 受控测试模型。依赖轮子版本和 sha256 见
`runtime_dependencies.json`；colcon 构建不要继承 worker 的依赖 PYTHONPATH。
测试故意终止自己的 worker 子进程，退出时清理全部子进程。

该模型按 wall clock 积分实际 ZOH 命令，不暂停计算期间运动；输入为合成空数组，
故障时再注入当前同位置的合成 track 等。它验证真实 ROS / BT / action / plugin
链路，**不是 Gazebo、tracker、物理净空或公平 STVL/MPPI 对比**。
同一个 NavigateToPose 目标在测试末尾人为 cancel（status=5），不是到达成功。
取消到下一直接 FollowPath 是两个显式控制会话；连续性只在每个活跃会话内计，
总体最大间隔同时保留。

固定 Humble 原始接口依据：[Controller Server 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)、
[FollowPath action](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/plugins/action/follow_path_action.cpp)、
[ControllerSelector](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/plugins/action/controller_selector_node.cpp)。
当前构建还直接验证该镜像安装头文件和真实加载结果。
