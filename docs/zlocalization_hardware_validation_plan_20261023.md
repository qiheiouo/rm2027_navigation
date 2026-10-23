# Z-Localization Hardware Validation 入口（2026-10-23）

本轮没有 Hardware Go。以下 gate 等新车实际数据；不再增加 synthetic 测试来替代。

1. **传感器与几何**：确认选定 MID360 内部 IMU 原点/轴、LiDAR–IMU 外参和时间同步；
   测量 base↔gimbal↔sensor、home/回差与时变机械误差。验证 full-base 启动姿态/重力、
   chassis heading 的物理定义及 cloud 与 odom 的实际源时刻。
2. **session 与停车 reset**：同录 MCU sample/boot/reset、FAST-LIO session/zero-start、
   source/receive 时间。验证整组重启流程，先闭合不可见 in-process rebase 的 metadata；
   有依据后再决定是否需要 ResetFrontend 适配，当前不构建假 reset 服务。
3. **一组真实定位 A/B**：冻结同一 FAST-LIO、同一独立 prior map/query episode；
   确认 3D prior 与 Nav2 occupancy map 的 map 原点/方向一致；
   对比原定位与 Z 候选的位置/姿态、correction 跳变、有效性和恢复错误接受。
   少量覆盖正常行驶、云台运动/高速自转、坡道/横坡、狗洞与遮挡；无需先做全场 benchmark。
4. **地图与统计资格**：实测 map alias、Scan Context recall、错误 basin 与真实恢复成功率；
   检查 model covariance coverage/创新相关性、共享 scan/map 误差。阈值、噪声、雷达高度、
   descriptor 参数由此校准，不继承上游示例为实车事实。
5. **计算与执行**：目标 NUC 上观察真实 CPU、温度、内存和卡顿；另行验收 MCU 时间、
   ESTOP/停稳、完整最终 motion authority。定位 trusted 不自动授予速度权限。

只有这些证据到位，再讨论受控低速 Nav2 A/B 与正式链迁移。新车真实 J、轮级约束和
MCU 分配是独立控制验收问题，不属于本轮定位软件修改。
