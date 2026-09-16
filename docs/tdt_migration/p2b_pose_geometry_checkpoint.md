# 离线姿态几何检查器开发检查点

2026-09-16：已实现 test-only 的 `pose_geometry` 库，未链接进 Nav2 插件。
19 项新回归覆盖连续区间下界、旋转/平移中间碰撞、显式长旋转、253/255/边界、
预算耗尽、无效输入与独立 long-double 多边形 oracle（48 条固定种子运动）。
常规 CTest 实际通过 78/78（core 52、plugin 4、工具 18、ROS 4），日志在
`build/tdt_p2b/pose_geometry_logs/`。此检查点尚未运行新库的 sanitizer。

用户随后提出优先做 path-heading-follow A/B，本模块暂停扩展，不接入规划器。
统一 sanitizer 将在下一项构建中覆盖本库；它的通过也不代表静态导航通过。
证书仅适用于线性中心、显式线性 yaw 的合成运动，尚无真实完整 raw snapshot 回放、
候选连接生成或 MPPI 姿态约束。`RawCostmapInput` 拒绝 Grid/PreparedGrid 对象转换，
人工构造字节时仍由调用方保证 raw 来源；类型本身不能识别手动复制的膨胀字节。
