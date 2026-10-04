# P2B 静态仿真验证回传（复制为新报告后填写）

状态：未填写。不要将预期结果写成实际结果。

## 版本与环境

- 交接提交 / 实际 HEAD / 分支：
- 验证日期、操作者/模型：
- git status；是否存在源码偏离：
- 镜像 ID、CPU、可用内存：
- P2B_SERIES / 持久运行目录：
- 旧 static_v1/static_v2/static_v3 证据及原回传报告哈希核对：
- sanitizer 独立目录、source_commit/image_id、chain_audit.json、ldd.txt、求解器 SHA、宏与插桩符号核对：
- profile、场景及 aggregate 路径/SHA：

## 实际执行记录

| 步骤 | 命令 / 开始结束时间 | 退出码 | 实际结果 / 日志绝对路径 |
| --- | --- | --- | --- |
| 依赖（如执行） | | | |
| 构建 | | | |
| CTest：25 + 4 + 13 + 4 预期 | | | |
| sanitizer 构建链审计、边界用例 1 项 | | | |
| ASan/UBSan/LSan：25 核心预期（含边界用例） | | | |
| 四配置核对 | | | |
| 汇总 | | | |

## 静态矩阵

| 方案 | 已运行编号 | Nav2 成功数 | 有效证据数 | 静态检查通过数 | 首个失败 / 未执行原因 |
| --- | --- | --- | --- | --- | --- |
| Navfn | | | | | |
| Smac2D | | | | | |
| T-DT A* | | | | | |
| T-DT A*+QP | | | | | |

- 每次 metadata/summary/trajectory/plans/commands/events/costmap 路径见 aggregate：
- fixture spawn 与实际 costmap 标记核对：
- canonical TF/恒等 map-odom 前提核对：
- 间隙、到点/yaw、停止命令、恢复结果：
- 轨迹图与 global-plan revision；cross-track 的解释：
- snapshot 并发更新拒收：仅计 costmap or footprint changed，或明确“未覆盖”：
- 端点拒收：起点/终点、原格值、输入模式、radius、地图几何；不要混作更新拒收：
- 完整预检查 action 延迟（含往返），非纯 core 时间：

## 问题与证据

每个问题写：分类（代码/配置语义/安全/输入或记录/设备约束/未知）、最小复现命令、
首次有效错误、error_type/完整 traceback、相关原始日志路径、是否停止该组、为何判断原因。
若无调用栈，区分源码推断与运行定位，不猜测历史回调位置。不得仅凭耗时或退出码
推断根因；不把当前设备性能限制当作淘汰方案。

## 验证结论与边界

- 已实现但未运行：
- 实际通过的检查：
- 失败、未知、缺失与待设计者复核：
- 当前设备资源观测及待新设备验证：
- 本轮未执行：移动障碍、LIO/双 STVL/MPPI 全负载、实车验收。
- 是否修改任何源码/配置/断言（预期没有；如偏离必须说明）：
- 不宣布部署验收，不 push/merge；等待设计者复核。
