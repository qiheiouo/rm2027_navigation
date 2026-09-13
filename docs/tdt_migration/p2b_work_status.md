# P2B 持久工作记录

状态：**static_v1 共享记录器中断；修复已编写，等待 Terra 重验。**
P2A 基线：`ca608e5`；首轮 P2B 交接：`b1c40e6`；分支：
`experiment/tdt-planner-phase2`。修复交接提交以设计者交付号与 `git rev-parse HEAD` 核对。

## 持久目录

- 源码：`/home/wpie/worktrees/rm2027_tdt_phase2`。
- 求解器依赖/构建/配置/运行：该目录的 `build/tdt_p2b/`（现有 git ignore）。
- 内部临时目录：`build/tdt_p2b/tmp/`，不依赖主机 `/tmp`。
- 失败系列：`build/tdt_p2b/runs/static_v1/`，已存在，完整保留。
- 修复重验系列：`build/tdt_p2b/runs/static_v2/`，由验证者首次执行时创建。

## 已取得与待验证

- b1c40e6 原验证构建成功；19 core + 3 plugin + 13 工具测试通过，已复核原日志。
- Navfn 首例 exit=2，记录器在目标前退出；其他组按共享错误停止，无有效静态矩阵。
- 原始回传报告和失败原始文件保留，见
  [原报告](evidence/p2b_static_20260913/validation_20260913.md)。
- 设计者查明代码中 `Log.level` 整数与 `Log.WARN` 字节常量的比较错误，修为
  `LoggingSeverity.WARN`；旧日志没有栈，原报告的 costmap 回调定位已另文更正。
- 已补 4 个真实 ROS 消息回归用例和异常调用栈留存。修复版预期 39 项，**尚未执行**。
- 设计者本轮只做源码与静态语法检查，不修改算法、实车配置、MPPI、场景或安全门。

## 下一执行者

Terra 优先按 [记录器修复重验交接](p2b_recorder_fix_20260913.md) 执行，再按
[完整交接](p2b_terra_handoff.md) 的相同门推进静态矩阵，用
[回传模板](p2b_result_template.md) 创建新报告。原 `p2b_authoring_checks.json`
属于 b1c40e6，不是当前修复验证报告。

复用已验证依赖，增量构建、39 项测试和四配置核对后，先独立跑 static_v2/Navfn 1。
共享故障停止全部仿真；如会话中断，先核对已有运行目录和报告，不覆盖任何编号。
功能/安全问题交回设计者；设备性能只记录并待新设备，不做性能补丁或淘汰候选。
移动障碍、全负载和实车均未进入本轮执行范围。
