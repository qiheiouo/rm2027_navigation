# P2B 持久工作记录

状态：**static_v3 中断；常规 45 项通过，混合构建的 sanitizer 失败；新构建链待 Terra 验证。**
分支 `experiment/tdt-planner-phase2`；本轮开发基线 `74f68e2`，交付提交执行前核对。

## 已有证据

- P2A ca608e5：3,200 次离线比较通过相应几何检查；不是闭环导航验收。
- static_v1：记录器错误中断。static_v2 e85b076：39 项通过，Navfn/Smac2D
  各 5/5 有限静态通过；T-DT 首例各恢复 16 次后失败，停止重复。
- static_v3 74f68e2：166 份旧证据完整；构建、45 项常规测试和四 profile 核对通过。
  首个核心测试两次 ASan heap-buffer-overflow，未启动任何仿真试次。
- static_v2 的单次 LIO teardown -11 另案保留，尚未归因规划器。

## 本轮实现与待验

- 已确认原 Release OsqpEigen 与 ASan 调用方的 Eigen 分配宏不同（1/0），与
  库内分配、调用方析构读取指针前 8 字节的报告吻合。修正先前提供的混合构建入口。
- 固定 SHA 不变；新目录中由同一 GCC、Debug、ASan/UBSan 构建 OSQP/OsqpEigen/core。
  旧源码/Release 前缀只读，独立源码快照承接 OSQP 生成头文件；记录与审计实际宏及加载库。
- 新增独立的求解器取解/析构回归。预期常规 46 项；sanitizer 先单验该项，随后
  执行包含它的 25 项核心测试。以上是待验预期；原始失败仍保留。
- 本轮没有改动规划 core、vendor、MPPI、costmap 语义、安全间隙、场景或目标。

## 下一执行者

Terra 按 [构建链修复交接](p2b_sanitizer_chain_handoff.md) 在
`/home/wpie/worktrees/rm2027_tdt_phase2` 执行，依赖、构建和日志保存在
`build/tdt_p2b/`。保留 static_v1/v2/v3，使用 static_v4；sanitizer 每次生成独立
`sanitizer_runs/chain_<时间>.<随机值>/`，中断后保留，禁止覆盖旧编号。

先编译/常规检查、sanitizer 构建链审计和 25 项核心通过，才逐组启动静态首例；
首例和人工审查通过后才补 2–5。当前约 55% 规划迁移进度估算保持，P2B 未通过。
设备性能限制只记录待新设备，不做性能补丁或淘汰方案。移动障碍、全负载与实车未验。
