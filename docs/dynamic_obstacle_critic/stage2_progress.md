# 第二阶段进度记录

本文件随阶段提交更新；不是部署通过结论。

## 2026-10-01 / guard 拒绝 witness

- 复核项目 workflow、TF/topic 契约、分支基线和第一阶段证据。
- 在保持检查顺序、阈值、时域及 pass/brake 行为的前提下增加拒绝分支、
  未来 pose/velocity、reserve 和 raw costmap cell 诊断。
- 试次观察器新增原始 costmap 与 canonical odometry；诊断 double 用完整
  精度文本记录，便于关联源时间戳。
- 当前修改的 11 项模型/几何测试与 2 项 pluginlib/DDS 测试通过；既有
  tracker 18 项历史结果仍在原 overlay 中，未将其算作本轮重新执行。
  `colcon test-result` 汇总为 33（含 2 个 CTest 汇总条目），无失败。
- 实际 guard 进程的 7 个 DDS 场景全部通过，拒绝分支和 raw cell 诊断
  也通过检查；最终编译无警告。YAML 契约与 diff 检查通过。
- 预运行结果保存于 `stage2_evidence/preflight/`。固定物理诊断试次随后运行，
  将按同一 policy 保存 FAILED/PASS，不替换第一阶段失败。
