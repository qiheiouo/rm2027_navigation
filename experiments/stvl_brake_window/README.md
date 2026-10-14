# 有限物理制动窗口

[报告](../../docs/research_history/stvl_brake_window_20261014.md)。仅两个预注册停止时刻，沿固定的既有软件执行夹具；不实现在线NoSafe控制策略。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
bash experiments/stvl_brake_window/run.sh 0.4
bash experiments/stvl_brake_window/run.sh 0.8
```

这两个运行已完成，目录不允许覆盖。运行依赖已有镜像、R4安装资产、上一阶段readonly SDF/observer，保留≥3GiB。run_fixture.py用固定Git版本读取源，仅插入一个停止时刻偏移；原夹具不改，适配的实际源与身份一起保存。

分析：在既有Humble镜像内运行 `python3 experiments/stvl_brake_window/analyze.py`，只把本轮cache和evidence挂为可写。完整归档只读复算用 `python3 experiments/stvl_brake_window/verify.py`，/tmp作为临时可写空间即可；它比较两次所有核心物理指标，不触发新仿真。完整native与失败contacts均入小归档，manifest记录成员身份。

所有门限来自上一阶段；按同一规则400ms无contact但净空不足，800ms停车后contact。不是导航目标成功率、统计性安全证书或实车参数。不能从四个不同停止时刻推导精确最大允许延迟。
