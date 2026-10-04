# Temporal MPC offline prototype

独立研究候选；`COLCON_IGNORE`阻止常规workspace构建自动接入。核心没有TF/costmap/goal/velocity发布者、Nav2插件或串口依赖。
`integration/`有未接入正式launch的选择监督器及明确标记为示例的未来双插件/BT片段。正式MPPI与所有已有研究保留。

设计、审计与条件：[架构](../../docs/dynamic_navigation/temporal_mpc_architecture.md)、
[预登记](../../docs/dynamic_navigation/temporal_mpc_experiment_registration.md)、
[进度和结果](../../docs/dynamic_navigation/temporal_mpc_progress.md)。
`PublicAdapter`消费既有v1/v2公共状态；冻结项目tracker→公开字段的适配有测试。
分支直接从main分出；`reference_inputs/provenance.json`记录四个选择性离线参考，
没有继承旧研究运行包/配置/launch，也没有合并旧分支。
既有size只能作为visible_extent代理，不能当作完整observed polygon。

旧SLSQP使用已有NumPy/SciPy。当前实时QP另用固定OSQP 1.0.5，
只安装到临时目录；T-DT选择性导入及依赖登记见`frontend/provenance.json`和
`dependency_intake.json`。没有复制HWS代码。
`pyproject.toml`记录依赖边界，实际版本由运行manifest锁定。HWS的固定版本、
MIT许可和只读文件哈希在`temporal_mpc_upstream_audit.json`；不是运行依赖。

旧SLSQP失败对照的复现命令，在本目录执行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 run_experiment.py --output /tmp/mpc-new-matrix
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 run_experiment.py \
  --output /tmp/mpc-new-ablation --scenarios wait_then_pass --modes observed_polygon \
  --consumptions temporal current_only future_union
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 run_experiment.py \
  --output /tmp/mpc-new-horizons --scenarios wait_then_pass --modes observed_polygon \
  --horizons 0.5 1.0 1.5 2.0
python3 verify_evidence.py /tmp/mpc-new-matrix --output /tmp/mpc-new-matrix-replay.json
```

输出目录必须全新；重跑使用新目录。每周期证据立即写出，即使中断也保留已执行
轨迹；完整运行另有汇总与环境/源码manifest。时间超限会改变控制分支，
有deadline miss时结果不能称为跨机器逐位确定性复现；物理执行轨迹可独立重放。

求解器使用机体系acceleration直接shooting、速度/几何/终态停止硬约束。
`Result.command`是下一个dt末的base_link速度提案，离线plant在这个dt内按
`Result.acceleration`连续爬升；实车能否实现该响应尚未辨识。
`model_feasible`只表示声明的模型/输入下通过约束，不等于物理安全认证。
`invalid_input_brake`没有预测支持；`unsafe_brake`表示连刹停路径都无模型证书。
离线10Hz循环另记录未来20Hz目标的50ms超期，0.5s开发预算不是实时接受门。

独立oracle用0.65×0.60m完整机械矩形、凸polygon精确距离和≤0.01s扫掠网格，
再扣除区间运动reserve。安全门为0.05m；物理接触与仅reserve下界为负分开记录。
`current_only/future_union`是消费机制消融，不是STVL/MPPI实现。没有配对B0安全
见证，`false_block_*`、ROS action success和recovery保持null。
CPU指标是进程计算需求，不包含完整ROS/定位/感知/串口负载。

后续Gazebo shadow、完整raw costmap支持和Nav2 Controller plugin有独立进入门；
本目录没有提供可误连实车的启动入口。暂不声称任何部署净收益。


## 当前实时切片复现

在本目录执行，已有NumPy 1.26.4 / SciPy 1.11.4 / pytest 7.4.4、C++20及Eigen3：

```bash
python3 -m pip install --target /tmp/mpc-deps --no-deps osqp==1.0.5
bash frontend/build.sh /tmp/mpc-tdt-frontend
PYTHONPATH=/tmp/mpc-deps OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q
PYTHONPATH=/tmp/mpc-deps OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 run_realtime.py --frontend /tmp/mpc-tdt-frontend --output /tmp/mpc-qp-new
PYTHONPATH=/tmp/mpc-deps OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 run_realtime.py --verify-only --output /tmp/mpc-qp-new
```

`realtime_qp.py`只解决固定yaw、wz=0的全向平移；需要转动时请求MPPI。
20Hz执行、1.5s horizon、15节点、15ms solver/40ms core预算、400迭代上限、
最多4个相关动态障碍。每次返回前重验整条轨迹及全量tracks。失败先重新验收
移位的旧序列，再提供有界制动，并记录FollowPathMPPI回退请求。离线runner
**不执行回退交接**，所以此矩阵属于MPC shadow核心验证，不是完整策略闭环。
静态map由已迁移T-DT生成简化路径和认证走廊；静态搜索在控制周期外。

`run_realtime.py`的control_elapsed计入局部参考采样、输入预测、QP组装/求解、
几何验收；planning另记。Linux调度和接口阻塞需要真实wall watchdog，不用
离线host计时声称硬实时。5,659周期全部小于50ms，但迎面接触及停滞保留。
70项最终测试包含实际T-DT静态绕行/unknown封路、solver fault/timeout、上一条
可行解新输入重验、以及双向selector及故障保持。OSQP缺失时该模块测试skip；
验收必须安装realtime依赖并确认无skip。

正式Nav2双插件加载、实际跟踪消息/TF/raw-grid接入、Gazebo及实车、MPPI
handoff均未完成；[集成边界](integration/README.md)明确下一阶段的验收门。

已保存的QP图可由`python3 plot_realtime_evidence.py /tmp/mpc-qp-new`重建（需要matplotlib，非控制运行依赖）。


2026-10-04继续轮已实现隔离 Humble 原生双插件和异步 QP，运行命令、约束与
尚未完成的真实输入/部署门见 [integration/README.md](integration/README.md)。
独立主线派生分支和默认src/正式配置保持；历史离线碰撞与失败证据不覆盖。
