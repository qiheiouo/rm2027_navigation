# R4 HWS-style prediction consumption — offline core

隔离研究核心，从 main 新分支实现。`COLCON_IGNORE` 防止正式 colcon 发现；没有 ROS publisher、Nav2 插件、正式 launch/YAML 修改或实车部署。

已实现：

- 冻结 tracker 的精确 detection→track 输出钩子；同次聚类的 source member IDs → centroid-local 0.05m observed raster。未确认/静止物体保留，coasting 保留最后关联形状。
- v2 CV 与实验 sidecar 的 frame/schema/完整性、源时刻、观测400ms、状态/最后发送命令150ms、source-order/jump/成员预算校验。不可由 size 补造 shape。
- 深层不可变 cycle snapshot；绝对 stage 时刻从 source anchor 补偿一次；同源预测不得热改，同拍无新 prediction 的整段 revalidation。
- fixed-yaw omni Follow：50ms ZOH 执行、100ms 15节点决策、自由弧长进度、contour/lag/速度投影、命令rate/jerk、running soft dynamic residual、terminal=0。OSQP 1.0.5，一次局部 QP，400迭代、15ms solver、40ms总消费预算。
- 原机械半尺寸(.325,.300)+.03 padding；soft configuration-space 支持另外加.02 margin和每cell半径。fixed-yaw rectangle 的 world AABB 是保守 soft 近似，不证明隐藏完整体。
- 当前原始占用图的 padded footprint 连续50ms扫掠、unknown/inscribed/lethal≥253、静态版本撤销、75ms命令租约、唯一输出策略、WAIT保持/下拍恢复和无证书的有界Stop。`mppi` offer 使用同一接口，**尚未连接实际MPPI**。

代码入口是 `CycleConsumer.compute(...)`：获取一次输入→freeze→原有/新receipt检查→Follow求解→command offer。`OutputArbiter.tick(...)` 应由独立输出所有者调用，发布成功后才 `sent(output)`；失败周期撤销旧offer。计算过程不得直接发布。offline fault probe 在独立进程运行输出策略；这不等于 Nav2/实车独立输出验证。

`frontend.py` 原样复用冻结 R3。`PreparedRoute.from_frontend` 只复制其 raw-static certificate、弧长及corridor，不调用其按wall time推进的 reference。T-DT桥接在控制周期外执行，explicit executable/hash 写入证据。

```bash
# 在此目录；使用此前已有的隔离OSQP安装，不写正式ROS环境。
R4_TDT_BRIDGE=/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/build/temporal_mpc_timing_20261004/binaries/frontend \
PYTHONPATH=/tmp/temporal_mpc_deps_20261004:. python3 -m pytest -q

PYTHONPATH=/tmp/temporal_mpc_deps_20261004:. python3 tools/validate_core.py \
  --frontend /home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/build/temporal_mpc_timing_20261004/binaries/frontend \
  --output evidence/core_checks_new_run.json
```

没有T-DT executable时pytest跳过唯一实际桥接检查；本次执行有指定 executable，未跳过。跨机器应在冻结来源构建相同桥接并记录hash，不把本机路径当已安装依赖。

与HWS的刻意差异：本切片使用最新关联raster+coasting保留，不加入HWS形状历史union/0.8 decay；保留公共v2无衰减CV、1.5s/0.1s；真实机器人support进入soft field；求解是局部Gauss–Newton QP而非FDDP。命令target→rate变量的精确线性换元保持QP目标/可行域，并改善数值尺度。soft field使用8·exp(-16·max(clearance,0))，0.4m截断；内部满代价平台零梯度是已记录限制，未人为加escape方向。

输入是源时刻map-frame候选端点；本模块不替代LaserScan/点云静态剔除和source-time TF。实验sidecar目前为Python/JSON合同，未新增正式公共ROS消息。shape仅为已观测支持，缺失、合并、未见面和预测误差必须继续记录；可接受body净空≥0.05m、padded>0的physical oracle门仍待闭环检查。

本次证据只覆盖离线合同/数值/独立输出探针。100拍理想端点等待释放探针不使用Gazebo扫描、完整体oracle或B0，不占五场景物理paired run，也不证明优于B0/R3。Nav2同步计算、真实sidecar发布、实际MPPI回退、五场景进入门/预登记/配对、continuous物理净空及最坏实时性仍未完成。

来源及开发记录见 [intake](../../docs/dynamic_navigation/r4_hws_prediction_consumption_intake.md)、[A02](../../docs/dynamic_navigation/r4_hws_prediction_consumption_progress.md)、[implementation](../../docs/dynamic_navigation/r4_hws_prediction_consumption_implementation.md)。不修改/重跑冻结R3；有限基础验证失败后按A01停止规则冻结复杂预测控制，不扩为R5/R6。
