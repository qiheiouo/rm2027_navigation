# 无安全候选与旧决策执行研究

Research阶段；[结果报告](../../docs/research_history/stvl_lifecycle_hold_20261014.md)。不要进入实车或正式导航。依赖现存Humble镜像、原R4安装资产和a880只读工作区，不完整复制或重建ROS。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
# 无节点、无运动输出的原生异常探针
bash experiments/stvl_lifecycle_hold/build_probe.sh
# 各次应使用新HOLD_LABEL，已有目录不允许覆盖；保持至少3GiB
HOLD_LABEL=review_timing bash experiments/stvl_lifecycle_hold/run.sh timing
# reference、brake、brake_delayed同入口，均只有软件模拟设备
```

本轮5启动（1工程失败、4有效）已经全部运行。不要为归档重复启动。48s空场序列和3×9s C83执行回放不含在线MPPI/BT；与原闭环有明确区别。原behavior直接Spin脉冲没有重建，未增加速度发布者。

只读重算入口（在已有镜像内运行，网络none、只读主仓库，输出挂载本轮cache；详情参照run.sh的容器边界）：

```bash
python3 experiments/stvl_lifecycle_hold/history.py
python3 experiments/stvl_lifecycle_hold/fixture_metrics.py
python3 experiments/stvl_lifecycle_hold/application.py
python3 experiments/stvl_lifecycle_hold/deadline.py
python3 experiments/stvl_lifecycle_hold/candidate_matrix.py
python3 experiments/stvl_lifecycle_hold/figures.py
```

`pack.py`仅精选本轮文件，三个大型派生历史表无损压缩；`verify.py`在临时目录解包、校验归档成员并从物理原始数据重算全部5尝试的核心指标，两个制动组同时重算1ms target变化。不含的空场/reference native记录不会被伪称已复算。`compress.py`无损转换本轮completed streams，逐字节验证，保留原manifest；已有gz不能当作删除唯一数据的理由。

输入合同：冻结控制生成/发布计划、缺周期/迟到标记、原障碍运动、原物理模型和smoother参数。输出：上游Twist、20Hz平滑、stub消息、1mslimited target、物理odom、真值及接触事件。标准Twist无header；用DDS双时钟和已有cycle关联，重复同值命令不伪造唯一应用身份。真值仅物理驱动及离线指标，夹具不进行未来真值导航。

安全评价：动态机械bbox净空、墙重叠、真实Gazebo轮子接触；contact消息数不是独立碰撞次数。body near-stop要求平移速率及yaw<0.02持续6个20ms样本；停车距离为逻辑锚点至近停位置的直线位移，路径长度另外记录。reference锚点不代表该时刻实际发出了停止请求。prefix 25样本依据protocol容差；无目标成功率评价。

`evidence/execution_selected.tar.gz`含每次全部输入、物理观察、真值、事件、日志、SDF、参数，另含两个制动组完整native。`archive_manifest.json`列成员校验和。完整5次流（含失败）保留在 `build/lifecycle_cache`，未push，唯一未纳入Git的1ms时序数据必须继续保存。构建small_build可以重建，原始时序数据不能保证重新运行得到同样字节。
