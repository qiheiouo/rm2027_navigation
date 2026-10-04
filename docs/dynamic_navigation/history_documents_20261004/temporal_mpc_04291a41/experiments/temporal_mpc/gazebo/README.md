# Isolated actual Gazebo research

2026-10-04用户要求研究冻结；[冻结记录](../../../docs/dynamic_navigation/temporal_mpc_freeze_20261004.md)。
所有物理失败与原始证据保留，以下命令/下一步为历史资料，不启动新试次。

[登记](../../../docs/dynamic_navigation/temporal_mpc_gazebo_registration_20261004.md)、
[结果与限制](../../../docs/dynamic_navigation/temporal_mpc_gazebo_results_20261004.md)、
[进度](../../../docs/dynamic_navigation/temporal_mpc_progress.md)。
动态验收=false：上一轮横穿两策略接触；本轮共同保护四轮均超时，连续性门也未全部通过。
保留独立main派生分支与默认MPPI。

在已有固定Humble容器中挂载worktree为`/workspace`，不使用host网络或设备：

```bash
bash /workspace/experiments/temporal_mpc/gazebo/build.sh
bash /workspace/experiments/temporal_mpc/gazebo/run.sh /workspace/build/mpc-gazebo-shadow-new shadow crossing
bash /workspace/experiments/temporal_mpc/gazebo/run.sh /workspace/build/mpc-calibration-new calibration actuator
# 完成audit_ros_capture、audit_run、calibration_audit及联合entry_gate后才能传入MPC模式。
bash /workspace/experiments/temporal_mpc/gazebo/run.sh /workspace/build/mpc-gazebo-candidate-new mpc crossing /absolute/entry_gate.json
```

run目录须全新，脚本拒绝覆盖。源码在启动前打包；所有动态真值仅记录。
`simulation.launch.py`复用主线description/localization/chassis adapter，实验固定map原点
取代动态identity占位stub；用真实测量odom，不用stub的指令反馈当测量。
`scene_node.py`只发布静态map、真实扫描端点云、T-DT规划Path，不发布预测/TF/速度。
冻结tracker只发布shadow预测；QP worker只发布proposal；两控制器同时在Controller
Server中，由既有标准BT/action选择。VelocitySmoother输出`/temporal_mpc/smoothed_cmd_vel`，共同ExecutionGuard是最终
`/cmd_vel`唯一publisher；无搜索/优化/oracle输入，只允许限差通过或减速。
`actor_target.py`只给独立物理PD joint target，不提供未来真值给控制。

只读审计：

```bash
python3 gazebo/audit_ros_capture.py /absolute/run  # Humble overlay：CDR和exact-source canonical TF
python3 gazebo/audit_run.py /absolute/run
python3 gazebo/audit_execution.py /absolute/run  # 精确回放共同保护输入及原生模型/净空拒绝
python3 gazebo/audit_model_occupancy.py /absolute/run  # 当前模型目标位置容差区/认证截面，拒绝覆盖
python3 gazebo/calibration_audit.py /absolute/calibration
python3 gazebo/entry_gate.py /absolute/shadow /absolute/calibration /absolute/entry_gate.json
python3 gazebo/static_frontend_audit.py /absolute/frontend /absolute/static-audit.json
```

运行前需要已有独立private Python deps和frontend binary，见integration/README.md。
`COLCON_IGNORE`确保正常工作区不自动引入实验overlay。shadow/scenario隔离，不是实车入口。
源码pin与license记录在ros2/tracker_intake.json和已有frontend/runtime dependency intake。

`audit_run`保留整轮测量域失败；`entry_gate`只依据校准+目标前状态批准开始实验，
不能宣称整轮部署接受。正式配对每场景一轮，内部MPPI噪声seed未固定。
物理oracle读取带epoch的model+link、投影完整机械3D外包体，绝不取tracker框；
50Hz距离和reserve只称诊断下界，连续误差/速度界未认证。Contact正消息证明接触，
零消息不单独证明绝无接触。结果、终止、接触、拒绝、output gap全部保留。

本轮执行模型/回退保护登记见[登记](../../../docs/dynamic_navigation/temporal_mpc_execution_registration_20261004.md)。
QP/native同用50ms held-velocity模型；T-DT SFC maxRange由2m到6m，仍经raw-cell完整足迹证书。
run.sh每个子进程独立process group以清理Gazebo后代；一轮一个全新容器避免旧clock。
输入进入门拒绝旧sim clock的轮次属于启动失败，不能算MPC性能试次。
保护层失流/异常仍限差减速，certificate仅指模型，不代表实际连续安全。
原来的未保护B0证据与本次共同保护B0分别保留，不混称同一基线。

[本轮结果与限制](../../../docs/dynamic_navigation/temporal_mpc_execution_results_20261004.md)：
4轮共同保护配对全部超时，无正接触消息；迎面cmd间隔和8故障fixture连续性门失败。
91项测试及原生14故障门通过，不等于动态接受。共同保护合成fault复现：
`bash /workspace/experiments/temporal_mpc/integration/run_guard_humble.sh /absolute/new-output`。
# 侧向参考继续轮

本轮登记见`docs/dynamic_navigation/temporal_mpc_lateral_registration_20261004.md`。
默认实验worker在现有T-DT认证矩形内产生侧移/通过/回原参考的局部偏好；仍只有
一个有界QP，所有硬约束保持。共同保护改为等价精确方格索引查询，新增生产端
monotonic与CPU字段，不将低CPU耗时或接收间隔误当硬实时证书。
每新run启动前生成`runtime_identity.json`，记录实际frontend/native二进制hash、
Python依赖版本及CPU affinity。仍每次新隔离容器、唯一输出目录、完整失败保存。
论文新线索只登记对照；未导入T-MPC++、Scenario或CBF实现，未改变正式v2消息含义。

v2投影独立物理复验见[结果](../../../docs/dynamic_navigation/temporal_mpc_projection_results_20261004.md)。
四轮控制源码/实际启动前二进制身份一致；横穿175/176、迎面62/63次MPC检查通过后
未来净空拒绝回退，任务全取消。横穿模型在所有目标后有效源采样排除固定yaw=0
位置容差区；只读工具不缩几何、不补造完整物体中心、不将当前空间当时域证书。
原始记录、失败与首版审计筛选修正均保存，动态接受仍false。

目标开放/有限候选继续轮见[登记](../../../docs/dynamic_navigation/temporal_mpc_candidates_registration_20261004.md)
和[结果](../../../docs/dynamic_navigation/temporal_mpc_candidates_results_20261004.md)。
可选TEMPORAL_MPC_FIXTURE_PROFILE=legacy（默认）/open_long；后者地图240×200、
origin(-1,-5)、目标(8.5,0)，T-DT corridor_range=10（原默认6，桥接有界[1,12]）。
TEMPORAL_MPC_STRATEGY=single（默认）/portfolio，后者共享15ms/390迭代预算，
单项5ms/130迭代、最多3局部参考，内部动态planning_buffer=.03m，原生硬门不变。

```bash
TEMPORAL_MPC_FIXTURE_PROFILE=open_long TEMPORAL_MPC_STRATEGY=single \
  bash /workspace/experiments/temporal_mpc/gazebo/run.sh /absolute/new-b0 shadow head_on
# 完成CDR、audit_run及校准联合进入门，另启全新禁网容器：
TEMPORAL_MPC_FIXTURE_PROFILE=open_long TEMPORAL_MPC_STRATEGY=portfolio \
  bash /workspace/experiments/temporal_mpc/gazebo/run.sh /absolute/new-portfolio mpc head_on /absolute/gate.json
python3 /workspace/experiments/temporal_mpc/gazebo/audit_reanchor_inputs.py /absolute/finished-run
python3 /workspace/experiments/temporal_mpc/gazebo/verify_static_capture.py /absolute/new-dds-fixture
```

原始静态TF采用可靠transient-local depth100，ready需确实记录两条必要静态边，
失败在固定phase前拒绝，不补TF。输入分解工具只适用于Gazebo event schema；
实际/shadow分开，旧测量替换是反事实，不能当传播后的plant或已执行轨迹。
六有效试次仍全部取消，另外保留缺静态TF失败基线；不形成部署接受。

时序/提案老化继续轮见[登记](../../../docs/dynamic_navigation/temporal_mpc_timing_registration_20261004.md)
和[结果](../../../docs/dynamic_navigation/temporal_mpc_timing_results_20261004.md)。
新实验worker_timing记录全部回调disposition、严格signed clock与monotonic身份，
原请求wire和100ms/非未来门不变。安装的Humble executor无MessageInfo，DDS时刻
明确null；不能把同主机总interval解释成唯一传输/排队时延。

```bash
# 新run采集结束后执行，只读且拒绝覆盖各自结果：
python3 /workspace/experiments/temporal_mpc/gazebo/audit_worker_timing.py /absolute/finished-run
python3 /workspace/experiments/temporal_mpc/gazebo/audit_proposal_age.py /absolute/finished-run /absolute/new-age-audit.json
# 实际DDS/worker受控clock工程检查；不是物理/检测接受：
python3 /workspace/experiments/temporal_mpc/integration/verify_worker_clock.py /absolute/new-clock-fixture /workspace/build/temporal_mpc_ros2/frontend
```

本轮双方strategy=portfolio匹配shadow负载，B0进入门通过后各一个新容器迎面run。
1,998请求均current，候选仅实际1个通过周期，shadow新状态未来净空拒绝后回退。
双方40s取消，候选最终命令接收gap95.474ms；保留失败，不追跑挑选。老化反事实
中的未修停止尾移位不是可执行提案，完整slack重放不等于连续物理证书。
