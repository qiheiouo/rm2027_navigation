# Isolated actual Gazebo research

[登记](../../../docs/dynamic_navigation/temporal_mpc_gazebo_registration_20261004.md)、
[结果与限制](../../../docs/dynamic_navigation/temporal_mpc_gazebo_results_20261004.md)、
[进度](../../../docs/dynamic_navigation/temporal_mpc_progress.md)。
动态验收=false：横穿两策略接触，迎面两策略超时。保留独立main派生分支与默认MPPI。

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
Server中，由既有标准BT/action选择。VelocitySmoother是最终cmd publisher。
`actor_target.py`只给独立物理PD joint target，不提供未来真值给控制。

只读审计：

```bash
python3 gazebo/audit_ros_capture.py /absolute/run  # Humble overlay：CDR和exact-source canonical TF
python3 gazebo/audit_run.py /absolute/run
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
