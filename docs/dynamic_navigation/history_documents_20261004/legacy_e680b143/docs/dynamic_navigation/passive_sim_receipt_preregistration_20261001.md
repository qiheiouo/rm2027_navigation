# 新设备被动仿真 raw receipt / TF 前置门预注册

## 背景与假设

只读 inventory 已找到现有 Gazebo/Humble 组件，但未启动实际数据流。假设原仿真（关闭 Nav2/RViz）可以产生 canonical scan/odom，并使 observer 在扫描 callback 当时按源时间查询 TF。此门只检查新设备输入链，不把 observer receipt 当成 tracker/controller receipt，不作预测、MPPI或安全结论。

## 方法与冻结

在当前研究分支先提交本协议，再实现独立监督脚本。固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，仅 source 新 workspace。prepare 固定脚本、原 launch/world/bridge/adapters/URDF/TF合同、实际安装 executable/module SHA、命令、环境和输入/参数 hash；input envelope 提交后才能启动。所有原资产只读，不改 world、采样率、gimbal placeholder、TF拓扑/所有者、footprint或正式配置。

原 `ros2 launch rm_simulation phase1_5_gazebo.launch.py headless:=true use_nav2:=false use_rviz:=false`。Docker network none、无设备/图形硬件挂载，2 CPU、6 GB memory、drop all capabilities；软件渲染 `LIBGL_ALWAYS_SOFTWARE=1`、`QT_QPA_PLATFORM=offscreen`，隔离 `IGN_PARTITION=rm2027_passive_tf_20261001`，ROS_DOMAIN_ID73 / ROS_LOCALHOST_ONLY1 / FastRTPS。没有 goal、速度 command、tracker、V1 critic 或 Nav2启动；原 chassis stub 的仿真输出按原代码保留。

先启动已有 raw recorder（30 s wall duration，use_sim_time，/scan，scan frame sim_lidar_link，target/prediction frame map），等待其 events 文件出现（最多5 s）再启动原 launch。启动后15 s抽样 ROS graph，在独立只读 node 中 spin 2 s，记录 node/topic/publisher endpoint身份和QoS；不能以此归因每个收到的TF边。记录进程 exit、完整 stdout/stderr、原始 raw CDR、callback clocks、TF字段/异常及 manifest。进程组 SIGINT，最多8 s后 SIGTERM，最多2 s后 SIGKILL；仅本次创建的进程组。监督超时和渲染失败全部作为失败保存，不重试或改参数凑通过。

## 判据、数据与限制

捕获完整 manifest 和原始文件 hash；clock须有至少两个不同正值，scan/odom须存在且 frame合同通过；graph无controller_server/planner_server/bt_navigator等导航节点。按预注册同时报告全部scan及启动后固定5 s的scan：源时刻正值、frame、source-after-observer、TF available/unavailable/invalid数量。严格源时刻TF门要求全部适用scan available；5 s子集单独报告，不替代全部门，不删除startup失败。扫描TF不能查询latest、等待未来包或补identity。

原始输入来自simulation ground-truth odometry，这是原sim定位来源；本阶段无动态预测器，因此没有将未来真值喂给预测。以后预测协议须重新划清线上可获得输入。若scan/render/TF不通过，停止并记录否定结果，再预注册诊断；不能为此调TF频率、改变frame/所有权或降低安全门。

本项只是一条被动设备前置记录，不是独立闭环成功，也不能修复旧image/capture gate。后续因果TF replay需另行冻结本次真实事件后独立验证，完整周期仍需要实际消费者与rollout/critic捕获。结果写入 docs/dynamic_navigation，旧阴性与证据完整保留，main不改、不merge/push/deploy。

## 启动前身份登记

监督脚本提交 `b5f1685f24c7ff13e7662b2b1101e5a19cdb3af4`；113项源/安装资产，input SHA `b670bdf7a89abf502cf1ce8d03cb1bc57f4a9199e2de46c4bcde9e2474be115b`，参数SHA `f98f132df965ed8cefb1af0cf2db6603676011cefbedecc4c261a641d4f75a19`。[完整输入](evidence/passive_sim_receipt_inputs_20261001/inputs.json)先提交，再运行。本次源代码包含原C++ adapter实现与安装二进制、installed launch/资源的身份；没有更改它们。
