# 独立 Navfn+V1 相位 2 采集预登记（2026-09-29）

## 目的与固定输入

本试次只补充一个此前未按结果挑选的闭环输入，用于检验旧周期 162 的预测输入、候选排序与控制聚合结论能否延续。试次相位在启动前固定为 **2 s**，与此前分析过的 0 s、4 s 归档不同；只运行一次，结果无论碰撞、恢复、通过或启动失败都保留。它不是跨场景稳定性统计，更不是部署验收。

固定使用历史 `candidate_navfn_1/profile.yaml`（SHA256 `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`）：Navfn、原 V1 Prediction critic、MPPI batch 300、30 步、0.1 s 步长及原噪声、温度、足迹、padding 和其余安全门。仿真为现有 Phase 1.5D 的 0.45×0.55 m 移动箱体与参考八边形机器人；不连接实车。旧归档的 52 个文件在准备阶段按原 manifest 重验。保留 tracker 与 T-DT 源码、原运行插件及比赛配置不变。

运行镜像固定 `rm2027_navigation:dynamic-prediction-20260924`，镜像 ID 为 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6`。这不是旧试次已遗失的原镜像，而是当前设备已核对的镜像。运行时将当前工作树只读挂载，采集输出单独写入忽略的 `build/`，禁网、限制 2 CPU、6 GiB RAM、8 GiB RAM+swap、256 进程、关闭额外权限；`ROS_DOMAIN_ID=176`，`IGN_PARTITION=dynamic_prediction_phase2_holdout_20260929`。单线程构建，未用多线程编译。

## 当前构建和验证基线

可执行源来自分支 `codex/dynamic-differential-risk` 的提交 `148a3a59e0f7936a5c06bd65a8ec2309306c0aef`。按当前 `prepare_trace.py` 和 Nav2 1.1.20 固定源码生成 82 个插桩源文件，`trace_sources.json` SHA256 为 `5b172004c82a3cdb07f6cb32b38eb9547977fd7c9df57110c38a18bcfe03de99`。在该镜像中使用 `colcon --executor sequential --parallel-workers 1`、`MAKEFLAGS=-j1`、`CMAKE_BUILD_PARALLEL_LEVEL=1` 重新构建 7 个运行包及 2 个插桩包，均成功；新库的 `ldd -r` 未见缺失库或未定义符号，包查找指向此次新构建目录。

MPPI 和 V1 critic 的 76 项 gtest 均通过。测试套件整体不能标成通过：生成的插桩 Nav2 源码有 xmllint 1、uncrustify 4、cpplint 104 项失败；它们与功能测试分开记录，没有据此改动上游源码。准备脚本还会检查当前提交与上述编译提交间运行源码无变化，核对生成源码、原 V1 critic 源码、镜像、profile、实际二进制和试验入口的 SHA256。准备时写出 `plan.json`；启动时再次逐项核对，已有 container ID 或退出记录时拒绝再次启动。

## 固定分析规则

1. 保存所有 MPPI 周期、原始局部代价地图、机器人位姿与速度、tracker 消息及 source timestamp、路径、初始控制序列、rollout 和逐 critic 分数，并保留 Gazebo 箱体与八边形真值。检查每一项在实际归档中的完整度，不能以记录器消息代替控制器消费帧。
2. 以采样真实八边形对物理箱体的**本体**间隙判主事件，原门为 0.05 m。若首次低于此门，选择至少早于该事件 0.25 s 的最后一个完整且被控制器接受的 MPPI 周期；否则选至少早于全程最小采样间隙 0.25 s 的完整接受周期。若无合格周期，明确写成无法按规则选取，不能换有利周期。
3. 对选中周期独立复核 300 条逐条滤波候选：真实/预测最小间隙、首次预测冲突时间、V1 及主要原生 critic 分数、候选权重、聚合与滤波输出、原 raw local costmap 静态 CostCritic，以及完整控制序列与目标推进。按“输入错误、无安全候选、有安全候选但排序或聚合失败、未复现这些失败”分类，不以一次通过认定稳定。
4. 如果状态覆盖足够，再把此试次作为独立**输入诊断**，检查源时刻机器人在箱体哪一侧、箱体纵向位置与速度、换向附近九步预测残差。此前的 Navfn/T-DT 记录已参与方案选择，这条新数据保持留出，不用来选核宽、相位窗口或 critic 权重；若本相位未覆盖关键状态，保留缺口，不为补覆盖重跑同一相位。
5. 此次不改 batch、noise、horizon 或 critic 参数。只有该独立输入完成验收审计后，才对其冻结周期按 300/600/1000/2000 做离线同周期敏感性；结果仍需区分逐条安全候选与最终聚合控制，不能当四次独立闭环。

入口为 [phase2_holdout.py](../../experiments/dynamic_prediction_v1/frozen_cycle/phase2_holdout.py) 与 [run_phase2_holdout.sh](../../experiments/dynamic_prediction_v1/frozen_cycle/run_phase2_holdout.sh)。运行前还需提交此预登记和入口，随后将精确运行提交与所有哈希写入忽略的 `build/dynamic_prediction_phase2_holdout_20260929/plan.json`。原始大数据只在该 `build/` 目录，不进入 Git；结果摘要与核验哈希提交到 `docs/dynamic_navigation/`。
