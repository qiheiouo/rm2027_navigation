# 相位 6 首次启动失败与固定环境修正（2026-09-30）

[原预登记](phase6_control_holdout_preregistration_20260930.md)提交 `d8e2765` 的第一次容器尝试已原样保留于 `build/dynamic_prediction_phase6_holdout_20260930/candidate_navfn_1/`。容器退出码 2，观察器在发送导航目标前直接拒绝：`observe_dynamic.py: error: use the documented isolated localhost simulation domain 174`。该脚本的运行环境误设 `ROS_DOMAIN_ID=178`；物理 pose 和 tracker prediction 文件均 **0 字节**、MPPI 周期 0 条，不能被解释成导航、预测或碰撞实验结果。原目录、计划、profile、日志和错误码不覆盖。

本次只将新运行脚本的 ROS 域改为观察器要求的 `174`；相位仍固定 **6 s**，Navfn+V1 原 profile、batch 300、镜像、单线程构建插件、障碍运动、目标、选周期规则和所有安全门均保持。先提交修正，再在**全新** `build/dynamic_prediction_phase6_holdout_20260930_setupfix/` 目录登记当前 HEAD 和所有哈希；如果此后获得物理/预测/目标数据，不再因任何轨迹或安全结果重跑。原“只尝试一次”的防挑选约束针对能产生评价数据的试次；这次尚未发目标的环境错误作为一次明确报告的设置失败，例外仅限这一行启动域修正。这与此前相位 2 的无目标设置失败处理口径一致，但不会把失败归档从记录中抹掉。

[失败尝试的文件哈希清单](evidence/phase6_startup_failure_20260930/manifest.json)保存了原计划、profile、容器和观察器日志、退出码及两个空数据文件的 SHA256；MPPI 周期文件数为 0。该清单在修正提交前生成，后续试次不会覆盖它。
