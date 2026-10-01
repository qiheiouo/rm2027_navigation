# Controller里程计输入契约实验

2026-10-02从`b89eb1d`建立`experiment/controller-odom-contract`。
前序SDK数值复现通过但349次native速度输入全0，295个canonical源
样本有运动。实际controller inactive配置读回默认`odom`和`/odom`
订阅；canonical发布在`/odometry/lio`。有效首速度契约失败，前序
完整三秒反事实只对原生零输入上下文有效，物理见证尚未建立。

## 执行前边界

新增显式`nav2_cv_controller_odom.yaml`只比native snapshot profile
多一个`controller_server.ros__parameters.odom_topic: /odometry/lio`。
不改原七critic、采样器、SG、优化器、Dynamic/Static、tracker、guard、
footprint、场景和速度界；不发布或复制里程计，不增TF owner，不用
额外remap绕过正确参数层级。旧profile与历史manifest保持。

driver新增可选明确要求的controller odom topic：lifecycle配置完成后
读回实际参数及controller实际nav_msgs/Odometry订阅，核对canonical
发布者；不把constructor阶段未声明的参数当成已验证默认值。预检
失败不发送目标，保存基础设施失败；原cmd路由与goal/窗口/相位不变。

先在固定Humble1.1.20 inactive节点实际配置读回，确认新profile的
订阅是`/odometry/lio`且原profile仍是`/odom`。通过后执行预定同条件
完整物理试次并保留全部失败：实际body/全机械≥0.05m、padded>0、
raw203、bounds容差5e−5、严格推进与action goal success全部独立。
native捕获需在真实canonical运动时确有非零速度输入，保留threshold
处理/来源时刻差异，不能要求所有静止时刻非零或用单次非零掩盖丢失。

新SDK重建仍先完整输入/输出逐位核对，然后再验证有效原生速度下的
完整三秒、实际历史与独立几何见证。该参数修复不证明CV隐藏几何或
未来支持；旧CA/sampler/ranking保持冻结，main/feature不提升。所有
配置、readback/graph、源码/ELF身份、原始数据与失败门都单独归档。

## 参数路由预运行

仓库工具`test_controller_odom_preflight.py`在隔离DDS域顺序configure
两个实际Humble controller，均停在inactive。旧profile读回`odom`并
实际订阅`/odom`，按预定要求拒绝；新profile读回并实际订阅
`/odometry/lio`，canonical唯一Odometry发布者检查通过。夹具仅声明
publisher，未发布消息、TF、命令或目标；不据此推断有效速度输入。

最初临时脚本只结束ros2 wrapper，两个用相同名字的controller存在
进程/发现隔离问题，第二项断言失败。保留该失败后改为独立进程组
清理，并等待DDS发现收敛；临时脚本重试及仓库工具新执行均通过。
完整证据见`stage2_evidence/controller_odom_preflight/manifest.json`。

安装新YAML后项目critic、guard、命令观察器及原生MPPI/critics、
controller/nav2_util的七个ELF身份均与前序一致。配置语义比较只多
controller层odom_topic。driver在明确STARTUP完成后检查实际参数/
订阅/发布者，失败保存readback且不发送目标。下一步仍是固定物理
试次；所有机械、三秒见证、有效速度和任务门尚未在新配置下验收。
