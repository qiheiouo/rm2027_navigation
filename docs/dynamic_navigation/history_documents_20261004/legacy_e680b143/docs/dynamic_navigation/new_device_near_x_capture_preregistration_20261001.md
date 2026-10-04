# 新设备near-X完整输入capture预注册

## 背景与假设

V1快照、actual滤波history、组合八critic及可移植launch门已通过；原历史全量replay仍失败，旧/work runner不执行。假设在固定新设备研究环境中，同一actual latest prediction、锁内costmap、sampled rollout、八critic评分、softmax/聚合与actual history可完整冻结，为同设备逐周期replay提供充分输入。此阶段先采集/冻结，不评价预测、排序或安全有效性。

## 场景与所有权

只用已通过v2的七scene资产，input SHA `26afaa9431e99dd6c9026be50d00022345ab835d77734119fdf954cda1a4bc3d`，preflight summary SHA `23e0f33bc24dfb57f0c2d4d40981d91aa9dd08f27c99b4bcb36d759b7979387d`。world/model/profile全部原byte（profile d00f722e...），四launch仅已登记路径改变。robot start(5.25,1.10,0)，goal(5.25,-1.10,0)，X障碍spawn(4.9,0)、amplitude=.9、period=6。原phase参数6、release公式8*ceil((sim-6)/8)+6及原ready条件（sim>=16、scan/odom/map/wall/actor/action server）保持。

固定81b325镜像、network none、2CPU/6GB、cap-drop ALL，无硬件。使用localhost research Domain174，以保持既有goal observer的域门；旧诊断Domain73不用作新scene。overlay顺序workspace→guarded→history→snapshot，三个库SHA对应已通过组合门，不覆盖原安装/源码。

canonical TF仅map_odom_stub、lio_adapter、robot_state_publisher；Gazebo、tracker、记录器、goal工具、chassis均不增加TF。保持原bridge/scan adapter/launch层级和mock chassis，真实设备/比赛配置/main完全不动。

## 采集方法与有界控制

开发前提交协议。新独立supervisor复用原portable dynamic launch、原static_map.py（/prediction_v1/static_map，frame odom，原静态body）、已通过consumer_trace_probe。tracker使用原shadow YAML，仅沿用旧runner的CLI use_sim_time=true、map_topic=/prediction_v1/static_map、map_frame=odom、scan_topic=/scan、prediction.velocity_decay_tau=0；不调整算法、TF timeout=.08或其他参数。

goal接口工具复用原benchmark observer的readiness/phase、map pose构造和ComputePathToPose/NavigateToPose API，单独新有界run方法记录全部实际接受/结果/取消。一次goal后固定记录8模拟秒（wall上限90s），即使goal提前成功/失败仍保留该固定窗口；若届时未结束则取消。此8s是采集边界，不改变预测窗口或3s安全门。parent goal等待上限240s；额外原record_physical_tail保证3.2模拟秒（wall60s），仅作未来离线labels，不反馈预测或controller。整体wall上限约320s，失败/timeout全部保留，不为有利结果重复场景。

goal前只读核对controller FollowPath/相关速度门与controller_frequency、planner GridBased、local/global costmap geometry/参数、moving obstacle与tracker实际参数，和原YAML+明确CLI override一致；不加旧QP插件未声明参数。保存实际source/header时间、服务响应与差异。图检查canonical TF、单一tracker/static map与正确control链；控制节点在本研究sim中允许，硬件节点禁止。

raw observer复用已验证Recorder，只在新独立进程扩充OccupancyGrid静态map和三个Twist命令流：/cmd_vel_nav、/cmd_vel、/simulation/chassis/cmd_vel。同其余scan/odom/prediction/clock/tf/tf_static完整raw CDR、source/observer receipt及因果TF查询一起保存，声明observer receipt不属于消费者。consumer保存actual scan entry、源时刻lookup原参数/结果、prediction publish；V1副本保存同cache latest typed CDR/receipt/实际age；history副本保存actual滤波前后history。GZ pose transport单独完整记录，不能作为预测输入。

父进程在固定窗口和tail完成后写planned stop marker，raw observer自然关闭并记录原因；unexpected interruption或自身deadline不能标为完整成功。原consumer按已有SIGINT/finalize行为关闭；原有退出warning若出现照实保存。所有process group有界终止，trace目录不预创建，检查writer status。捕获过程及终止事件逐项保存，不用goal success替代输入/trace完整性。

## 输入与结果冻结

scene、profile、tracker/collector/goal/supervisor/helper source、原源码提交、已通过门/库、installed资产/overlay、源运行参数和env，input/parameter/image/binary hash必须先冻结提交。原observer helper仅在独立build复制并替换import路径，保留源/差异；新goal方法不同于旧长时间等待，不声称旧observer性能或旧run复现。未来truth记录器只输出独立文件。

采集后先将全部raw/typed/cycle/goal/parameter/graph/process/ROS/GZ stdout-stderr日志逐文件SHA冻结、commit与image写manifest并归档，之后才能开发/执行评价。磁盘不够只停止并保存，不删除旧实验/evidence。

## 后续固定选择与验收范围

后续独立replay协议固定选择goal之后按cycle id最先三个eligible cycle，不能依据安全标签或排名选取。eligible仅接口可重放条件：原300×30、iteration1、八critic及rollout/weights/update/filter/output唯一完整、无fallback/exception/fail、schema/frame/complete/age/finite confirmed V1 accepted、同cache receipt/CDR与history完整。所有其他cycle及拒绝原因保留，少于三个则报告输入门失败，不另挑有利run。

先核对实际库/原运行参数/TF所有者和全capture writer完整性，再确认consumer→V1 native消息身份与因果receipt、锁内map、候选姿态/评分、权重/聚合/滤波/returned control。同capture三个cycle共享输入环境，不是三次独立成功。旁路命令流无header，若重复/歧义不能冒充实际应用控制receipt；端到端tracker map缓存与velocity smoother内部状态不在本次MPPI replay声明中。

此协议不把完整采集或replay成功当安全/预测有效，不改变body gap>=.05、padded gap>0、raw CostCritic阈值203或full30-step3s判据。此前所有否定结果、V1 Research Frozen、历史FAIL、main/正式配置/TF契约保持。

## 下一步

严格冻结新capture后，另预注册首三个cycle同设备评分/聚合/滤波replay。通过后才能用实际receipt/frame数据研究在线风险排序及输出一致性；任何方向否定照实记录，不回调batch/std/temperature/window/权重求成功。
