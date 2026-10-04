# 冻结replay只读消息身份与接收证据记录器预注册

## 背景与假设

旧周期43–47有scan/pose源时间，却没有这些observer回调的receipt和完整pose frame，故只能条件化分析。此次假设是：在独立实验observer中保存源时间、回调入口时间、frame、原始CDR及源时刻TF查询，可以复现其自身的消息可见顺序，并准确识别迟到、重复、同stamp异payload、错误frame及ROS时间重置。

**observer receipt不能证明tracker/controller自身的receipt。** 两者是独立DDS消费者；记录器不补造旧字段，也不把observer时间当运行节点时间。未来需要实际消费者内部证据才能证明其在线可用性。此次不研究几何安全、风险权重或控制性能。

## 最小实现与合同边界

实现仅在 `experiments/dynamic_prediction_v1/frozen_cycle/`，standalone Python工具，不新增src包、不改正式launch/参数/消息定义。prediction的expected frame与TF target必须显式登记：公共合同是map，旧sim实验采用odom不能偷换。读取显式给定的LaserScan、规范 `/odometry/lio`、现有版本化prediction、`/clock`、`/tf`和`/tf_static`；与冻结controller cycle JSON/bin在离线阶段按源stamp和payload关联，暂不宣称完整历史capture replay。

每个回调入口先取steady/wall/ROS整数纳秒；事件包括本次run全局递增序号、ROS时间epoch、topic/type、header/processing stamp、frame/child frame、message CDR SHA与长度、可用DDS publisher GID。CDR原始字节按内容hash保存，事件重复仍逐条记录；目录必须新建，已有证据不覆盖。meta保存commit、源码/参数/镜像身份及观察者用途，结束manifest核对每个事件和payload。

TF订阅只填私有Buffer。scan查询显式实验target frame（登记为map或odom）`target <- header.frame_id` 的**准确非零源stamp**，timeout=0；当时失败保留异常，不查询latest、不合成identity、不等待未来TF回填。保存query时刻、返回transform原始CDR及frame/stamp；静态返回stamp为0与query源stamp分开记录。ROS时间回退后增epoch、清Buffer（含静态项；须再次接收），跨epoch相同源时间不会被合并。不创建任何应用publisher（ROS节点固有parameter事件不授予运行权限），没有TF、velocity、goal、action或diagnostic发布，无硬件与运动权限。相应TF/topic边界只在本实验文档说明，公共运行合同不变。

## 输入冻结和检验

先提交实现与固定夹具计划，再生成合成ROS消息的CDR/预期事件身份，登记input/parameter/source/commit/image SHA后evaluate。镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；网络隔离容器、独立ROS domain、无机器人/Gazebo/controller。

固定夹具包括正常scan/odom/prediction、无源时刻TF（保留失败）、时间0禁止latest、相同CDR重复、同源stamp异CDR、较旧源stamp后到、错误odom frame/child frame、未来source/processing stamp、ROS时间回退后同stamp，以及回退后静态/动态TF缺失与重新接收。TF source轨迹用两个不同translation样本，查询中间stamp必须返回插值结果，防止错误使用latest。将原生序列化/反序列化再序列化的CDR字节门、事件顺序与时间域、hash/缺损检测及TF行为分别报告。纯调用回调夹具与真实DDS接收烟测分开，不用合成时钟数据假装真实耗时。

失败停止该工具验收，保存失败日志和修复来源；测试必须验证证据含义而非仅镜像实现。输出不能证明运行节点收到消息、在线prediction有效或闭环动态安全。所有结果、限制与下一步写入 `docs/dynamic_navigation/`，main/正式运行配置/critic/tracker/TF发布者/安全门均不变。

## 评价前冻结登记

首次源码 `5c01c68` prepare因geometry消息x字段要求float而传入int失败，未冻结CDR；原参数目录和失败日志保留。修复源码 `05aaf96f84e79fb440719e0985134f5fa890267c` 明确转换为float；25个固定案例不变，新目录 `replay_receipt_fixture_inputs_v2_20261001/`，input SHA `acdddaa3ad749e29b9d9fd2140b8b744ed00d1c97d6c0bf4b83affcb2ca88d86`，参数SHA `ad7b44bbfe48cf5dea45800c2b1fd7e2206cd5df6b068b70e87095f3b0f0dd22`。尚未evaluate，[完整输入及失败prepare](evidence/replay_receipt_fixture_inputs_20261001/manifest.json)先提交。
