# 现有shadow tracker的consumer trace离线转发门预注册

## 背景与假设

真实observer的TF失败不能推断tracker行为。现有tracker已使用0.08s timeout和2-thread executor，本项不修改它们。假设实验目录中的最小trace包装可以记录实际scan callback entry、lookup参数/返回/异常及原发布消息，同时完整转发现有实现。通过此接口门后才能另行考虑隔离shadow采集；本项不是新的预测算法。

## 方法与固定数据

继承现有DynamicObstacleTrackerNode，scan入口记录后调用super；包装本节点private Buffer lookup_transform，完全转发原args/kwargs，返回同一对象或重新抛出同一异常；包装三个原publisher，记录后调用原publish。不换Buffer、TF listener、callback groups、executor、topic/QoS、参数、tracker/KF或算法。不修改src、正式launch/config、TF合同或critic。

先提交本协议，开发trace与fixture；prepare固定现有node/core/config、安装Python模块、trace/fixture源码、镜像、参数及生成fixture消息字节，提交input envelope后再evaluate。固定七例：无map、空scan frame、零stamp、stale、future、有效TF正常扫描、TF异常。原生scan包含finite/inf/NaN，记录完整native字段身份；这些是typed消息重序列化CDR，必须标注不是原始DDS bytes。callback、lookup及publish时钟分别标注本consumer入口/调用，不冒称observer。

fixture在network none的固定Humble容器中实例化原node与trace node，不spin、不运行DDS场景；两边同样替换clock/TF provider及最终publisher为本进程sink，调用原_on_map/_on_scan。以同一7s ROS clock及原参数0.08/.4/.05运行七例；正常例free静态测试map、固定7s scan和固定planar transform，不使用未来真值。所有application输出由sink接收，不发送DDS；ROS内置日志/参数机制保留。此门只验证包装，不测试async TF等待。

## 判据与限制

每例原node与trace node的prediction/markers/diagnostics完整native字段及状态必须相同；diagnostics中原耗时字段明确作为非确定测量独立保留、不纳入语义相等门。processing stamp在固定clock下必须相等，header/frame/schema/authority、全部轨迹/geometry/value字段不得排除。全量native fields记录使用明确数字位模式或规范形式，无padding跨进程字节一致假设。

扫描记录须一条entry/exit、完整native字段身份、相同header；TF只在实际调用时记录，args/kwargs/frame/source stamp/timeout必须与baseline相同，返回对象/异常身份必须相同。publisher日志数量/内容与真实sink相同；手动异常lookup转发负对照必须重新抛同一异常。损坏trace或fixture身份应拒绝。任何门失败即保存/停止，不调runtime参数或放宽判据。

时钟及日志开销会改变运行时序，因此转发语义门不能证明零扰动、线上receipt、等待成功、预测效果或MPPI安全。后续真实consumer capture须新预注册，将trace开销和typed-vs-raw身份限制写清；不拼接旧capture或回填旧receipt。结果/负例/commit/input/params/image/module hashes全部保存docs/dynamic_navigation，main不改、不merge/push/deploy。

## 评价前冻结登记

源码 `dcd0abb637b6c0682ec4319988493f3895285e62`；input SHA `733e10871783107b84714ea83c3bac0c42187fc74b777e13c5fa54035bee5212`；参数SHA `3f83685374bbf40ea282be22360d31a5b86858a4b24611dc387f99b5f92007a7`。21个CDR输入冻结。[全部输入](evidence/consumer_trace_fixture_inputs_20261001/manifest.json)先提交再evaluate。fixture实例化原node constructor默认参数；原YAML仅保存来源身份，未加载，0.08/.4/.05三项与它一致。明确耗时字段为 `latency_ms`，除此之外所有消息字段保留比较；正常例必须产生非空track几何输出，不以空输出通过。

## 首次失败后的修复限定（新prepare前）

首次评价前三例通过，第四例baseline固定clock的SYSTEM_TIME/ROS_TIME不匹配中断，见 [失败记录](consumer_trace_fixture_clock_failure_20261001.md)。仅修正fixture显式ROS_TIME，新增prepare reuse旧733e108 envelope内全部21CDR字节，源参数与七例不变；不重新序列化输入。旧native descriptor注明为pre-serialization，另冻结解码后全字段身份并在evaluate校验，处理float32 wire量化，不放宽CDR或语义门。新源码/input/params镜像身份提交后才能重新evaluate，旧部分结果与FAIL保留。

## 修复版本评价前登记

源码 `2d7cbc504e5f54570e3737b04ff2e2ee04e35996`；新input SHA `7b3c339b8b808888f809dd2347ee7a4c3b15236ca565e703c3547469b180b633`，参数SHA `5fbf55a53c851a459e044cc185709699e46e5a43d08059be9662421ab7a604b7`。[修复输入](evidence/consumer_trace_fixture_inputs_v2_20261001/manifest.json)全部21CDR SHA与旧输入一致，14项pre-native和解码native不同（float32量化），两种身份明确分开冻结；不是修改输入数值。lookup记录增加原Time clock_type身份，转发门保留。新input提交后才evaluate，原FAIL不覆盖。
