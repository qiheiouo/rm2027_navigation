# 两telemetry副本组合加载的累计评分门预注册

## 背景与假设

V1 actual snapshot 12/12门及history 16/16门分别通过，但不同overlay单项通过不能直接证明组合加载/ABI或累计评分一致。假设同固定镜像内组合history controller/critics与snapshot V1，不改变同输入的七标准critic加V1累计分数、fail和消息记录身份。

## 方法

不改任何C++源码、不重构评分或新建controller。复用已验证的prediction_snapshot_worker二进制与既有12条sole-message fixture的CDR/decoded字段、原300×30 map/rollout/control及八顶点padded footprint全部字节。仅在新独立fixture元数据中将critic列表设为原七标准critic顺序加PredictionV1Critic；所有参数、输入age/物理尺寸/guard保留。不是运行配置改动。

原版role加载guarded原controller/critics、workspace原V1；组合role加载history overlay controller/critics与snapshot overlay V1。各用新独立进程、固定ROS_TIME7s、Domain73、网络与硬件隔离；不执行导航、TF或控制。worker不调用evalControl，history已由独立actual-filter门验证，此门不声称完整周期。

开发前提交本协议。prepare前核对两个通过summary、原输入envelope及相关source/library/worker身份；输入、参数、源commit、镜像、二进制和两个overlay顺序全部冻结并提交。两role全部capture/log先冻结后独立评价。

## 验收门

至少保持原12路径。全部300最终score字节、每个critic的300累计score blob及对应fail记录，在原版/组合间全部逐字节一致；最终score须等于最后critic blob。八critic都必须实际执行、无fail/exception；writer attempted=written=1、无drop/error并关闭。

三critical实际加载module路径必须唯一且分别对应冻结controller、critics、V1 SHA，不允许/opt同名混入。locked raw map/padded footprint/path/pose必须保持输入身份。组合V1 enabled/snapshot/typedCDR全部native字段、同cache source/frame/receipt/实际age保持旧12路径语义（guard之前不产生age，missing/disabled无payload对应行为）；不能用原版饱和分数代替snapshot身份。记录/identity缺失或任何字节变化均失败并保留，不以数值容差放宽此门。

## 数据、限制与下一步

同一输入和单一内容重复传送仅证明局部接口/计算，不能证明异步选择、实时性、预测效果或安全。七critic累计分数可能因V1全饱和仍无法排序，本协议不把字节一致当算法有效。原历史FAIL、risk否定结果、V1 freeze、main与正式配置/TF/安全门全部保持。

通过后另预注册新设备可移植near-X完整capture：必须登记旧/新路径绑定差异、原scene/model/profile字节、实际consumer receipt与V1/history/cycle数据完整性；新capture先冻结再replay。未通过本门，不启动该场景。

## 执行前冻结

实现/prepare提交 `e57119a7944c5b60c63f654282bcd560412a092a`；input SHA `7884873fbec5cab240012b6e75790d39572900f4193371125a5c9e81ce129332`，parameter SHA `65338333b5ddc6ed01461673e27a5ce27698f9b061197fb6cc245199fca6753f`。未重新编译任何C++。全部CDR/decoded字段及原map/control、sole-V1 golden、8critic元数据/两个已通过门及module顺序身份先冻结在 [evidence](evidence/combined_trace_fixture_inputs_20261001/manifest.json)。此外要求每role最后一次V1加法恰等于float32(第7累计+原sole-V1)，raw map blob与CPM1输入body全部字节一致。

两role各12进程均return0，完整196文件先核验冻结至 [capture evidence](evidence/combined_trace_fixture_capture_20261001/manifest.json)。original/combined capture manifest SHA分别 `7efb7b6c71c528fa9c8883490f3cae6f8a416082701918ceca51f16d06a9b4af` / `43a57dec7675dfca3dd8e8b4a6a69b7c76ddbe2b8dfe5e51e3776b7968a010e7`；累计评分/快照评价尚未进行。
