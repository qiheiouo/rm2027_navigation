# 两telemetry副本组合加载：八critic离线结果

## 背景与假设

V1快照与filter history各自通过局部门，仍需确认组合overlay的实际加载及累计评分。按 [预注册](combined_trace_fixture_preregistration_20261001.md)，比较原版和组合的七标准critic加V1，同输入全字节一致才通过。

## 方法与数据

源码/prepare `e57119a7944c5b60c63f654282bcd560412a092a`；执行 `7a966393ce98d6f64c3e4476cd958691a81c00a1`；评价 `46ce2da04aad93e0716d0a08d43fc468acad388c`。input SHA `7884873fbec5cab240012b6e75790d39572900f4193371125a5c9e81ce129332`，parameter SHA `65338333b5ddc6ed01461673e27a5ce27698f9b061197fb6cc245199fca6753f`；固定81b325镜像，完整image/binary/source身份在input。

没有新C++修改或构建。复用已通过worker SHA `01a095ae36a7f0f2c659c6bc5657ed3bb1b5ad23267feace705a40a25f4d090b`，12条旧sole-message CDR/decoded字段及原300×30地图、轨迹、控制、八顶点padded footprint全部字节。新fixture仅将critic列表设为原七项加V1，参数及guard保持；不是运行配置改动。

原版加载guarded controller/critics与workspace V1；组合加载history controller/critics与snapshot V1。每role12新进程，固定ROS_TIME7s、Domain73，未调用evalControl、导航、TF或控制。196原始文件先SHA核验归档后评价。

## 结果

12/12路径、558个门全部通过：

- 24进程return0，writer attempted=written=1、无drop/error并完整关闭。
- 每进程controller、critics、V1实际maps路径唯一，SHA分别对应冻结原版/组合库；无/opt同名混入。
- 八个critic各300累计评分数组和最终评分原版/组合全部字节一致，八项fail均false；最终评分等于最后critic blob。
- 最后一次加法与float32(第七累计+原sole-V1 golden)逐字节相同，确保V1的既有增量保持。
- raw map blob与CPM1输入body全部字节一致；两role锁内map metadata、padded footprint、path、pose、speed、frame一致，原worker footprint门保持。
- 组合enabled、typed snapshot全部native字段、source/processing/frame/schema/authority/complete/tracks、同cache receipt和实际age符合原12路径语义。missing、disabled及原guard前无age行为保持，无telemetry异常。

summary SHA `ee456fa347ad7114088f7fac301a017417fd8d5fdc48f46a4f145e6e728bcaff`。完整证据：[输入](evidence/combined_trace_fixture_inputs_20261001/manifest.json)、[24次capture](evidence/combined_trace_fixture_capture_20261001/manifest.json)、[评价](evidence/combined_trace_fixture_result_20261001/manifest.json)。

## 限制

这只是同输入的局部接口、累计计算与动态加载门。没有完整evalControl周期、实际异步多消息选择或实时开销验证。字节一致不表示预测准确、风险可排序或输出安全，V1饱和行为和此前风险否定结果不变。history独立actual-filter门已通过，但不能把两项门与本组合门自动当完整capture replay。

原历史完整replay失败、V1 Research Frozen、main及正式源码/配置/TF/安全门保持。

## 下一步

先预注册七项可移植near-X场景资产的精确路径重绑定和只读launch依赖门。原evidence中的reference/comparison launch同样存在且符合manifest SHA；此前inventory只列五项，不能推断另外两项丢失。新设备协议不伪造旧/work安装或旧镜像。该门通过后，另预注册完整研究capture，先冻结数据再replay。
