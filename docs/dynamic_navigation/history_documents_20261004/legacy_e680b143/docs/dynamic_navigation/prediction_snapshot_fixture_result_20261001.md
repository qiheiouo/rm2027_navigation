# V1实际prediction快照：隔离副本离线结果

## 背景与假设

现有cycle trace无法确认V1实际选择的latest prediction。按 [预注册](prediction_snapshot_trace_preregistration_20261001.md)，只在独立build源码副本增加同cache消息的typed CDR、callback ROS/steady receipt及原score实际使用age，验证其是否保持原评分。原V1源码与安装均未覆盖。

## 方法与数据

固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。源码提交 `da1d700169a4a82f499bb49d2f8a50df0801f4ff`；评价提交 `c2720b928b54d64ab4a1f1185808c69617ff56d9`。输入SHA `4bfd43e383d56ce336e36489891286cebcd6e9299d86d8c2fcbe1f1d6207f947`，参数SHA `3ad8cfaf0bdc7d402ae436f538784be31e5d3b153caaa7df0243e37329b2c2dd`。

worker SHA `01a095ae36a7f0f2c659c6bc5657ed3bb1b5ad23267feace705a40a25f4d090b`；原V1库SHA `7da70180f7272ffc4a02e85b7f4b3dc613297ea6bb8027456ab0feaadb595010`，副本库 `cca4de6c4bd9d1be0d41b11ce1e9c43551cc2f7e49e9debe7fc90c71d1cef7dc`。副本单包及worker单线程Release构建exit0，源码删去追加块可恢复原CPP全部字节。

原周期47的300×30 CPM1/CTR1完整字节、costmap及八顶点padded footprint保持；worker执行原footprint核对。ROS_TIME固定7s，普通source6.9s。仅隔离V1评分，12条合成接口路径各在original/copy两个独立进程运行；无导航、控制、TF或未来truth。两组capture先归档后评价，196文件全部SHA核验。

## 结果

12/12路径全部通过；24进程return0，原版与副本全部300评分及fail记录逐字节一致。writer均attempted=written=1、dropped=errors=0、closed=true。每进程实际加载controller/V1唯一库身份符合冻结记录。

| 路径 | 原版/副本成本范围 | 实际记录age |
|---|---:|---:|
| missing / disabled | 0 | 未计算 |
| collision | 全部1666.6666259765625 | 0.1s |
| clear | 0 | 0.1s |
| uniform_overlap | 1927.9376220703125–1947.7525634765625 | 0.1s |
| schema / authority / incomplete / frame拒绝 | 0 | 原guard前未计算 |
| stale / future | 0 | 1.0s / -0.1s |
| no_confirmed | 0 | 0.1s |

所有present快照typed CDR解码后与冻结sole输入的全部native字段一致；source/processing/frame/schema/authority/complete/tracks及同cache callback ROS/steady receipt匹配。missing正确无payload/receipt，disabled保持原early-return、无snapshot/age；无telemetry异常。summary SHA `27c7a6739dd74fa1bcfefddf410a73f5329091abecb9ad4de99f751847a67abb`。

完整证据：[源码](evidence/prediction_snapshot_trace_source_20261001/manifest.json)、[worker与构建](evidence/prediction_snapshot_worker_source_20261001/manifest.json)、[输入](evidence/prediction_snapshot_fixture_inputs_20261001/manifest.json)、[24次capture](evidence/prediction_snapshot_fixture_capture_20261001/manifest.json)、[评价](evidence/prediction_snapshot_fixture_result_20261001/manifest.json)。

## 限制

这是单一消息内容的接口与公式门。三次重复发送内容相同，不能证明异步多消息选择或未instrumented时序不变。typed CDR不是原DDS字节；序列化、锁和记录有开销，未验收性能。合成碰撞全饱和与uniform可排序属于既有行为，不证明预测有效、候选安全或MPPI输出安全。没有改运行配置、源时刻安全门、TF或physical尺寸。

原历史完整replay失败、所有风险组合否定结果和V1 Research Frozen结论保持。任意cycle filter history[4]仍未记录，因此还不能宣称完整输入replay。

## 下一步

先独立预注册control_history[4]的只读telemetry副本，验证原版/副本实际evalControl滤波及输出字节一致和记录身份。两项输入缺口补齐且独立协议明确后，才考虑新的研究容器完整周期capture；本次不启动完整控制场景。
