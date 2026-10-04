# V1实际prediction快照telemetry隔离副本预注册

## 背景与假设

链接门通过，但当前V1没有保存实际score用的latest消息/receipt，旁路数据及饱和分数不能反推输入。假设只读telemetry可以在不改变公式、cache选择、参数或输出的条件下，把actual snapshot保存进当前cycle trace。原V1及既有库不改；完整周期运行暂不推进。

## 副本与方法

先提交本协议。在独立build root复制当前冻结V1源码/package/geometry与已验证guarded接口，记录原文件SHA、生成脚本、完整copy和精确diff。保持原package/plugin/class/参数命名，不改score公式、source age门、物理尺寸、footprint、rank mode、track过滤、权重或TF。副本只追加：callback入口ROS/steady receipt、同一typed消息的一次完整CDR序列化缓存；与latest指针在同一mutex下关联；score复制原latest指针时同时取关联telemetry，保存present/error/receipt及typed CDR blob；在原age计算处记录实际使用的age。声明typed CDR不是原DDS字节。记录失败不改变原输入选择或输出，缺记录必须使后续capture身份门失败。

使用现有tdt_trace接口，active cycle内记录；score之外不创建logger节点/TF/publisher或改topic/QoS/线程。新overlay仅写独立build/install，不覆盖原source、workspace/guarded/audited安装；单线程编译、固定镜像81b325。即使build通过，未获离线语义门前不得用于sim/controller运行。

## 离线门与冻结

离线worker复用原冻结300×30 map/poses、8顶点padded footprint和原参数输入，构造固定ROS clock和sole frozen prediction消息；不使用未来truth。先冻结元数据、CDR、参数及原版/副本worker和library/source/image hash，提交后评价。插件实例只在隔离fixture里initialize/score，不运行导航或发布控制/TF；prediction输入仅本进程/隔离domain fixture传送。

至少覆盖missing、confirmed collision、clear、uniform overlap、schema/authority/incomplete/frame拒绝、stale/future、no confirmed、disabled等独立接口路径；case参数先登记，不依据结果调整。clock必须显式ROS_TIME，age固定；accepted collision/overlap须产生非零成本，clear/reject按原行为核对。原版与副本所有300个score/fail输出必须逐字节相同；telemetry CDR解码后与sole输入全部字段一致，receipt/frame/stamp/实际age/错误和缺失路径对应正确，原costmap/footprint门保持。

标准七critic/聚合/概率/滤波已有独立门，本次不声称其新runtime已验收；后续history快照仍需独立预注册/验证，再核对完整输出。telemetry源码删除指定追加块后必须还原原文件全部字节，CMake仅最小链接/依赖补充，接口无额外控制权。

## 限制与下一步

序列化/锁/写入开销改变运行时序，不能证明零开销、并发包选择不变或未instrumented性能。单消息离线fixture仅证明公式/接口转发和snapshot身份，不证明真实异步场景/预测效果/MPPI安全。任何byte/identity/footprint门失败即保存/停止，不放宽安全判据或直接改线上critic。若通过，只允许另行预注册history及完整研究capture；原V1 Research Frozen结论、历史FAIL、main/正式配置/TF/安全门全部保留。

## 副本构建前登记

生成器提交 `2bafd817c7a31953af5cf981903cbadaf6dadaa7`；原CPP SHA `767ffc49d8740d1c30f53d84d783b9e51331cf94e9b863781ae3666d44b2ff1f`，副本 `c3471603ebc344132ee129ba5b5078f782ef700bca41209c323fa91140e667d2`。仅src/prediction_critic.cpp变化，geometry/package/CMake及测试字节不变；去除指定telemetry块还原原CPP全字节门通过。[完整副本/diff/manifest](evidence/prediction_snapshot_trace_source_20261001/manifest.json)先提交。构建在独立 `prediction_snapshot_trace_overlay_20261001`，固定镜像81b325、workspace+guarded setup、MAKEFLAGS=-j1、sequential/parallel-workers1、Release(-O3 -DNDEBUG)、BUILD_TESTING=OFF，原安装不覆盖；编译并不允许运行场景或宣称语义通过。

## 隔离构建与worker源码冻结

V1副本单包构建exit0（19.5s），库SHA `cca4de6c4bd9d1be0d41b11ce1e9c43551cc2f7e49e9debe7fc90c71d1cef7dc`，不代表语义门通过。固定clock/sole-message worker源码SHA `23fa02ba892a250faf589dee2af5a56b7c25bc0f5d1278353c3fb8f67b44907d`，来自既有冻结scorer，完整源码/CMake/生成manifest和构建日志先保存于 [evidence](evidence/prediction_snapshot_worker_source_20261001/manifest.json)。worker同样只在独立目录、固定镜像与Release单线程构建。

## Sole-message fixture实现与冻结前参数

worker已在固定镜像单线程Release构建exit0，SHA `01a095ae36a7f0f2c659c6bc5657ed3bb1b5ad23267feace705a40a25f4d090b`。`prediction_snapshot_fixture.py`先提交，再prepare并冻结输入：固定ROS_TIME=7s，普通source=6.9s、stale=6s、future=7.1s，processing=7s；collision在原robot pose、clear中心(100,100)，confirmed=2，size(.45,.55)，15个常值预测中心。12路径为missing/collision/clear/uniform_overlap/bad_schema/bad_authority/incomplete/bad_frame/stale/future/no_confirmed/disabled；仅uniform路径选已有uniform_center_overlap，禁用路径仍发送同一消息以验证early-return。全部物理与age/horizon参数沿用原V1默认，case成本须遵守预登记positive或zero预期，不依结果调参。

原版/副本各自新进程、隔离Domain73、同一worker；先冻结两组完整输出/trace/log/实际加载库路径，再独立evaluate。逐字节score/fail、writer完整性、ROS clock、全字段typed snapshot/receipt及原guard位置对应age全部必须通过。fixture只选择V1以隔离语义检查，不用于运行配置或安全结论；原costmap/footprint门原样执行。

输入已冻结：源码 `da1d700169a4a82f499bb49d2f8a50df0801f4ff`，input SHA `4bfd43e383d56ce336e36489891286cebcd6e9299d86d8c2fcbe1f1d6207f947`，parameter SHA `3ad8cfaf0bdc7d402ae436f538784be31e5d3b153caaa7df0243e37329b2c2dd`；12路径CDR/meta/native字段、原map/control及buildcache/log已完整保存于 [冻结evidence](evidence/prediction_snapshot_fixture_inputs_20261001/manifest.json)。评价前先执行并归档两role输出。

两role各12进程return0，仅表示fixture完成；全部trace/score/ROS日志已在 [capture evidence](evidence/prediction_snapshot_fixture_capture_20261001/manifest.json)逐文件SHA核验并先冻结。original/copy capture manifest SHA分别 `55533d584fdc9028852a1e9fe535682d56730884c330becdf6b66f786a7492ee` / `cbedff6fa0130294bee482c7f69986efe0989afb968876a8295c4884cb7b2dd3`，评价尚未执行。
