# 冻结observer事件顺序的独立TF replay预注册

## 背景与假设

raw入口及DDS字节记录门通过，仍需证明离线重建保持当时可见TF，而不是把全录包的未来TF预先载入。假设只按冻结事件顺序填独立Buffer、按epoch清缓存，可以复现所有scan的准确源时刻查询成功/失败和返回全部字段。observer可见性仍不是实际消费者可见性，完整历史controller replay仍未通过。

## 固定数据与方法

使用已提交的raw结果全部两个capture：controlled 25事件 manifest SHA `3effa16ca065f5102b6c90b12f6e56b585aedff676f023d24d6a92c2887e02af`、DDS六事件 manifest SHA `37af4a751cafe5a566e2eaccd4cb36b7496b384ee623b6c8ab3d1cd28e353862`。不得筛选scan或补字段。每个manifest和文件/CDR hash先复核，新源码/parameter/input envelope在evaluate前冻结。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`不变。

实现独立离线reader，不调用Recorder.observe_raw，不创建ROS节点/订阅/publisher/client，不读取真值。读取原始CDR构建私有TF Buffer，按原事件全局序号处理；每个epoch开始新Buffer；只在原TF到达事件插入，所有scan用非零准确源stamp且timeout=0，无latest。无效source保留not_queried；TF不可用保留unavailable，异常文本不要求跨运行一致。

基线逐条核对全部query status、target/source/requested stamp、返回header/frame/child与translation/quaternion每个float64字段的位模式（忽略CDR alignment padding而不改它）；任何差异停止replay验收，保存failed gate和日志。原入口payload本身仍要求完整字节身份，不用字段门替代。原query实际steady耗时只是原观察记录，不伪造成此次replay耗时。

否定对照仅在独立新Buffer预装同epoch所有未来TF，再按原scan stamp查询；完整报告各scan与原记录的差异。该arm显式无效，不作预测候选或成功算法。预期controlled中原缺失的一些query被错误变为available，至少一条须暴露；DDS中若本来全部TF已先收到，差异可以为0，不要求制造失败。

## 判定、限制与下一步

只验收因果TF/消息身份子流程，不验收tracker内部receipt、pose/scan实时处理、不丢包、时延性能、local costmap/critic/聚合完整replay或动态安全。失败不以latest/未来TF补救。源数据、旧失败和evidence都保留，不改运行节点/TF合同/main/正式配置/安全门；结果、限制及后续最小实验写入docs。

## 评价前冻结登记

源码/prepare `c746f36b73ea1f77a75ec068beb6387cd87b5f76`；input SHA `9a478709c87ad91fab319ae58f196fda7f4e7b91c465c57a2a879c09a1f2414d`，参数SHA `a8683afa713a330883590f18711f7ee8333dca3ef09c7f42108bc853feaa25af`。两份manifest与所有入口/返回payload hash已复核；[输入envelope和日志](evidence/replay_event_tf_inputs_20261001/manifest.json)先提交，尚未evaluate。
