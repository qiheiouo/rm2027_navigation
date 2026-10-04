# Typed消息重新序列化不能作为原始CDR身份

## 背景、假设与数据

[原预注册](replay_receipt_recorder_preregistration_20261001.md)要求先过CDR反序列化/重新序列化完整字节门，再检验记录顺序与DDS。首次prepare `5c01c68`严格float字段失败，修复源码/prepare `05aaf96f84e79fb440719e0985134f5fa890267c`；输入SHA `acdddaa3ad749e29b9d9fd2140b8b744ed00d1c97d6c0bf4b83affcb2ca88d86`、参数SHA `ad7b44bbfe48cf5dea45800c2b1fd7e2206cd5df6b068b70e87095f3b0f0dd22`。镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

## 结果与归因

评价提交 `08dbaf7`在第3个消息dynamic TF bracket的完整字节门停止；只记录2个受控事件，manifest标aborted，DDS烟测尚未执行。工具不能验收为原始消息字节记录器。

另进程只读完整25消息归因中，解码字段25/25一致，重新序列化18/25字节一致。先[追加预注册](replay_receipt_raw_cdr_preregistration_20261001.md)，再提交独立严格CDR字段游标 `b91e38b`，以原生反序列化字段逐项核对每个primitive、string、sequence、array及嵌套消息，并记录CDR1对齐padding。25/25完整字段解析与native核对通过；所有变化都位于明确padding偏移，无尾部未知编码。该进程重新序列化只有12/25字节一致，说明跨分配/进程的padding字节也不能假定确定。游标结果SHA `6b16beca3e2d5c64623feaf7b9f79c4536316cf3ab827dc91176b60ce3928dd4`。

**字段相同不等于原始CDR字节相同；原门仍失败。** typed callback后重新序列化只能记录新构造payload，不能代表DDS收到的原字节。停止该接口验收，没有风险、控制、安全或消息到达性能结论。

## 限制与下一步

字段游标只支持此冻结夹具用到的little-endian CDR1类型，未知类型/编码即停，不是通用CDR库。未修改ROS serializer或把padding清零；完整失败、两个归因进程的不同字节结果均保留。

下一协议用Humble `raw=True`订阅直接保存入口bytes，再独立解码；原25个CDR与预期保留，不重生成有利数据，新的source/parameter envelope先冻结。六消息独立进程DDS要求 `publish(bytes)`到raw callback完整字节相同，不能替换成字段门。observer receipt仍不能证明实际tracker/controller receipt，也不追补旧字段。

[原输入及失败prepare](evidence/replay_receipt_fixture_inputs_20261001/manifest.json)、[aborted capture、完整归因及日志](evidence/replay_receipt_typed_failure_20261001/manifest.json)保留。main、运行配置、消息/TF合同和安全门不变。
