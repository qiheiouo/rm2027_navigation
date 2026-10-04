# 原始CDR observer协议预注册

## 背景与停止条件

前一[typed callback记录器](replay_receipt_recorder_preregistration_20261001.md)在原始CDR反序列化/重新序列化字节门失败，仅记录2事件，未执行DDS烟测，验收停止。固定输入 `acdddaa3ad749e29b9d9fd2140b8b744ed00d1c97d6c0bf4b83affcb2ca88d86`。初次完整归因25消息解码字段全一致，重新序列化字节18/25一致，其余7个消息有少量偏移变化；归因不是通过原字节门。

假设新协议可以从Humble `create_subscription(raw=True)`保存回调收到的精确CDR，再独立解码header/frame做诊断；字节身份从回调bytes读取，不从typed对象重新构造。修改仅在独立实验工具，不接运行tracker/critic/TF。先使用严格CDR游标按冻结ROS字段类型读取所有值并登记对齐padding，确认上述差异是否全部落在padding，任何解析/字段差停止归因。禁止把padding诊断当原字节验收。

## 固定输入与方法

25个旧合成输入CDR及独立预期保持原字节和原身份，不重新生成有利payload；新input envelope只登记旧manifest和新源码/参数/镜像。支持raw回调的实现首先记录实际入口时间，再解码，不对收到的bytes清零/规范化。CDR payload必须与传入bytes逐字节相同；TF返回值由Buffer新构造，记录它自己的CDR SHA但不声称该字节在跨运行确定。

固定夹具改为将同一旧bytes直接传入raw入口，所有25个预期仍固定。六消息独立进程DDS烟测采用原生 `Publisher.publish(bytes)`；raw subscriber收到的完整CDR必须逐字节与冻结输入相同，不仅比较解码值。typed重新序列化门继续记为失败，不作为新协议的前置假定；改动理由是更换记录接口以保存真实字节，而非放宽相同bytes门。

正常/坏frame、未来、重复/异payload、迟到、零stamp禁止latest、TF插值/失败保留、时间回退清Buffer、损坏证据拒绝和无应用publisher/client保持。独立observer的receipt仍不能证明tracker/controller收到，旧数据不回填。正式配置、公共消息/主题/TF合同、main及安全门不变，原失败目录/evidence保留。

## 身份与限制

镜像仍 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，源/参数/新envelope/原CDR/提交在evaluate前冻结。只验证记录协议，不接受闭环动态安全或消息不漏包/延迟性能；原生ROS序列化的padding是否确定须另行报告。本协议不使用真值，不调风险或控制。

## 评价前冻结

raw源码/prepare提交 `ff8cc65c72f6f275277de848614c0604323180f2`；input SHA `b898d9b58249097238da83bd21fed9378bc44d72d523a40d652c4d27f1ee1796`，参数SHA `50b0a06a2cefe454b1b8b4ff2c82ec6a3ebe6ac415c741ee61291361dfa59f2b`。25个原CDR及所有receipt/expected rows已与原envelope逐字节/逐字段核对，不变。[输入及prepare日志](evidence/replay_receipt_raw_inputs_20261001/manifest.json)先提交，尚未evaluate。
