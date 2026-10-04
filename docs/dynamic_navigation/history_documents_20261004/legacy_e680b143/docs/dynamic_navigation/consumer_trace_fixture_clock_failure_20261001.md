# Consumer trace fixture固定时钟类型错误：失败记录

## 背景、方法与数据

[预注册](consumer_trace_fixture_preregistration_20261001.md)的转发门首次评价中断。源码 `dcd0abb637b6c0682ec4319988493f3895285e62`，评价 `80fe084cda1b4acfca7c6d35bd70cb695f10a872`；input SHA `733e10871783107b84714ea83c3bac0c42187fc74b777e13c5fa54035bee5212`，参数SHA `3f83685374bbf40ea282be22360d31a5b86858a4b24611dc387f99b5f92007a7`，镜像81b325。[原始部分结果、trace和日志](evidence/consumer_trace_fixture_clock_failure_20261001/manifest.json)全部保留。

## 结果与归因

waiting_for_map、empty_scan_frame、invalid_scan_stamp三例门通过。第四例stale_scan的原baseline在 `self.get_clock().now() - Time.from_msg(scan.header.stamp)` 抛 `TypeError: Can't subtract times with different clock types`。fixture的 `Time(nanoseconds=7e9)` 默认SYSTEM_TIME，未落实预注册固定ROS clock；原scan stamp转换是ROS_TIME。此错误在baseline、trace之外，不是runtime tracker回归；后四例/sentinel/损坏拒绝未执行，不作完整转发通过声明。

另一个输入身份标注需校正：旧prepare的native_fields_sha256来自序列化前的message对象；ROS float32 scalar在deserialize后会量化。旧CDR字节SHA才是本次实际消费输入，旧native descriptor不能冒称解码后的字段身份。

## 限制与下一步

保留首次协议FAIL，不覆盖源码/input/evidence或原runtime。仅修正实验clock为显式ROS_TIME，新增prepare reuse旧input选项，逐字节复制全部21个旧CDR，冻结新的“序列化前字段身份”及“实际解码后字段身份”，evaluate严格验证后者；不改扫描值/TF/参数/输出门。新envelope先提交再评价。此阶段无live采集、Nav2或算法/正式配置/TF修改。
