# 原始CDR只读observer记录协议通过隔离夹具

## 背景与方法

按[raw协议预注册](replay_receipt_raw_cdr_preregistration_20261001.md)复用typed失败实验的全部25个CDR及独立预期，只改为raw subscriber保存入口bytes，再独立解码header/frame。没有清零padding、重生成payload或放宽完整字节门。旧[typed字节门失败](replay_receipt_typed_failure_result_20261001.md)保持。

## 数据身份

源码/prepare `ff8cc65c72f6f275277de848614c0604323180f2`，评价 `ba3de94`。输入SHA `b898d9b58249097238da83bd21fed9378bc44d72d523a40d652c4d27f1ee1796`，参数SHA `50b0a06a2cefe454b1b8b4ff2c82ec6a3ebe6ac415c741ee61291361dfa59f2b`。原25个payload及所有receipt/expected rows与旧输入 `acdddaa3ad749e29b9d9fd2140b8b744ed00d1c97d6c0bf4b83affcb2ca88d86`完全相同。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；结果summary SHA `530b7c0e43422533e76ca6f22d63fd153b7ac298a316aaff4f2a3170b5d22730`。

## 结果

- 25/25受控原始CDR完整字节通过；入口bytes从未由typed消息重新构造。受控receipt明确标为synthetic，不能当实际耗时。
- 固定TF插值返回12.5 m及11.5 m（两动态样本加static .5 m），准确源stamp一致；13 s缺失事件在晚到TF后仍保留unavailable，下一重复scan才变available（16.5 m）。
- 时间0与空frame不查询latest；错误frame、未来source/processing时间、旧source后到、重复/同stamp异CDR按预期识别。
- ROS回退后epoch增加，私有Buffer重建；static再到仍不补造dynamic，完整TF重到后才成功，跨epoch不合并CDR重复。
- 有意生成的三种损坏test-copy（payload变化、payload缺失、event截断）全部被manifest/hash检查拒绝。原capture未修改，坏副本及拒绝原因保留。
- 独立进程DDS实际六消息（static TF、dynamic TF、scan、odom、prediction、clock）通过完整原始CDR字节门，receipt来自真实回调入口，先于callback的发布steady时间另存；scan源时刻TF成功。一个epoch，正常关闭、6事件完整manifest。
- Recorder源码无应用publisher/client调用，无TF、命令、goal/action权限；fixture publisher仅存在隔离ROS domain 73、无网络/硬件/controller的容器。Humble executor未提供DDS MessageInfo，GID明示null/unavailable。

## 限制与下一步

**通过的是独立observer记录协议，实际tracker/controller receipt仍未证明。** 六消息DDS只验证接口与字节传递，不能声称消息不漏包、延迟性能或真实场景在线可用。TF Buffer返回消息是新构造的结果，保存它本次CDR身份但不声称padding跨运行确定。CDR解码异常会停止并封存已有事件；当前协议不声称保存畸形CDR的失败入口。没有真值、几何安全或闭环验收。

工具位于 `experiments/dynamic_prediction_v1/frozen_cycle/replay_receipt_recorder.py`，显式选择scan topic/frame、TF target、prediction expected frame及镜像。公共prediction合同为map；旧sim采用odom必须单独登记，不能静默改frame。use_sim_time由显式flag控制；仅手工独立运行，尚未加入旧holdout或正式launch。

下一步冻结本次25事件和六事件capture，独立按事件顺序重建TF并核对查询成功/失败及全部返回字段；用预装未来TF的否定对照检查因果顺序是否影响结论，不补回原缺失事件。该阶段仍不宣称完整controller cycle/critic历史replay已通过。

[已冻结输入](evidence/replay_receipt_raw_inputs_20261001/manifest.json)、[全部capture、损坏副本、日志与结果](evidence/replay_receipt_raw_result_20261001/manifest.json)保留。main、运行tracker/critic、TF发布者、正式配置和安全门不变。
