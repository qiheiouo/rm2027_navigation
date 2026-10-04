# 快运动输入：源时间可构造，实际在线可用尚未证明

## 背景与方法

按[预注册](fast_x_online_input_audit_preregistration_20261001.md)完整审计周期43–47，各取实际消费source及此前四个完整单confirmed source。只查消息身份、同源scan、在线pose bracket、consumer时间和到达字段；未读取物理未来、计算几何或风险。

执行提交 `7563874`；输入SHA `f9e3a28a96ec1f74101de9f2114e003d9b768681c5a1a41fbf4cda7363a57501`，参数SHA `4c5de5bc710c052d91a2464f4814ff07fd179e2a6cad01df4d1efbf9795a36c9`；镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。迁移manifest和五份json/bin及三条流逐一核对，未放宽旧回放数值门。

## 结果

| cycle | 消费source / consumer时刻（s） | 源时间/字段构造门 | 实际可用证明 |
| --- | --- | --- | --- |
| 43 | 23.563 / 23.624 | 通过 | 缺失 |
| 44 | 23.629 / 23.673 | 通过 | 缺失 |
| 45 | 23.629 / 23.701 | 通过 | 缺失 |
| 46 | 23.695 / 23.737 | 通过 | 缺失 |
| 47 | 23.695 / 23.772 | 通过 | 缺失 |

五周期最新recorded track的id/state/xy/vxy/size、last observation及prediction均与critic消费内容一致；每周期四帧跨度0.198 s、同源扫描存在且frame为sim_lidar_link、tracker处理与recorded receive均不晚于consumer。在线pose插值的两个header也均不晚于consumer，未使用consumer之后的pose或latest扫描替代。五周期共享source，只有6个不同历史源，不是20个独立观测。

**实际在线可用门0/5验收。** 每周期均缺scan callback receipt、pose callback receipts和pose frame原字段。源header时刻较早不证明消息及时到达；录包中插值可计算也不证明当时已有精确timestamp TF。这是证据缺失，不是已证明消息迟到、frame错误或运行链故障。

## 限制与下一步

保留所有五周期及对应scan/track/pose bracket输入，不删失败、不推断missing字段。不能据此将扫描规则称为在线可用或改runtime TF/critic。旧记录不会被补造接收时间或frame。

下一项可单独预注册**条件于这些源时间输入可得**的冻结二维占用反事实，复用既有规则、先固定预测再看未来覆盖，明确不构成在线输入验收。若要实际可用证明，需要新研究录制记录header+接收时间+frame/TF查询输入及到达顺序，不能从旧数据恢复不存在的字段。

完整输入及门结果见[证据清单](evidence/fast_x_online_input_audit_20261001/manifest.json)，9项hash均核对。main、正式配置、TF设计、tracker/KF、安全门和旧evidence不变。
