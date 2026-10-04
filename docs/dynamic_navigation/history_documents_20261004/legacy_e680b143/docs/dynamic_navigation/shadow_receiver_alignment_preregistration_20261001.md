# Shadow真实consumer/observer字段与TF结果对齐预注册

## 背景与假设

新shadow capture已经冻结，原tracker参数未改。假设共享source的scan完整native字段可对齐，并且实际consumer的0.08s查询可与observer timeout0结果并列。不能从后来TF到达推断本次consumer结果，必须读其实际trace；也不能以单次结果接受预测或MPPI算法。

## 数据与方法

唯一输入为 `shadow_consumer_capture_20261001/`，observer manifest SHA `27dce5d9c7901e288c68355d926a1fde94b6b3acdf6c462defe0527616094112`，consumer manifest SHA `e8d3e3a1226b16a0328157cff9850022aaf53b6c784f5f047d732d565b482a01`，capture identity SHA `b2fe9cdc3bbee0ad1ab9f4c034d6ec924001e8ce9f6521aa7a4c1ba654b8e745`。先冻结source/全部输入文件集合/字节hash、镜像/参数，export/commit后才evaluate。

扫描按header source stamp/frame建立两侧多值索引，再逐字段比较全部native值（含ranges/NaN/inf/intensities/时序角度）；不同source不得互相填补。consumer native输入来自callback typed对象，observer来自原raw CDR解码；不比较二次CDR padding。相同source唯一pair的字段不同即门FAIL；歧义/窗口不重叠/单侧缺失均列出，不删除，不解释成已证明DDS丢包。

逐consumer scan入口sequence核对唯一exit、lookup entry/return或exception和publication关联；lookup必须使用该scan源stamp、sim_lidar_link→map、ROS_TIME、timeout80,000,000 ns，记录原参数、真实entry/结束ROS/steady时刻及duration，不能把返回/异常身份当source时间。每个原prediction发布要求header源stamp对应关联scan、frame map/schema/authority shadow_only、processing stamp保留；与observer prediction按完整native字段hash对齐，未匹配/歧义列出。诊断status来自consumer发布的原native内容，不调整标签。

observer同源scan的原TF状态与consumer实际TF状态交叉计数；同时报告全量及固定launch后5s口径，单侧记录仍单列，不用warmup覆盖原结果。map原始body与已冻结fixture解码字段比较（只排除实际publisher刻意更新的header stamp），frame/分辨率/origin/全cells必须一致；未捕获consumer map callback，不冒称实际map receipt。

## 门、限制与下一步

所有输入manifest/file hashes、事件序号、scan entry/exit和lookup完整配对、参数、共同唯一source字段、同源prediction语义身份必须通过；否则保存具体失败并停止该门，不改算法/TF/等待/QoS/判据。未匹配自身是数据覆盖限制，不能用它强行推断字段相等或运行成功。报告原warning和日志hash，无重跑。

若接口门通过，只能说明这一份instrumented consumer在此fixture/map/资源条件的实际输入/TF行为；不证明零扰动、未instrumented性能、独立闭环、检测/预测准确性或MPPI安全。无truth、rollout/critic/risk排序。保持原所有FAIL、main/正式配置/TF/安全门不变，结果包含背景/假设/数据/方法/结果/限制/下一步。

## 评价前冻结登记

源码 `b40223a65f0cb294843615567941a0e3c727bc10`；9279个capture内文件/日志全hash冻结，input SHA `a8009c4b64dd3635e2028746f52a3509a31759a549d3490bceb71f87dd82cfa4`，参数SHA `d4c7bfec0c15f425ae2dce16aa3591e92775e0fb6c010e2cc9c06f98c280c3b9`。[完整输入](evidence/shadow_receiver_alignment_inputs_20261001/manifest.json)先提交再evaluate。consumer typed scan另解码核对callback native身份；共同唯一source prediction比全字段，不把native hash未命中而同source存在的差异当普通缺失。
