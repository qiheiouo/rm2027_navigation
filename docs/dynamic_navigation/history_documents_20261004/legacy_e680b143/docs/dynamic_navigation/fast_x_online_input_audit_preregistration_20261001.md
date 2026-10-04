# 快运动冻结周期在线输入可用性审计预注册

## 背景与假设

固定连续风险虽有整体区分信号，相关候选四组增强critic首选仍不安全。回到输入层，先检验既有扫描/四帧因果规则所需数据是否在已保存周期中有充分的时间与身份支持，而不是直接改中心/速度或使用真值。

假设周期43–47保存的tracker消息、同源扫描和在线机器人姿态能形成四帧输入；也可能出现消息不一致、历史缺失、姿态需要消费者之后的样本，或缺少真实到达证明。本项仅审计输入，不计算动态几何、未来覆盖、风险或控制。

## 固定范围与方法

完整检查迁移已保存的五个周期43/44/45/46/47，不换周期、不看结果增补。原 selected 周期47与原输入hash固定。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。冻结全部周期json/bin及三条流：predictions SHA `fd8d7eabfe0bd0cec9b5668412deb2b91890faa7ae01c14118d2fdc5627eb2bd`、scans SHA `f97b1c9f51d0666d23708bf0bd53e143b7920750ee98eaa8c5722993eb645ee1`、trajectory SHA `0d87068962966c6791fc159dc2e2978e0b69dfef578dab4d0fcd055ec85d8a3d`。

每个周期只取critic实际收到消息source及其之前最后四个完整、单confirmed tracker source，跨度必须0.15–0.30 s。最新源与实际消费track的id/state/xy/vxy/size/last_observation/prediction逐项一致；frame/schema/complete与原消费时间和source age核对。匹配相同source的扫描，不允许latest扫描替代。

姿态沿用既有规则的source时刻在线pose插值，记录前后bracket与header时间；两端均须不晚于当前critic consumer，间隔≤0.2 s。tracker另外核对processing/recorded receive_ros_t≤consumer。保留所有源行、扫描和pose bracket的原字段及hash，明确source与receipt是不同证据。

扫描及trajectory若只有header时间、无callback接收时间，不得推断当时已到达；trajectory若未记录frame，也不得宣称完整TF身份门通过。分别报告“按header/字段可构造”和“在线实际可用已证明”两种状态，未知后者按未验收，不用改判据把缺失变成通过。

## 数据、限制与下一步

执行前源码/预注册提交；输出输入manifest/参数/镜像/执行commit hash，完整五周期结果和失败原因。未来真值文件不读取，仅保持旧迁移证据。未过身份/时间构造门时，停止对应周期的预测构造；不能丢弃该周期。若仅receipt/frame证明不足，可另行预注册明确条件化的离线几何反事实，不能称在线输入可用或修改运行插件。

不改变TF发布者、frame设计、tracker/KF、critic、main/正式配置、footprint或安全门。需要新录制接口时也必须独立预注册并先在研究脚本留证，不能临时改正式录制/运行配置。
