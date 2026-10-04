# 实验基础设施统一notes

## 当前冻结状态

从用户2026-10-01方向纠正起，已有采集/回放/TF/身份/证据体系默认足够使用，停止扩展。既有工具失败与详细证据不删除；本notes统一记录后续直接阻塞明确算法判定的最小修复，不再为微小工具修复新增大量主研究章节。

## 当前capture

v4已启动，source6486643、execution f420a39，input3565a62e...、parameter6f84d923...、image81b325。此前三次失败分别为rclpy属性误调用、错误排除原Humble behavior publisher、NumPy UUID日志JSON类型；已保存原始数据和否定记录，v3六周期不完整。原运行配置/TF/control链/安全门没有改变。

仅完成当前v4并保存结果，随后直接开展三层算法holdout；若再遇工具问题，先判断是否直接阻塞Sampling/Ranking/Aggregation或当前capture。不能以追求额外字节/身份/时间层继续完善工具。

### v4完成与关闭失败

原执行仍为FAIL：固定8秒窗口22.102–30.102 s完成，goal取消响应0、result status5；物理tail到33.496 s完成。150/150 MPPI周期已写出，dropped/errors=0且writer closed。监督器随后等待observer关闭10秒超时；observer已写manifest，在遍历验证时被退出信号打断。完整日志和失败状态保留，不修关闭逻辑、不为此重跑。

原始44963项文件归档在 `evidence/new_device_near_x_capture_v4_20261001/`，capture_files SHA256 `d14e3cb590872da30f6ce9853c960722c45d60ee938b8735509da73f672b1238`，archive SHA256 `5fa05bede5508fa722ca88795032a4bd28b5e205003fd5fb9fb44d83e75fe7e3`。逐文件归档内容已核对，build原件保留。该关闭问题不阻塞已保存周期的Sampling/Ranking/Aggregation；若所选周期必要字段缺失，直接报告限制，不能按安全结果更换周期。

三层分析读取必要数组时使用events中的array记录；旧reader不支持dtype=cdr，本次局部读取CDR为bytes，其余数组沿用NumPy f4/u1。该适配直接支持实际prediction输入解析，不新增记录器或验证体系。一次只读检查错误假定meta含arrays键，产生KeyError；原格式实际为events，不影响原数据。

固定风险评价前的一次git文档提交自动审批超时，未执行；按工具允许重试一次成功，没有改变权限或动作范围。一次只读路径检查错误猜测DynamicObstacleTrack.msg，实际接口为DynamicObstaclePrediction.msg；字段解析直接按现有接口进行。risk_only均匀展示字段不作为算法通过或实际weight mass，主判断以真实总分/原生概率为准。

kinematic risk在首次prepare前只读检查发现AxisBox位置参数顺序错误，改为具名bounds后才生成风险；没有旧实验结果被覆盖。host在线风险计算Python3.12.3/NumPy1.26.4，原生控制评价在固定81b325镜像；源JSON/hash记录和文档明确两者角色，采用冻结数值阈值，不为跨环境字节一致重跑。

独立CA试次01复用原capture完成窗口/tail和451周期，再现退出等待超时；执行FAIL保留，数据足够则直接三层分析。未修关闭逻辑。新归档复用原capture_files.json并放入tar，manifest只索引原hash/执行状态与archive，避免在主Git文档重复数万行文件表。

独立试次02在原ready/phase限时前未满足就绪，goal未发送、MPPI0周期，原错误“original near-X readiness/phase timeout”；数据不能承担预登记holdout评价。完整输出归档，未放宽时间/TF/graph，未重试。该输入失败不能等同算法安全失败，也不能从完整01推断02通过。
# 物理执行C诊断中的只读调用

第一次Twist离线解码Docker命令未传`-i`，stdin未接入，exit0但无分析输出。补上`-i`后读取相同已冻结CDR；未修改capture/observer或算法，结果记录在physical execution文档。Matplotlib因默认配置目录不可写自动使用/tmp缓存，图已生成；不修改用户全局配置。

结论交付的临时只读hash核对两次错误假定旧manifest的files为dict、capture归档名为result.tar.gz，分别在读取时退出。按原manifest的list/dict及实际capture.tar.gz核对后，所引十个阶段24个归档/摘要hash通过；没有修改manifest/数据，没有新验收门或算法重跑。

风险形状阶段只读源码查询一度把analyze导入的polygon_distance当作本文件定义，并猜错simulation_geometry.py位置，命令退出未改变数据；实际算法直接复用已有导入函数，不写新geometry体系。Nav2文档/Raw网页缓存请求失败，GitHub对应1.1.20正文可读，公开margin实现依据取该primary源；不影响实验输入或控制。算法执行未发生基础设施失败。
