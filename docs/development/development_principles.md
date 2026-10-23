# 日常开发原则

适用于 Research / Integration / Deployment；验证强度由[研究与验证](research_and_validation.md)决定。
本页负责实现取舍，Git 和文档维护分别见 [Git 工作流](git_workflow.md)、[文档策略](documentation_policy.md)。

## 最小充分实现

默认采用最小充分实现，不追求最大防御实现。
修 bug 或加功能只改目标直接需要的范围；不顺手重构相邻模块、统一风格或修复无关问题。
范围外发现可以记录影响，不自动变成当前任务。

不要为了“更稳健”默认增加：

- 同一错误的多层保护、重复 fallback 或无真实失败场景的异常恢复。
- 大量 speculative checks，为现实不可达状态建立复杂状态机。
- 没有实际需求的兼容层、为未来功能预建的抽象。
- 仅为架构完整而增加的 wrapper / manager / adapter。
- 同一信息的多个 owner、未经验证需要的冗余缓存。
- 用大量 runtime assert 代替明确的接口设计。

必要防御用于已知真实故障、外部不可信输入、硬件/通信/文件 IO、运动授权，
或 Deployment 的明确要求。新加复杂保护必须能回答：**它防止哪个现实故障？**
回答不了则默认不加；这不构成删除现有安全保护的授权。

## 抽象必须有收益

三行重复代码通常比错误抽象便宜。
出现一次类似代码不构成建设 framework、generic manager、plugin system、复杂模板或扩展接口的理由。
只有真实复用需求或清晰稳定的边界才抽象；先选简单、局部、易理解、可验证、易回退的实现。

## 单一职责与唯一 owner

ROS2 的 TF、canonical odometry、motion authority、mission goal 和 serial transport 必须有明确 owner。
同一输出或状态不同时由多个模块决定；用 contract 明确生产者与消费者，不能靠叠加 adapter 解决所有权争议。
具体边界直接维护在 [contracts](../contracts/)，本页不复制车型与节点合同。

以下变化必须同步对应合同：TF、topic、action、service、message semantic、serial protocol、
motion authority、chassis command、coordinate frame、timestamp / clock domain。
运行组合变化同时更新 [runtime profiles](../operations/runtime_profiles.md)；只更新受影响的权威说明。

## 配置优于复制

- 参数变化优先使用 YAML、launch 参数或 profile，不复制整套节点或只有少数数字不同的 YAML。
- 避免 magic number；参数说明物理意义、单位、frame、时间单位与时钟域。
- 实验配置与正式配置明确区分；一次实验不能悄悄覆盖正式 baseline 或永久改变默认值。
- 引入外部模块先确认 adapter boundary；按[文档策略](documentation_policy.md)登记实际依赖。

## 机器人边界

Simulation Go ≠ Hardware Go；offline replay Go ≠ field Go；build success ≠ functional acceptance。
真实运动和真实串口必须显式 opt-in，不能绕过 motion authority；固件/串口合同未确认时不猜测实车协议。
实验结论说明适用边界；普通软件修改不因此自动升级为整车 Deployment。
