# near-X采集工具属性修复v2预注册

## 背景与假设

首次capture已独立冻结提交 `7e805eccb9dd0065890483287bfb1fa02ce5327e`，在graph写入前因rclpy int属性误调用FAIL。七actual参数门通过，但没有发送goal、没有MPPI周期或安全结果。假设仅修复 `now().nanoseconds()` 为 `now().nanoseconds` 可消除明确的采集工具TypeError；不假设graph、完整capture、replay或安全会通过。

## 最小方法

只修改新研究goal工具的一处属性读取。supervisor只改v2输出目录、IGN partition、临时XDG目录并增加本协议及native API检查身份；原scene/profile/phase/ready/所有参数、graph判据、300×30/八critic、8s窗口/3.2s tail、TF timeout、process termination/所有时间上限保持。helper仍两处import-only绑定至全新input目录，旧source和failure evidence不覆盖。

先在固定Humble镜像中读取实际rclpy Time.nanoseconds类型与Clock.now()返回值，保存源码/API版本及多个Time值读取结果，确认是int属性；不创建导航goal或TF发布。AST/syntax检查后固定工具source commit，prepare冻结原378资产及新协议/API检查，完整input/parameter/image/binary证据提交之后执行一次v2 capture。

## 数据与评价边界

新inputs `new_device_near_x_capture_inputs_v2_20261001/`，新output `new_device_near_x_capture_v2_20261001/`，partition `rm2027_near_x_capture_v2_20261001`，XDG `/tmp/rm2027_near_x_runtime_v2_20261001`。相同81b325、Domain174、2CPU/6GB/network none。完整输出先freeze/commit再分析，失败停止并保存。此重试为未发送goal的工具修复，不按任何安全标签挑选成功run。

graph缺门、parameter差异、action不接受、超时、writer不完整均不能以属性修复成功替代。原首三个eligible周期选择及后续独立replay协议要求保持；本阶段不分析排名/几何安全，不引入future truth作为输入。

## 结果、限制与下一步

本文件在修复前提交，尚无v2结果。native API检查只证明该属性读法，不证明完整工具路径；完整capture仍需实际所有门。main/比赛配置、原安装/source/TF、安全判据与全部否定研究结论保持。
