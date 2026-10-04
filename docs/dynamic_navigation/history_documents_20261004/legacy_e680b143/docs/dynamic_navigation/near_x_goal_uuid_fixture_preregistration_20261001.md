# near-X原生goal UUID记录fixture预注册

## 背景与假设

v3 FAIL已冻结提交cd0035d：原参数/图门均过，goal accepted后UUID NumPy uint8元素不能直接JSON写入，窗口/tail不完整。假设新纯记录helper将合法16-byte整数序列复制为Python int列表，可保持UUID完整值和原对象/CDR，同时拒绝非法长度/非整数/范围错误；不改变action handle或算法。

## 方法与固定fixture

goal工具暂不改。固定81b325内原生unique_identifier_msgs/UUID和rclpy serializer用于数据生成与评价，无node/TF/goal。三固定vector：全0、0..15、边界0/1/127/128/254/255重复到16；每vector四种容器：Python int list、array('B')、NumPy uint8、原生ROS UUID经过typed CDR serialize/deserialize后的字段，共12正例。负例7：长度0/15/17、float、bool、negative、256，全部拒绝。

每正例记录旧list→json成功/TypeError、容器/元素原生类型、完整CDR/输入值/期望JSON。输入先source commit/freeze input/parameter/image/module hash并提交后才评价。新helper必须返回全部Python int、长度16、值完全相等；原数组/ROS对象身份与内容不变、重新序列化CDR逐字节相等。JSON覆盖原accepted event字段结构。不得把typed CDR说成DDS原始bytes。

## 限制、结果与下一步

没有结果。19case全部满足原协议预期才另预注册goal记录接入和v4 capture；不修改v3失败、不使用其六周期当完整采集。此门只证明logging类型转换，不能证明action异步时序、窗口/tail、replay或安全。main/TF/config/运行control链/原参数与所有安全判据保持。
