# 原Humble控制图精确身份fixture预注册

## 背景与假设

只读归因已证明新tool的cmd_vel单publisher预期与固定Humble source冲突，原FAIL保留。假设独立tool检查可精确识别原controller→smoother→chassis及既有四behavior command publisher，同时拒绝未知/缺失/错误身份。不是修改运行控制链或放宽物理/风险/安全门。

## 方法与冻结数据

纯离线新helper只检查控制图，goal工具暂不使用。正对照唯一取已冻结v2图快照；source证据为固定Humble launch+TimedBehavior header+原profile四behavior plugins，归因summary511b735d...。输入先冻结source/input/parameter/image hash并提交后评价。

要求controller、behavior、velocity smoother、chassis各单一node且namespace为/。精确publisher Counter：cmd_vel_nav=controller×1，cmd_vel=behavior×4+smoother×1，simulation/chassis/cmd_vel=chassis×1；均geometry_msgs/msg/Twist、根namespace，各topic GID非零且唯一。要求smoother订阅cmd_vel_nav、chassis订阅cmd_vel，身份/类型/namespace正确，且各恰一endpoint。四behavior数量仅适用于被冻结profile，其他profile必须另协议，不按capture结果推导允许集。

固定负对照：未知cmd publisher、缺/多behavior endpoint、错误namespace、错误Twist类型、重复GID、缺controller、缺smoother、缺chassis转发、缺/错误smoother订阅、缺chassis订阅、多behavior node。全部应拒绝。记录各门，不通过即停止，不调整fixture求通过。原七graph门中的另六门保持；此fixture只证明tool的控制图身份检查。

## 限制与下一步

图发现GID只能核对endpoint集合，raw observer仍无每消息publisher GID，不能宣布cmd_vel来自哪个plugin或实际应用。通过之后另预注册v3 tool接入、新input/output capture；不得提前修改goal工具或launch/TF/配置。尚无结果，main/安全门/旧failure evidence不变。
