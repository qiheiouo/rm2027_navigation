# near-X capture v3原生UUID记录失败

## 背景与假设

按v3协议接入离线通过的原Humble控制图helper，原runtime/scene/phase/profile/时限保持。source `d092420ffce03894d6338fb08f9a12ede3e020ec`，执行 `19937017c09b3e8d2a0ad83ce3f5318ee9801011`。

## 方法与数据

固定81b325、Domain174、2CPU/6GB/network none，input SHA `a254e18eed7f8009ecf049394a5dfe7841c54458ac494e8bbd0336e1f5319381`，parameter SHA `5fee08326781c8903a39a08971b75728d956e719487dbcff4e3b227ad7c69906`。八actual参数组完全匹配、七graph门和九command endpoint详细门全部通过；原phase22s后ComputePath返回status4/path87，约22.208s发送NavigateToPose并收到accepted handle。

## 结果与归因

capture FAIL，planned_stop_completed=false。新工具将 `list(handle.goal_id.uuid)` 写JSON，原生UUID数组元素为NumPy uint8，标准json拒绝 `Object of type uint8 is not JSON serializable`。错误在accepted handle之后的记录语句，监督器随后按异常流程终止场景；固定8s窗口、action outcome和3.2s physical tail未完成。已有少量MPPI周期保留，但不能作为完整capture或首三个周期实验输入。

全量 `capture_files.json` SHA `1af8a6b43854295982cf9b794b8272bc2d13a3f60de6117624e6e71eb2cd78e8`，完整raw/typed/MPPI trace/graph/参数/goal/process/GZ/ROS日志在 `evidence/new_device_near_x_capture_failure_v3_20261001/`，逐文件与tar解包身份核对。没有评价周期评分、排名或未来几何标签，不能把初始周期存在当算法或安全成功。

## 限制与下一步

v3协议停止，旧三个FAIL全保留。下一项先预注册实际Humble ROS UUID/NumPy类型的离线JSON序列化正负对照与仅记录转换修复；通过再写新capture接入协议。不得修改handle、UUID内容、goal/control/参数或任何安全门，不把此前三个工具失败隐藏于算法成功分母。main/source/config/TF/control运行链不变。
