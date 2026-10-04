# 新设备near-X首次完整采集工具失败

## 背景与假设

按完整capture协议，在原near-X scene/profile/phase上采集可重放的actual MPPI输入。输入/参数在启动前冻结，source `ffb637eb6d42e4c9d51eb50978efbaa724075368`，执行提交 `2bd166af08950c556ab78c36a220ad8890248624`。

## 方法与数据

固定镜像81b325、2CPU/6GB、network none、只读仓库，Domain174。input SHA `f5f0cb93af62c658e045d309bf47c9c99e40728d4deacc2c28e74667fe33eb50`，parameter SHA `b73762c50c8e493aa378d7ead789751c000d76c3c33f8a4b811723e3e6ce65df`。原observer readiness/phase运行到约22.174模拟秒；controller、planner、velocity smoother、两costmap、tracker、moving obstacle七组实际参数服务响应与冻结YAML/CLI一致，全部原始响应保留。

## 结果与归因

capture FAIL，supervisor exit1，planned_stop_completed=false；未发送ComputePath或NavigateToPose goal，未得到MPPI周期。新goal工具graph写入调用 `self.get_clock().now().nanoseconds()`，但Humble rclpy该接口是int属性，抛 `TypeError: 'int' object is not callable`。这是采集工具错误，不是算法、安全、TF所有权或参数门失败；graph结果在异常前尚未写入，不能宣布actual graph通过。

`capture_files.json` SHA `6d8b9497c03ac0309aae68655b229383051adfa41c2fc71411a18317dc320ea1`。observer没有planned marker，按原预注册记录interrupted；所有typed/raw、goal原始stream、参数、GZ transport、process/signal、ROS/stdout/stderr和退出warning完整冻结到 `evidence/new_device_near_x_capture_failure_20261001/`，逐文件SHA与解包核对。启动期scan TF extrapolation、网络地址提示和退出日志原样保留。

## 限制与下一步

该数据不是完整动态控制capture，不能用于首三个周期replay，不能以七组参数匹配替代graph/cycle/安全门。原协议停止且不覆盖此失败。另预注册仅修复rclpy属性读法的新协议和全新input/output绑定，保留原scene/profile/phase/门/时间限；修复目标是工具完整性，未发送goal的旧run不构成可选择的安全样本。

main、原src/config/launch、安装、TF契约及所有安全判据不变，不部署、不merge、不调MPPI参数。
