# near-X采集v2前置图门失败与原始数据冻结

## 背景与假设

按v2协议修复首次rclpy属性误调用，完整采集仍要求实际参数、canonical TF、单一consumer/map/control链门全部通过。source `68a65b47323e499b05c424e0377af4e968a0beb7`，执行 `5fdfa54cfe4044d99ff8841b5105c88a3ca24fb7`；首次FAIL证据保持。

## 方法与数据

固定81b325镜像、Domain174、2CPU/6GB/network none，scene/profile/phase/原门与时间限全部保持。input SHA `173f8369ced1c406b194f54bd0ab6a1b37f47d74dd5b86bb8cac1240cb21b38e`，parameter SHA `880edc1782ad1022a6c7e96121e70968c284988e5301629c0f8840ad0e8a4426`。启动运行到约22.095模拟秒，七actual参数响应全部匹配。

## 结果

v2 capture FAIL，supervisor exit1，planned_stop_completed=false。属性错误消失，实际graph写入成功，但 `canonical TF/single consumer/control-chain graph gate failed`，工具在发送ComputePath/NavigateToPose前停止。无goal周期可用于replay；loaded_maps/writer_status文件即使存在也不等同有控制周期。

全量 `capture_files.json` SHA `84ed404fcf36f77b4e2c5e00281f232a5dcaf181718a7d83ea05f3b2dec58306`，原始graph、参数、consumer/raw数据、process/signal、GZ transport、ROS日志及退出信息归档于 `evidence/new_device_near_x_capture_failure_v2_20261001/`。全文件身份与tar解包核对，冻结后才能分析具体图门差异。此文档仅记录原始失败，不宣称具体TF违规原因。

## 限制与下一步

v2协议停止，不重跑以选择有利图或安全结果。下一项先独立预注册只读graph/source归因，核对实际publisher/subscriber身份、命名空间、原Humble navigation launch remap及图判据来源；如是工具错误必须保留否定证据后再写最小修复协议，不改变runtime TF/控制链。没有MPPI完整capture、评分、排序、几何安全或部署结论。

main/source/config/launch/安装、全部安全门与旧否定研究结果保持。
