# near-X采集v2输入冻结

## 背景与假设

首次工具FAIL已保存，按v2预注册仅修正rclpy属性读取，验证有界完整输入capture能否执行。没有重设架构或修改算法、场景、TF、参数、安全门。

## 方法与数据

协议commit `c521afe6753084cce466c4391cb7f954c1635db1`，工具source `68a65b47323e499b05c424e0377af4e968a0beb7`。同镜像81b325中原生API检查四种Time值读取全为相同int；actual Clock.now().nanoseconds也为int，descriptor为property。rclpy time.py SHA `9e77229d85033ed96af0b11b803964a958e04d6fd1e1b689199d6002f7efac05`，检查记录SHA `3ac1385879a6af2c7f82bef9bc002bb25f19f856591348ced5c96ee6cff626d0`。三个工具AST检查通过；检查没有创建node、goal或TF。

全新 `new_device_near_x_capture_inputs_v2_20261001/inputs.json` SHA `173f8369ced1c406b194f54bd0ab6a1b37f47d74dd5b86bb8cac1240cb21b38e`，parameter SHA `880edc1782ad1022a6c7e96121e70968c284988e5301629c0f8840ad0e8a4426`，380源码/installed资产，15 package prefix，原组合库身份保持。两helper import绑定逆变换还原全字节；scene/profile仍原portable v2输入。

## 结果

prepare exit0，API/AST/既有portable/组合库身份和helper逆变换门通过，完整准备input tar/gzip和原native API检查保存于 `evidence/new_device_near_x_capture_inputs_v2_20261001/`，逐文件解包SHA核对通过。此文档和证据在v2 capture前提交。

## 限制与下一步

API读取正确不证明actual graph、action、writer或完整capture。下一项只执行一次注册v2采集，新output/partition/XDG绑定，所有原时间上限与判据保持；完整输出先冻结，失败则保存停止。首三个eligible周期与replay需后续独立协议。此前29564文件失败证据和全部旧否定结论保持，main/src/config/launch不改。
