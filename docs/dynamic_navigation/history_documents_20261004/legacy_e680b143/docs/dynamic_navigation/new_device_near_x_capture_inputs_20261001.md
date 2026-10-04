# 新设备near-X完整采集输入冻结

## 背景与假设

按 `new_device_near_x_capture_preregistration_20261001.md`，验证actual MPPI输入能否冻结为同设备replay数据。组合八critic、V1缓存快照与滤波history离线门已通过；本阶段只准备输入，没有启动场景或评价安全。

## 方法与数据

源码提交 `ffb637eb6d42e4c9d51eb50978efbaa724075368`。新增三个独立研究工具：有界goal、扩充raw observer、采集监督器。原observer helper只在独立build目录替换两处import路径，每处一次；全字节逆变换门通过，完整副本和diff随输入归档。

输入 `build/dynamic_research_device_20260930/new_device_near_x_capture_inputs_20261001/inputs.json` SHA `f5f0cb93af62c658e045d309bf47c9c99e40728d4deacc2c28e74667fe33eb50`，参数SHA `b73762c50c8e493aa378d7ead789751c000d76c3c33f8a4b811723e3e6ce65df`；冻结378项源码/installed资产、15 package prefix、三个实际研究库及既有portable门。固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

scene/profile复用已通过portable v2输入，物理/预测/MPPI配置不改。Domain174、unique IGN partition、原phase6、单goal固定8模拟秒、额外3.2秒物理tail；真实参数/TF所有者/control链在goal前只读检查。旁路命令receipt与actual消费者receipt区分保存；future truth仅独立输出。

## 结果

三个源码AST检查通过，prepare exit0，helper逆变换与既有场景/installed资产/组合库身份门通过。完整输入tar/gzip及逐文件SHA位于 `evidence/new_device_near_x_capture_inputs_20261001/`，解包全文件身份核对通过。此记录在capture执行前提交。

## 限制与下一步

准备输入不证明实际capture完整、异步时序等价、预测准确或控制安全。下一步只执行预注册的这一次capture，先冻结所有原始数据、warning、失败和退出记录，之后另写replay协议。main仍 `d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`，接手基线后src/config/launch无差异，原安全门及历史FAIL保持。
