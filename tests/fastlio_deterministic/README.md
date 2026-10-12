# FAST-LIO 定向复现

仅服务 AUD-038 与默认 bundle 的 AUD-046。[验证报告](../../docs/new_car_software_saved_20261012.md)说明固定对象、证据和范围。没有真实新车输入；ROS plane/IMU 消息为软件夹具。

在已具备真实 Livox SDK/ROS Humble 依赖的原镜像中，将本任务独立 checkout 挂到 `/workspace`，以非 root、禁网、无设备、私有 IPC 运行：

```bash
bash tests/fastlio_deterministic/build_and_test.sh
bash tests/fastlio_deterministic/ros_and_original.sh
```

第二个入口需要 `/workspace/build/original_fast_lio`：在自己的 ignored build 下独立克隆 FAST-LIO，detach 到 `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`，初始化 ikd-Tree `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`。不 checkout/reset 本任务的修复子模块或其他工作区。

- `imu_fixture.cpp`：完整原 IMU 类、真实 ESKF，private 可见性仅供观察初态；明确给定时间区间作过程模型对照。没有人为设置原未初始化 double 来制造失败。
- `measurement_fixture.cpp`：完整 bundle translation unit、真实 IKD/PCL/ESKF，多计数、迭代、Jacobian 和分配稳定性。原始 pin 的同载体 matched 应在 libstdc++ bounds assertion 中 abort；unmatched 应正常返回。
- `prepare_reference.py`：只把原 Git 方法的两个 clear 改为 resize(input count) 生成合法存储数学对照，明确记录原函数 hash；不是宣称未定义原实现输出正确。
- `ros_probe.py`：原审计 900 点平面/200 Hz 重力 IMU 输入，完整原 executable 的 DDS/callback/滤波/发布路径，先建图后施加无匹配输入。无 canonical TF、Nav2、串口或实体运动。

产物均在 ignored `build/fastlio_validation`；必要小型结果另存报告目录。测试脚本不安装依赖、修改原镜像或启动真实设备。`COLCON_IGNORE` 避免普通仓库构建发现这组手动定向夹具。
