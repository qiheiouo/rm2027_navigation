# 双MID360定向软件验证

固定ae5 bundle与现有ROS组件，全部输入为标记的合成fixture，无串口/底盘设备。不是新车标定、运动去畸变质量或双定位Integration Go。结论见[复用矩阵与结果](../../docs/old_car_reuse_dual_mid360_20261012.md)。

## 构建与运行

固定Humble镜像rm2027_navigation:humble，id 81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3。初始化FAST/ikd/Livox固定pin后，在独立checkout执行：

```bash
docker run --rm --network none --ipc private --user 1000:1000 \
  --cpus 2 --memory 6g --pids-limit 256 --cap-drop ALL \
  --security-opt no-new-privileges \
  -v "$PWD:/workspace" -w /workspace rm2027_navigation:humble \
  bash tests/new_car_dual/build_and_probe.sh
```

仅挂自己的独立checkout，无device；输出全在ignored build/dual_validation。默认只构建bundle，async/adaptive不启用。现有FAST native测量、IMU三模式、driver适配运行测试与canonical测试原样复用。源码发现限于src，避免ignored旧车导出成为重复包。首次环境若缺ROS依赖，安装到相同受控镜像后再运行，不伪造通过。

新增check_config验证：null/非有限/反射/矛盾刚体/错误主IMU拒绝，默认零节点，改IP时一SDK两adapter及JSON身份一致性。test IP为RFC5737文档地址，不能当真实网络配置。

probe_bundle执行完整bundle实际DDS：正常、40ms到达差、25ms源begin差、右LiDAR断流/恢复、主IMU断流/恢复。直接看真实raw odom及stamp，保留恢复旧对/积压边界；静止合成平面有限输出不能证明运动deskew正确。

probe_dynamic_adapter运行实际RSP、gimbal_state_adapter(use_input=true)、lio_adapter(sensor_tf/finite_difference)，使用现有URDF明确的仿真占位几何：纯云台、纯底盘、反向旋转；刻意在源测量后更新最新TF，检查源时刻选择、单TF GID（复用S2 C++观察器的可选edges参数）、pose/twist与原始协方差行为。canonical-parent三例是正控制；sensor_initial动态错误是预期反例，不能删它或据此称全链通过。没有改生产模型去适应fixture。

## 独立候选启动

```bash
# 默认零节点，不触碰硬件：
ros2 launch rm_lio_bringup new_car_dual_mid360_experimental.launch.py
# 显式启用且缺标定必失败：
ros2 launch rm_lio_bringup new_car_dual_mid360_experimental.launch.py enable_experiment:=true
```

new_car_dual_mid360_pending.yaml只有null外参。另存配置，填写实测ROS double数组（带小数点）、revision，才能显式enable_experiment:=true config_file:=... calibration_revision:=...启动backend。左右frame/IMU的固定外参需与URDF一致；现入口只起quarantine backend，无driver/adapter/Nav2/serial，不能用于完整新车导航。软件fixture文件只生成在ignored输出并标记software_fixture，不入生产配置。

驱动已有dual_mid360_driver支持新config_file、left_device_ip、right_device_ip；必须一起指向真实同一JSON两设备，主IMU首选left。原旧车参数默认保持；新车无生产IP，不使用旧IP来表示完成标定。SDK extrinsic_parameter必须与backend/TF合同核对，避免重复变换。

## 既有定位状态回归

本轮接口不变。如重跑原完整gate/bridge/consumer probe，按tests/localization_validity/README.md构建rm_competition_mission与rm_system_monitor等依赖，PROBE_INSTALL指向自己的install，PROBE_OUTPUT指向ignored build，独立DDS域分别执行PROBE_BACKEND=amcl/gicp；synthetic消息不等于原生AMCL/GICP双传感器实车回归。
