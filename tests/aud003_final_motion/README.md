# AUD-003 最终运动授权软件验证

本夹具只运行受控虚拟速度源、真实 `managed_motion_ingress_node`、真实
`competition_v2_transport_node` 与 V2 codec。MCU heartbeat、定位状态和底盘
模式均为 fixture；最终观察对象是 `/serial/v2/mock_tx` 的 ChassisCommand 字节，
不会打开串口或设备。

覆盖：无授权、正常运动、Mission/owner 撤权、定位 LOST、授权超时、命令超时、
非有限命令、未知速度 publisher 注入，以及 guard 重启后旧 endpoint/旧租约拒绝与
新 session 重新授权。ingress 只过滤来源，唯一运动授权判定仍在 V2 编码前完成。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/aud003_log build \
  --base-paths src/rm_competition_interfaces src/rm_chassis_interface src/rm_serial_driver \
  --build-base /tmp/aud003_build --install-base /tmp/aud003_install
bash tests/aud003_final_motion/run.sh
```

结果写入 `/tmp/aud003_final_motion_results`，不进入长期仓库资产。
