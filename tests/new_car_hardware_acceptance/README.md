# 新车验收采集器无硬件自检

本 fixture 只发布标准 IMU、Odometry 和 TF 测试消息，启动真实
`new_car_hardware_acceptance` 只读采集器。它不打开设备、不创建命令 publisher，
也不启动 Nav2。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=231 ROS_LOG_DIR=/tmp/rm2027_acceptance_ros_logs
python3 tests/new_car_hardware_acceptance/run_fixture.py \
  --output /tmp/rm2027_acceptance_fixture_good

export ROS_DOMAIN_ID=232
python3 tests/new_car_hardware_acceptance/run_fixture.py --regress \
  --output /tmp/rm2027_acceptance_fixture_regression
```

第一条必须为 `GO`；第二条注入一次时间回退，并断言采集器子进程以退出码 2 和
`STOP` 结束。fixture 自身断言通过后返回 0。输出只保存在指定临时目录。
