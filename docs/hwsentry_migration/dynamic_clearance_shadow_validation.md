# 动态通道准入 Shadow 验证

## 当前边界

`rm_dynamic_clearance` 只把标准 Path、同 revision 的 annotated path 和版本化动态目标
预测组合成 `CLEAR/BLOCKED/UNKNOWN` 报告。它在 prediction 源时间戳读取 canonical
`map -> base_link` 并投影到未裁剪的全局 Path，禁止 latest-TF。launch 默认关闭，节点
不发布 Path、TF、costmap、Nav2 action、姿态请求或速度命令；现有 mission 与狗洞执行器
均不消费该报告。

Linux 隔离验证已完成：`rm_competition_interfaces`、`rm_path_annotations`、
`rm_dynamic_obstacle_tracking`、`rm_dynamic_clearance` 全新构建成功，合计 57 项测试
零失败；实际 ROS 消息回环已得到 revision-bound `BLOCKED/predicted_overlap`。

## Mock 门

至少覆盖以下场景，并保存相关输入、输出和 TF topic：

1. 新鲜空 prediction：动态准入区输出 `CLEAR`；
2. confirmed 目标在预计到达点重叠：`BLOCKED`，带 blocking track ID；
3. tentative 目标重叠：`UNKNOWN`，不能成为进入许可；
4. coasting 目标：按额外 margin 保守判定；
5. prediction stale/future：`UNKNOWN`；
6. prediction horizon 短于通道采样 ETA：`UNKNOWN`；
7. tracker 超过有界输出数量、`complete=false`：`UNKNOWN/prediction_track_limit`；
8. Path/annotation revision、frame、长度或 schema 不一致：`UNKNOWN/contract_error`；
9. timestamped TF 缺失、机器人离 Path 太远或自交投影有歧义：`UNKNOWN`；
10. 无动态准入区：`CLEAR/no_admission_required`，且没有 region ID。

```bash
ros2 launch rm_dynamic_clearance dynamic_clearance_shadow.launch.py enabled:=true

ros2 bag record \
  /plan \
  /navigation/annotated_path \
  /perception/dynamic_obstacles_shadow/predictions \
  /navigation/dynamic_clearance_shadow \
  /tf \
  /tf_static
```

## 场地门

当前场地与旧地图不匹配时不要做结论性准入验证。地图工作已冻结；以后使用你从同一
CAD 来源导出的匹配 PCD/PGM 即可，无需为本模块新增建图功能。先完成 tracker D01-D07，
再在通道入口分别做空通道、人员横穿、目标停留、短遮挡和预测时域不足测试。

正式接线前必须用实测速度确定 nominal speed/ETA，量测机器人等效半径、目标 footprint、
margin、prediction P95/P99 age 和完整通道所需 horizon。任何 `UNKNOWN`、revision 不匹配
或过期报告都必须解释为“不得进入”，而不是 `CLEAR`。

## 执行接线门

只有上述实车证据通过后，才在 `main-new-car` 已有狗洞执行器上增加消费者。消费者必须
保持 `competition_mission` 为单一 Nav2 action owner；进入 COMMITTED 前要求同 revision
的 fresh `CLEAR`，进入后只允许 emergency/safety/显式 abort 打断。不得新建第二套狗洞
FSM，也不得让 shadow 节点直接取消目标或控制底盘。
