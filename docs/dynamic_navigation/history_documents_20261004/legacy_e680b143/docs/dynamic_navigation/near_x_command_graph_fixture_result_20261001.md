# 原控制图精确身份fixture结果

## 背景与假设

原tool错误排除了Humble已有behavior publisher；按独立协议只评价来源约束下的control图身份helper，未接入goal工具或runtime。

## 方法与数据

source d5a2d9c，evaluation commit `9792968dd08f94da1e9db0a55b052416a7ff6de6`。input SHA `6f1c0d2ae6d81c6f4d01d0462b5ceb9ea63a549fe4573cdf781d228bee5fa43e`，parameter SHA `437e77e603a27dc66bc1e5bf74e92d77df9c04b3b92924d2b416515fb091eaa6`。来源为81b325实际Humble capture/source，评价是host纯Python离线；result image_id表示来源镜像，不表示host有该runtime。

## 结果

14/14预注册case通过：唯一原native图被接受，13种未知publisher、behavior缺/多endpoint、错误namespace/type、重复GID、controller/smoother/chassis输出缺失、smoother订阅缺失/错误、chassis订阅缺失、多behavior node全部被拒绝。正对照九个详细门通过，负对照失败门全保存。

summary SHA `3700ce194c5be3109cdc3cb2753592494bbd884e92bb2f297b50a58c943311ec`，完整结果在 `evidence/near_x_command_graph_fixture_result_20261001/`。未修改输入或根据结果调整允许集。

## 限制与下一步

helper只证明被冻结四behavior profile的endpoint身份集合检查，不能证明raw命令归属、DDS异步时序或安全。原v2 graph FAIL保留。下一项另预注册v3工具接入、actual behavior参数读验证和新capture目录；原六TF/单consumer等门保持，所有物理/风险/控制安全判据不变，不改runtime控制链/TF/config/main。
