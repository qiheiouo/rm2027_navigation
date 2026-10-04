# near-X图门来源归因结果

## 背景与假设

v2完整capture前置graph失败，先冻结数据后只读辨别runtime问题或tool预期错误。按独立预注册执行，归因工具source/evaluation `27033fb15c8af1038b052948cd312a716c1ab24d`。

## 方法与数据

input SHA `09302a417d924c9553f971358b673e5a61c2aa829bedae5f0909c6c222c1e4d4`，parameter SHA `be3f50d1538aa9ad9014a1a68cd45dcf9ce58d03e4d0d4399c7b9bf7a9983f2a`；补充native source输入SHA `021c837e16fa4952b726072127d9d8a2bb65e89ae0a31bd1da3e277d8db56cf2`。固定81b325内Humble Nav2 bringup/behaviors/velocity smoother均1.1.20；完整source、query和version已先提交。最初误猜nested include header路径FileNotFoundError保留，后用dpkg实际清单找到唯一 `/opt/ros/humble/include/nav2_behaviors/timed_behavior.hpp`，不改旧失败。

## 结果与来源

原七图gate重算与记录逐项一致，六pass、仅nav_command_chain FAIL；五个来源检查通过。实际 `/cmd_vel_nav` publisher只有controller_server，smoother订阅存在；`/cmd_vel` 有velocity_smoother一endpoint和behavior_server四endpoint，chassis订阅和唯一转发publisher存在。canonical TF、单一tracker/static map/controller和无hardware门全部通过。writer written0，仍无导航周期。

固定Humble `navigation_launch.py` 的非composition Node分支：controller将cmd_vel映射cmd_vel_nav；behavior只用原TF remappings，没有该command remap；smoother将输入映射cmd_vel_nav、输出映射cmd_vel。原 `TimedBehavior::configure` 第130行建立 `geometry_msgs::msg::Twist("cmd_vel",1)` publisher，header SHA `db7260945fc77b8e4323abf2d8dc1a399a2bbadcadb7be67534a90d6635390cf`。因此新工具“cmd_vel只有smoother”预期与原固定source冲突。四behavior endpoint与原四plugin配置一致，此数量关系是来源支持的解释，不声称逐DDS消息或逐plugin归属。

summary SHA `511b735d4cde5245bf2fd80660f611faa39d3bde6a2fea5f43228b1f775ac192`，完整结果在 `evidence/near_x_graph_attribution_result_20261001/`。

## 限制与下一步

没有安装新graph判据或修改runtime，原图门FAIL保持。新协议可最小修复工具：精确匹配原source约束下的identity/namespace/type/endpoint数量及原链，而非任意放行新publisher；必须先离线正负fixture验证，再考虑全新capture。behavior可直接发布cmd_vel，后续不能默认每条cmd_vel来自MPPI+smoother；旁路raw receipt没有publisher GID，这个归属限制保持。

不改TF/运行control链、配置、MPPI算法或安全门；main和所有旧否定实验保留。
