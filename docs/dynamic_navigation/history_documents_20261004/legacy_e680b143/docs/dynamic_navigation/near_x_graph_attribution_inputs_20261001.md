# near-X图归因输入冻结

## 背景与假设

按 `near_x_graph_attribution_preregistration_20261001.md` 只读定位v2图门差异，暂不认定是runtime或tool错误。协议source commit `3d530b790286f5bb00adc86fdd46e7e06ec2b5e3`。

## 方法、数据与结果

固定81b325容器中核对capture envelope SHA84ed404f...，七capture目标文件分别与已冻结SHA一致。复制14项capture/installed/source材料至独立输入，包括实际 `/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py`，不读主机Jazzy launch作为依据。

input SHA `09302a417d924c9553f971358b673e5a61c2aa829bedae5f0909c6c222c1e4d4`，parameter SHA `be3f50d1538aa9ad9014a1a68cd45dcf9ce58d03e4d0d4399c7b9bf7a9983f2a`。16个完整输入文件及逐文件SHA在 `evidence/near_x_graph_attribution_inputs_20261001/`，prepare exit0，无sim、node或goal。

## 限制与下一步

此阶段仅冻结来源；尚未评价图门差异。下一项重算原七gate并逐项核对actual endpoints及固定Humble source，未知身份不得按出现结果加入允许集合，不能推断DDS逐事件所有权或安全有效性。原failure数据、main/config/TF/安全门保持。
