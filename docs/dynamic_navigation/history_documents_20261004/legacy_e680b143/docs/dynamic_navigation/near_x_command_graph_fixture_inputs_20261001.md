# 原控制图精确身份fixture输入冻结

## 背景与假设

按独立预注册，纯离线检查能否识别固定Humble原控制图，并拒绝13种未知/缺失/错误/重复身份。goal工具仍未使用helper，原capture FAIL与runtime保持。

## 方法、数据与结果

source `d5a2d9cfc3eb301ddde6e932120da2f92f316d79`，graph/source来自已冻结v2图和归因summary511b735d...；正对照1、负对照13，prepare exit0。input SHA `6f1c0d2ae6d81c6f4d01d0462b5ceb9ea63a549fe4573cdf781d228bee5fa43e`，parameter SHA `437e77e603a27dc66bc1e5bf74e92d77df9c04b3b92924d2b416515fb091eaa6`，固定81b325来源身份；本纯Python评价运行于host，不声称镜像中新的runtime行为。

16个完整输入文件在 `evidence/near_x_command_graph_fixture_inputs_20261001/` tar/gzip归档，逐成员SHA解包核对。未评价，输入在评价前提交。

## 限制与下一步

下一项只评价全部14fixture，不按实际通过结果修改输入或门。helper的identity/GID集合不提供raw消息逐发布者归属、实际控制应用或安全证明；通过才另协议考虑接入。main/config/TF/安全门不变。
