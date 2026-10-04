# 可移植near-X只读preflight v2预注册

## 背景与假设

首次 [失败](portable_near_x_preflight_failure_20261001.md)明确为harness把topic/日志当文件和CLI参数位置错误。假设仅修复文件语义识别与CLI选项顺序即可执行原只读门；不改变场景、physical/profile或安全标准。

## 方法与数据冻结

先提交本协议后改harness。原四launch/world/model/profile/runner都重新从旧七资产严格SHA验证，在全新portable_near_x_inputs_v2_20261001目录执行同路径前缀重绑定及全字节逆变换。除了新目录路径，旧scene内容不改变；profile d00f722e...原字节保持。原input fbc2d8aa.../原summary2545bdc6...及所有失败证据不覆盖。

AST文件上下文明确为world变量的绝对路径赋值、PythonLaunchDescriptionSource绝对路径、Node arguments中-file后的绝对路径、parameters中config_file绝对路径。要求六个引用的完整预登记集合：world、base/comparison/reference三个include文件、X障碍模型、dynamic bridge配置；不得只按存在性筛选、不排除任何真实文件。topic和LogInfo为其他类型字符串，仍保留并记录，不作为filesystem目标。保持无旧/work或/ws runtime引用门、全部原资产逆变换/物理byte门、196资产/15包/overlay身份门及输入不变门。

CLI使用 `ros2 launch --show-args <dynamic launch> params_file:=<profile>`，选项先于位置参数；只展开参数，禁止启动场景、TF/controller或goal/control。固定81b325镜像与同四overlay，sourcecommit/input/params/image/library/package全部冻结提交后执行。完整stdout/stderr/ROS日志和通过/失败记录追加保存。

## 判据、限制与下一步

六文件集合严格相等、全部存在、15查询/overlay及source/physical identities保持、show-args return0才通过；任何失败保存并停止，不放宽footprint/guard等判据。此门不证明运行、预测或安全。若通过，仍需另预注册有界完整capture及冻结后replay，原历史FAIL和V1 freeze保持。

## 执行前冻结

修复/prepare提交 `d97fac7f791cb2b2c060e5dd7fb61d2a2d8850ec`；v2 input SHA `26afaa9431e99dd6c9026be50d00022345ab835d77734119fdf954cda1a4bc3d`，parameter SHA `2cf65fd2d3d4f7c8ef4a3bb6f247dbfbe4a66e49ea2ed06f3c4b43c95e7f48d5`。196资产/15包、原七资产及四launch逆变换、完整新scene/source manifest均冻结于 [v2 evidence](evidence/portable_near_x_inputs_v2_20261001/manifest.json)。旧失败及原输入保持，未执行场景。
