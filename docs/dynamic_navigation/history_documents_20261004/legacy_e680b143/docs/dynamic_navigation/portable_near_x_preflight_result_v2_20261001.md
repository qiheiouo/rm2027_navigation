# 可移植near-X只读preflight v2结果

## 背景与假设

首次门因文件分类与CLI参数排列错误失败，完整 [旧失败](portable_near_x_preflight_failure_20261001.md)已保留。按 [v2预注册](portable_near_x_preflight_v2_preregistration_20261001.md)，只修复harness，原scene/profile/TF/安全标准保持。

## 方法与数据

修复/prepare `d97fac7f791cb2b2c060e5dd7fb61d2a2d8850ec`，执行 `ac4b833edaede9f8c104509163f025e33311cc9b`；input SHA `26afaa9431e99dd6c9026be50d00022345ab835d77734119fdf954cda1a4bc3d`，parameter SHA `2cf65fd2d3d4f7c8ef4a3bb6f247dbfbe4a66e49ea2ed06f3c4b43c95e7f48d5`，固定81b325镜像。

新独立目录portable_near_x_inputs_v2_20261001；七原资产全部符合旧manifest。四launch只变旧绝对路径，完整逆变换还原原文；world/model/profile byte保持，profile SHA `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`。旧runner只存参考，不执行。

冻结196 src/installed资产、15包prefix、组合门及关键库身份。AST按world assignment、launch source、Node -file、config_file识别六个完整文件引用，topic/LogInfo另行保留。只执行 `ros2 launch --show-args <dynamic launch> params_file:=<profile>`；未启动仿真、controller、TF或goal/control。

## 结果

13/13门全部通过：原source逆变换与world/model/profile不变、15包查询return0、history/snapshot overlay prefix正确、四launch无旧runtime前缀、六文件集合严格相等且全部存在、show-args return0、查询后输入/source不变。

summary SHA `23e0f33bc24dfb57f0c2d4d40981d91aa9dd08f27c99b4bcb36d759b7979387d`。完整 [输入evidence](evidence/portable_near_x_inputs_v2_20261001/manifest.json) 与 [查询结果/ROS日志](evidence/portable_near_x_preflight_result_v2_20261001/manifest.json)逐文件SHA核验保存。原失败门及两个归因日志不覆盖。

## 限制

参数展开与路径存在仅证明依赖可解析，不证明组件运行、实时性、预测效果或闭环安全。新设备镜像/安装路径已另冻结，没有恢复旧/work环境或历史replay。原全量历史FAIL、所有风险否定结果、V1 freeze、main和正式配置/TF/footprint/安全门保持。

## 下一步

独立预注册有界完整研究capture，同七scene/profile与原goal phase；明确actual consumer receipt、V1最新快照、history及cycle完整性，核对canonical TF、实际库和运行参数。新capture先冻结后评价，再按固定首三个eligible cycle规则replay；不选安全结果，不将共享单run的三个cycle作为独立成功。
