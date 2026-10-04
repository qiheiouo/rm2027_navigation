# 新设备独立周期采集前置条件只读核查

## 背景与假设

已有raw observer与因果TF子流程通过，历史周期标准评分capture仍不完全复现，尚无本设备完整独立cycle数据。假设当前固定Humble镜像和已构建工作区可能提供Gazebo、现有sim launch、V1插件及trace overlay，但不能假定旧设备 `/work` 安装、旧镜像或derived场景仍存在。

## 方法与冻结

先固定本分支commit、镜像、明确资产清单和只读命令/参数hash。读取原world/model/bridge、研究profile、launch、TF/topic合同、V1插件与trace源码hash；现有文件只读，不生成新场景或调整参数。使用镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`、已构建workspace setup，查询Gazebo版本/帮助、dpkg版本、ROS package前缀和executables、launch `--show-args`。独立超时命令保留stdout/stderr/exit及缺项，缺项不能改成通过。

核对新prefix、旧绝对prefix、现有V1库/trace库及二进制hash；资产存在不等于可加载或已运行成功。只读源码梳理预期TF单一所有者：map_odom_stub、lio_adapter、robot_state_publisher及gimbal来源；Gazebo pose不桥接canonical TF。没有启动Gazebo server、Nav2、tracker、observer、controller或行动客户端，没有ROS topic/TF/command发布，show-args只是解析。

## 判定与限制

报告每个命令和资产，不将“package存在”判为完整replay/闭环门；特别分清sim依赖可用、旧prefix缺失、controller trace/插件安装是否准备好与历史scene能否忠实恢复。旧holdout image门、source/hash与正式配置不改，不调用原holdout去绕过缺失镜像。

如前置能力可用，下一独立周期协议仍须另行预注册并冻结scene/profile/源码/镜像/模块加载身份，保留actual消费者与observer receipt边界；如缺少组件，只考虑在隔离build root编译已冻结原源码，先做数值/接口核对，不直接运行或修改critic。禁止拼接本次合成observer capture与旧cycle冒充同一输入。

文档/evidence保留全部阴性输出；main、正式运行配置、算法代码、TF合同和安全门不变。此阶段纯环境/源码静态检查，不提供算法安全结论。

## 评价前冻结登记

初次host prepare `02b9d62`输出host绝对路径，不适用于container执行；未evaluate，原envelope和日志保留。源码 `d631ff50b8a3159db0e967cf93769020360cd9ad`增加命令/执行路径解释一致门，在实际容器路径重新prepare；input SHA `1d80218730b3b40cd52ff1b200abeeb9eec06a5ad15d4fe9675c8520bb70736e`、参数SHA `2c01263a8009f60d2cd1b627a695fa2b350ec1f4512fe81a6ddb03874a0391ff`。26项资产hash固定，未执行版本查询或launch。[两次准备与有效清单](evidence/new_device_capture_inventory_inputs_20261001/manifest.json)先提交。

## 只读路径补充核查（执行前）

原inventory已完成，候选 `libnav2_mppi_controller.so` 不存在不能推断trace没编译；CMake/插件XML实际目标是 `mppi_controller` 和 `mppi_critics`。trial目录缺derived路径也不等于portable evidence缺场景。追加只读核对actual library路径及 `near_x_fast_motion_20260930/scenario/` 的五个原artifact，并对migration manifest SHA `50c6f347d0558393224b19af7face0ba3caba5d9427d8d685f700f2cb7f6cae6`逐项确认源身份。不覆盖原阴性路径输出，不加载库或运行launch。[补充输入/参数](evidence/new_device_capture_artifact_supplement_20261001/inputs.json)在新路径字节查询前提交。
