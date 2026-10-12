# 2026-10-12 Git远端保存收口

本轮仅保存Git资产和恢复记录。GitHub添加现有公钥后，SSH识别qiheiouo，指定fork实际写入成功；Gitee沿用原origin，通过SSH只写两个独立分支。没有force、all、mirror、tags推送，没有修改正式main/main-new-car/main-old-car或默认分支。AUD-003和动态双雷达定位仍Modify，不开展开发或硬件正授权。

## 已保存身份

| 仓库 / 远端ref | 已验证代码节点完整SHA |
|---|---|
| git@github.com:qiheiouo/FAST_LIO_MULTI_ROS2.git；refs/heads/integration/rm2027-bundle-fixes | ae5bef29c89ba7a1004d75657db482bb277e603a |
| git@gitee.com:qiheiovo/rm2027_navigation.git；refs/heads/integration/new-car-software-remote-20261012 | b5177b9b818b6cbeb3dac4a014e915f9e6d69ec1 |
| 同一Gitee；refs/heads/integration/new-car-dual-candidate-20261012 | 9d4be4ed0814f3fe976b5f5b356d687f94f273ef |

这两条RM分支随后仅追加本轮文档收口，源码/测试/配置/.gitmodules/gitlink与上表代码节点逐字节一致；最新文档提交完整HEAD由远端ref及交付清单记录。保留上表原提交，不改写历史。FAST main保持e7864a63a7e2a1ad62ec8fd75fcb9149db08e321；其修复提交的父提交亦为e786。RM正式三线保持fcd9a05b03c9539b4b5ea44de0f09d30c4468455、bd6cf689d3d5aca5c8edd430ff760ca5ceb27496、820179457b70789b0f46697758dbe3d947ec1d4a。

.gitmodules FAST URL保持https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git，因此公开源码恢复不需要GitHub写权限。pin始终ae5；嵌套ikd-Tree来自https://github.com/engcang/ikd-Tree，pin 0438b0daa6ccfaaad9f65f45a3addc318b19ae7a；Livox来源https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble.git，pin 2a2029a6e62a2196b280be6ec00bb2418065b8e0。第三方许可证及版权文件原样保留，未修改或重新授权。

## 实际恢复与发布校验

- GitHub维护ref推送后，空对象库通过匿名HTTPS fetch --depth 1取得精确ae5，不依赖本地已有对象。
- 从新空目录SSH shallow clone Gitee保存分支，submodule update --init --recursive --depth 1取得FAST、ikd、Livox全部精确pin。随后同一干净checkout只从Gitee fetch独立候选，避免第二次大克隆；两个代码节点均验证通过。所有对象均来自远端，无bundle、reference、alternates或本地对象共享；禁用全局Git URL重写/credential配置以排除本地替代。
- 原代码节点与本轮纯文档追加版本分别记录恢复结果；检查root及全部子模块clean、对象连通性、origin URL、固定HEAD、许可证SHA256、必要测试源码和场地资源。场地STL SHA256为cb73b46fe01de83308f05430e895d6d963215a78fbfc5b7e320218f064514556。
- 相对正式bd6的原保存版新增100对象，原候选共126对象；压缩增量约2.04/2.07MB。FAST相对e786仅5对象、约16KB压缩增量。唯一新增超过1MiB的blob是必要且已记录Apache-2.0来源的5,711,784B场地STL。后续只新增小型文档对象。
- 仅检查所选新增可达内容：密钥/token/密码候选、历史聊天与附件元数据、禁入文件类型、对象大小、来源与许可证；没有发现候选，原精简去元数据成果保持。原研究分支/审计证据不是新增提交祖先，build/install/log、rosbag/PCD输出、批量JSON、冷备均未上传。
- 双雷达候选默认enable_experiment=false、外参null，未加入正式软件默认profile；完整active入口、真实动态定位及最终运动授权的原Modify边界保持。源码Git恢复通过不等于运行Integration/Deployment Go。

原publish_after_auth.sh已核对：只推精确FAST和原保存ref、不写正式线，但会重复完整下载且不处理候选；本轮逐步执行等价范围，并改成浅恢复和复用一个checkout。原脚本保留为原SHA计划，文档追加后其固定HEAD前置会拒绝运行；后续恢复使用下列入口和当前远端ref，勿删其校验绕过。

## 最小异地源码恢复

```bash
# 仅在新空目录执行；选择保存版，或把branch换为独立dual候选。
git clone --depth 1 --single-branch \
  --branch integration/new-car-software-remote-20261012 \
  git@gitee.com:qiheiovo/rm2027_navigation.git rm2027_recovered
cd rm2027_recovered
git submodule update --init --recursive --depth 1
git rev-parse HEAD
git submodule status --recursive
git status --short
git submodule foreach --recursive 'git status --short'
```

这证明指定快照的源码、依赖和必要测试可远端恢复；浅checkout不是完整离线历史镜像。本轮没有重新构建、执行ROS/仿真测试或验证硬件，不继承旧运行结论作为新运行结果。

## 备份与停止

原FAST完整冷备144394828B、SHA256 a762e1ecae688137c4c6ee7324948c1dddf37c40928344e06dbccf33473abc5e，及原superproject增量和证据均保留；不上传冷备、不清理其他工作区。详细远端refs、最新HEAD、恢复和许可证校验receipt另存既有备份目录，原失败阶段清单不覆盖。

远端Git源码恢复已通过，但本机同盘bundle仍不是异机冷备。仍建议将必要单ref离线备份和校验清单另存用户控制的独立介质/异机，以降低账户、服务或上游依赖不可用风险；Docker/ROS环境、原始传感器证据与真实硬件验收是独立事项。本轮暂停，不合并正式线，不恢复算法研究。
