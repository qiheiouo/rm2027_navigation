# 代码来源与修改范围

plugins.cpp、controller/critics.xml、package.xml及CMake基础来自本仓库
`experiments/stvl_temporal`（dc7897e包含353薄观测器最终修复），RootCostProbe保留原
Samsung Research/Open Navigation Apache-2.0 attribution。
许可证见 `../stvl_temporal_roots/LICENSE.Apache-2.0`，其原NOTICE仍适用。
vendor/model.hpp只读引用原组件，没有引入新tracker/控制框架。

本轮仅添加shadow.hpp观测调用、单周期字段和诊断耐久性；原Critic评分、原生优化器、
SG、Twist选择/速度owner不修改。prepared::Snapshot/classify直接复用上一轮core。
相同包名仅为隔离overlay复用既有插件配置，不能与旧research DLL同时装载同一个控制器。
没有整体merge研究历史，未修改原文件，也不用于正式导航。
