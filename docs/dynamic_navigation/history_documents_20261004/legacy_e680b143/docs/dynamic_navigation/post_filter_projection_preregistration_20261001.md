# 冻结过滤输出bounds投影反事实预注册

## 背景与假设

32组原生输出的27条整段越界，包括几何正对照#18。假设原after-filter逐轴投影到原bounds后，部分正对照仍几何安全；也可能失效。该项不是运行控制器修改或安全策略验收。

## 方法与固定输入

完整32组，来源audit输入 SHA `7d4b48c554abe444f4f298a6a7bf4b07f27acb0a61c372ad7c977ec37f1fff90`；所有源字节先核对，10条raw replay失败仍缺失。只执行一次 `clip(original_after, original_bounds)`，不再次滤波，不调bounds、footprint/padding、dt、horizon、当前速度首步或安全门。

prepare先保存原/投影矩阵、case映射、参数JSON与输入/参数/源码/执行commit hash，评价前提交。镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

评价原与投影64条三秒轨迹，当前速度首步，未来完整箱体仅标签，原生raw map检查器复核；报告body>=0.05 m、padded>0、静态门、目标推进、返回点变化和整段bounds。原几何对旧记录差须<=1e-4 m，失败则停止解释。投影bounds是运算性质，不作为算法收益证明。

## 限制与下一步

prepare执行 `21f14bb`，输入 SHA `381c9d7809ed24ca8faedb0d1ab56915d6bd120bdc6a017a2d8f333e1738a327`、参数 SHA `f3379ebc1316d21ba72738444d6d2b92a7ecac7d5992ae06ad63bb2ca93142e9`。原/投影矩阵已在评价前冻结并登记。

不将投影结果写回旧标签。正对照仍安全只能给出此状态下满足原nominal约束的采样动作见证；风险迁移仍失败则保留否定结论。记录投影是否损害几何/推进，不假定单调改善。无加速度/轮速/连续安全/闭环或部署声明，无运行clamp/critic/supervisor修改；之后再预注册时间结构覆盖或在线输入研究，不重复调参。
