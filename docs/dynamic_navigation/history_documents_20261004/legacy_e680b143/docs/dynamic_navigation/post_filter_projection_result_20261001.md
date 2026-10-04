# 冻结过滤输出原 bounds 投影结果

## 背景与假设

32 组原生输出中 27 组整段越界，之前的几何正对照不满足完整 nominal velocity 约束。本项按[预注册](post_filter_projection_preregistration_20261001.md)检查一次原 bounds 投影是否仍保留几何安全见证，不修改运行控制器。

## 方法与数据

全部 32 组原 after-filter 序列逐轴投影至原 vx/vy/wz bounds，不再滤波；原/投影 64 条轨迹沿用当前速度首步、30 步、0.1 s、原 body/padded footprint、raw map 和 body ≥ 0.05 m / padded > 0 安全门。未来箱体只生成离线标签。10 条原型 raw replay 失败依然缺失，未补齐或改标签。

准备提交 `21f14bb`；评价提交 `0d240d7a9bb900ee1c70cbeda49fdfdfd961d0e8`。输入 SHA `381c9d7809ed24ca8faedb0d1ab56915d6bd120bdc6a017a2d8f333e1738a327`，参数 SHA `f3379ebc1316d21ba72738444d6d2b92a7ecac7d5992ae06ad63bb2ca93142e9`；镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，raw map 检查器 SHA `ce1c81c3f86bb5d87b188529e313e2694c10addafd9aedcc1807b1f0095d7faa`。全部原输出几何复现门通过，未借此重验收历史 capture 失败。

## 结果

| 组别 | 数量 | 原几何联合安全 | 投影后几何联合安全 |
| --- | ---: | ---: | ---: |
| 单原型成功 replay | 17 | 6 | 6 |
| 原型聚合 | 3 | 2 | 2 |
| std4 原基线 | 4 | 0 | 0 |
| std4 固定风险 | 8 | 0 | 0 |

原矩阵有 564 个逐轴越界元素，最大超限 0.0671363；投影矩阵严格越界数为零，无 tolerance。32 组 body/padded 最小间隙均未变化，目标推进最大变化 0.00847 m。#18 投影后 body/padded 为 0.147608/0.117608 m，推进 1.36399 m；#21 为 0.137698/0.107698 m，推进 0.624942 m。两条原型及重复使用 #18 的两个聚合正对照均保留。

唯一原实际返回越界 seed0 raw 的 vy 从 −0.50779098 投影为 −0.5，但动态间隙仍为零。投影没有修复固定随机集合的 0/8 风险迁移失败。

## 限制与下一步

这是同一个已查看冻结状态下的 nominal velocity 与采样几何见证，不是运行 clamp、安全算法、加速度/轮速、连续时间或闭环验收；8 条包含相同 #18 的重复使用，不是 8 个独立成功。原序列和旧标签不覆盖，运行 critic、TF 与比赛配置不变。

恒定动作可行而四组随机候选仍无安全覆盖，下一步预注册固定样本的时间相关结构覆盖对照，不增加 batch、不改 std、不调评分权重。相关噪声改变协方差，不能直接套用原对角正则声称 MPPI 输出正确。

完整输入、64 条 raw map 检查、严格 bounds 审计与日志见[证据清单](evidence/post_filter_projection_20261001/manifest.json)，11 项压缩与解压内容 hash 均核对。
