# 快运动源时间条件化的既有二维占用迁移预注册

## 背景与假设

五个冻结周期的源时间字段可构造，实际scan/pose到达及pose frame证明缺失。本项明确条件于审计保存的源时间输入可得，复用已存在的扫描端点/近面四帧速度区间，不作为在线可用验收。假设同一规则可能覆盖快运动未来；此前周期73只覆盖2/9，故迁移也可能失败。

## 固定输入与方法

完整周期43–47及输入审计SHA `f9e3a28a96ec1f74101de9f2114e003d9b768681c5a1a41fbf4cda7363a57501`，五周期共享6个源。只用保存的同源scan、当时tracker及consumer之前pose bracket构造。frame解释沿用旧研究假设，receipt/frame缺失继续标记未验收，不补造。

严格复用 `cycle613_face_xy_rank` 中常量和公式：箱体半宽/高0.225/0.275 m，端点allowance .05 m，y源allowance .05 m，径向sigma .01 m，四帧速度斜率与 `3*sigma/sqrt(Sxx)+A*(last_t-mean_t)`，A固定0.5551652475612764；age用实际critic source age，dt和九步沿原周期。静态扫描筛选仍用原probe静态map夹具、dynamic_candidates(.25,True)、tracker附近x±.35/y±.50窗口，观测north/south侧必须四帧一致。

不改中心/速度规则、箱体尺寸、A、allowance、历史跨度或支持宽度，不拟合未来残差。源点无效/区间倒置/侧别变化时停止该周期构造并保留为九步失败，不fallback或丢弃周期。prepare先保存所有选点、源区间、速度区间、五组预测框、规则/静态map源码与输入/参数/commit/镜像hash；在读取未来标签前提交。

## 评价与判据

评价完整45个周期步的物理完整箱体二维覆盖，严格坐标包含；同时报源时刻中心区间、x/y分项覆盖、逐步最小覆盖margin、宽度和原V1支持对照。保留失败周期/源，分母不缩小，五周期不是独立场景。物理真值只在evaluate标签阶段读取，不用于生成框或选规则。

若有任一步漏包，固定规则在此集合的完整占用门失败，停止直接迁移，不回调A/allowance/中心以追求覆盖。全覆盖也只为条件化同周期几何支持，不能替代真实到达、硬安全界、MPPI排序/聚合或独立闭环。

## 限制与下一步

旧激光极值噪声和reference acceleration不是物理硬界，已知轴对齐单箱、选点关联和静态map夹具仍有局限；快速箱体动态不受该reference约束证明。本项不评分候选，不增加batch/std，不修改TF/运行critic/main/正式配置/footprint/padding或原安全门。镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

## 评价前冻结登记

准备/源码提交 `0bea930`；输入manifest SHA `503807f99c94c3d7b0d64a0958844cce4e4b14211a98ed1b3d46563d85b05028`、参数SHA `207fadc745d9f8656308792fea91f817b5dc5bc2f1f5924ed97481dadfed1e3c`。五周期均能按旧规则生成9框，选点、源区间、速度半宽及静态map源码hash见[冻结输入](evidence/fast_x_face_coverage_inputs_20261001/manifest.json)。未读取未来标签，实际receipt/frame仍未验收。
