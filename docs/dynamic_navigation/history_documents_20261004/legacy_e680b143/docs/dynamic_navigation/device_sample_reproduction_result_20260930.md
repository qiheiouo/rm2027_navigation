# 周期 47 新设备样本字节复现否定结果

## 背景与假设

按 `device_sample_reproduction_preregistration_20260930.md` 检验不同 CPU 数值 dispatch 能否恢复四个历史过滤姿态样本 SHA。第一次 std4 输入门失败仍保留，不改阈值。

## 方法与数据

执行提交 `03e6991`，固定原四项输入、历史控制、四 seeds、2000×30、原噪声与 NumPy 1.21.5。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。先默认执行，再按预注册禁用 AVX2/FMA3 与 AVX512 相关特性；机器本就不支持 AVX512，警告保存。源和参数由 cycle/profile/history 以及结果中的 dispatch 设置 hash 链锁定。

## 结果

默认条件：header、footprint、raw map 和 x/y 姿态全部字节相同。seed 1/3 整个文件 SHA 相同；seed 0 有 5 个 yaw 元素不同，seed 2 有 1 个不同；最大绝对差 `2.9802322387695312e-8`，最大 2/1 ULP。

禁用扩展后：四组控制矩阵 SHA 与默认设置全部相同，但 x/y 姿态差异增多；yaw 的 6 个差异保持。四组 map 仍不能完整字节复现。**否定“两种已登记 dispatch 设置之一能恢复全部旧样本 SHA”的假设。原 std4 协议停止，不解释其排序。**

## 限制

没有旧设备控制矩阵或完整 NumPy 数值环境，不能证明旧样本噪声完全相同，也不能将差异唯一归因于 NumPy 版本、CPU、过滤或积分。小误差不等于通过既定 hash 门，也不解除历史 capture 数值门失败。

## 下一步

另行预注册新设备控制矩阵的字节冻结。新诊断保留原 cycle、profile、history、seeds、std4、原生有界评分与安全门，明确仅研究当前设备离线反事实，不宣称历史分布完整复现。输入先写出并 hash，再执行评分，未来真值只作独立标签。

[证据 manifest](evidence/device_sample_reproduction_20260930/manifest.json)包含第一次失败日志/样本、两种设置的全部控制与姿态、依赖和 CPU 信息。旧证据不覆盖。
