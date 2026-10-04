# 新设备周期 47 样本字节复现差异预注册

## 背景

执行提交 `3b52412616b2d24e5507033be596f944f63c7455` 的 std4 诊断在 seed 0 标准噪声样本复现门停止，尚未生成 std4 控制或解释排序。第一次输出保留于 `build/dynamic_research_device_20260930/fast_x_scale4/`，不得覆盖。guarded 独立数值门通过不代表历史 capture replay 通过。

## 假设

迁移后的 NumPy 或 CPU 数值实现可能造成过滤后姿态的字节差异，也可能存在真实采样或历史输入差异。先定位差异所在，不能直接将 hash 门改成宽松容差。只在找到可以完整复现四个历史样本文件的确定性执行设置后继续原 std4 诊断。

## 方法与冻结数据

沿用 `near_x_fast_coverage.EXPECTED` 四项输入 hash、cycle 43–46 历史、原 profile、seeds 0/1/2/3、2000×30 和原噪声。迁移 manifest 仍是 `50c6f347d0558393224b19af7face0ba3caba5d9427d8d685f700f2cb7f6cae6`。参考文件由 `coverage_inputs_47/inputs.json` 锁定。

镜像固定 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。先比较二进制 header、footprint、raw map 与 pose；统计 pose 的非零差异、最大误差、ULP 和索引。然后最多检查两种 NumPy CPU dispatch 设置：默认及禁用 AVX2/FMA3/AVX512 相关实现。保存设置、依赖版本、执行提交、脚本与输入 hash。不得改输入、噪声、过滤公式或积分公式以凑 hash。

## 判据与结果记录

继续原诊断要求四个标准噪声样本 map 文件 SHA256 全部与迁移文件完全相同。若仍不能字节复现，保存否定结果，停止 std4 原协议；后续另行预注册可移植输入设计，不宣称本次历史复现成功。几何、安全门和 formal 配置保持原判据。

## 限制与下一步

这是离线可复现性诊断，不是预测算法验证。执行结果追加到独立 result 文档及 evidence；原失败日志和 map 文件保留。
