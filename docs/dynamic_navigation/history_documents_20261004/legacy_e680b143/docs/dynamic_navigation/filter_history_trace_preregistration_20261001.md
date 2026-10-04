# MPPI滤波history只读快照副本预注册

## 背景与假设

V1 sole-message快照离线门12/12通过，但现有trace只有before_filter/after_filter控制，没有任意周期实际control_history[4]。假设在原evalControl的滤波调用前后追加显式12个float32的只读blob，可记录真实输入及更新结果，同时保持原计算与输出字节。完整周期场景仍不启动。

## 副本方法与源码门

以新设备已验证guarded源码为基线，optimizer.cpp SHA `1de9019fe85ccdeba5edbd07327023befe8da2003870fcfcbef3df6b5f968cfc`。复制整个已登记Nav2 package到独立build root，只在evalControl既有before_filter与after_filter位置追加显式history序列化：顺序[h0.vx,h0.vy,h0.wz,...,h3.wz]。不依赖struct内存布局。blob dtype=f4、shape=[4,3]，不增加ROS节点/TF/topic/控制权或改变filter、reset、fallback、bounds、聚合、regularizer及source guard。

生成器必须硬核对原CPP SHA，移除追加块后恢复原CPP全部字节；除optimizer.cpp外所有副本文件与基线字节相同。完整源码、精确diff、license/intake来源与manifest先冻结，再在固定镜像81b325、Release(-O3 -DNDEBUG)、MAKEFLAGS=-j1、sequential/parallel-workers1构建独立overlay。原workspace/guarded/audited安装均不覆盖。

## 离线fixture门

worker复用已冻结map/footprint核对与原Optimizer，暴露仅fixture加载/读取控制及history的方法。直接调用实际Optimizer::evalControl。fixture显式iteration_count=0以隔离滤波/返回/shift，不做sampling或critic/MPPI有效性声明；仍保留原300×30、地图、八顶点padded footprint及所有物理bounds。使用固定ROS_TIME与消息header，不运行导航、TF或控制publisher。

至少六路径：zero history、原43–46 history、固定非对称合法history、相同历史shift-off、reset后history、连续两次evalControl。普通ctrl来自既有冻结initial_controls，reset路径重载该ctrl但不重载history；shift-off仅fixture controller_frequency=20（原dt=.1），其余10。两次路径第二次用第一次原返回后的内部sequence/history，不从未来或旁路推断。

原版与副本worker及库、元数据、expected输入history f4字节、参数/source/image hash在执行前冻结。原版/副本全部返回control、内部shift后sequence和history输出字节一致；旧before/after_filter记录、原版/final输出和新history blob对应一致。副本writer完整且无drop/error，实际加载controller/critics库唯一身份正确；第一cycle before-history等于冻结输入（reset路径等于zero），after-history等于worker读取的真实更新结果，连续case第二cycle before-history等于前cycle after-history。禁用logger路径也应保持输出字节。

另复用既有native replay的冻结正/负/zero/additive对照，核对新controller库加载下score/聚合/filter输出与既有通过版本全字节一致。该门验证局部copy构建无计算偏差，不把原否定的安全结果改成成功。每阶段结果与失败全部追加保存。

## 数据与限制

所有fixture输入先冻结、两role raw输出先归档再评价。synthetic非对称history仅接口正对照，非实际运行状态；iteration0不是比赛/研究运行参数改动，不能作为完整MPPI周期证明。记录会改变执行时长，未证明零开销、实时性或异步行为。原历史replay失败、V1 freeze、TF所有者和正式配置/安全门保持。

## 下一步

本门通过后仍需独立预注册新的完整研究capture，冻结新设备场景绑定、source/receipt/history/trace完整性并检验实际加载库与完整周期replay。未通过则保存并停止，禁止直接覆盖原controller/V1或降低安全门。

## 构建前冻结

生成器提交 `34863413da8b5ddfc8a19df2dc9534bf5ebc363c`；副本optimizer SHA `18998c7ccf88a629dfb5ef3277fe97acb539eaf2a0c26b75e4b9a852825e8a43`。仅optimizer.cpp变化，删除两处追加块恢复原CPP全部字节，其他75个文件原样保持。完整源代码、license、差异和manifest见 [source evidence](evidence/filter_history_trace_source_20261001/manifest.json)。独立build/install为filter_history_trace_overlay_20261001，构建不允许运行完整周期。

## Worker与fixture实现登记

actual evalControl worker源码提交 `c2fd5661d699485cd35ae50c71b374b8f4c72975`，SHA `1dc2810eb32478a130e48f9978234e9ee080175475aba1693cf89503337c8de2`，完整源码先冻结于 [worker evidence](evidence/filter_history_worker_source_20261001/manifest.json)。fixture明确7路径（六history情形及logger_off），读取原initial controls并按声明注入/重置history；原版/副本before/after全部90 float及returned/internal-shift/history全部字节比较。回归固定5类native输出×6文件，包括原18项baseline/zero门及固定raw/actual_filtered，另4非法项均须return1且不写full scores；仅复用旧golden字节，不读未来truth或重算安全标签。native动态链接用LD_DEBUG真实calling-init路径结合冻结库SHA核对。

输入冻结源码 `db43c2c810a04ee75369faeaefe4aa18a5856d89`，input SHA `c623e53f8b5deed82e29d75be7c917f6da50a7c3bf8b4b22ed4255480f6ad7f8`，parameter SHA `8ea2d05c2c3804aba9e3f99bf841c5881cce33aea2621927e9d4d896a6656276`；worker SHA `c082b69a4f83ff544807332e01d2a0b14a5df54b7dea9396f494894e1df0c46f`。独立副本controller/critics SHA分别 `7baebf7641c29b7a0cfcff9a5b68f085f6243e721b6b66f1e4f5f764c26e15e5` / `d8b41a3976d142f85231f0c0b77bc89a1dbe4d4e15cf4bf497f07a875e5d71ee`。完整输入、golden、buildcache/log先冻结于 [evidence](evidence/filter_history_fixture_inputs_20261001/manifest.json)，尚未执行评价。

两role各7滤波fixture+5正常native+4非法native均按预期完成（正常return0、非法return1）；尚未解释评分/身份。全部280文件先SHA核验归档至 [capture evidence](evidence/filter_history_fixture_capture_20261001/manifest.json)。original/copy capture manifest SHA分别 `1df4f18a1ca369ee4e39b89b9a677a61cd771ff73e399ef124b850af565af0a4` / `de47e28b687283820cf41fe420a2fea56810bdb031375271f77c1038a376ca68`，随后独立evaluate。
