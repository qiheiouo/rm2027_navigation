# 两个局部合同的定向复现

仅软件输入。实际 bundle callback/timer/ESKF、DDS、RSP、gimbal adapter 和
lio_adapter 均参与；动态参考系探针的 raw pose 是解析合成输入，不经过动态
FAST-LIO 估计。静态平面不是定位精度或可观测性验收。没有硬件、串口或 Nav2
运动输出。原 `tests/new_car_dual` 和原始结果不修改。

环境：`rm2027_navigation:humble`，镜像身份见阶段结论。仅挂载自己的工作区：

```bash
docker run --rm --network none --ipc private --user 1000:1000 \
  --cpus 2 --memory 6g --pids-limit 256 --cap-drop ALL \
  --security-opt no-new-privileges -v "$PWD:/workspace" -w /workspace \
  rm2027_navigation:humble bash
```

容器内可运行整理入口（需要充足空间，不执行其他人的清理）：

```bash
FINAL_DUAL_PHASE=after bash tests/final_dual_closure/run_humble.sh
```

入口配置 bundle `-O0 -g0 -D_GLIBCXX_ASSERTIONS`，只构建 bundle、driver 消息和
四个相关包；不构建 async/adaptive、整车或 Gazebo。每阶段禁止覆盖已有输出。
本轮实际使用同等分步命令和既有缓存执行；新整理脚本通过语法检查，未再次从零
全套执行。不把脚本整理当作一次新的 clean rebuild。

重现修复前，在宿主机本任务根目录导出固定源码，不 reset 任何工作区：

```bash
mkdir -p build/final_dual_reproduction/baseline_fast build/final_dual_reproduction/baseline_super
git -C src/fast_lio_multi archive ae5bef29c89ba7a1004d75657db482bb277e603a \
  | tar -x -C build/final_dual_reproduction/baseline_fast
git archive 412aa4915ca4af8a5d6a9a2a1e73ba84553dc080 src/rm_localization_adapters \
  | tar -x -C build/final_dual_reproduction/baseline_super
rmdir build/final_dual_reproduction/baseline_fast/include/ikd-Tree
ln -s "$PWD/src/fast_lio_multi/include/ikd-Tree" \
  build/final_dual_reproduction/baseline_fast/include/ikd-Tree
```

再在容器内分别运行，保留 before 与 after 独立构建目录：

```bash
FINAL_DUAL_PHASE=before \
  FASTLIO_SOURCE="$PWD/build/final_dual_reproduction/baseline_fast" \
  ADAPTER_SOURCE="$PWD/build/final_dual_reproduction/baseline_super/src/rm_localization_adapters" \
  bash tests/final_dual_closure/run_humble.sh
FINAL_DUAL_PHASE=after bash tests/final_dual_closure/run_humble.sh
python3 tests/final_dual_closure/check_results.py build/final_dual_reproduction \
  --bundle after_bundle --reference after_reference
```

若原 probe 遇到退出超时，保留日志和非零结果、停止该次入口；不要放宽期限或将
失败改成通过。必要时使用另一输出根做一次复核，并明确报告两次结果。

资产：

- `prepare_visible_bundle.py` 只生成测试副本：private→public，并省去重复包含的
  未加 include guard 的 IMU header；生产方法和数学原样，记录原文件 SHA256。
- `bundle_state_trace.cpp` 复用首批 IMU 夹具及完整生产 callback/timer，记录真实
  filter pose/P、last_lidar_end_time、测量发布候选时间及实际迭代更新数。
  `publish_stamp` 是内部候选时间，不等于 DDS 已发布；ROS probe 单独观察消息。
  `recovery` 参数让故障后再输入新鲜数据，证明锁定不可由输入解除。
- `probe_epoch_ros.py` 是实际 DDS 正常/回填/缺口/回退/设备重启/重复/新进程 epoch
  对照，直接计数真实 Odometry；0.1 秒仅为软件预算。
- `probe_source_reference.py` 两个 epoch、非零 raw 初值/杆臂/初始云台角，保存同一
  base-reference 坐标中的输入、预期、输出、误差及真实 TF publisher GID。
- `check_results.py` 比较原始/修复状态、已有 AUD-038/046 stdout 和 ROS 输出；
  默认目录名对应本轮保留的第一次失败与纠正后的复核。摘要输出不自动进入 Git。

状态 trace 用固定 900 点平面、200 Hz 样本、10 Hz 配置与既有 0.15 秒帧间隔；
正常最大 IMU 源间隔 0.045 秒，完整回填 0.005 秒，真缺口 0.455 秒。
左右 40 ms 到达差和 25 ms 源时差仅检验处理连续/有限，未证明动态去畸变准确。
