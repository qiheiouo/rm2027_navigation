# AUD-001 定向复现

所有脚本启动真实 ROS 节点，仅产生边界消息或已有合成地图/点云；不连接硬件、Nav2 运动或串口。结果写入 `PROBE_OUTPUT`，可执行文件从 `PROBE_INSTALL` 读取。基线为 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`。

使用已有镜像 `rm2027_navigation:humble`（身份见验证报告），在本任务 worktree 根目录启动：

```bash
docker run --rm --network none --ipc private --user 1000:1000 \
  --cpus 2 --memory 6g --pids-limit 256 --cap-drop ALL \
  --security-opt no-new-privileges -v "$PWD:/workspace" -w /workspace \
  rm2027_navigation:humble bash
```

容器内构建（保留足够磁盘空间；不要清理别人的 cache）：

```bash
set -eo pipefail
source /opt/ros/humble/setup.bash
colcon --log-base build/aud001/patched_log build \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
    src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  --packages-select rm_competition_interfaces rm_relocalization_bridge \
    rm_competition_mission rm_system_monitor rm_gicp_relocalization \
  --parallel-workers 1 --build-base build/aud001/patched_build \
  --install-base build/aud001/patched_install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source build/aud001/patched_install/setup.bash
export ROS_DOMAIN_ID=172 ROS_LOG_DIR=/workspace/build/aud001/repro_ros_logs
export PROBE_INSTALL=/workspace/build/aud001/patched_install
export PROBE_BACKEND=amcl PROBE_OUTPUT=/workspace/build/aud001/repro_amcl
python3 tests/localization_validity/probe_ros.py
export PROBE_BACKEND=gicp PROBE_OUTPUT=/workspace/build/aud001/repro_gicp
python3 tests/localization_validity/probe_ros.py
export PROBE_BACKEND=amcl PROBE_OUTPUT=/workspace/build/aud001/repro_native_amcl
python3 tests/localization_validity/probe_backends.py
export PROBE_BACKEND=gicp PROBE_OUTPUT=/workspace/build/aud001/repro_native_gicp
python3 tests/localization_validity/probe_backends.py
export PROBE_BACKEND=amcl PROBE_NO_NOMOTION=1 PROBE_OUTPUT=/workspace/build/aud001/repro_stationary
python3 tests/localization_validity/probe_backends.py
unset PROBE_NO_NOMOTION
export PROBE_OUTPUT=/workspace/build/aud001/repro_timing
python3 tests/localization_validity/probe_timing.py
export PROBE_OUTPUT=/workspace/build/aud001/repro_profiles
python3 tests/localization_validity/probe_profiles.py
export PROBE_EXPECT_FIXED=1 PROBE_OUTPUT=/workspace/build/aud001/repro_original_fixed
python3 tests/localization_validity/original_gate_probe.py
```

`probe_profiles.py` 仅作原生 launch 参数求值，不执行 launch 节点。`probe_ros.py` 等待正常 DDS 接线完成，但故障撤销仍采用生产预算，不放宽超时。`probe_backends.py` 沿用既有二维世界和审计非对称 GICP 几何；AMCL 默认显式调用无运动更新，`PROBE_NO_NOMOTION=1` 取消该调用，保留两个独立回归场景。

原有测试：

```bash
colcon --log-base build/aud001/test_log test \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
    src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  --packages-select rm_relocalization_bridge rm_competition_mission rm_system_monitor rm_gicp_relocalization \
  --build-base build/aud001/patched_build --install-base build/aud001/patched_install
colcon test-result --test-result-base build/aud001/patched_build --verbose
```

修复前原 probe 需要独立基线，不使用首批 FAST-LIO 分支或其他人的 build。先在**宿主机的本 worktree**导出源码（容器没有挂载共享 Git 元数据）：

```bash
mkdir -p build/aud001/baseline_src
git archive bd6cf689d3d5aca5c8edd430ff760ca5ceb27496 src/rm_relocalization_bridge \
  | tar -x -C build/aud001/baseline_src
```

再进入上述容器，构建和运行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base build/aud001/baseline_log build \
  --base-paths build/aud001/baseline_src/src --packages-select rm_relocalization_bridge \
  --build-base build/aud001/baseline_build --install-base build/aud001/baseline_install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source build/aud001/baseline_install/setup.bash
unset PROBE_EXPECT_FIXED
export PROBE_INSTALL=/workspace/build/aud001/baseline_install
export PROBE_OUTPUT=/workspace/build/aud001/repro_original_baseline
python3 tests/localization_validity/original_gate_probe.py
```

已有基线 native AMCL 的补充观察可先 source 修复后 install 提供新消息的 Python 类型，再 source 基线 install，令 `PROBE_INSTALL` 指向基线，同时设置 `PROBE_BASELINE=1 PROBE_NO_NOMOTION=1 PROBE_OBSERVE_ONLY=1`。此模式只观察基线节奏，不输出稳定性通过结论。

测试源码原样保留自753131e6；历史报告和运行输出留原独立分支，不随本精简分支上传。当前版本、验证范围与恢复见[软件保存摘要](../../docs/new_car_software_saved_20261012.md)。
