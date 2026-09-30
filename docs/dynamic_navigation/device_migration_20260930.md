# 动态预测研发设备迁移记录（2026-09-30）

## 当前状态

代码和本轮精简证据在 `codex/dynamic-differential-risk` 分支；应把该分支推到用户确认的 `https://gitee.com/qiheiovo/rm2027_navigation.git`，**不合并或覆盖** `main` 与 `experiment/dynamic-prediction-v1`。新设备以推送结果中显示的确切 commit 为起点。本机原有 `~/.ssh/id_rsa` 属于别人，未用于此次推送，也未加入用户账号。为迁移新建的专用 Ed25519 私钥是 `/home/wpie/.ssh/id_ed25519_rm2027_gitee`，公钥指纹 `SHA256:GPV5cWKCYloufXoQJcCWpIZwEiQmZ4j/dfWHPhCeQDU`；用户已把公钥加入自己的 Gitee 账号，迁移完成后可从账号中撤销该公钥。

[阶段决策](dynamic_prediction_stage_decision_20260929.md)与[较快箱体结果](near_x_fast_motion_result_20260930.md)分别记录研究门槛和最新留出数据。运行 V1 仍未经稳定安全验收，任何新设备构建也不自动改变该结论。下一算法诊断是固定周期 47 的 std ×4 **离线**扩展，按已提交的[预登记](near_x_fast_motion_preregistration_20260930.md)执行，不重复寻找有利相位。

## Git 与较大数据的边界

1. `docs/dynamic_navigation/evidence/near_x_fast_motion_20260930/`：可由 Git 搬走的约 6.5 MB 精简证据，包含选中周期及邻近周期的冻结输入、300 条 rollout、原始地图、预测流、物理真值、原生评分及固定种子覆盖。`manifest.json` 列出每件文件的 SHA256；导出器 `experiments/dynamic_prediction_v1/frozen_cycle/export_near_x_fast_migration.py` 在本机重新读取原归档并验证选周期和原始分析。
2. 本机隔离工作树下 `build/dynamic_prediction_near_x_fast_motion_20260930/`：一次仿真的完整 538 周期归档，约 234 MB。其独立 `tar.zst` 副本和 Git bundle 放在本机 `/home/wpie/rm2027_navigation/build/dynamic_differential_risk_worktree/build/migration/`，均被 `.gitignore` 排除，**不会随着 Git 推送上传**。完整仿真压缩包 SHA256 为 `063f643161c408fb3bd043da91a25bd59484bd1e5458f91512267fb02b7c3343`；Git bundle 的最新 SHA256 在同目录 `SHA256SUMS`。搬新设备时若需要完整时间序列，须另行拷贝压缩包并按 SHA256 核对。
3. `/home/wpie/tdt_p2b`：用户提供的约 5 GB 历史原始归档，仍只在本机。Git 中有从它导出的选择性证据和哈希，不能声称整个目录已云备份。需要重新分析未导出的原始日志时，应另外传输此目录。
4. Docker 镜像 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6` 是本机运行身份记录，**不是 Git 文件**。新设备须按当前分支 Dockerfile/依赖重新建环境；检查镜像、代码、插件和运行 profile 与目标 HEAD 一致，不能直接复用旧镜像 ID 或旧编译产物。

## 新设备核验

在新设备上使用**该设备自己的** Gitee 凭据检出 `codex/dynamic-differential-risk`，无需复制本机私钥。先检查 `git rev-parse HEAD`、`git status --short` 与推送结果中报告的 commit。下面的核验只需要 Python 标准库，针对 Git 中的精简证据：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib, json
root = Path('docs/dynamic_navigation/evidence/near_x_fast_motion_20260930')
manifest = json.loads((root / 'manifest.json').read_text())
for name, record in manifest['files'].items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    assert actual == record['artifact_sha256'], name
print('verified', len(manifest['files']), 'fast-X evidence files')
PY
```

如另行传输完整归档或 Git bundle，再用同目录的 SHA256 清单核对。开发仍遵守 [`docs/development_workflow.md`](../development_workflow.md)：隔离分支、文档同步、单线程构建；不改 T-DT 核心、tracker/KF、footprint、padding 或原 `0.05 m` 安全门。高算力结果仍要在实际设备全栈负载下验收。
