# 新设备磁盘清理记录

## 背景与假设

用户授权磁盘紧张时清理绝对无用内容。构建后文件系统仅约 412 MB 可用；若旧失败 Docker 构建没有任何可写数据、挂载或标签，则其容器及中间镜像可删除。

## 方法与数据

执行提交 `03e6991`。审计 2026-06-25/26 的五个失败 Livox SDK2 git clone 构建容器：均 exit 128、无挂载、`docker diff` 为空、SizeRw=0，镜像均无标签。删除前再次检查。保存准确 ID、镜像 hash、命令、状态与写入大小于 [before.json](evidence/device_disk_cleanup_20260930/before.json)。

仅执行列出的五个 `docker rm` 与五个 `docker image rm --no-prune`；未 force 或全局 prune。tagged 镜像、旧 navigation 容器、core、研究数据、地图、录包和 evidence 均保留。

## 结果与限制

两项命令 exit 0。可用空间增加 `15,558,705,152` bytes，清理后 `15,970,885,632` bytes；此数字是文件系统前后可用空间差，期间其他进程活动也可能影响测量，不是 Docker 单独核算的精确层回收量。详见 [after.json](evidence/device_disk_cleanup_20260930/after.json)。

## 下一步

磁盘已允许继续离线研究，继续监测可用空间。未知文件和历史实验不作为清理目标。
