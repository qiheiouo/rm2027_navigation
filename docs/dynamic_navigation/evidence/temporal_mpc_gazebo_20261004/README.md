# Actual Gazebo evidence checkpoint

[结果与限制](../../temporal_mpc_gazebo_results_20261004.md)。每个events.jsonl.gz是
原始完整CDR+JSON记录的无损gzip；原字节sha256和压缩文件sha256见manifest。
复制或解压到新临时目录运行只读审计，不覆盖本证据目录。

正式固定配对：shadow06/mpc01（横穿）、head_on02/head_on_mpc01（迎面）。
每场景world/nav2/tracker/bridge逐字节相同；每轮启动source_snapshot.tar.gz，实际
binary与镜像身份固定。paired_summary含终态、接触、原生/worker时延、接收间隔、
真实双向选择与相关重验拒绝；来源延迟/TF/公共契约见每轮audit/ros_capture_audit。
calibration02及shadow05是修正SDF后的进入门见证。static_frontend_audit仅静态course。
source_tf_replay只从canonical /tf和/tf_static重建，不使用oracle位姿或伪造源stamp。

早期shadow01..04、calibration01、head_on01与全部启动错误原样保留，运动结论中
namespace受损的轮次撤回。无启动source快照的调试记录不声称完全源码复现。
旧错误Contact topic缺消息不可当作无接触；最终正接触来自物理传感器。

全部动态接受=false；安全B0见证缺失，false_block=null，单轮seed未锁定，无净收益
结论。图中sampled clearance只是机械外包体诊断，不是连续安全证书。85 tests两环境
通过不消除物理失败。原工作区与保护refs完整性见preservation.json。
