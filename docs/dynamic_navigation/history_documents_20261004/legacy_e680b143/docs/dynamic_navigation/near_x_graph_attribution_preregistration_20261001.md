# near-X v2图门失败只读归因预注册

## 背景与假设

v2完整原始capture已冻结于提交 `1197c90ce111c8a5b704b15cdd7ff8759a9dd43d`。七参数门通过，但goal前graph门失败，无控制周期。假设失败可能来自actual TF/control所有权问题，也可能来自新工具对原Humble graph/remap的错误预期；归因不预设是哪一种。

## 方法与数据

只读固定v2 `goal/graph.json`、七参数响应、execution/capture_files/process/log和原已冻结源码、固定镜像installed launch与节点源。先验证capture envelope与目标文件SHA，冻结归因输入（source/input/parameter/image）后才执行评价。逐项重算原七graph gate，原判据不改；列出所有实际node/publisher/subscriber及命名空间/endpoint，找出每个不匹配项。

对每个控制topic，核对固定Humble Nav2 navigation_launch的Node/remappings及velocity smoother/chassis原launch/source。对TF、tracker/map检查此前契约、原源码与实际身份。读取具体installed文件，不能引用主机Jazzy版本当Humble依据。保存全部来源文件、路径、SHA及查询日志；没有新sim、goal、配置改动。

## 结果判据、限制与下一步

若发现runtime实际所有权违规，停止完整capture推进并记录问题，未经另行协议不改runtime。若发现tool预期与原固定source冲突，记录原门FAIL与来源证据，再独立预注册最小判据纠错；保持所有原TF/物理/风险/安全门。未知publisher不能仅因出现而加入允许集，发现源约束不能解释的identity即停止。graph发现快照不证明每条DDS消息publisher或执行时序，不做安全/预测结论。

本文件在归因前提交；没有结果。所有旧failure evidence保持，main/原src/config/install/launch不修改。
