# 可移植near-X首次只读preflight失败

## 背景与假设

按 [原预注册](portable_near_x_assets_preregistration_20261001.md)，验证只经路径重绑定的原near-X资产可解析。此阶段未启动仿真、controller、TF或goal/control。

## 方法与数据

源码/prepare `143b20fbcd392f5416bcf81a4cdfe0ff9ca9fcee`，执行 `13876ae21dd38e3ec9c12eb9f4c47b613478cbd4`；input SHA `fbc2d8aa842a3b1e49769fce7b05050222ebdabf81303daeedf61a23f9515602`，parameter SHA `370d98bf82a6acf348d6dfa75b3b07f2acdfbd05ef04655fcb62463227330d71`，固定81b325镜像。196源/installed资产、15包prefix已冻结；四launch逆变换及world/model/profile字节门通过。

## 结果与归因

preflight总体FAIL、进程return1，summary SHA `2545bdc696ce054749dd94e35a428b8588d6bd001f5b68287e6775d4543d7bf0`。两个失败门：

- 文件引用检查错误地把所有以“/”开头的AST字符串当文件。六个真正文件引用全部存在；`/scan`、`/cmd_vel`等topic及LogInfo文字被误判不存在。不能据此声称场景资产缺失。
- `ros2 launch <file> --show-args params_file:=...`被当前Humble CLI拒绝（return2，unrecognized arguments）。直接读取安装的LaunchCommand.add_arguments，确认其位置参数含可选launch_file_name和nargs=*的launch_arguments；本次选项穿插位置导致解析失败，未进入场景运行。

第一次parser探查错误导入不存在的ros2launch.verb模块，return1；其帮助输出与异常同样保留。随后正确读取ros2launch.command.launch的安装源码，成功。全部失败和归因文件逐SHA核验保存在 [evidence](evidence/portable_near_x_preflight_failure_20261001/manifest.json)，不覆盖原input或结果。

## 限制与下一步

此失败属于验证harness，尚不能推断完整launch可运行。原绑定逆变换、profile/world/model、15包身份保持；历史FAIL、TF、正式配置、安全门不变。

先独立预注册v2：以AST实际文件上下文识别world assignment、PythonLaunchDescriptionSource、Node的-file参数及config_file；要求完整六引用集合存在，ROS topic/LogInfo仍保留但不作为文件。CLI选项放位置参数之前。新源码/参数/输入目录与hash重新冻结再查询，旧失败保持；不启动场景。
