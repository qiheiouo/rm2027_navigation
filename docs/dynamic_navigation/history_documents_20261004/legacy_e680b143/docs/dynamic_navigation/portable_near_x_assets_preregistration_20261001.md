# 新设备near-X可移植资产与只读launch门预注册

## 背景与假设

新设备原安装路径不同，旧/work runner不能执行；历史replay失败结论保持。组合telemetry八critic门12/12通过后，先验证原near-X七资产能仅经路径重绑定形成可审计的新设备场景。reference/comparison launch实际存在且SHA符合旧manifest；此前五项inventory是子集。

## 方法

固定81b325镜像，仅当前研究分支；先提交协议，再生成独立build目录，源码/输入/参数/镜像/库身份冻结提交后执行只读preflight。七原资产为crossing.sdf、x_axis_obstacle.sdf、base/reference/comparison/dynamic四个launch、run_near_x.sh；逐文件核对旧manifest source/artifact SHA。profile.yaml继续使用已迁移原byte，SHA `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`。

world/model/profile原字节不变。四launch只将旧 `/work/dynamic_prediction_near_x_fast_motion_20260930`前缀换为独立新build路径；dynamic launch唯一 `/ws/src/rm_simulation/config/course_dynamic_bridge.yaml`换为当前workspace绝对路径。每锚点数量与完整逆变换必须符合原源字节，记录精确diff；不改变任何参数、timer、spawn位置、物理尺寸、TF或launch层级。旧runner完整byte保存作参考，明确不执行、不伪造旧安装；完整采集需要后续独立新supervisor协议。

保留robot start(5.25,1.10)、goal(5.25,-1.10)、X-slider spawn(4.9,0)、amplitude=.9、period=6；此门不启动或评价运动。静态walls、scan/odom/canonical TF/chassis stub仍由原包与原launch拥有。

## 验收门与数据

prepare冻结所有原/新场景文件、profile、contracts、相关src与installed package资产、当前image和已通过组合门/三库SHA，记录预期overlay顺序。使用已验证workspace→guarded→history→snapshot环境，仅执行ros2 pkg prefix以及 `ros2 launch <new dynamic launch> --show-args params_file:=<frozen profile>`。该CLI只展开参数，不启动仿真、controller、TF或发送goal/control。

原source/derived identities、全字节逆变换、world/model/profile不变、所有运行launch引用的文件存在且不含旧/work或/ws前缀、实际required package prefixes和controller/V1 overlays正确、show-args exit0及完整source/参数不变门必须通过。原profile的batch300/dt.1/steps30/std/temp/footprint/padding/critics/安全相关字段不改。preflight所有stdout/stderr/identity结果先保存，不覆盖旧evidence。失败则保存并停止，不换小footprint或放宽判据。

## 限制与下一步

路径与参数展开成功只证明新设备scene绑定/依赖可解析，不证明组件可运行、预测有效或闭环安全，不恢复旧镜像身份。原runner仍不可用于新设备。门通过后独立预注册有界完整研究capture（同原scene/profile、actual consumer receipt、V1/history/cycle完整性、canonical TF/真实模块身份），新数据先冻结再replay，不以单次成功证明算法有效。

## preflight前冻结

prepare源码 `143b20fbcd392f5416bcf81a4cdfe0ff9ca9fcee`；input SHA `fbc2d8aa842a3b1e49769fce7b05050222ebdabf81303daeedf61a23f9515602`，parameter SHA `370d98bf82a6acf348d6dfa75b3b07f2acdfbd05ef04655fcb62463227330d71`。196相关src/installed资产、15包prefix查询及三库/已通过组合门身份，四launch精确diff/逆变换、world/model/profile原字节和旧runner参考已冻结于 [evidence](evidence/portable_near_x_inputs_20261001/manifest.json)。独立scene目录portable_near_x_inputs_20261001/scene，尚未执行launch。
