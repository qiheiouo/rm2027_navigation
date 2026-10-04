# 原shadow tracker实际consumer capture：数据冻结

## 背景、假设与方法

按 [预注册](shadow_consumer_capture_preregistration_20261001.md)运行原headless sim、关闭Nav2/RViz，增加已通过离线转发门的tracker实验记录包装。加载原YAML，只绑定simtime、/scan及独立research fixture map输入；TF/.08s/两线程/算法参数未改。raw observer与typed consumer各20s wall采样，无目标、速度command、critic或硬件。

研究map复用已冻结ok_map.cdr，经独立topic以1Hz发布，header为实际sim clock，几何不变；这是接口fixture，不能验证动态检测或预测准确性。raw observer只在独立进程扩展map消息catalog，原工具源码未改。

## 数据身份与结果

采集源码 `d331b6a50cb8a1be6c59d47f64cced5015faede4`，执行 `ab2dd2fbd94295665eeecb83ca0bad1dcd2f841c`。input SHA `363be19416798e4e3c52aeedb396f082e1789d60513350b37ca2dd166d4e94fe`，参数SHA `0c8a1eb033f5a663d63af6757ade4e6bdfaa9ca815a90ca844da23ddf9b41faf`，124资产/模块；镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

| 冻结对象 | SHA / 数量 |
|---|---|
| capture identity | `b2fe9cdc3bbee0ad1ab9f4c034d6ec924001e8ce9f6521aa7a4c1ba654b8e745` |
| observer manifest | `27dce5d9c7901e288c68355d926a1fde94b6b3acdf6c462defe0527616094112`；8881事件、8731注册文件 |
| consumer manifest | `e8d3e3a1226b16a0328157cff9850022aaf53b6c784f5f047d732d565b482a01`；754事件、538注册文件 |

四个本次进程最终return0；两个采集进程在监督等待内自然退出，raw manifest为complete。graph满足无导航控制节点、/cmd_vel无publisher、canonical TF现有publisher集合一致的门。两个manifest逐文件集合/大小/SHA核验通过。[全部raw/typed payload、事件、graph及日志](evidence/shadow_consumer_capture_20261001/manifest.json)在任何跨接收方分析前无损打包/验证/提交，build原件保留。

## 限制

consumer日志有一次源0.002s、earliest TF0.020s的过去外推警告，以及退出阶段 `The following exception was never retrieved: cannot use Destroyable because destruction was requested`。全部保留；return0和manifest完整不证明异步退出完全无异常。此处只核验存储/身份门，不分析callback或lookup成功率，也不以这次录制宣称算法成功。

consumer typed CDR由本地serializer生成，不能当原DDS字节；observer receipt不能代替consumer entry。测量开销、2CPU/software-render、不同启动/结束窗口都影响时序。fixture map不是实际静态环境解释；没有未来truth或MPPI rollout/critic数据。旧observer严格TF门及监督FAIL、历史capture数值FAIL、V1冻结结论全部保持。

## 下一步

另行预注册跨接收方native字段对齐：按source stamp/frame寻找scan，再比较全部native字段，不用重序列化CDR padding作相等门；所有未匹配/歧义保留。核对实际consumer query的frame/stamp/Time clocktype/0.08s参数、入口/返回/异常时间、退出配对及prediction source/processing身份，并与同源observer原timeout0结果并列。先冻结本次原文件hash/参数，后评价；不能补未来TF或把窗口外/缺失记录抹掉。
