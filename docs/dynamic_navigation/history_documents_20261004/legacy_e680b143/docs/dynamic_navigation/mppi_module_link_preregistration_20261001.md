# 新设备MPPI / V1动态链接身份前置门预注册

## 背景与假设

实际shadow consumer字段/TF门通过，仍未有新设备完整MPPI cycle capture。guarded/controller/critics和V1库只被确认存在，不能假定runtime选中同一模块。假设source新workspace+guarded overlay后，固定Humble镜像能解析这些已编译模块及依赖，且实际loaded controller/critics来自guarded、V1来自workspace。

## 冻结、方法与范围

先提交本协议，再开发独立Python只读探针。prepare冻结commit、镜像81b325、明确controller/critics/V1文件SHA和路径、setup/manifest/插件XML/optimizer trace source/critic source、旧研究profile、portable near-X场景及runner引用；保存指定AMENT_PREFIX_PATH和LD_LIBRARY_PATH的实际值及参数hash，input提交后才evaluate。

Docker network none、2CPU/6GB/drop capabilities，仅sourceworkspace与guarded local setup，不使用缺失/work wrapper，不伪装旧image。读取package prefix与原capture接口源码。使用ctypes.CDLL按controller→critics→V1顺序RTLD_NOW|RTLD_LOCAL加载准确库路径；只检查链接、static registration初始化与/proc/self/maps实际加载身份，不创建controller/critic实例、不configure、不启动ROS节点/仿真、TF或publisher，不评价算法。

expected controller `5a17595b01311d3ba4133a02b403c97bca30e950c690bd830902eee471c3d2ad`，critics `f92f2a5de291b0455dcf1242680fbaecd58460bf68d32d41fff6c191fec307f1`，V1 `7da70180f7272ffc4a02e85b7f4b3dc613297ea6bb8027456ab0feaadb595010`。实际loaded target必须与这些身份相同，不允许悄悄混入/opt的同名controller/critics。记录所有可读loaded .so路径/SHA及每次load成功/错误，源码各trace字段/成本分项接口只读梳理；源码存在不等于capture已工作。

## 判据、限制与下一步

source/input/parameter/environment hash先检查；所有目标module解析成功，/proc maps中critical库身份严格相同且唯一，否则保留失败停止，不换库/改RPATH/降低判据。旧capture数值FAIL和V1冻结结论不变。loaded基础依赖属于固定镜像和overlay环境，不宣称任意ABI/类初始化/score行为已验证。

通过只允许进一步预注册独立完整cycle采集协议，必须保持原scene/profile/footprint/padding/TF/critic/安全门并验证plugin实例、实际consumer/costmap/rollout/critic/filtered output身份。禁止直接修改critic/正式配置、单纯扩大batch/调temperature/权重或将新数据冒称旧capture恢复。结果及全部阴性/哈希写docs/dynamic_navigation，main不改，不merge/push/deploy。

## 加载前冻结登记

源码 `92dc8a1268e2ade76ea5d23756042dc16fcde0b6`；29资产input SHA `b5110d4c6cad5a4384d77c4306b9da24710e060c19a073a1272aba50ca4b9134`，参数SHA `f0ca7a31ef3809aba9ebefc93bd4e7d5e25fdb7def9c2707b3d08edc559d02a3`。[全部输入](evidence/mppi_module_link_inputs_20261001/manifest.json)先提交再load；真实overlay环境字符串冻结，不绑定缺失旧prefix。
