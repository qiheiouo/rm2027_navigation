# T-DT前端桥接

选择性复用已迁移且冻结的YAstar、collision-aware路径简化、SfcSquare；来源/上游pin/许可/范围/逐文件哈希见[provenance.json](provenance.json)。vendor源码逐字保留旧迁移补丁。桥接独立编写，唯一改变是输入契约适配及验证，没有修改vendor。

搜索/静态走廊在控制循环之外执行，不发布速度；完整MinimumSnap/规划器迁移仍在冻结research提交e680b143中，未删除或废弃。本切片不加载完整ROS插件或旧动态路线，可以用既有后端生成的路径替换这里的简化路径，仍必须提供同样静态走廊证书。

YAstar采用地图原点；SfcSquare冻结版本采用不同offset约定，桥接在零原点局部坐标调用SfcSquare后显式平移。第一次小地图测试触发Eigen断言，已定位为该桥接原点约定错误；修复未缩小足迹或改unknown。Python StaticRoute独立逐cell认证矩形走廊，并仅在同一证书通过时合并局部squares。地图原点、占用判定和走廊内容均留证据。

以C++20、Eigen3构建：`bash frontend/build.sh /tmp/mpc-tdt-frontend`（在原型根目录）。GCC13对未调用的vendor _removeRedundant产生stringop-overflow警告，原构建日志已保存；当前桥接调用getBound，不调用该冗余合并函数。尚未取得全vendor运行域的安全接受。

替换/删除本目录只影响离线原型，正式planner与Nav2保持原状。


继续轮新增 `--path` 输入模式：Nav2/既有前后端给出路径，仅使用冻结 SfcSquare
构造走廊，并独立验原始地图；该模式不会执行 YAstar 搜索或改变路径拓扑。
密集 Nav2 路径只删除共线点；256点以上、碰撞/未知路径或不支持的map输入
明确拒绝，由规划前端压缩/重规划，不能让 MPC 改道。离线默认搜索模式保留。
