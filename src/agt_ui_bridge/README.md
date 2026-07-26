# agt_ui_bridge

主界面采用项目维护的
[`Aldoubt/Ros_Qt5_Gui_App:agt-navigation-v2`](https://github.com/Aldoubt/Ros_Qt5_Gui_App/tree/agt-navigation-v2)，
固定源码快照位于 `third_party/ros_qt5_gui_app`，原项目 GPL-2.0 许可证与署名保持不变。
GUI 负责地图显示、机器人位姿、速度控制、重定位、单点/多点导航、路径显示、
栅格和拓扑地图编辑，以及版本化任务组的离线编辑和基础栅格校验；本包负责 V2 配置、启动和地图 I/O。

## 构建主界面

首次构建前安装上游依赖：

```bash
sudo apt-get install -y \
  qtbase5-dev qtbase5-private-dev libqt5svg5-dev \
  libsdl2-dev libsdl2-image-dev libeigen3-dev libgtest-dev
```

然后在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
./tools/build_ros_qt5_gui_app.sh
```

构建过程会按上游 CMake 配置下载 Advanced Docking System、yaml-cpp、nlohmann/json 和
topology_msgs，产物写入 `build/ros_qt5_gui_app`。固定 fork 提交以
[`third_party/README.md`](../../third_party/README.md) 为唯一 provenance 记录，构建产物
不会提交 Git。

启动脚本只使用本仓库构建产物，找不到时会提示执行
`tools/build_ros_qt5_gui_app.sh`。如需显式指定构建或 runtime 根目录，可设置
`ROS_QT5_GUI_BUILD_DIR` 和 `ROS_QT5_GUI_RUNTIME_DIR`。

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_ui_bridge ros_qt5_gui.launch.py profile:=mapping
ros2 launch agt_ui_bridge ros_qt5_gui.launch.py profile:=navigation
ros2 launch agt_ui_bridge ros_qt5_gui.launch.py profile:=teach \
  map:=/absolute/path/to/map.yaml start_map_io_bridge:=false
```

以上是 GUI/地图 I/O 的独立启动入口，不会启动 Nav2、定位、安全或
`ExecuteWaypointTask` server。在线任务执行必须使用完整 `agt_bringup` navigation 链，命令和
断车默认见任务库工作流文档。

首次启动会把对应模板复制到 profile 独立目录：
`runtime/gui/ros_qt5_gui_app/<profile>/config.json`。GUI 修改只保存在各自 runtime，互不覆盖，
也不会污染 vendor 源码。启动预检会从新版 profile 补齐缺失的能力键，但不覆盖已保存的用户值。
需要恢复仓库默认配置时执行：

```bash
ros2 run agt_ui_bridge start_ros_qt5_gui_app.sh \
  --profile navigation --reset-config
```

## V2 默认映射

Task Library 的完整离线编辑、地图绑定、旧格式迁移和执行流程见
[`docs/workflows/qt5_offline_task_group_editor.md`](../../docs/workflows/qt5_offline_task_group_editor.md)。
Qt5 主窗口将它内嵌在右侧 `任务中心 / Task Center`，并与原有拓扑任务通过页签切换；
切换离开 Task Library 或隐藏任务中心会退出地图点位编辑模式。
版本化默认值来自 `config/task_library.yaml`，启动时合并到 profile runtime 配置；已保存的操作者
设置不会被覆盖，`TaskLibraryRoot` 始终由启动脚本解析为当前工作区的绝对 `runtime/maps` 路径。

- mapping 地图：`/agt/map/mapping_occupancy`
- navigation 地图：`/agt/map/global_occupancy`
- 机器人里程计：`/agt/mapping/odometry`
- 重定位初值：`/initialpose`
- 单点导航目标：`/goal_pose`
- 多点任务：`/agt/navigation/execute_waypoint_task` Action
- 手动速度：`/agt/cmd_vel_manual`
- 全局/局部路径：`/plan`、`/local_plan`
- 示教只读 profile：`/agt/teach/reference_path` 与 transient-local
  `/agt/teach/route_annotations`；显示方向、左/右转、掉头和原地转向，任务库、规划预览和执行均关闭
- 全局/局部代价地图：`/global_costmap/costmap`、`/local_costmap/costmap`；大地图
  profile 默认 `EnableCostmapDisplay=false`，需要调试时显式改为 true，常规查看使用 RViz
- 机器人 frame：`base_footprint`
- GUI 显示 frame：mapping 为 `odom`，navigation 为 `map`
- 拓扑地图：`/agt/map/topology`、`/agt/map/topology/update`

单点 `/goal_pose` 由项目桥接为 Nav2 `NavigateToPose`；Qt 的 **Start Task Chain** 直接调用
项目 `ExecuteWaypointTask` Action，并以 Nav2 Action 反馈、结果和 missed waypoint 判断成败。
添加导航点采用两次点击：第一次固定位置，第二次从位置指向朝向点并计算 yaw；
右键、Esc 或切换工具会丢弃未完成的点。拓扑线装饰刷新固定为 10 Hz，避免抢占大地图交互。
手动速度已经接入 `agt_safety -> agt_chassis`，但必须先显式使能安全层。GUI 可直接打开、
编辑并保存 PGM/YAML；错误 YAML 会提示而不退出，切换地图会先清空旧拓扑并拒绝同名 sidecar
中的越界点。保存后的导航地图放在 `runtime/maps/`，下一次启动 map server 时使用对应 YAML。

mapping profile 的 `FixedFrameId=odom` 与 FAST-LIVO2 建图链一致；navigation profile 固定为
`map`，并要求 NDT/ICP 已发布 `map -> odom`。mapping profile 的
`EnableTaskExecution=false` 会同时禁用按钮并在 ROS2 channel 再次拒绝多点执行；navigation
profile 显式设置为 true 才允许调用项目 Action，该键缺失时按 false 拒绝处理。该开关不是安全层替代，
真实运动仍必须通过 `agt_safety`。

## 备用工具

轻量 Python 编辑器仍保留用于无上游 GUI 或地图 I/O 调试：

```bash
ros2 launch agt_ui_bridge map_editor.launch.py
ros2 run agt_ui_bridge map_io_bridge.py
```

它通过标准 `nav2_msgs/srv/LoadMap`、`SaveMap` 服务加载和保存地图，并将编辑结果发布到
`/agt/map/edited`。主界面优先使用 `ros_qt5_gui.launch.py`。

## 农业语义标注边界

语义地图 1.0 合同已经定义，详细格式见
[`docs/interfaces/semantic_map_schema.md`](../../docs/interfaces/semantic_map_schema.md)，机器可读
约束位于 `config/semantic_schema.yaml`。所有语义几何使用 `map` frame 和米制坐标，独立保存
为 GeoJSON 与 `coverage.yaml`，不得写回基础 PGM。

当前已完成 schema、合法/非法样例，以及以下无 ROS、无 GUI 基础库：

- `map_transform.py`：grid、PGM image、Qt scene 与 `map` world 坐标互转；
- `semantic_model.py`：FeatureCollection 与 coverage 参数数据模型；
- `semantic_io.py`：GeoJSON/YAML 重载、地图哈希检查、只读降级和原子写入；
- `semantic_validation.py`：结构、frame、ID、必需对象和基本 Geometry 检查；
- `semantic_scene.py`：对象状态与 undo/redo。

这些模块不发布 topic、不发布 TF，因此没有 QoS 和 TF 责任。TASK-03 已在本包内新增独立
Qt5 语义编辑器；当前编辑器除语义对象外，也支持直接对底图 PNG 做本地三值修图，
用于把 CloudCompare 观测图逐步整理成正式导航图；底图编辑支持连续自由画笔、像素宽度加减和
拖动预览后松开提交的直线工具。`map_editor_qt5.py` 仍保留用于 ROS
话题式 OccupancyGrid 调试，两者 I/O 方式不同。
TASK-04 已引入 Shapely 完成完整几何合法性检查。

运行编辑器前安装几何依赖：

```bash
sudo apt-get install -y python3-shapely
```

## 启动语义编辑器

编辑已有 Nav2 地图时，显式传入地图 YAML 和车辆 profile：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP_YAML=runtime/maps/greenhouse_01/greenhouse_01.yaml
PLATFORM_PROFILE=profiles/platforms/bunker.yaml
ros2 launch agt_ui_bridge semantic_editor.launch.py \
  map:="$MAP_YAML" \
  platform_profile:="$PLATFORM_PROFILE"
```

重载已保存任务时再传入 GeoJSON；程序会从同目录读取 `coverage.yaml`：

```bash
SEMANTIC_MAP=runtime/maps/greenhouse_01/semantic/semantic_map.geojson
ros2 launch agt_ui_bridge semantic_editor.launch.py \
  map:="$MAP_YAML" \
  semantic_map:="$SEMANTIC_MAP" \
  platform_profile:="$PLATFORM_PROFILE"
```

编辑器支持 field boundary、exclusion zone、row centerline、access lane、entry pose 和 work direction
绘制，选择对象后可拖动顶点，提供对象图层显隐、footprint 预览、undo/redo、保存/另存、重载
及未保存退出提示。地图首次显示时自动适配窗口，光标位于地图上时可使用滚轮缩放、中键拖动
平移，也可通过“适配地图”恢复全图视野。语义线采用深色分类色和白色外描边，绘制预览、已完成
对象及 footprint 在黑白栅格区域上均保持可见。使用“地图障碍 / 地图自由 / 地图未知”工具时，
可直接对基础 PNG 做本地三值修图，并可在“自由画笔 / 画直线”之间切换；保存时会先原位写回底图 PNG/YAML，再保存
`semantic_map.geojson` 与 `coverage.yaml`。基础地图哈希不匹配时自动降级为只读，禁止覆盖。

区域右键完成前会检查边界自交；无效草稿会保留，可用 `Backspace` 撤回最后一个点。
完成对象后自动切换到“选择/顶点”，拖动黄色控制点可重复编辑；选中控制点后可用方向键按
`1 px` 精调，`Shift+方向键` 按 `0.2 px` 精调。底图完整笔划和语义对象修改均支持撤销/重做。

“路线预览”面板提供三种只读离线输入解释：区域自动覆盖、标注线即道路、作物行间道路。
“作物行间道路”按作业方向统一各行端点后，将相邻作物行两端分别取中点，临时生成 `N-1`
条道路中心线；作业方向线的位置不作为道路或首行，只使用其首尾向量的角度。该过程不改写源
GeoJSON。“通行道路”工具绘制独立 `access_lane` 折线，启用时会与上述派生道路合并，适合标注
温室边缘道路和入口联络线；其中心线必须位于作业区内且不穿过内部障碍。面板会显示当前解释
规则，并可选择允许倒车的 Reeds-Shepp 或仅前进 Dubins；二者都是不读取栅格与语义障碍的
几何连接，仅用于产生待审计候选。入口位姿不属于 OpenNav action 输入，面板会显示入口到
候选首点的距离，并明确标记入口 approach 尚未规划。
启动固定 `execution_enabled=false` 且 `start_rviz=false` 的 Coverage 预览，在当前画布叠加橙色
候选路线，并以红色叉号显示 canonical footprint 与黑色/unknown 底图、field 外部或 semantic
exclusion/keepout 的冲突采样。面板显示点数、长度、预计时间、转弯、倒车距离、底图/语义碰撞
数量和稳定错误码；安全审计始终不可执行，不替代 TASK-10。编辑器为该预览创建专属
ROS Domain，并让订阅端与子进程使用同一 Domain，避免已退出或
异常遗留的预览节点向新一轮面板回灌状态；停止预览时优先等待 ROS launch 正常清理子节点。
启动编辑器前必须 source 外部 coverage 工作区，否则面板会明确报告找不到规划包。

新建空任务只显示一条绘制顺序引导；开始绘制后，尚缺的必需对象显示为“待绘制”，而不是系统
故障。底层校验和保存门禁不变，缺项、自交、越界等问题在保存前仍会阻断。

该进程仅进行本地文件编辑，不创建 ROS topic、service、action 或 TF，也没有 QoS 责任。
结构和几何错误会显示稳定 code/object ID、高亮关联对象并阻止保存。当前检查多边形自交、
exclusion/entry 包含关系、基础地图范围、入口 footprint 与 field/exclusion 的碰撞，以及显式
边界净距。覆盖路径本身的可执行性属于 Fields2Cover 接入后的 Validator 任务。

## 启动语义地图服务器

先启动发布 `/agt/map/global_occupancy` 的 Nav2 map server，再启动独立语义服务器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

SEMANTIC_MAP=runtime/maps/greenhouse_01/semantic/semantic_map.geojson
PLATFORM_PROFILE=profiles/platforms/bunker.yaml
ros2 launch agt_ui_bridge semantic_map_server.launch.py \
  semantic_map:="$SEMANTIC_MAP" \
  platform_profile:="$PLATFORM_PROFILE"
```

检查状态与服务：

```bash
ros2 topic echo /agt/map/semantic_status --once
ros2 service call /agt/map/semantic/validate std_srvs/srv/Trigger "{}"
ros2 service call /agt/map/semantic/reload std_srvs/srv/Trigger "{}"
ros2 service call /agt/map/semantic/load nav2_msgs/srv/LoadMap \
  "{map_url: 'runtime/maps/greenhouse_01/semantic/semantic_map.geojson'}"
```

markers、mask 和状态均为 reliable/transient-local/depth 1。TASK-06 的
`/agt/map/keepout_mask` 将 enabled exclusion/keepout 和默认 field 外部写为 `100`，可通行区域
写为 `0`；分辨率、尺寸、origin/yaw 和 frame 与基础地图完全一致。加载使用候选态事务，失败
状态不会清除上一份有效产品。TASK-07 已由 Nav2 global costmap 通过
`/agt/map/keepout_filter_info` 消费该 mask；启动与 fail-open 注意事项见 `agt_navigation`
README。完整合同见
[`docs/interfaces/semantic_map_server.md`](../../docs/interfaces/semantic_map_server.md)。
