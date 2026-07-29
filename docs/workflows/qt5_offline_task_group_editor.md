# Qt5 离线任务组编辑与执行

## 适用范围

Task Library 用于维护绑定到一个不可变地图版本的有序导航点任务组。Qt5 负责本地编辑、
文件管理、基础栅格校验、Action 提交和反馈显示，不实现任务执行、安全、定位或
`FollowWaypoints` 状态机。执行权威始终是：

```text
Qt5 Task Library
  -> /agt/navigation/execute_waypoint_task
  -> agt_navigation WaypointTaskServer
  -> Nav2 FollowWaypoints
  -> agt_safety
  -> agt_chassis
```

任务坐标是 `map` frame 下的米制坐标，朝向是弧度。Qt scene 或图像像素坐标不会写入任务文件。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_interfaces agt_navigation agt_ui_bridge
source install/setup.bash
./tools/build_ros_qt5_gui_app.sh
```

Qt5 源码是 `third_party/ros_qt5_gui_app` 中的 GPL-2.0 固定快照；构建产物位于
`build/ros_qt5_gui_app`，不写回 vendor 目录。

## 离线启动

地图必须来自正式版本目录：

```text
runtime/maps/<map_id>/versions/<map_version_id>/
  manifest.yaml
  navigation/map.yaml
  navigation/map.pgm
  tasks/
```

不需要启动 Nav2、定位、底盘或其他 ROS 节点即可编辑：

```bash
MAP_YAML="$(realpath runtime/maps/<map_id>/versions/<map_version_id>/navigation/map.yaml)"
ros2 run agt_ui_bridge start_ros_qt5_gui_app.sh \
  --profile offline --map "$MAP_YAML"
```

此入口仍启动 Qt 进程自身的 ROS2 channel 插件，但不要求其他节点在线；offline profile 的
`EnableTaskExecution=false` 会保持执行按钮禁用。navigation/offline profile 都隐藏底图编辑、
地图保存和旧拓扑任务页签；加载的 READY PGM/YAML 只读，任务保存只写当前版本的 `tasks/`。
若只需直接启动 GUI，也可在对应 runtime 配置已准备后运行构建产物。

## 编辑流程

1. 在右侧 `任务中心 / Task Center` 选择“新建”。navigation/offline profile 只显示版本化 Task Library；新任务在第一次成功保存前保持未保存状态，并自动进入“布置任务点”模式。
2. 填写任务名称、说明和有限循环次数。循环次数必须是 `1..maximum_loops`。
3. 第一次点击地图确定位置，第二次点击确定朝向；右键、Esc、关闭“布置任务点”或切换工具会取消未完成点。
4. 已有拓扑点可从“拓扑点”下拉框选择，再点击“添加选中点”。拓扑新增、删除、改名或移动会立即刷新候选项；添加操作只把当时的点名、`map` 坐标和 yaw 快照到任务，不建立隐式联动，也不复制拓扑连线。
5. 表格支持名称、X、Y、Yaw、备注和启用状态编辑；按钮支持添加、复制、删除、上移、下移、反转及批量启停。
6. 地图上的点、朝向手柄与表格行同步选择。拖动点修改位置，拖动朝向手柄修改 yaw。
7. 保存前检查地图绑定和校验状态。保存只校验各任务点端点，不把相邻点的显示连线当作机器人直线路径。保存使用完整覆盖、原子替换、索引更新和轮转备份，不追加旧内容；成功后自动退出布点模式。空任务会提示先布置任务点，不再只显示底层英文校验错误。
8. 在 navigation bringup 或下文 `waypoint_preview.launch.py` 已提供规划器时，至少有两个启用点即可点击“预览路径”。Task Library 会显示当前规划段/总段数，并在 `/plan` 显示逐段拼接的实际规划路径；每段有有限超时。单独启动 offline GUI 只支持编辑保存，点击预览会立即提示预览服务未启动，不再停留在“已提交”；预览允许 Nav2 绕过占据格，但不代表任务获准执行。
   每个后续分段从上一段 Nav2 实际返回的终点开始，因此 planner 在目标容差内调整中间点时，
   预览路径仍保持连续；任务点顺序和保存内容不会被改写。

预览与 Bunker 正式 global costmap 使用相同的 `0.75 m` 膨胀半径和 `4.0` 代价衰减系数。
膨胀半径表示障碍物对周围规划代价的影响范围；它不会修改只读 PGM，也不会替代平台 profile
中的 canonical footprint。路径仍需结合定位、动态障碍、安全状态和实车制动距离验收。

关闭窗口、切换地图或切换当前任务时，存在未保存修改会显示“保存 / 放弃 / 取消”。删除任务需要
再次确认；删除实际把 JSON 移入同一任务目录下的 `archive/`。

底图像素需要修订时，不要在当前 READY 版本目录中覆盖保存。使用
[`mapping_task_navigation_workflow.md`](mapping_task_navigation_workflow.md) 生成候选；Qt
`candidate` profile 只允许原位编辑该候选，完成后通过 ManageMappingSession COMMIT 登记新版本，
再在新版本上创建或显式迁移任务。实时 `mapping` profile 不开放 Task Library，因此不会再用
尚无 `map_version_id` 的建图快照创建任务。

## 文件操作

任务保存在：

```text
runtime/maps/<map_id>/versions/<map_version_id>/tasks/
  task_index.json
  <task_group_id>.json
  <task_group_id>.json.bak.1
  archive/
```

“另存为”和“复制”要求新的安全任务 ID，不会静默覆盖已有 ID。“重命名”修改操作者显示名称；
稳定 `task_group_id` 只有另存或复制时改变。`task_index.json` 是本地可重建索引，任务 JSON
才是任务内容来源。

## 导入与导出

“导入旧 JSON”接受原 Qt 格式：

```json
{"points":[{"name":"P1","x":1.0,"y":2.0,"theta":0.0}]}
```

导入器把 `theta` 转为 `yaw`、生成 `wp_0001` 等稳定 ID，并绑定当前地图版本。导入不会修改或
覆盖源文件；导入结果在显式保存前保持未保存状态。“导出旧 JSON”只导出启用点，用于兼容旧工具。
项目 Action server 同时读取旧格式和 schema v1，因此 headless 客户端无需 Qt 临时转换文件。

## 地图绑定

`MATCHED` 要求地图版本、几何、YAML/image 哈希和定位 PCD 哈希一致，允许编辑、保存和执行。

`CONTENT_CHANGED` 表示几何相同，但地图 ID/version 或内容身份发生变化。任务可查看并重新进行全部
任务点端点检查和规划预览，但执行保持禁用。确认这是同一坐标系下的内容更新后，选择“更新内容绑定”并保存。

`GEOMETRY_MISMATCH` 表示 resolution、width、height、origin 或 origin yaw 变化。原任务只读，
不能自动平移、缩放或旋转。选择“复制到当前地图”创建新 ID 后，操作者必须逐点人工确认并保存。

仅修改 PGM/PNG 像素而不改变尺寸会触发内容变化；裁边、缩放、旋转或修改 YAML origin 会触发
几何失配。任务文件内只保存相对地图版本的 `navigation/map.yaml` 路径，Action 提交时才解析任务
JSON 的绝对路径。

## 离线校验

离线校验器检查 schema、有限数、稳定 ID、点数/循环上限、相邻重复、整条路径重复，并检查每个
启用任务点端点的地图范围和基础栅格状态。世界坐标转换处理 origin yaw、PGM 左上原点、
OccupancyGrid 左下原点和图像 Y 轴翻转。相邻点之间的界面连线不做栅格采样，因为它不是 Nav2
必须跟踪的直线轨迹。

`unknown_cell_policy` 默认为 `reject`，也可配置为 `warn` 或 `allow`。occupied 和地图外始终失败。
这些检查只针对基础栅格中的任务点端点，不使用完整车辆 footprint，也不证明 Nav2 可以生成或跟踪
点间轨迹。至少两个点时使用“预览路径”检查当前 Nav2 planner 能否逐段绕行；运行时定位、
TaskReadiness、Nav2、`agt_safety` 和底盘 watchdog 门禁仍不能省略。

## 在线执行

在线提交必须启动完整 navigation bringup，而不是只启动 GUI profile。显式提供同一个不可变
地图版本的二维地图、定位 PCD、处理记录和地图身份：

```bash
VERSION_ROOT="$(realpath runtime/maps/<map_id>/versions/<map_version_id>)"
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$VERSION_ROOT/navigation/map.yaml" \
  global_map_pcd:="$VERSION_ROOT/pointcloud/localization_map.pcd" \
  global_map_processing_record:="$VERSION_ROOT/pointcloud/localization_map.processing.yaml" \
  map_id:=<map_id> map_version_id:=<map_version_id> \
  start_gui:=true start_chassis:=false
```

`start_chassis:=false` 适合断车配置与门禁检查；完成 CAN、方向、急停和 watchdog 实车验收后，
才可由操作者显式改为 `true`。单独运行 `ros_qt5_gui.launch.py profile:=navigation` 只启动 GUI
和地图 I/O，不启动项目 Action server、Nav2、定位或安全执行链。

执行按钮只在 profile 允许、任务已保存、绑定为 `MATCHED` 且离线校验通过时启用。点击执行后，
Qt 只把 `map_id`、`map_version_id`、`task_group_id`、`task_revision`、
`expected_content_sha256`、有限循环参数和 UUID `client_request_id` 发送到
`/agt/navigation/execute_waypoint_task`。机器人端 Task Registry 根据 ID 解析任务 JSON，
并校验 revision 与内容 hash；Qt 不再发送本机绝对 `task_file` 路径。任务未保存、仍有脏改动、
缺少 content hash 或机器人端尚未同步时，界面显示 `任务尚未同步到机器人`，不会退回到本地路径执行。
反馈显示当前点、最终结果和 `missed_waypoints`；取消会请求项目 Action 取消 Nav2 child。
Qt 不调用运动使能服务，也不发布底盘速度。

服务端再次检查 schema、当前 OccupancyGrid、地图 ID/version、YAML/image/PCD 内容身份、新鲜已接受
定位状态、TaskReadiness 与 `agt_safety`。mapping 和 offline profile 均保持执行禁用。
`/agt/navigation/session_status` 是机器人端权威会话状态，使用 reliable + transient-local。
Qt 断联不会取消当前任务；重连后客户端应读取最近 session，而不是根据本地 Action handle 推断任务已停止。

## 常见错误

- `selected map cannot be assigned to a map version`：地图不在正式 `versions/.../navigation/` 目录或缺少 manifest。
- `CONTENT_CHANGED`：显式重校验并更新内容绑定，不要直接执行旧文件。
- `GEOMETRY_MISMATCH`：复制为新任务并人工迁移，不要修改旧任务坐标。
- `occupied / unknown / outside`：调整点或地图策略；unknown 默认为拒绝。
- `任务尚未同步到机器人`：任务仍是本地草稿，或机器人端 Task Registry 没有相同 ID/revision/hash。
- `NO_ACTIVE_MAP`：运行时 map manager 没有发布 READY 活动地图身份。
- `LOCALIZATION_PCD_HASH_MISSING`：活动地图缺少可验证定位 PCD hash。
- Action 被拒绝或执行中止：检查定位、TaskReadiness、`agt_safety`、Nav2 lifecycle 和 missed waypoint。

JSON 字段合同见 [task_group_schema.md](../interfaces/task_group_schema.md)，Action 合同见
[waypoint_task_action.md](../interfaces/waypoint_task_action.md)。
