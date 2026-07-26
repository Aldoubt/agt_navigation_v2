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
`EnableTaskExecution=false` 会保持执行按钮禁用。若只需直接启动 GUI，也可在对应 runtime
配置已准备后运行构建产物。

## 编辑流程

1. 在右侧 `任务中心 / Task Center` 的 `任务组库 / Task Library` 页签中选择“新建”。新任务在第一次成功保存前保持未保存状态。切换到“拓扑任务”页签或隐藏任务中心会自动退出地图编辑模式。
2. 填写任务名称、说明和有限循环次数。循环次数必须是 `1..maximum_loops`。
3. 选择“地图编辑”。第一次点击地图确定位置，第二次点击确定朝向；右键、Esc 或切换工具取消未完成点。
4. 表格支持名称、X、Y、Yaw、备注和启用状态编辑；按钮支持添加、复制、删除、上移、下移、反转及批量启停。
5. 地图上的点、朝向手柄与表格行同步选择。拖动点修改位置，拖动朝向手柄修改 yaw。
6. 保存前检查地图绑定和校验状态。保存使用完整覆盖、原子替换、索引更新和轮转备份，不追加旧内容。

关闭窗口、切换地图或切换当前任务时，存在未保存修改会显示“保存 / 放弃 / 取消”。删除任务需要
再次确认；删除实际把 JSON 移入同一任务目录下的 `archive/`。

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

`CONTENT_CHANGED` 表示几何相同，但地图 ID/version 或内容身份发生变化。任务可查看和重新进行全部
点/线段检查，但执行保持禁用。确认这是同一坐标系下的内容更新后，选择“更新内容绑定”并保存。

`GEOMETRY_MISMATCH` 表示 resolution、width、height、origin 或 origin yaw 变化。原任务只读，
不能自动平移、缩放或旋转。选择“复制到当前地图”创建新 ID 后，操作者必须逐点人工确认并保存。

仅修改 PGM/PNG 像素而不改变尺寸会触发内容变化；裁边、缩放、旋转或修改 YAML origin 会触发
几何失配。任务文件内只保存相对地图版本的 `navigation/map.yaml` 路径，Action 提交时才解析任务
JSON 的绝对路径。

## 离线校验

离线校验器检查 schema、有限数、稳定 ID、点数/循环上限、相邻重复、整条路径重复、地图范围、
点栅格和相邻点线段采样。世界坐标转换处理 origin yaw、PGM 左上原点、OccupancyGrid 左下原点
和图像 Y 轴翻转。线段采样步长为 `resolution * line_check_step_ratio`，默认是半个栅格。

`unknown_cell_policy` 默认为 `reject`，也可配置为 `warn` 或 `allow`。occupied 和地图外始终失败。
这些检查只针对基础栅格中的点和线段，不使用完整车辆 footprint，也不证明 Nav2 可以生成或跟踪
轨迹；运行时定位、TaskReadiness、Nav2、`agt_safety` 和底盘 watchdog 门禁不能省略。

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
Qt 把已保存任务文件的绝对路径和有限循环参数发送到
`/agt/navigation/execute_waypoint_task`。反馈显示当前点、最终结果和 `missed_waypoints`；取消会
请求项目 Action 取消 Nav2 child。Qt 不调用运动使能服务，也不发布底盘速度。

服务端再次检查 schema、当前 OccupancyGrid、地图 ID/version、YAML/image/PCD 内容身份、新鲜已接受
定位状态、TaskReadiness 与 `agt_safety`。mapping 和 offline profile 均保持执行禁用。

## 常见错误

- `selected map cannot be assigned to a map version`：地图不在正式 `versions/.../navigation/` 目录或缺少 manifest。
- `CONTENT_CHANGED`：显式重校验并更新内容绑定，不要直接执行旧文件。
- `GEOMETRY_MISMATCH`：复制为新任务并人工迁移，不要修改旧任务坐标。
- `occupied / unknown / outside`：调整点或地图策略；unknown 默认为拒绝。
- `active map_id ... required`：运行时 TaskReadiness 没有提供激活地图身份。
- `active map content hashes are not configured`：导航启动或定位状态未提供完整 YAML/image/PCD 身份。
- Action 被拒绝或执行中止：检查定位、TaskReadiness、`agt_safety`、Nav2 lifecycle 和 missed waypoint。

JSON 字段合同见 [task_group_schema.md](../interfaces/task_group_schema.md)，Action 合同见
[waypoint_task_action.md](../interfaces/waypoint_task_action.md)。
