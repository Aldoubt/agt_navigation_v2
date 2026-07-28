# 核心接口草案

当前导航启动顺序、Qt5/Nav2/安全数据流以及门禁矩阵见
[`current_navigation_startup_and_dataflow.md`](../architecture/current_navigation_startup_and_dataflow.md)。

## Topics
- `/agt/sensors/lidar/custom`：`livox_ros_driver2/msg/CustomMsg`，MID360 原始输入。
- `/agt/sensors/lidar/custom_filtered`：`livox_ros_driver2/msg/CustomMsg`，由
  `agt_livox_self_filter` 在 FAST-LIVO2 之前发布；保留通过点的完整字段和原始顺序。
- `/agt/sensors/lidar/self_filter/boxes`：`visualization_msgs/msg/MarkerArray`，可选的
  `base_footprint` 自滤除盒可视化。
- `/diagnostics` 中的 `agt_livox_self_filter`：profile、帧、计数、无效点、TF 失败和耗时诊断。
- `/agt/sensors/lidar/points`：通用 `sensor_msgs/msg/PointCloud2` 输出，不是当前 MID360 原始驱动输入。
- `/agt/sensors/imu/data`
- `/agt/mapping/odometry`
- `/agt/mapping/registered_cloud`
- `/agt/mapping/status`
- `/agt/perception/ground_cloud`
- `/agt/perception/obstacle_cloud`
- `/agt/perception/semantic_cloud`
- `/agt/localization/status`
- `/agt/localization/status_text`
- `/agt/localization/global_pose`
- `/agt/localization/coarse_pose`
- `/agt/localization/candidate_pose`
- `/agt/localization/aligned_points`
- `/agt/localization/relocalize`

`LocalizationStatus.map_hash` is the active localization PCD content identity in the canonical
`sha256:<64 lowercase hexadecimal characters>` form. It is recomputed at map readiness time and is
used to bind configured candidates and last-pose records; a processing record `pcd_sha256` is checked
when present.

`LocalizationStatus.ERROR_STALE_SCAN` and `ERROR_INVALID_SCAN_TIMESTAMP` are fail-closed errors for
the latest registered cloud. The localization node reports the cloud stamp and computed age; dynamic
TF used for that attempt is queried at the same stamp.
- `/agt/navigation/cmd_vel`
- `/agt/navigation/cmd_vel_raw`
- `/agt/navigation/status`
- `/agt/safety/cmd_vel`
- `/agt/safety/emergency_stop`
- `/agt/safety/status`
- `/agt/chassis/cmd_vel`
- `/agt/chassis/monitor_cmd_vel`：只读 CAN 监测模式的故意无人发布占位输入；不可用于导航或安全链。
- `/agt/chassis/odometry`
- `/agt/chassis/status`
- `/agt/chassis/connected`
- `/battery`：`sensor_msgs/msg/BatteryState`，BUNKER 状态桥接提供电压，百分比未知时保持未知。
- `/agt/experiment/events`
- `/agt/map/mapping_occupancy`: OctoMap 建图过程中的二维工作图，采用
  `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`；它是持久监看快照，不是固定周期的健康流，
  也不是受管会话最终候选。FINALIZE 会固定其 PGM/YAML 作为 `online_preview` 后离线重建候选
- `/agt/map/octomap_occupancy`: 离线静态障碍补全使用的射线/free-space 基图（内部 topic）
- `/agt/map/static_obstacle_evidence_status`: 离线重复观测障碍补全统计 JSON，仅用于审计
- `/agt/map/global_occupancy`: 导航模式下由 Nav2 map server 发布的已保存静态地图
- `/agt/teach/reference_path`: `nav_msgs/msg/Path`，绑定示教资产的 transient-local 只读参考线
- `/agt/teach/route_annotations`: `visualization_msgs/msg/MarkerArray`，由项目后端根据版本化阈值生成的
  方向、转弯、掉头和原地转向标注；Qt 仅显示，不作为验证或执行批准

建图健康证据同时检查 `/agt/mapping/odometry`、`/agt/mapping/registered_points_lidar` 和
`/agt/map/mapping_occupancy` 的消息类型和持久快照可用性；该话题不使用三秒新鲜度门限。Web 只读预览会对二维栅格做有界降采样，
不发布新地图，也不参与建图算法或导航决策。Web 还可以对
`/agt/mapping/registered_points` 的 `PointCloud2` 做固定体素降采样预览；该缓存不等于持久化
导航 PCD，不能馈入定位、规划、验证或控制。

`agt_chassis` 的 `operation_mode:=monitor` 只接收 BUNKER CAN 状态，启动状态桥接和驱动但不启动
`agt_safety` 或命令 guard；真实导航必须使用
`/agt/navigation/cmd_vel_raw -> /agt/navigation/cmd_vel -> /agt/safety/cmd_vel -> /agt/chassis/cmd_vel`。
CAN 接口的 up/down 配置不属于 ROS/Web 接口，必须由主机管理员预先完成。
- `/agt/navigation/preview_footprint`: planner-only 离线预览起点处的 canonical 多边形车体
- `/agt/navigation/waypoint_preview_request`: Qt 提交的 planner-only `PoseArray`；无预览适配器时
  Qt 必须立即报不可用，不得显示为已经开始规划
- `/agt/navigation/waypoint_preview_status`: advisory 字符串状态；逐段发布
  `planning:<current>/<total>`，终态为 `succeeded:*`、`failed:*` 或 `rejected:*`
- `/agt/map/semantic_markers`: 语义服务器发布的 transient-local 标注可视化
- `/agt/map/keepout_mask`: 语义服务器发布、与基础地图严格对齐的 transient-local 语义 mask
- `/agt/map/keepout_filter_info`: Nav2 Costmap Filter Info Server 发布的 transient-local keepout 元数据
- `/agt/map/semantic_status`: 语义加载、校验和产品构建诊断
- `/agt/coverage/path_raw`: Fields2Cover 原始覆盖路径，永远禁止直接执行
- `/agt/coverage/path_components`: swath 与连接段组件
- `/agt/coverage/path_reconstructed`: 从 PathComponents 重建的扁平 Path
- `/agt/coverage/path_semantics`: 原始 Path 全区间 SWATH/CONNECTION 分类及稳定 swath ID JSON
- `/agt/coverage/swaths`: 覆盖 swath MarkerArray
- `/agt/coverage/headland`: field 与 planning field MarkerArray
- `/agt/coverage/status`: 覆盖请求、规划结果和稳定错误码诊断
- `/agt/coverage/path_validated`: TASK-10 全部检查通过时的 Path，失败时为空
- `/agt/coverage/collision_poses`: 插值后发生完整 footprint 碰撞的 PoseArray
- `/agt/coverage/footprint_markers`: 无效采样 footprint 的 MarkerArray
- `/agt/coverage/validation_report`: 含无效 component/swath ID 的稳定键序 JSON 报告
- `/agt/coverage/path_repaired`: TASK-12 成功替换无效 CONNECTION 并最终复验后的 Path
- `/agt/coverage/repair_report`: 修复数量、耗时、保留 swath ID 和最终验证 JSON
- `/agt/coverage/task_status`: TASK-14 当前阶段、swath 进度和剩余距离诊断
- `/agt/coverage/simulation_report`: 离线运动学时间、路径长度、换向和语义分段统计 JSON
- `/agt/coverage/comparison/markers`: 多候选路线的只读彩色 MarkerArray，不提供可执行 Path
- `/agt/coverage/comparison/report`: 多候选长度、时间、方向、语义和面积指标的稳定键序 JSON
- `/agt/coverage/comparison/status`: 离线候选比较进度与错误诊断

## 语义地图服务
- `/agt/map/semantic/load`: `nav2_msgs/srv/LoadMap`，输入 GeoJSON 路径或 `file://` URL
- `/agt/map/semantic/reload`: `std_srvs/srv/Trigger`
- `/agt/map/semantic/validate`: `std_srvs/srv/Trigger`

完整 QoS、状态和事务规则见 [`semantic_map_server.md`](semantic_map_server.md)。

## 系统管理器

- `/agt/system/health`：`agt_interfaces/msg/SystemHealth`，周期结构化健康快照。
- `/agt/system/task_readiness`：`agt_interfaces/msg/TaskReadiness`，共享 fail-closed 任务门禁。
- `/agt/system/robot_state`：`agt_interfaces/msg/RobotState`，2 Hz 与输入变化即时发布的统一读模型；
  reliable transient-local depth 1，字段仍需独立检查 known/freshness。
- `/agt/system/get_health`：`agt_interfaces/srv/GetSystemHealth`。
- `/agt/system/get_robot_state`：`agt_interfaces/srv/GetRobotState`。
- `/agt/system/evaluate_task_readiness`：`agt_interfaces/srv/EvaluateTaskReadiness`。
- `/agt/system/change_mode`：`agt_interfaces/action/ChangeSystemMode`，只接受白名单 profile。
- `/agt/localization/set_mode`：`agt_interfaces/srv/SetLocalizationMode`，有界重定位策略。

Web、Qt bridge 和 Action server 都消费这些机器接口；不得解析旧
`/agt/localization/status_text` 参与控制。

## 业务 manager 接口

- `/agt/maps/list`：`agt_interfaces/srv/ListMapVersions`。
- `/agt/maps/manage`：`agt_interfaces/srv/ManageMapVersion`。
- `/agt/maps/active`：`agt_interfaces/msg/MapVersionSummary`，reliable transient-local depth 1。
- `/agt/data/bags/list`：`agt_interfaces/srv/ListBagSessions`。
- `/agt/data/bags/manage`：`agt_interfaces/srv/ManageBagSession`。
- `/agt/data/bags/status`：`agt_interfaces/msg/BagSessionSummary`，reliable transient-local depth 1。
- `/agt/data/experiments/list`：`agt_interfaces/srv/ListExperiments`，返回
  `ExperimentSummary[]`，不向客户端暴露实验 manifest。

完整操作、错误码、路径与 owner 规则见
[`business_manager_services.md`](business_manager_services.md)。

## 覆盖规划服务
- `/agt/coverage/plan`: `std_srvs/srv/Trigger`，按当前语义任务发起一次异步规划
- polygon action：`/agt/coverage/polygon/compute_coverage_path`
- annotated rows action：`/agt/coverage/rows/compute_coverage_path`
- `/agt/coverage/repair`: `std_srvs/srv/Trigger`，显式修复当前无效连接段
- `/agt/coverage/execute`: `agt_interfaces/action/ExecuteCoverageTask`，串联语义加载、规划、验证、可选修复和 Nav2 `FollowPath`
- `/agt/coverage/compare`: `std_srvs/srv/Trigger`，重新执行离线多候选比较

转换、GML、Validator、状态和安全边界见 [`coverage_planning.md`](coverage_planning.md)，统一任务
接口字段和代码生成边界见 [`coverage_task_action.md`](coverage_task_action.md)。

## Nav2 语义过滤
- global costmap 顺序：`StaticLayer -> KeepoutFilter -> InflationLayer`
- `/global_costmap/keepout_filter/toggle_filter`: `std_srvs/srv/SetBool`，运行时启停语义层
- FilterInfo/mask 缺失时 Humble 插件 fail-open 并告警；实车运动前必须确认语义状态 `LOADED`
- 语义成本只存在于 costmap 过滤层，不写回 `/agt/map/global_occupancy` 或基础 PGM

## 导航动作与速度链
- `/navigate_to_pose`: `nav2_msgs/action/NavigateToPose`
- `/navigate_through_poses`: `nav2_msgs/action/NavigateThroughPoses`
- `/follow_waypoints`: `nav2_msgs/action/FollowWaypoints`，仅由项目任务桥接调用
- `/agt/navigation/execute_waypoint_task`: `agt_interfaces/action/ExecuteWaypointTask`
- `/agt/navigation/task_status`: `std_msgs/String`，项目多点任务 JSON 状态
- `/goal_pose`: Qt5/RViz2 的 `geometry_msgs/PoseStamped` 兼容入口
- `/agt/navigation/cmd_vel_raw -> /agt/navigation/cmd_vel -> /agt/safety/cmd_vel -> /agt/chassis/cmd_vel`

`/goal_pose -> agt_goal_pose_bridge -> NavigateToPose` 当前是兼容/调试路径，不订阅
`TaskReadiness`，也不执行任务版本绑定和 chassis-connected 共享门禁；正式多点任务必须使用
`/agt/navigation/execute_waypoint_task`。

## TF 责任
- `map -> odom`: `agt_relocalization` 唯一发布
- `odom -> base_footprint`: `agt_mapping_fast_livo2_adapter` 唯一发布
- `base_footprint -> base_link`、`base_link -> lidar_link/imu_link`: `robot_state_publisher`
  （`agt_description`）集中管理
- FAST-LIVO2 backend 的 TF 发布关闭；BUNKER driver 的 odom TF 默认关闭，不能添加第二个
  publisher

当前 `agt_system_manager` 对这些 edge 使用 `lookup_transform(..., Time())` 判断可查询性，
尚未比较 transform header 时间戳与当前时间；因此当前门禁语义是“TF 可查询”，不是严格的
“TF 新鲜”。
