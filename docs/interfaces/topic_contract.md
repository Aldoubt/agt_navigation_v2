# v2.5 Topic Contract

本文是仓库内 **canonical cross-module ROS topic** 的唯一真源。代码、launch、bag profile、
健康检查、Qt/Web 展示和其他文档引用跨模块正式 topic 时必须使用下表名称；topic 名称不是可随意
替换的显示标签。单个 package 的内部、debug、可视化 topic 可以由对应 package/architecture 文档
定义，但不得冒充第二套跨模块 canonical interface。

## 正式 topic

| Topic | 类型 | frame / 语义 | owner | 主要消费者 |
| --- | --- | --- | --- | --- |
| `/agt/sensors/lidar/custom` | `livox_ros_driver2/msg/CustomMsg` | MID360 原始输入 | Livox driver | self-filter、bag |
| `/agt/sensors/lidar/custom_filtered` | `livox_ros_driver2/msg/CustomMsg` | 过滤后的 MID360 输入；仍保持原始 LiDAR message frame 与逐点字段 | `agt_livox_self_filter` | FAST-LIVO2 |
| `/agt/sensors/imu/data` | `sensor_msgs/msg/Imu` | IMU 数据 | sensor adapter/driver | FAST-LIVO2 |
| `/agt/sensors/camera/image` | `sensor_msgs/msg/Image` | 相机图像 | camera adapter | mapping/perception（可选） |
| `/agt/sensors/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机标定 | camera adapter | mapping/perception（可选） |
| `/agt/sensors/gnss/fix` | `sensor_msgs/msg/NavSatFix` | GNSS fix | GNSS adapter | localization（可选） |
| `/agt/mapping/odometry` | `nav_msgs/msg/Odometry` | `odom -> base_footprint` | FAST-LIVO2 adapter | localization、health |
| `/agt/mapping/registered_points` | `sensor_msgs/msg/PointCloud2` | canonical registered cloud；`odom` frame | FAST-LIVO2 adapter | localization、perception、map processing |
| `/agt/chassis/odometry` | `nav_msgs/msg/Odometry` | 底盘里程计 | `agt_chassis` | health、read model |
| `/agt/perception/ground_cloud` | `sensor_msgs/msg/PointCloud2` | `base_footprint` | `agt_perception` | Nav2/local inspection |
| `/agt/perception/obstacle_cloud` | `sensor_msgs/msg/PointCloud2` | `base_footprint` | `agt_perception` | Nav2 obstacle layer |
| `/agt/perception/ground_plane` | `not implemented in baseline` | `base_footprint` | reserved perception interface | diagnostics/consumers |
| `/agt/map/local_occupancy` | `nav_msgs/msg/OccupancyGrid` | map-local working grid | reserved map interface | mapping UI/audit |
| `/agt/map/global_occupancy` | `nav_msgs/msg/OccupancyGrid` | `map` | Nav2 map server | global costmap/planner |
| `/agt/map/waypoints` | `geometry_msgs/msg/PoseArray` | `map` | reserved task/UI interface | preview/task authoring |
| `/agt/system/health` | `agt_interfaces/msg/SystemHealth` | structured health snapshot | `agt_system_manager` | all clients |
| `/agt/system/task_readiness` | `agt_interfaces/msg/TaskReadiness` | shared fail-closed task gate | `agt_system_manager` | navigation/safety/clients |

## Package-local / debug topic boundary

例如 self-filter 的 `/agt/sensors/lidar/self_filter/geometry`、removed-points 可视化和
`/diagnostics` 属于 package-local/debug evidence，不是 FAST-LIVO2 或导航模块之间的新正式数据
接口。其类型、QoS 与启停规则由
[`livox_custom_self_filter.md`](../architecture/livox_custom_self_filter.md) 定义。

## 命名收口

`/agt/mapping/registered_points` 是唯一正式注册点云 topic。以下名称均为禁止新增、禁止
在运行时配置中使用的历史称呼，不是 alias，也不保证存在：

- `registered_cloud`（旧 component/display 标识）
- `/agt/mapping/registered_points_lidar`
- `/agt/mapping/registered_cloud`

迁移旧 bag 或旧配置时，应在边界适配器中一次性映射到 canonical topic，并在迁移记录中
注明来源；不得让下游节点继续订阅历史名称。公共合同要求该 cloud 的 `frame_id` 为 `odom`，不能
通过在 topic 名中加入传感器 frame 形成第二套接口。

## 规则

- 一个 canonical topic 只能有明确的 runtime owner；兼容 remap 不能制造第二个正式名称。
- 表中标为 reserved/not implemented 的 topic 是 v2.5 预留名称，不得被伪造为已发布能力；实现时仍须遵守本文 owner/type/frame 约束。
- package-local/debug topic 不得成为跨模块控制、定位或安全状态的隐式依赖；需要升级为正式接口时必须先进入本合同。
- 需要新增或修改 canonical topic 时，必须同时更新本文、相关接口文档、`AGENTS.md` 与
  `docs/roadmap/v2_5.md`，并增加 contract test。
- `/agt/map/global_occupancy` 是导航地图；不得把 semantic keepout 或临时障碍写回基础地图。
- topic liveness 不等于 readiness；运动前必须满足结构化 health、TaskReadiness、定位和安全门禁。
