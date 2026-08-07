# agt_map_processing

职责：把全局 PCD 或注册点云转成二维栅格、可通行性地图和对比输出。

## OctoMap 二维投影 baseline

当前已迁移旧仓库的在线 OctoMap 投影链：

- 输入：`/agt/mapping/registered_points` (`sensor_msgs/msg/PointCloud2`，`lidar_link` frame)。
- 建图输出：`/agt/map/mapping_occupancy` (`nav_msgs/msg/OccupancyGrid`)。
- 默认分辨率：`0.05 m`。
- 默认仅把 `0.10 m <= z <= 1.00 m` 的点作为投影障碍候选。
- 全图投影输入默认节流到 `0.2 Hz`，先以 `0.10 m` 体素和每帧 `8000` 点上限压缩，再通过
  `/agt/mapping/octomap_points` 送入 OctoMap；
  节流器只保留尚未处理的最新一帧，并用 steady-time timer 取帧；它保留原始
  `lidar_link` 和点云时间戳，使 OctoMap 使用同一时刻的动态 TF，而不会在处理落后时继续
  投递已经退出 TF cache 的旧帧；每次投递后还会等待对应 OccupancyGrid 发布完成才释放
  下一帧，等待上限默认 `60 s`，避免 OctoMap 自身序列化期间形成第二层订阅积压；
  FAST-LIVO2 原始注册点云仍以其正常频率发布，局部障碍链不经过该节流器。该限制用于避免
  大型 OctoMap 序列化速度低于 10 Hz 输入而造成 Message Filter 积压，实际频率应在独立 bag
上根据地图更新延迟和峰值内存重新验证。

Humble 的 `octomap_server` 2.3.1 使用参数名 `point_cloud_min_z` /
`point_cloud_max_z`。它的 `incremental_2D_projection=true` 分支在当前源码中不会调用
`update2DMap`，会得到有尺寸但全 `-1` 的二维图，因此本 baseline 明确使用完整二维投影。
OctoMap 以 volatile QoS 发布内部 `/agt/map/mapping_occupancy_raw`，节流节点确认投影完成后
再把同一消息以 transient-local QoS 中继到公共 `/agt/map/mapping_occupancy`；Qt、SaveMap 和
bag 的公共接口不变。`latch=false` 避免在没有订阅者时仍序列化所有完整 3D 输出。

OctoMap 是持久 3D 八叉树，会同时保存射线经过的 free 节点和终点 occupied 节点；`0.05 m`
分辨率、`40 m` 最大射线和 MID360 原始点数的组合会让节点数随覆盖区域持续增长。在线 baseline
已将最大射线改为 `15 m`，启用增量二维投影和压缩发布。`Message Filter queue is full` 或
`timestamp is earlier than all the data in the transform cache` 表示投影处理/TF 时间轴已经落后，
不是可以通过增大队列解决的内存泄漏；增大队列只会保留更多待处理点云。
建图 bag 同时记录 `/agt/mapping/octomap_points`，用于核对进入 OctoMap 的实际帧、header 和 TF。

只需要 FAST-LIVO2 PCD 时关闭全图 3D 投影，避免在长 bag 回放中建立持久 OctoMap：

```bash
ros2 launch agt_bringup mapping_mode.launch.py \
  use_sim_time:=true start_sensor:=false start_chassis:=false \
  start_octomap_projection:=false start_rviz:=false start_gui:=false
```

需要在线 2D 预览时保留投影，但可以进一步降低输入负载：

```bash
ros2 launch agt_map_processing octomap_projection.launch.py \
  use_sim_time:=true input_rate_hz:=0.1 \
  cloud_voxel_leaf_size:=0.15 cloud_max_points:=5000
```

最终全局 PGM 不应依赖这个无限增长的在线 3D 树；应使用离线“射线基图 + 重复点云证据”链，
并检查队列丢弃、位姿匹配和资源报告。

建图工作图与导航静态图分开：OctoMap 只发布 `/agt/map/mapping_occupancy`，导航模式的
`map_server` 才发布 `/agt/map/global_occupancy`。这样建图 RViz 不会误显示仍在运行的旧导航地图。

FAST-LIVO2 和投影节点都启动后，回放传感器 bag：

```bash
ros2 launch agt_map_processing octomap_projection.launch.py use_sim_time:=true
```

参数集中在 [`config/octomap_projection.yaml`](config/octomap_projection.yaml)。实测前重点根据
地面位置、机器人高度和作物冠层调整 `point_cloud_min_z`、`point_cloud_max_z`、
`occupancy_min_z` 和 `occupancy_max_z`。

生成地图后，在仓库根目录执行：

```bash
cd "$AGT_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p "$AGT_WS/runtime/maps"
ros2 launch agt_map_processing save_occupancy_map.launch.py \
  map_prefix:="$AGT_WS/runtime/maps/mid360_map"
```

会生成 `mid360_map.pgm` 和 `mid360_map.yaml`。保存节点使用 transient-local 订阅，投影节点
也保留最后一帧，因此可在回放结束后启动保存命令。该二维图是全局静态地图候选，不包含
Nav2 local costmap 的瞬时局部障碍。
保存默认使用 `free_thresh=0.196`、`occupied_thresh=0.65`，以保持 PGM 的 `205` unknown
像素在 Nav2 重新加载时不被解释为空闲空间。
保存入口不会覆盖已有的 PGM/YAML；需要重新生成时请使用新的 `map_prefix`，并保留已验证地图。

OctoMap 使用当前帧 `lidar_link` 点云和 `odom -> lidar_link` TF，因此射线原点会随机器人
运动。车辆 `base_link -> lidar_link` 外参完成标定和高度阈值调优前，输出地图只用于链路
验证与后端对比，不作为最终导航地图。

## 大包离线静态障碍补全

单纯使用固定世界坐标高度带投影，容易漏掉低矮障碍、细杆、较高障碍，也会在 LIO 的
`odom.z` 漂移后选择错误高度。大包离线处理采用“射线基图 + 重复点云证据”的混合方法：

- bag 内已有的 `/agt/map/mapping_occupancy` 提供射线清除后的 free/unknown 基图；
- `/agt/mapping/registered_points` 按每帧机器人 `base_footprint` 高度选择
  `0.05 m .. 2.00 m` 障碍点；
- 点云等待其时间戳两侧的里程计样本并插值 `x/y/z/yaw`，随后变换到
  `base_footprint`；非有限点和 canonical 多边形 footprint 向外 `0.12 m` 范围内的自体点
  被过滤；同一栅格必须被至少 3 个不同点云帧观测才写为 occupied；
- `obstacle_padding=0.05 m` 是显式的点证据栅格补偿，不代替 Nav2 InflationLayer；保存的
  PGM 不把机器人 footprint 写成占据代价，也不烘焙 Nav2 膨胀代价。
- 最后以 bag 中全部里程计位姿栅格化 canonical 多边形 footprint，并将车辆真实扫掠区域
  标为空闲；这只清除车体实际通过的空间，不能用圆形半径扩大清理轨迹两侧障碍。

回放使用 bag 自带基图时，将输入基图改名，避免和增强输出 topic 冲突：

```bash
ros2 launch agt_map_processing offline_static_obstacle_map.launch.py \
  platform_profile:="$(realpath profiles/platforms/bunker.yaml)" \
  rebuild_raytraced_baseline:=false

ros2 bag play runtime/rosbag/<mapping_bag> --clock --rate 1.0 \
  --topics /agt/map/mapping_occupancy \
           /agt/mapping/registered_points \
           /agt/mapping/odometry \
  --remap /agt/map/mapping_occupancy:=/agt/map/octomap_occupancy
```

只有 bag 没有有效 OccupancyGrid、并且包含完整 `/tf`、`/tf_static` 与 lidar-frame 点云时，
才设置 `rebuild_raytraced_baseline:=true` 并同时回放这些 TF/topic。回放结束后检查
`/agt/map/static_obstacle_evidence_status`，再从增强后的 `/agt/map/mapping_occupancy` 保存新
PGM/YAML；不要覆盖已验证地图。

正式候选地图使用 `1.0×` 回放，状态中的 `queue_overflow_drops` 和
`pose_mismatch_drops` 必须为零。更高倍速只适合快速调试，不能作为最终静态地图证据。

保存点云增强候选图后，直接读取 rosbag2 中全部里程计生成完整扫掠层，避免高倍速 ROS
回放丢失中间位姿：

```bash
ros2 run agt_map_processing apply_swept_footprint_to_map.py \
  --bag "$(realpath runtime/rosbag/<mapping_bag>)" \
  --input-yaml "$(realpath runtime/maps/<evidence_map>/<evidence_map>.yaml)" \
  --output-prefix "$(realpath runtime/maps/<swept_map>)/<swept_map>" \
  --platform-profile "$(realpath profiles/platforms/bunker.yaml)" \
  --clearance 0.05
```

工具要求输入为 map_saver 生成的 P5 PGM，拒绝原地覆盖，并输出消费的位姿数、完整扫掠
栅格数和实际改动像素数。最终审计必须确认轨迹中心附近伪障碍下降，同时 footprint 外的
障碍基本不变。

## 地面、时序动态与高度分层对照

`generate_traversability_variants.py` 默认仍只用于离线对照，一次直接读取 bag 并生成：

- `ground_only`：局部约束 RANSAC 地面平面以上的多帧障碍；
- `ground_temporal`：再要求同一栅格的观测时间跨度至少 `0.5 s`，抑制移动拖影；
- `ground_temporal_layered_provisional`：只保留 `0.10–0.65 m` 碰撞高度，忽略
  `0.65–2.0 m` 上方层。

```bash
ros2 run agt_map_processing generate_traversability_variants.py \
  --bag "$(realpath runtime/rosbag/<mapping_bag>)" \
  --baseline-yaml "$(realpath runtime/maps/<baseline>/<baseline>.yaml)" \
  --output-dir "$(realpath runtime/maps/<comparison_id>)" \
  --platform-profile "$(realpath profiles/platforms/bunker.yaml)"
```

默认地面候选范围为车辆周围 `1–20 m`、相对底盘 `-0.5–0.5 m`，RANSAC 距离容差
`0.08 m`、最大坡度 `20 deg`。障碍层为 `0.10–0.35 m`、`0.35–0.65 m` 和
`0.65–2.0 m`；每个栅格至少三帧，时序变体还要求跨度 `0.5 s`。所有变体最后应用完整
canonical footprint 扫掠。

受管建图会话会显式增加 `--rebuild-raytraced-baseline`，此时工具还会：

- 从 bag 的 `/tf_static` 取得 `base_footprint -> lidar_link` 传感器偏移；
- 按 `/agt/mapping/odometry` 时间插值注册点云位姿，以记录的传感器原点重建二维自由空间射线；
- 以显式 `maximum_evidence_range` 和 `grid_padding` 扩展画布；扩展像素初始保持 `205` unknown，
  只有射线经过的栅格才变成 free；
- 把超范围障碍、源/目标边界、射线帧数、地面拟合、位姿丢失、证据/扫掠裁剪和四边余量写入
  `comparison_report.json`；
- 只把保守 `ground_temporal` 标记为可能的 managed candidate。该标记仍固定
  `eligible_for_execution=false`。

当前受管默认值是已存在离线 OctoMap 基线的 `40 m` 证据/射线范围、`1.0 s` 射线采样间隔和
`2.0 m` unknown 画布余量。障碍 RANSAC、三帧、`0.5 s` 时序、`0.05 m` 障碍 padding 和
canonical sweep 参数没有改变。`agt_system_manager` 会额外要求全部注册点云有匹配位姿、全部
地面平面拟合成功、零裁剪、报告 occupied 数与 PGM 一致，并在通过前保持 Qt candidate 关闭。

高度分层图默认写入报告 `eligible_for_execution=false`。Bunker 产品车体高 `0.4 m`，但
MID360、支架和线束组成的整车最高点尚未实测，因此 `0.65 m` 只是对比阈值，不能据此
放行真实导航。

## 有界实时化候选参数

[`config/bunker_realtime_traversability_provisional.yaml`](config/bunker_realtime_traversability_provisional.yaml)
保存第一轮实时化基准，但当前没有节点消费它，`enabled=false` 且状态固定为
`provisional_not_runtime_connected`。它不能通过修改 launch 参数变成可执行地图链。

- `temporal_window` 是每个活动栅格保留观测证据的时间窗；超过窗口的旧证据不再参与当前
  静态判断。
- `cell_stale_timeout` 是最后一次观测后允许栅格继续驻留的时间；到期后可以从活动内存中
  清除，但已经接受为全局静态证据的 tile 必须先持久化。
- `max_active_cells` 限制活动稀疏单元总量，`max_active_tiles` 限制同时驻留的全局分块数量，
  `memory_budget_mib` 是进程软预算；三者任一触发都必须产生可诊断的降级状态。
- `tile_size_cells=256` 在 `0.05 m` 分辨率下对应 `12.8×12.8 m`。活动 tile 可按距机器人
  远近和最近访问时间淘汰，但只有已成功原子落盘的 tile 才能被淘汰。
- `10×10 m` 局部窗口覆盖当前约 `5 m` 射线清除范围；完整栅格只按 `1 Hz` 发布，内部证据
  可按 `5 Hz` 更新，避免 Qt/RViz 重复传输整幅大图。

候选时间参数以 MID360 `10 Hz` 为前提：三次观测的理论最短跨度只有 `0.2 s`，当前仍要求
至少 `0.5 s`，并保留 `2 s` 证据窗口。它只服务于静态地图判断；Nav2 local costmap 必须
继续无延迟消费当前障碍点，不能等待时序静态确认。正式节点应使用紧凑数组/分块结构，禁止
照搬离线 Python 字典，并在独立 bag 上测量峰值 RSS、积压、落盘延迟和地图差异后才能启用。

RANSAC 的 `0.08 m` 是点到候选地面平面的固定残差阈值，不随 URDF 外参自动缩放。每帧拟合
的平面会随正确变换后的点云改变，但外参修正后仍须重新扫描距离阈值、坡度、障碍最低高度，
并用独立数据验证。`base_link -> lidar_link` 车体外参与 FAST-LIVO2 的 LiDAR/内置 IMU 外参
是两个标定问题，不得用一套结果静默覆盖另一套。

## 后续后端

- 几何地面分割与可通行性栅格。
- 混合静态障碍图与几何地面分割结果对比。
