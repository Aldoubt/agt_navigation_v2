# agt_map_processing

职责：把全局 PCD 或注册点云转成二维栅格、可通行性地图和对比输出。

## OctoMap 二维投影 baseline

当前已迁移旧仓库的在线 OctoMap 投影链：

- 输入：`/agt/mapping/registered_points_lidar` (`sensor_msgs/msg/PointCloud2`，`lidar_link` frame)。
- 建图输出：`/agt/map/mapping_occupancy` (`nav_msgs/msg/OccupancyGrid`)。
- 默认分辨率：`0.05 m`。
- 默认仅把 `0.10 m <= z <= 1.00 m` 的点作为投影障碍候选。

建图工作图与导航静态图分开：OctoMap 只发布 `/agt/map/mapping_occupancy`，导航模式的
`map_server` 才发布 `/agt/map/global_occupancy`。这样建图 RViz 不会误显示仍在运行的旧导航地图。

FAST-LIVO2 和投影节点都启动后，回放传感器 bag：

```bash
ros2 launch agt_map_processing octomap_projection.launch.py use_sim_time:=true
```

参数集中在 [`config/octomap_projection.yaml`](config/octomap_projection.yaml)。实测前重点根据
地面位置、机器人高度和作物冠层调整 `pointcloud_min_z`、`pointcloud_max_z`、
`occupancy_min_z` 和 `occupancy_max_z`。

生成地图后，在仓库根目录执行：

```bash
cd /home/yangxuan/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p /home/yangxuan/agt_navigation_v2/runtime/maps
ros2 launch agt_map_processing save_occupancy_map.launch.py \
  map_prefix:=/home/yangxuan/agt_navigation_v2/runtime/maps/mid360_map
```

会生成 `mid360_map.pgm` 和 `mid360_map.yaml`。保存节点使用 transient-local 订阅，投影节点
也保留最后一帧，因此可在回放结束后启动保存命令。该二维图是全局静态地图候选，不包含
Nav2 local costmap 的瞬时局部障碍。

OctoMap 使用当前帧 `lidar_link` 点云和 `odom -> lidar_link` TF，因此射线原点会随机器人
运动。车辆 `base_link -> lidar_link` 外参完成标定和高度阈值调优前，输出地图只用于链路
验证与后端对比，不作为最终导航地图。

## 大包离线静态障碍补全

单纯使用固定世界坐标高度带投影，容易漏掉低矮障碍、细杆、较高障碍，也会在 LIO 的
`odom.z` 漂移后选择错误高度。大包离线处理采用“射线基图 + 重复点云证据”的混合方法：

- bag 内已有的 `/agt/map/mapping_occupancy` 提供射线清除后的 free/unknown 基图；
- `/agt/mapping/registered_points` 按每帧机器人 `base_footprint` 高度选择
  `0.05 m .. 2.00 m` 障碍点；
- 非有限点、车体 footprint 外接圆内的自体点被过滤；同一栅格必须被至少 3 个不同点云帧
  观测才写为 occupied；
- `obstacle_padding=0.05 m` 是显式的点证据栅格补偿，不代替 Nav2 InflationLayer；保存的
  PGM 不烘焙机器人 footprint 或 Nav2 膨胀代价。

回放使用 bag 自带基图时，将输入基图改名，避免和增强输出 topic 冲突：

```bash
ros2 launch agt_map_processing offline_static_obstacle_map.launch.py \
  platform_profile:="$(realpath profiles/platforms/bunker.yaml)" \
  rebuild_raytraced_baseline:=false

ros2 bag play runtime/rosbag/<mapping_bag> --clock --rate 2.0 \
  --topics /agt/map/mapping_occupancy \
           /agt/mapping/registered_points \
           /agt/mapping/odometry \
  --remap /agt/map/mapping_occupancy:=/agt/map/octomap_occupancy
```

只有 bag 没有有效 OccupancyGrid、并且包含完整 `/tf`、`/tf_static` 与 lidar-frame 点云时，
才设置 `rebuild_raytraced_baseline:=true` 并同时回放这些 TF/topic。回放结束后检查
`/agt/map/static_obstacle_evidence_status`，再从增强后的 `/agt/map/mapping_occupancy` 保存新
PGM/YAML；不要覆盖已验证地图。

## 后续后端

- 几何地面分割与可通行性栅格。
- 混合静态障碍图与几何地面分割结果对比。
