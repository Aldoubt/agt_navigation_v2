# BUNKER + MID360 + FAST-LIVO2 + RTAB-Map + GNSS + Nav2 坡地导航设计

日期：2026-08-27
分支：`feat/bunker-rtabmap-slope-nav`
状态：设计冻结，等待实现计划

## 1. 目标

在现有 `agt_navigation_v2` 的 BUNKER / MID360 / FAST-LIVO2 / Nav2 基线之上，增加一条可独立启用的 RTAB-Map 后端与坡地导航链，用于下周真机测试。

第一阶段目标不是替换现有 NDT/PCD 重定位方案，而是提供一条可 A/B 测试的新路径：

1. MID360 + IMU 继续由现有传感器层和 FAST-LIVO2 负责高频局部里程计。
2. RTAB-Map 消费 FAST-LIVO2 的外部里程计和注册点云，负责图优化、回环、2D/3D 地图产品，并预留 GNSS prior。
3. Nav2 全局层使用稳定的 2D OccupancyGrid；局部层使用坡地感知后的障碍 PointCloud2/VoxelLayer。
4. 增加离线 wheel-odom / LIO 联合标定工具，用 rosbag2 估计并验证 BUNKER `base_link -> lidar_link` 平面外参与时间偏移。
5. 保持现有 `agt_safety`、底盘命令 guard、TaskReadiness 和 TF 唯一所有权不被绕过。

## 2. 现有基线与必须保持的约束

现有仓库已经具备：

- MID360 raw CustomMsg：`/agt/sensors/lidar/custom`
- 自体过滤输出：`/agt/sensors/lidar/custom_filtered`
- IMU：`/agt/sensors/imu/data`
- FAST-LIVO2 backend + `agt_mapping_fast_livo2_adapter`
- 标准化 mapping odometry 与 `odom -> base_footprint`
- Nav2、BUNKER safety/guard 和现有全局定位模块

本设计遵守以下现有契约：

- 不允许多个节点发布同一条 TF edge。
- `agt_safety` 仍是导航速度到底盘的唯一安全入口。
- 不修改已验证的 MID360/FAST-LIVO2 参数作为默认值；新增参数以独立 profile/config 提供。
- 语义地图、Keepout、coverage 本阶段不改。
- RTAB-Map 路径默认关闭，必须显式启用。
- 架构和接口变更实现时同步更新 `AGENTS.md`、`docs/` 与 `docs/migration/migration_matrix.md`。

## 3. 推荐架构

```text
MID360 CustomMsg + IMU
        |
        v
agt_livox_self_filter
        |
        v
FAST-LIVO2
   |            \
   |             \ registered cloud
   v              v
/agt/mapping/   RTAB-Map backend <------ GNSS NavSatFix
odometry             |
   |                 | map products / global correction
   |                 v
   |             map -> odom
   |                 |
   +-------> TF chain +------------------------+
                                                |
                                                v
                                              Nav2
                                     +----------+----------+
                                     |                     |
                              global costmap          local costmap
                              StaticLayer             VoxelLayer
                              RTAB 2D map             slope-filtered cloud
```

### 3.1 TF 所有权

默认 TF 合同保持：

```text
map -> odom              RTAB-Map path启用时由 rtabmap_slam 唯一发布
odom -> base_footprint   agt_mapping_fast_livo2_adapter
base_footprint -> base_link / sensors   robot_state_publisher/static TF
```

RTAB-Map 路径启用时，现有 `agt_localization` 不得同时发布 `map -> odom`。launch 必须 fail-fast 拒绝这类非法组合。

第一阶段直接使用 RTAB-Map ROS2 官方外部 odometry 模式，不增加项目自定义 TF adapter：

```text
frame_id:=base_footprint
odom_topic:=/agt/mapping/odometry
odom_frame_id:=''          # 空值表示从 odom topic 获取里程计，而不是从 TF 反查
visual_odometry:=false
icp_odometry:=false
publish_tf_odom:=false
map_frame_id:=map
publish_tf_map:=true
```

因此 `rtabmap_slam` 只负责 `map -> odom`，FAST-LIVO2 adapter 继续唯一负责 `odom -> base_footprint`。如果实际安装版本的 RTAB-Map 参数行为与上述官方 ROS2 launch 合同不一致，启动测试必须 fail-fast，不通过增加第二个 TF 发布者来规避。

## 4. MID360 与 RTAB-Map 适配策略

### 4.1 不使用 RTAB-Map ICP odometry 作为默认前端

默认路径继续使用 FAST-LIVO2 作为 LiDAR-Inertial odometry，原因：

- 现有 MID360 输入、IMU、外参和时间链已经经过项目验证。
- FAST-LIVO2 对 Livox 非重复扫描模式适配更直接。
- 下周真机目标优先降低新变量数量。

RTAB-Map只消费：

- 标准外部 odometry
- 注册后的 `sensor_msgs/PointCloud2`
- IMU（用于图优化重力约束）
- GNSS `sensor_msgs/NavSatFix`（可选）

### 4.2 Topic 适配

项目侧不让 RTAB-Map直接依赖 Livox CustomMsg。统一通过 FAST-LIVO2 注册后的 PointCloud2 接入，避免再做一层 CustomMsg 转换。

固定第一阶段接口：

```text
RTAB odom              <- /agt/mapping/odometry
RTAB scan_cloud        <- /agt/mapping/registered_points_lidar
RTAB imu               <- /agt/sensors/imu/data
RTAB gps/fix           <- /agt/sensors/gnss/fix      # 可选
RTAB map output        -> /agt/rtabmap/map
RTAB obstacle/debug    -> /agt/rtabmap/*
```

对应 RTAB-Map launch 必须显式：

```text
subscribe_scan_cloud:=true
scan_cloud_topic:=/agt/mapping/registered_points_lidar
visual_odometry:=false
icp_odometry:=false
odom_topic:=/agt/mapping/odometry
imu_topic:=/agt/sensors/imu/data
gps_topic:=/agt/sensors/gnss/fix
map_topic:=/agt/rtabmap/map
```

所有 RTAB-Map 输出统一进入项目命名空间，避免裸 `/map`、`/cloud_map` 等 topic 与现有链冲突。

## 5. GNSS 接入

### 5.1 第一阶段定位

GNSS 用作 RTAB-Map 全局 prior / 图优化约束，不作为高频 `odom -> base` 来源。

输入必须满足：

```text
sensor_msgs/msg/NavSatFix
```

并保留：

- fix 状态
- covariance
- 时间戳
- 天线外参 `base_link -> gps_link`

RTAB-Map ROS2 官方 launch 的 `gps_topic` 是异步 GPS 输入，用于 SLAM graph optimization 和 loop-closure candidate selection。实现配置必须显式关闭“忽略 priors”的行为，使有效 GNSS 能实际进入优化；无 GNSS 时允许以纯 LIO + loop 模式运行。

### 5.2 质量门控

无有效 fix、covariance 不可信或时间戳异常时，不允许把该帧 GNSS 作为强 prior。

第一阶段不引入双 EKF `robot_localization + navsat_transform`，避免与 RTAB-Map/global TF 权限叠加。该方案保留为后续独立实验。

## 6. wheel odometry + LIO 离线联合标定

### 6.1 目的

使用 rosbag2 同步记录 BUNKER 轮速里程计和 FAST-LIVO2 里程计，估计并验证：

```text
base_link -> lidar_link 平面外参: x, y, yaw
wheel/lio 时间偏移: dt
轮速尺度修正: k_v, k_w
```

`z / roll / pitch` 第一阶段由机械测量、地面平面和 IMU 重力方向确定，不与 BUNKER 滑移运动学一起自由优化。

### 6.2 标定模型

从两个 pose 序列构造相邻运动：

```text
A_i = inv(T_base_i) * T_base_{i+1}
B_i = inv(T_lidar_i) * T_lidar_{i+1}
```

用 hand-eye 初值：

```text
A_i X = X B_i
```

再做鲁棒非线性 refinement。

BUNKER 是 skid-steer 平台，wheel odom 不能作为绝对真值。优化中：

- 直线与大半径转弯权重高。
- 原地急转、明显打滑、强坡滑移段降权或剔除。
- 必须用独立 bag 验证，不允许在同一数据上只报告拟合残差。

### 6.3 标定 rosbag 资产

至少录制：

```text
/agt/sensors/lidar/custom
/agt/sensors/imu/data
/agt/chassis/odometry
/agt/chassis/status
/agt/mapping/odometry
/agt/mapping/registered_points_lidar
/tf
/tf_static
```

GNSS 可用时额外记录：

```text
/agt/sensors/gnss/fix
```

标准激励轨迹：静止、前后直线、大半径左右转、8字、±90°/±180°、缓坡上下坡、静止。

## 7. RTAB-Map 地图产品与 Nav2 对接

RTAB-Map 的输出按用途分三类，不混用。

### 7.1 全局静态导航层

RTAB-Map 生成/发布 2D `nav_msgs/OccupancyGrid`，作为 Nav2 global costmap 的 `StaticLayer` 来源。

约束：

- 原始 OccupancyGrid 不预先烘焙 inflation。
- 不把局部瞬时障碍永久写入静态图。
- 地图必须有明确 frame、resolution、origin 和版本身份。
- A/B 测试时 global costmap 的 `map_topic` 显式切到 `/agt/rtabmap/map`，不同时订阅旧地图源。

### 7.2 局部实时障碍层

注册点云先经过项目侧 slope-aware ground filter：

```text
registered cloud
      |
      v
gravity/base aligned
      |
      v
normal / local-ground segmentation
   |                     |
 ground              obstacles
                         |
                         v
                 sensor_msgs/PointCloud2
                         |
                         v
                 Nav2 VoxelLayer
```

局部 costmap 使用：

```text
VoxelLayer -> InflationLayer
```

不直接把全部 3D 点云二维投影为障碍。

### 7.3 3D/OctoMap 产品

RTAB-Map 3D occupancy/OctoMap 只作为：

- 调试
- 地图质量检查
- 坡地结构可视化
- 离线资产

第一阶段不要求 Nav2 直接在 OctoMap 上规划。

## 8. 坡地障碍分割设计

### 8.1 问题

基于全局 Z 高度阈值的简单 obstacle filter 会把坡面随着距离增加的高度变化误判成障碍。

### 8.2 第一阶段策略

使用重力方向和局部法向/局部地面模型判断可通行地面，而不是固定 `z > threshold`。

设计要求：

- 输入必须先转换到明确的 base/gravity-aligned frame。
- 地面判断基于局部法向与重力夹角、邻域连续性和高度残差。
- `max_ground_angle` 明确配置，并与实际测试最大坡度分开记录。
- 对悬空点、垂直结构和明显高于局部地面的点输出 obstacle cloud。
- 对地面拟合失败、TF缺失、点云过期必须发布诊断并 fail-closed；不得把全部点静默标记为 free。

第一阶段保留 RTAB-Map 自身 Grid ground segmentation 参数作为对照，但 Nav2 local obstacle 主链由项目侧 filter 输出，避免后端地图参数变化直接影响安全实时层。

## 9. 包和文件边界

实现阶段预计新增/修改：

```text
src/agt_mapping/
  launch/rtabmap_lio_backend.launch.py
  config/rtabmap_bunker.yaml
  config/rtabmap_bunker_slope.yaml

src/agt_calibration/
  package.xml
  CMakeLists.txt/setup.py
  scripts/extract_odom_tracks.py
  scripts/wheel_lio_handeye.py
  scripts/validate_calibration.py
  config/bunker_mid360.yaml

src/agt_perception/
  slope-aware ground filter相关实现/配置

src/agt_navigation/
  config/nav2_bunker_slope.yaml 或现有参数文件中的独立profile

tools/bags/
  record_calibration.sh
  inspect_calibration_bag.py

docs/calibration/
  bunker_mid360_wheel_lio.md

docs/workflows/
  bunker_rtabmap_nav2.md
```

如果现有包边界已经能容纳对应脚本，不为了目录美观强行创建新 ROS package；实现前以现有包结构为准。

## 10. 启动模式

新增模式必须显式启用，例如：

```text
localization_backend:=rtabmap
```

默认仍保持现有 baseline。

非法组合必须拒绝启动：

- `agt_localization` 与 RTAB-Map 同时拥有 `map -> odom`
- RTAB-Map 模式没有可用外部 odom
- 需要 GNSS 模式但未声明 GNSS topic/TF
- slope local obstacle source 与旧 obstacle source 同时写入同一权威 local costmap observation source 而未显式 A/B 配置

## 11. 故障处理与诊断

必须覆盖：

- 外部 odom 过期/跳变
- registered cloud 过期、frame错误
- GNSS 无 fix / covariance异常
- RTAB-Map database/map identity 不匹配
- TF 多发布者
- slope filter 地面拟合失败
- costmap observation 超时

故障时保持：

- 不绕过 `agt_safety`
- 不因 RTAB-Map 节点存活就声明 TaskReadiness ready
- 定位或地图不可用时 Nav2 lifecycle 保持 inactive / paused

## 12. 测试策略

### 12.1 静态/合同测试

增加测试检查：

- launch 默认 RTAB-Map 关闭
- TF edge 唯一所有权
- topic remap 与 frame_id 合同
- Nav2 slope profile plugin 顺序
- vehicle geometry 仍来自 canonical platform profile
- 禁止硬编码 workspace/device/map 绝对路径

### 12.2 rosbag 回放测试

至少建立两类 bag：

1. 平地 calibration bag：验证 wheel/LIO 时间、外参、尺度。
2. 坡地 navigation bag：验证坡面不会大面积进入 obstacle cloud，同时实体障碍仍保留。

指标：

- wheel/LIO 相对运动残差
- 独立 bag 外参验证误差
- slope ground/obstacle 点数与失败率
- local costmap 被坡面错误占据的比例
- RTAB-Map odom/graph 输入频率
- TF continuity

### 12.3 真机 Gate

按顺序通过：

```text
G0 BUNKER cmd_vel/odom/safety
G1 MID360/IMU/time/TF
G2 wheel-LIO 离线外参验证
G3 FAST-LIVO2 稳定 odom
G4 RTAB-Map 建图/回环/GNSS输入
G5 RTAB-Map -> Nav2 global map
G6 slope obstacle -> local VoxelLayer
G7 平地连续导航
G8 缓坡连续导航
G9 5m/7m/10m 定点精度测试
```

## 13. 第一阶段明确不做

- 不把 wheel odom 在线紧耦合进 FAST-LIVO2 状态量。
- 不实现新的 3D planner。
- 不迁移到 ROS2 Jazzy。
- 不移植 Nav2 Jazzy Ground Consistency Layer。
- 不把 RTAB-Map OctoMap 直接作为 Nav2 唯一规划地图。
- 不修改语义/coverage 体系。
- 不在这轮重构现有 FAST-LIVO2 baseline。

## 14. 第一阶段完成定义

满足以下条件才算实现完成：

1. 独立分支可构建，现有 baseline tests 不回归。
2. 一条命令可启动 FAST-LIVO2 + RTAB-Map 后端，且 TF 权限唯一。
3. rosbag 录制脚本可同时记录 wheel odom、LIO、LiDAR、IMU、TF 和可选 GNSS。
4. 离线标定工具能从 bag/导出的轨迹输出外参候选、时间偏移、残差和独立验证结果。
5. RTAB-Map 2D OccupancyGrid 能进入 Nav2 global StaticLayer。
6. 坡地 filter 输出 obstacle PointCloud2 并进入 Nav2 local VoxelLayer。
7. 缓坡 bag/真机中，坡面不被大面积误判为障碍；真实立体障碍仍能触发 costmap。
8. 现有 `agt_safety`、底盘 guard、TaskReadiness 和默认定位路径不被绕过。
9. 文档、`AGENTS.md` 和 migration matrix 与实际实现一致。
