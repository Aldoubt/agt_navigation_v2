# 自动与手动重定位使用说明

本文只覆盖 BUNKER + MID360 + FAST-LIVO2 + Nav2 基线。定位成功后，`agt_localization` 才会发布
唯一的 `map -> odom`；FAST-LIVO2 adapter 继续发布 `odom -> base_footprint`。定位失败不会启动
Nav2 运动链，`agt_safety` 也不会因为定位成功而自动使能底盘运动。

## 先建图再导航

### 1. 建图并保存

在仓库根目录、已 source ROS 2 Humble 和本工作区的终端中启动建图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=<map_id> \
  start_rviz:=true start_mapping_gui:=false record_bag:=true
```

建图运行期间保持 FAST-LIVO2 和总控不退出。在另一个已 source 的终端保存二维地图：

```bash
tools/save_mapping_outputs.sh <map_id>
```

确认二维地图保存成功后，对建图总控执行正常 `Ctrl+C`，等待 FAST-LIVO2 完成 PCD 落盘。然后
确认以下文件都存在：

```text
runtime/maps/<map_id>/<map_id>.yaml
runtime/maps/<map_id>/<map_id>.pgm
runtime/maps/<map_id>/pcd/localization_map.pcd
runtime/maps/<map_id>/pcd/localization_map.processing.yaml
```

`localization_map.processing.yaml` 必须有 `state: ready` 且 `map_file` 必须指向同目录的
`localization_map.pcd`。启动定位时会重新计算 PCD SHA-256；旧记录没有 `pcd_sha256` 时可以
读取，但会发出未验证告警。不要使用 `kill -9` 代替正常保存。

### 2. 启动导航的共同参数

自动和手动模式都必须使用同一批次生成的 PGM/YAML、PCD 和 processing record：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$PWD/runtime/maps/<map_id>/<map_id>.yaml" \
  global_map_pcd:="$PWD/runtime/maps/<map_id>/pcd/localization_map.pcd" \
  global_map_processing_record:="$PWD/runtime/maps/<map_id>/pcd/localization_map.processing.yaml" \
  map_id:=<map_id> \
  start_semantic_map_server:=false \
  start_coverage_planning:=false
```

导航模式默认 `auto_relocalize_on_start:=false`，因为定位节点不会从零进行无界全地图搜索。
Nav2 lifecycle manager 初始保持非活动，定位状态进入 `TRACKING` 后才由 gate 发送标准
`STARTUP`。即使 gate 启动 Nav2，运动仍需单独检查安全状态并显式调用 motion enable。

### 点云时间与 tracking validation

`agt_localization` 只缓存最新一帧注册点云；一次候选搜索的全部候选共用同一条点云和
`cloud_stamp`。将点云转换到 `tracking_frame`、查询动态 `odom -> tracking_frame` 和计算
`map -> odom` 都使用这个时间戳，TF 不存在时不会回退到最新值。`base_link -> lidar_link`
是静态外参时可使用静态 TF。

初始手动/Action/配置候选仍使用 candidate 粗初值。进入 tracking validation 后，NDT/ICP 初值改为：

```text
map -> tracking_frame predicted
  = map -> odom * odom -> tracking_frame(cloud_stamp)
```

质量 innovation 比较当前 odom 传播预测与本次配准结果；验证只更新 supervisor 状态和质量字段，
不更新 `map -> odom`，也不覆盖最近有效位姿。`global_pose` 和 `aligned_points` 的测量时间为
点云时间，20 Hz 持续 TF 重发才使用当前 ROS 时间。

tracking validation 在进入 TF 查询和配准前会原子预留本次 `cloud_stamp`，并且只接受严格大于
上一帧的纳秒时间戳。相同时间戳只有在点云仍通过新鲜度门禁时才跳过；bag 与 `/clock` 一起暂停
时，fresh duplicate 不执行 NDT、不计成功或失败，也不发布状态。实车 ROS 时间继续而点云停发
时，缓存帧超过 `max_cloud_age_s` 后会以 `ERROR_STALE_SCAN` 失败，不会被 duplicate 门禁掩盖；
连续周期将推动 `TRACKING -> DEGRADED -> RECOVERING -> LOST`。future 或 invalid duplicate 也按
对应时间错误拒绝。

若 ROS 时间回退且当前帧新鲜，该帧成为新的序列基线但不执行验证；再次出现同一时间戳时按
fresh duplicate 跳过，下一帧时间戳严格增加后才恢复。回退帧和 fresh duplicate 都是 skip：
不更新 supervisor，不执行 NDT，也不发布新的 `LocalizationStatus`。

一次非 skip tracking validation 只在外层 worker 完成一次 supervisor 更新并发布一次最终权威
状态。`runCandidates()` 在 tracking 模式仅计算并返回 accepted/rejected/skipped，不发布 map/cloud
准备失败、TF 失败、取消、超时、`VERIFYING`、候选质量结果、候选耗尽或歧义等中间状态；外层对
失败统一调用一次 `trackingValidation(false)`，成功调用一次 `trackingValidation(true)`。因此成功
验证不会短暂发出 `pose_valid=false`，不会误取消 teach-repeat 或使 Nav2/safety gate 抖动。该路径
仍不更新 `map -> odom`，也不覆盖 last valid pose。

最终状态的 `has_converged` 是 NDT/ICP 后端真实收敛值，和质量接受是两层语义。后端可收敛但因
fitness 或 innovation 被拒绝，此时 `has_converged=true`，而 `localization_accepted=false`、
`pose_valid=false`；只有完整质量门禁接受时后两者才为 `true`。

`tracking_confirmations_required` 当前只允许 `1`。任何非 `1` 值都会在节点启动时以
`multi-frame bootstrap confirmation is not implemented` 明确失败。未来多帧启动确认需要临时
`map -> odom`、严格的新点云门禁、N 帧独立确认和最终 TF 提交，本次没有实现，也不能通过修改
YAML 提前启用。

点云新鲜度默认由 `max_cloud_age_s: 0.5`、`max_cloud_future_tolerance_s: 0.1` 和
`require_nonzero_cloud_stamp: true` 控制。拒绝时状态会报告 `ERROR_STALE_SCAN` 或
`ERROR_INVALID_SCAN_TIMESTAMP` 及实际 stamp/age。

使用历史 bag 做定位回放时必须启用 ROS clock：

```bash
ros2 launch agt_localization relocalization.launch.py \
  use_sim_time:=true \
  global_map_pcd:=<same-map-localization.pcd> \
  global_map_processing_record:=<same-map-processing.yaml>
ros2 bag play runtime/rosbag/Benchmark-BAG-260725 --clock
```

回放中的点云、`/tf` 和 `/clock` 必须同时覆盖待测试的测量时刻；节点不使用系统墙钟判断点云
新鲜度。该 bag 含注册点云和 TF，可用于时间一致性/过期拒绝回归；定位精度仍需使用同源 PCD
和明确的初始位姿另行验收。

## 自动重定位

自动模式是一个启动后只发送一次的、有候选上限和超时的 `Relocalize` Action 请求。客户端会
先等待首帧 `/agt/mapping/registered_points` 和 Action Server，满足后才发送请求。它按
顺序使用以下可选来源：

- 带当前 `map_id`/`map_hash` 的 `last_valid_pose.yaml`；
- 带当前 PCD `sha256:<64位小写十六进制>` 的 configured candidate YAML；
- 当前时间窗口内、`map` frame 且协方通过校验的外部 coarse pose。

这不是 Scan Context 或无界全地图 place recognition。首次在一个新地图上使用自动模式时，
至少应提供 configured candidate YAML。以 `config/candidates.example.yaml` 为模板建立部署
配置，并把 `map_hash` 写成实际 PCD 摘要：

```bash
PCD="$PWD/runtime/maps/<map_id>/pcd/localization_map.pcd"
sha256sum "$PCD"
```

候选 YAML 中的 `map_id` 和 `map_hash` 必须与当前地图一致，候选坐标使用 `map` frame。然后
使用自动启动参数：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$PWD/runtime/maps/<map_id>/<map_id>.yaml" \
  global_map_pcd:="$PWD/runtime/maps/<map_id>/pcd/localization_map.pcd" \
  global_map_processing_record:="$PWD/runtime/maps/<map_id>/pcd/localization_map.processing.yaml" \
  map_id:=<map_id> \
  configured_candidates_yaml:="$PWD/runtime/maps/<map_id>/localization_seeds.yaml" \
  auto_relocalize_on_start:=true \
  auto_relocalize_timeout_s:=30.0 \
  auto_relocalize_max_candidates:=128 \
  start_semantic_map_server:=false \
  start_coverage_planning:=false
```

验证定位结果：

```bash
ros2 topic echo /agt/localization/status --once
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
```

成功条件是 `state: 3`（`TRACKING`）、`pose_valid: true`、`localization_accepted: true`、
`error_code: 0`，并且 `map -> odom` 持续存在。候选耗尽、PCD hash 不匹配、点云未到达、TF
缺失或质量门禁失败都会保持 fail-closed；启动客户端不会自动无限重试。需要再次恢复时，
使用手动 `/initialpose` 或显式 Action 请求。

也可以不通过启动客户端，导航已经启动后手动发送一次同样的自动 Action：

```bash
ros2 action send_goal /agt/localization/relocalize \
  agt_interfaces/action/Relocalize \
  "{mode: 0, use_last_valid_pose: true, use_configured_candidates: true, use_external_coarse_pose: true, timeout_s: 30.0, publish_debug: true}"
```

## 手动给定位姿

手动模式默认开启 `manual_initialpose_enabled:=true`。先使用共同导航参数启动，但显式关闭
自动启动：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$PWD/runtime/maps/<map_id>/<map_id>.yaml" \
  global_map_pcd:="$PWD/runtime/maps/<map_id>/pcd/localization_map.pcd" \
  global_map_processing_record:="$PWD/runtime/maps/<map_id>/pcd/localization_map.processing.yaml" \
  map_id:=<map_id> \
  manual_initialpose_enabled:=true \
  auto_relocalize_on_start:=false \
  start_semantic_map_server:=false \
  start_coverage_planning:=false
```

在 navigation Qt/RViz 操作界面使用 `2D Pose Estimate`，第一点给出 `map` 中的机器人位置，
拖动方向给出 yaw 后释放。该入口发布 `/initialpose`，定位节点将它转换为同一内部配准路径，
但只测试一个初始候选。

没有图形界面时可以直接发布一个示例初值。下面的四元数表示 yaw=0；实际使用时必须替换为
操作者在 `map` 中测得的坐标和方向：

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

等待 `LocalizationStatus` 达到 `TRACKING` 后再检查 Nav2 和安全层。手动初值很差时可能被
innovation、fitness、点数或 TF 门禁拒绝；拒绝不会直接写 `map -> odom`。

## 技术原理对比

| 项目 | 自动重定位 | 手动给初值 |
| --- | --- | --- |
| 操作者输入 | 候选 YAML、last pose 或外部 coarse pose | `/initialpose` 的一个 `map -> base_link` 初值 |
| 搜索范围 | 候选周围有界 SE(2) 展开，按优先级和上限顺序测试 | 只测试一个候选，不展开全局搜索 |
| 配准后端 | 与手动模式相同的 NDT/ICP | 与自动模式相同的 NDT/ICP |
| 质量门禁 | fitness、点数、创新量、候选歧义和后续 supervisor | 同一套质量门禁 |
| 适合场景 | 已知若干安全起点、上次有效位姿或有外部粗定位 | 操作者能在地图上识别当前车位，首次联调和错误恢复 |
| 失败行为 | 一次 Action 超时/拒绝后停止，不无限重试 | 发布明确失败状态，等待再次给初值 |
| TF 责任 | 只有质量接受后由 `agt_localization` 发布 `map -> odom` | 完全相同 |
| 安全关系 | 不发布速度、不使能安全层、不绕过 Nav2 | 完全相同 |

两种方式不是两套定位算法。差异只在初值来源和候选数量：最终都必须通过同一个配准、质量、
supervisor、TF 和 Nav2/safety gate。自动模式也不会把“Action Server 在线”当作定位成功。

## 排查顺序

```bash
ros2 topic echo /agt/localization/status --once
ros2 topic echo /agt/localization/status_text --once
ros2 topic info /agt/mapping/registered_points -v
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 service call /agt/safety/set_motion_enabled std_srvs/srv/SetBool '{data: false}'
```

先处理 `map not ready`、`scan too small`、`TF unavailable`、`map hash mismatch` 或
`no candidates`，确认定位稳定后才进行任何运动测试。正常停止使用 `Ctrl+C`，不要用 `kill -9`。
