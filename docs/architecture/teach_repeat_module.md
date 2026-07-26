# 示教-复现模块架构

## 定位与所有权

`agt_teach_repeat` 是 FAST-LIVO2/地图资产和现有 Nav2 之间的可选客户端，不是新的 SLAM、定位、
TF、地图管理、控制器、安全或底盘所有者。第一版使用独立 launch；`agt_bringup/system.launch.py`
仍只有 mapping/navigation 两个顶层 mode。System Manager 只增加只读 `teach_preview` profile，
不提供 Web 一键实车执行。

仓库实际接口如下：

| 项目 | 实际合同 |
| --- | --- |
| 示教真源 | `/agt/mapping/odometry`, `frame_id=odom`, `child_frame_id=base_footprint` |
| Nav2 Action | `/follow_path`, `nav2_msgs/action/FollowPath` |
| Controller ID | `FollowPath` (MPPI) |
| 定位门禁 | `LocalizationStatus`: TRACKING + pose_valid + localization_accepted + ERROR_NONE + !status_stale |
| Safety 门禁 | `/agt/safety/status` 中 `agt_safety/tracked_controller`: motion_enabled=true, estop_latched=false |
| 共享门禁 | `/agt/system/task_readiness.ready=true` 且 map_id 匹配 |
| 几何真源 | `profiles/platforms/<platform>.yaml: platform.geometry.navigation_footprint` |

## 数据层

提取器使用 `rosbag2_py.SequentialReader`，不会调用 `ros2 bag play`。topic 缺失时固定报错：

```text
odometry topic unavailable; replay mapping inputs through FAST-LIVO2 before extraction
```

每个 demo 是独立目录：

```text
runtime/teach_repeat/<demo_id>/
├── manifest.yaml
├── raw/{raw_path.csv,source_bag.yaml}
├── processed/{reference_path.yaml,reference_path.json,reference_path.csv,
│              task_control_points.json,processing_report.json}
├── audit/{path_validation.json,corridor_conflicts.json,corridor_markers.json}
└── runs/<run_id>/{executed_path.csv,localization_samples.csv,mapping_odometry.csv,
                  chassis_odometry.csv,safety_samples.csv,metrics.json,report.md}
```

内部文件使用临时文件、fsync 和 `os.replace`；NaN/Inf、未知 schema、非法 demo ID 被拒绝。已存在
demo 默认拒绝，只有显式 `overwrite:=true` 才允许替换单个文件。raw bag、map YAML、PCD、processing
record 和 reference path 都记录 SHA-256。

manifest schema-v1 示例：

```yaml
schema_version: 1
demo_id: greenhouse_route_001
created_at: "2026-07-26T08:00:00+00:00"
source:
  bag_path: /data/bags/teach_001
  bag_sha256: "sha256:<64 lowercase hex>"
  odometry_topic: /agt/mapping/odometry
map:
  map_id: teach_base_v1
  map_yaml: /data/maps/teach_base_v1/map.yaml
  map_yaml_sha256: "sha256:<64 lowercase hex>"
  localization_pcd: /data/maps/teach_base_v1/localization_map.pcd
  localization_pcd_sha256: "sha256:<64 lowercase hex>"
  processing_record: /data/maps/teach_base_v1/localization_map.processing.yaml
  processing_record_sha256: "sha256:<64 lowercase hex>"
platform:
  profile: /absolute/path/to/profiles/platforms/bunker.yaml
frames:
  source_frame: odom
  source_child_frame: base_footprint
  execution_frame: map
  map_from_teach_odom: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
processing:
  resample_distance_m: 0.10
  smoothing_enabled: true
  smoothing_method: moving_average
  smoothing_window: 5
  max_smoothing_deviation_m: 0.05
execution:
  controller_id: FollowPath
  maximum_linear_speed_mps: 0.20
assets:
  reference_path: processed/reference_path.yaml
  reference_path_sha256: "sha256:<64 lowercase hex>"
  task_control_points: processed/task_control_points.json
  task_control_points_sha256: "sha256:<64 lowercase hex>"
  processing_report: processed/processing_report.json
lifecycle: {session_id: "", growth_stage: "", parent_map_id: ""}
```

`map_from_teach_odom` 只支持 SE(2) 平移+yaw，但保留 z。它必须显式写入，不能假设每次示教 odom
都与后续 navigation map 重合。

## 预览与验证

publisher 以 reliable + transient-local 发布 map-frame Path、控制点 MarkerArray 和诊断。validator
复用 `agt_coverage_planning.path_validator.validate_path()` 的完整 footprint、距离/角度插值、未知/
越界、占据阈值和曲率检查，车辆几何只从 profile 加载。失败或输入不完整时 validated path 为空。

corridor auditor 对同一插值样本计算 swept footprint 覆盖的唯一 occupied/unknown cell，仅输出审计
JSON/markers/conflict poses。它没有 PGM/OccupancyGrid 写接口，且
`eligible_for_automatic_map_edit=false` 固定不变。

## 执行门禁

`execution_enabled` 默认 false。启动前必须同时满足 manifest/schema、reference/map/PCD/record hash、
processing record ready、几何报告 eligible、匹配 map identity 的新鲜 TRACKING 定位、操作员已使能
safety、急停清除、TaskReadiness ready 和 `/follow_path` 可用。`/agt/teach/start` 不自动使能 motion。

执行期间任何门禁 stale/失效、validated path 清空、资产 stat 变化、反馈超时、用户取消或横向误差
超过 hard limit，都会请求取消 Nav2 child。执行器只发布 Nav2 `SpeedLimit`，其值为 asset、P0 参数
和平台 forward limit 的最小值；不拦截或改写最终速度链。

## 评测与实验

最近参考线段投影给出 cross-track、along-track、heading error 和 goal distance；P95 使用固定线性
插值。误差位姿来自 `LocalizationStatus.global_pose`，因为 `/agt/mapping/odometry` 实际在 odom frame，
不能直接与 map reference 相减。mapping/chassis odometry 仍作为漂移和滑移证据订阅/录包。

这些指标是机载定位闭环的系统内部重复性，不是独立绝对真值。Experiment Manager 的
`record_teach_repeat_result()` 和 `record_failure_case()` 保存 Git snapshot、配置快照引用、地图/path
hash、执行结果和失败上下文；没有建立第二套实验目录管理器。

## Topic、Service、Action

| 类型 | 名称 |
| --- | --- |
| Path | `/agt/teach/reference_path`, `/agt/teach/path_validated`, `/agt/teach/executed_path` |
| Marker/poses | `/agt/teach/control_points`, `/agt/teach/collision_poses`, `/agt/teach/footprint_markers`, `/agt/teach/corridor_markers`, `/agt/teach/corridor_conflicts` |
| JSON/diagnostic | `/agt/teach/status`, `/agt/teach/validation_report`, `/agt/teach/corridor_report`, `/agt/teach/execution_status`, `/agt/teach/current_error`, `/agt/teach/metrics` |
| Service | `/agt/teach/start`, `/agt/teach/cancel` (`std_srvs/Trigger`) |
| Child Action | `/follow_path` (`nav2_msgs/action/FollowPath`) |
| Nav2 limit | `/speed_limit` (`nav2_msgs/msg/SpeedLimit`) |

本阶段未定义新的 project Action；执行器是标准 Nav2 Action client。后续如需要多前端统一提交，应先在
`agt_interfaces` 定义并生成项目 Action，而不是复制 FollowPath 或从 GUI 推断完成状态。

## 明确未实现

多会话/多生长周期融合、stable voxel/lifelong/delta map、语义植被过滤、自动 PGM 清理、PCD 合并、
独立真值、Qt 新任务页和 Web 实车一键执行均未实现。
