# System Health and Task Readiness

## ROS interfaces

- `/agt/system/health`: `agt_interfaces/msg/SystemHealth`, periodic structured snapshot.
- `/agt/system/get_health`: `agt_interfaces/srv/GetSystemHealth`, one-shot query.
- `/agt/system/task_readiness`: `agt_interfaces/msg/TaskReadiness`, shared task gate.
- `/agt/system/evaluate_task_readiness`: `agt_interfaces/srv/EvaluateTaskReadiness`, one-shot query.

Health component states are stable `UNKNOWN`, `OK`, `WARN`, `ERROR`. A required
component error contributes a blocker. Optional frontend, Web, and rosbag
components can warn without becoming a motion authorization.

The versioned contract is
`src/agt_system_manager/config/health_contracts.yaml`. The MID360 raw sensor
check consumes `/agt/sensors/lidar/custom` (`livox_ros_driver2/msg/CustomMsg`),
which is the point-timed input used by the current FAST-LIVO2 adapter. It covers
the raw MID360 cloud, the pre-FAST-LIVO2 filtered CustomMsg stream,
IMU, FAST-LIVO2 odometry, registered cloud, mapping occupancy grid, the three TF edges, BUNKER status and
odometry, `agt_safety`, structured localization, Nav2 lifecycle, map/costmap,
disk space, and optional process health.

The filtered stream is produced by `agt_livox_self_filter` from the canonical
BUNKER platform profile. Mapping, localization debug, and navigation health
require both the raw stream and `/agt/sensors/lidar/custom_filtered`; the raw
stream remains the preserved bag/replay input.

`/agt/map/mapping_occupancy` is a durable map snapshot, not a periodic telemetry
stream. Its publisher uses `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)` and the
health node and Web bridge use the matching transient-local subscription. The
contract marks this topic `persistent: true`: receipt proves that a valid map
snapshot exists, so its age is reported for observability but does not expire
the mapping component after three seconds. Cloud, odometry, sensor, and other
live topics retain their configured freshness limits.

## Readiness matrix

### V25-08 mode-aware readiness semantics

本阶段不修改 `EvaluateTaskReadiness.srv`。现有 `TASK_EXECUTION` 和
`RELOCALIZATION` 主要表达当前 MAP navigation baseline；未来 readiness 应按
导航模式拆分为：

```text
MAP_START_READY       MAP_CONTINUE_READY
ROUTE_START_READY     ROUTE_CONTINUE_READY
GLOBAL_CORRECTION_READY
LOCAL_READY
```

`ROUTE_CONTINUE_READY` 的持续门禁应要求 odometry、local control、安全和所需
local perception 健康，但不应持续要求 global map matching healthy 或近期刚有
`map -> odom` correction。`GLOBAL_CORRECTION_READY` 单独判断重定位条件，不能
被 ROUTE continue readiness 隐式替代。`LOCAL_READY` 不应要求 Global Navigation
Map；具体 srv/message 变更须在后续 versioned interface work 中单独提出。

| Check | Required value | Blocker code |
| --- | --- | --- |
| Main mode | `NAVIGATION` | `MODE_NOT_NAVIGATION` |
| Active map | READY map id/version and non-empty hash | `MAP_NOT_READY`, `MAP_ID_MISSING` |
| Assets | `SystemHealth` 从 `active_map.yaml -> manifest` 检查 READY、PGM/YAML/PCD/processing-record 文件存在；`agt_relocalization` 另行解析 processing record 内容和 PCD hash | `NAV_MAP_INVALID`, `LOCALIZATION_PCD_INVALID` |
| Localization identity | status map id/hash equals active map | `LOCALIZATION_MAP_MISMATCH` |
| Localization quality | fresh `TRACKING`, `pose_valid`, `localization_accepted` | `LOCALIZATION_NOT_TRACKING`, `POSE_INVALID`, `LOCALIZATION_NOT_ACCEPTED`, `LOCALIZATION_STATUS_STALE` |
| Safety/chassis | `/agt/safety/status` is fresh, `motion_enabled=true`, authoritative `emergency_stop=false`, safety reports `navigation_ready=true`, and `/agt/chassis/connected` is fresh/true | `EMERGENCY_STOP`, `CHASSIS_DISCONNECTED`, `SAFETY_NOT_READY` |
| Nav2/TF | required lifecycle nodes active and required `map -> odom -> base_footprint` edges are queryable | `NAV2_NOT_ACTIVE`, `TF_NOT_FRESH` |
| Task validator | current task is revalidated by the existing Action server before child dispatch | `TASK_INVALID` |

The frontend displays blocker codes and messages; it does not reimplement this
matrix. The Action server repeats runtime prerequisites and its existing task
validator protects against malformed or out-of-map waypoints. The health node's
`task_valid` default means no task-specific request is currently pending; it is
not permission to skip Action validation.

At startup, an absent `/agt/localization/status` is fail-closed: the
relocalization readiness profile remains blocked because localization map
identity and freshness are not yet evidenced. Once a fresh
`LocalizationStatus` carries a `map_id` and `map_hash` matching the READY
active-map identity, the same `/agt/system/evaluate_task_readiness` service
can recover to ready for `PROFILE_RELOCALIZATION`. This ordering is exercised
against the real `SystemHealthNode` in its ROS test.

`SystemHealth` and `TaskReadiness` are not identical outputs. The health contract
checks all three chassis evidence topics (`/agt/chassis/connected`,
`/agt/chassis/odometry`, `/agt/chassis/status`). The current readiness evaluator
only consumes fresh `/agt/chassis/connected` for its chassis condition; odometry
and status failures can therefore make `SystemHealth` unhealthy without adding a
dedicated TaskReadiness blocker.

V2.5 sensor-input evidence is published by `agt_sensor_monitor` as individual
diagnostics `agt_sensor_monitor/lidar`, `agt_sensor_monitor/filtered_lidar`, and
`agt_sensor_monitor/imu` on `/diagnostics`. `agt_system_manager` consumes their
structured `healthy` values as the required `sensor_input` component; it does
not independently calculate sensor rates or timestamp monotonicity. Camera,
CameraInfo, and GNSS monitor streams are disabled and optional by default.

`agt_safety` is the owner of the emergency-stop latch. The health node consumes
`emergency_stop`/`estop_latched` and `navigation_ready` from its diagnostic status;
the optional `/agt/safety/emergency_stop` input is not a required health topic.
This prevents a missing hardware-adapter publisher from being interpreted as an
active stop after the safety controller has explicitly reported a clear latch.
`navigation_ready` currently means motion enabled, no physical/latched stop, and
valid localization. Navigation command freshness is not part of that value: a
stale `/agt/navigation/cmd_vel` makes the safety controller output zero but may
leave `navigation_ready=true`.

The baseline localization status is emitted on the 5 s tracking-validation
period and can spend up to 3 s in registration. Safety, the lifecycle gate, the
waypoint Action, and the health contract therefore use a 10 s message freshness
window. The `TaskReadiness` snapshot itself remains a separate 2 s cache gate.

The system health TF check currently calls `lookup_transform(target, source,
Time())` and records whether each edge can be queried. It does not compare the
transform header stamp with current time. The existing blocker code remains
`TF_NOT_FRESH` for compatibility, but its current meaning is missing/unqueryable
TF rather than an enforced TF age threshold.

The compatibility `/goal_pose` bridge submits Nav2 `NavigateToPose` without
subscribing to TaskReadiness, active-map identity, or chassis-connected state. It
still remains subject to Nav2 lifecycle/ localization gating and downstream
`agt_safety`; it is not the formal version-bound task entry point.

For `MAPPING`, BUNKER chassis telemetry is optional because mapping can be tested
with MID360 and FAST-LIVO2 while the vehicle is disconnected. The Web console
reports the mapping phase from the three required mapping topics and exposes a
read-only, bounded preview of the occupancy grid and registered point cloud.
`start_chassis_monitor:=true` adds only read-only BUNKER CAN/status evidence;
it never satisfies navigation readiness and never enables motion. A CAN interface
being present or `operstate=up` is advisory and is not proof that status frames,
odometry, safety, or a connected vehicle are healthy.
