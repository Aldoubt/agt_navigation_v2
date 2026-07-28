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

| Check | Required value | Blocker code |
| --- | --- | --- |
| Main mode | `NAVIGATION` | `MODE_NOT_NAVIGATION` |
| Active map | READY map id/version and non-empty hash | `MAP_NOT_READY`, `MAP_ID_MISSING` |
| Assets | verified PGM/YAML and ready PCD processing record | `NAV_MAP_INVALID`, `LOCALIZATION_PCD_INVALID` |
| Localization identity | status map id/hash equals active map | `LOCALIZATION_MAP_MISMATCH` |
| Localization quality | fresh `TRACKING`, `pose_valid`, `localization_accepted` | `LOCALIZATION_NOT_TRACKING`, `POSE_INVALID`, `LOCALIZATION_NOT_ACCEPTED`, `LOCALIZATION_STATUS_STALE` |
| Safety/chassis | `/agt/safety/status` is fresh, `motion_enabled=true`, authoritative `emergency_stop=false`, connected chassis, safety reports `navigation_ready=true` | `EMERGENCY_STOP`, `CHASSIS_DISCONNECTED`, `SAFETY_NOT_READY` |
| Nav2/TF | required lifecycle nodes active and fresh `map -> odom -> base_footprint` chain | `NAV2_NOT_ACTIVE`, `TF_NOT_FRESH` |
| Task validator | current task is revalidated by the existing Action server before child dispatch | `TASK_INVALID` |

The frontend displays blocker codes and messages; it does not reimplement this
matrix. The Action server repeats runtime prerequisites and its existing task
validator protects against malformed or out-of-map waypoints. The health node's
`task_valid` default means no task-specific request is currently pending; it is
not permission to skip Action validation.

`agt_safety` is the owner of the emergency-stop latch. The health node consumes
`emergency_stop`/`estop_latched` and `navigation_ready` from its diagnostic status;
the optional `/agt/safety/emergency_stop` input is not a required health topic.
This prevents a missing hardware-adapter publisher from being interpreted as an
active stop after the safety controller has explicitly reported a clear latch.

The baseline localization status is emitted on the 5 s tracking-validation
period and can spend up to 3 s in registration. Safety, the lifecycle gate, the
waypoint Action, and the health contract therefore use a 10 s message freshness
window. The `TaskReadiness` snapshot itself remains a separate 2 s cache gate.

For `MAPPING`, BUNKER chassis telemetry is optional because mapping can be tested
with MID360 and FAST-LIVO2 while the vehicle is disconnected. The Web console
reports the mapping phase from the three required mapping topics and exposes a
read-only, bounded preview of the occupancy grid and registered point cloud.
`start_chassis_monitor:=true` adds only read-only BUNKER CAN/status evidence;
it never satisfies navigation readiness and never enables motion. A CAN interface
being present or `operstate=up` is advisory and is not proof that status frames,
odometry, safety, or a connected vehicle are healthy.
