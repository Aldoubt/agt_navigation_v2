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
`src/agt_system_manager/config/health_contracts.yaml`. It covers MID360 cloud,
IMU, FAST-LIVO2 odometry, registered cloud, the three TF edges, BUNKER status and
odometry, `agt_safety`, structured localization, Nav2 lifecycle, map/costmap,
disk space, and optional process health.

## Readiness matrix

| Check | Required value | Blocker code |
| --- | --- | --- |
| Main mode | `NAVIGATION` | `MODE_NOT_NAVIGATION` |
| Active map | READY map id/version and non-empty hash | `MAP_NOT_READY`, `MAP_ID_MISSING` |
| Assets | verified PGM/YAML and ready PCD processing record | `NAV_MAP_INVALID`, `LOCALIZATION_PCD_INVALID` |
| Localization identity | status map id/hash equals active map | `LOCALIZATION_MAP_MISMATCH` |
| Localization quality | fresh `TRACKING`, `pose_valid`, `localization_accepted` | `LOCALIZATION_NOT_TRACKING`, `POSE_INVALID`, `LOCALIZATION_NOT_ACCEPTED`, `LOCALIZATION_STATUS_STALE` |
| Safety/chassis | no emergency stop, connected chassis, safety allows navigation | `EMERGENCY_STOP`, `CHASSIS_DISCONNECTED`, `SAFETY_NOT_READY` |
| Nav2/TF | required lifecycle nodes active and fresh `map -> odom -> base_footprint` chain | `NAV2_NOT_ACTIVE`, `TF_NOT_FRESH` |
| Task validator | current task is revalidated by the existing Action server before child dispatch | `TASK_INVALID` |

The frontend displays blocker codes and messages; it does not reimplement this
matrix. The Action server repeats runtime prerequisites and its existing task
validator protects against malformed or out-of-map waypoints. The health node's
`task_valid` default means no task-specific request is currently pending; it is
not permission to skip Action validation.
