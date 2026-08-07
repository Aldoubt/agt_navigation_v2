# Mission schema v1

Mission 存放在 `runtime/missions/<mission_id>/<mission_version>/mission.yaml`，只支持顺序、有限、
可审计的 `WAYPOINT_TASK`、`WAIT_DURATION` 和 `WAIT_EVENT`。文件必须包含与 canonical 内容匹配
的 `content_sha256`，并绑定活动地图的 ID、version 与 manifest SHA-256。

```yaml
schema_version: 1
mission_id: greenhouse_demo_01
mission_version: v1
content_sha256: sha256:<64 lowercase hex>
map_binding:
  map_id: greenhouse_a
  map_version_id: map_20260728_120000_1234abcd
  manifest_sha256: sha256:<64 lowercase hex>
steps:
  - id: navigate_to_a
    type: WAYPOINT_TASK
    task_file: tasks/inspection_route.json
  - id: wait_30_seconds
    type: WAIT_DURATION
    duration_s: 30
  - id: wait_for_arm
    type: WAIT_EVENT
    event_type: manipulator.task_finished
    event_source: arm_controller
    correlation_id: operation-42
    timeout_s: 300
```

所有 ID 使用受限 portable component；资产路径必须相对、不得含 `..`，第一版 waypoint 资产
必须位于绑定地图版本的 `tasks/` 下。schema 不接受 shell、Python、launch、executable、循环、
隐式重试或未知字段。duration 和 event timeout 必须是正有限值且不超过 manager 配置上限。
步骤 ID 必须唯一。

`WAIT_EVENT` 只接受带非零时间戳、类型匹配、可选 source/correlation ID 匹配且不早于当前
等待步骤开始时间的 `MissionEvent`。Action cancel 是唯一取消协议；`SetMissionRunState` 仅支持
PAUSE 和 RESUME。
