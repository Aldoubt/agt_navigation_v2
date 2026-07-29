# Qt waypoint task Action

`/agt/navigation/execute_waypoint_task` is the stable project boundary between an
operator frontend and Nav2 waypoint execution. The maintained Qt5 fork is an
operator/editor and Action client; this project Action remains the execution authority.

## Action

Type: `agt_interfaces/action/ExecuteWaypointTask`

Goal:

- `map_id`, `map_version_id`, `task_group_id`, `task_revision` and
  `expected_content_sha256`: the formal execution input. The server resolves the
  task JSON through the robot-side Task Registry under `runtime/maps`;
- `client_request_id`: frontend-generated idempotency key for a start request;
- `loop_count`: always required to be `1..maximum_loops` and is finite;
- `task_file` and `poses`: deprecated same-machine CLI/debug compatibility
  inputs. They are disabled by default and must not be used by Qt/Web.

Exactly one formal task identity, legacy `task_file`, or direct `poses` input is
accepted. The maintained Qt5 navigation profile sends only task identity fields;
it never sends an absolute task path.

Result:

- `success`: true only if every Nav2 run succeeds;
- `error_code`, `blocker_code`, `operator_message`, and `technical_message`:
  stable machine, operator, and engineering failure information;
- `session_id`, `duplicate_request`, and `final_status`: authoritative robot-side
  Navigation Session state for reconnect/retry recovery;
- `missed_waypoints`: indices returned by Nav2 `FollowWaypoints`.

Feedback reports `state`, zero-based `loop_index`, current waypoint, total waypoint
count, and the current `NavigationSessionStatus`. `/agt/navigation/session_status`
is reliable + transient-local + keep-last 1. `/agt/navigation/task_status` remains a
deprecated concise JSON compatibility topic.

## Validation and safety

The server rejects unreadable JSON, empty/oversized chains, non-finite poses, adjacent
duplicates, exact repeated patterns caused by the vendor append-on-save defect, and
points outside the current `/agt/map/global_occupancy`. It also requires recent
`agt_safety` diagnostics with motion enabled and the emergency-stop latch clear.

For schema-v1 task groups it additionally compares OccupancyGrid geometry, active map
ID and version, map YAML/image hashes, and the localization PCD hash. Active map
identity comes from `agt_map_manager` through `/agt/maps/active`, with launch
parameters only as compatibility fallback. Missing or mismatched active identity fails
closed. Legacy files have no map binding and retain their existing runtime
map-boundary behavior only when `allow_legacy_local_task_file:=true` confines the path
below the configured `runtime/maps` root.

Duplicate `client_request_id` submissions do not create a second Nav2 child goal.
Different request IDs while a task is active are rejected with `TASK_ALREADY_ACTIVE`
and include the active session status. Idempotency records are in-memory and bounded;
restart-persistent idempotency is a later scope.

The server never enables motion and never publishes velocity. It sends only
`nav2_msgs/action/FollowWaypoints`; cancellation or loss of safety readiness cancels
the active child goal. A Nav2 abort or any `missed_waypoints` makes the parent task fail.

## Qt integration boundary

The maintained `agt-navigation-v2` Qt branch submits map/task identity and a UUID
`client_request_id`. If a task is dirty, unsaved, or has not been synchronized to the
robot registry, Qt reports `任务尚未同步到机器人` and does not fall back to a local path.
Start/Stop controls display Action feedback/result and missed waypoints, and forward
cancellation. Any future Qt/Web/autostart frontend must call this same Action and
must not duplicate its state machine.
