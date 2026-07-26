# Qt waypoint task Action

`/agt/navigation/execute_waypoint_task` is the stable project boundary between an
operator frontend and Nav2 waypoint execution. The maintained Qt5 fork is an
operator/editor and Action client; this project Action remains the execution authority.

## Action

Type: `agt_interfaces/action/ExecuteWaypointTask`

Goal:

- `task_file`: absolute path to a legacy Qt points JSON or schema-v1 task group;
- `poses`: portable `map`-frame input for future Qt/Web/autostart frontends;
- `loop`: enables finite repetition;
- `loop_count`: always required to be `1..maximum_loops`; it is ignored after
  validation when `loop=false`.

Exactly one of `task_file` and `poses` is required. This keeps legacy Qt files usable
without making future remote frontends depend on the robot computer's filesystem.

Result:

- `success`: true only if every Nav2 run succeeds;
- `error_code` and `message`: stable machine/human failure information;
- `missed_waypoints`: indices returned by Nav2 `FollowWaypoints`.

Feedback reports `state`, zero-based `loop_index`, current waypoint and total waypoint
count. `/agt/navigation/task_status` mirrors concise JSON state for operator displays.

## Validation and safety

The server rejects unreadable JSON, empty/oversized chains, non-finite poses, adjacent
duplicates, exact repeated patterns caused by the vendor append-on-save defect, and
points outside the current `/agt/map/global_occupancy`. It also requires recent
`agt_safety` diagnostics with motion enabled and the emergency-stop latch clear.

For schema-v1 files it additionally compares OccupancyGrid geometry, active map ID and
version, map YAML/image hashes, and the localization PCD hash. Missing or mismatched
active identity fails closed. Legacy files have no map binding and retain their existing
runtime map-boundary behavior.

The server never enables motion and never publishes velocity. It sends only
`nav2_msgs/action/FollowWaypoints`; cancellation or loss of safety readiness cancels
the active child goal. A Nav2 abort or any `missed_waypoints` makes the parent task fail.

## Qt integration boundary

The maintained `agt-navigation-v2` Qt branch submits an absolute saved schema-v1 task
path so the server can repeat binding/hash checks. Portable clients may still submit
`map`-frame poses. Start/Stop controls display Action feedback, result and missed
waypoints, and forward cancellation. Any future Qt/Web/autostart frontend must call
this same Action and must not duplicate its state machine.
