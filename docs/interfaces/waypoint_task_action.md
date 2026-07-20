# Qt waypoint task Action

`/agt/navigation/execute_waypoint_task` is the stable project boundary between an
operator frontend and Nav2 waypoint execution. The current Qt5 application remains an
unmodified map, topology and task-chain editor; its JSON output is validated by the
project server before motion is requested.

## Action

Type: `agt_interfaces/action/ExecuteWaypointTask`

Goal:

- `task_file`: absolute path to a Qt-compatible JSON containing `points`;
- `poses`: portable `map`-frame input for future Qt/Web/autostart frontends;
- `loop`: enables finite repetition;
- `loop_count`: required to be `1..maximum_loops` when looping.

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

The server never enables motion and never publishes velocity. It sends only
`nav2_msgs/action/FollowWaypoints`; cancellation or loss of safety readiness cancels
the active child goal. A Nav2 abort or any `missed_waypoints` makes the parent task fail.

## Current Qt compatibility boundary

The vendor **Start Task Chain** button still uses pose-distance polling and is not this
Action. For the baseline Demo, create points in Qt, save each task to a new JSON file,
then execute that file with the project client. A future project-owned Qt/Web frontend
can call the same Action without changing navigation, safety or chassis code.
