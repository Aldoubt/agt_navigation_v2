# P0 Reliable Operator Control Baseline

## Current Preview Chain

The maintained Qt5 Task Library stores schema-v1 task JSON below
`runtime/maps/<map_id>/versions/<map_version_id>/tasks/` and validates the
draft against the selected immutable Nav2 map raster. Planner preview remains
advisory: `TaskLibraryDock::PreviewTask()` emits a `TaskExecutionRequest` that
carries the current enabled points, and `rclcomm::PreviewTaskChain()` publishes
those points to `/agt/navigation/waypoint_preview_request`. The preview adapter
publishes `/plan` and a string status topic; no localization, safety enablement,
Nav2 control, or chassis output is started by the offline preview path.

## Current Execution Chain

`TaskLibraryDock::requestFromCurrent()` currently adds
`repository_.pathFor(task_.task_group_id)` to `TaskExecutionRequest::task_file`
when the task has no unsaved edits. `rclcomm::ExecuteTaskChain()` converts that
absolute path into `agt_interfaces/action/ExecuteWaypointTask.task_file` and
sends `/agt/navigation/execute_waypoint_task`. The Python
`waypoint_task_server.py` reads that file directly, loads either legacy
`points/theta` JSON or schema-v1 task groups, validates points against the live
`/agt/map/global_occupancy`, checks map binding and runtime gates, and then
sends Nav2 `FollowWaypoints`. `/agt/navigation/task_status` mirrors a deprecated
string JSON status for the GUI.

## Absolute `task_file` Failure Point

The execution request contains a frontend-local absolute path. That works only
when Qt and the robot share the same filesystem layout. A laptop editing under
`/home/user/...` and a robot running under `/home/robot/...` can preview from
the local draft but fail execution because the robot Action server cannot read
the submitted path. The path also bypasses the map manager as the runtime owner
of map-version assets and makes network reconnect recovery depend on Qt memory.

## Map Identity Sources

`agt_map_manager` owns the active map pointer and publishes `/agt/maps/active` as
a latched `MapVersionSummary`. `agt_system_manager` already derives
`TaskReadiness` map identity from `/agt/maps/active`. The waypoint server still
accepts launch parameters such as `current_map_id`, `current_map_version_id`, and
`current_map_yaml_path`, and may fall back to `TaskReadiness` or localization map
hashes. This creates multiple possible identity sources for execution.

## Session Recovery Gap

Execution state is held mainly by the Action goal handle inside Qt and the
waypoint server's in-memory `_active` flag. The compatibility status topic is a
volatile `std_msgs/String` publisher, so a restarted Qt client cannot reliably
recover the authoritative session ID, task revision, current waypoint, missed
waypoints, or terminal blocker. Qt disconnect does not intentionally cancel the
Nav2 child goal, but the UI cannot reconstruct the state after reconnect.

## P0 Packages And Interfaces To Change

This P0 slice keeps Qt/Web as clients and leaves Nav2, localization, safety, and
chassis ownership unchanged. The implementation will add a robot-side Task
Registry in `agt_navigation`, extend `ExecuteWaypointTask` to execute by
`map_id + map_version_id + task_group_id + revision`, add transient-local
`NavigationSessionStatus`, expose task registry/session services in
`agt_interfaces`, add an ID-driven map activation service wrapper in
`agt_map_manager`, and update the Qt fork execution request so it syncs task JSON
through the registry and no longer submits absolute task paths.

