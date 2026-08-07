# Behavior-tree execution boundary

V25-05 adds `agt_bt_executor` as a capability layer, not a second Mission
manager. The authoritative business boundary remains:

```text
/agt/missions/execute
        -> agt_mission_manager
        -> controlled BT backend (V25-06)
        -> BehaviorTree.CPP
        -> project Action nodes
        -> Relocalize / ExecuteWaypointTask / other project capabilities
```

`agt_mission_manager` owns Mission state, audit records, cancellation policy,
and the public `ExecuteMission` Action. BT nodes own only temporary tree data
and execution evidence. They do not publish `cmd_vel` or TF, inspect raw sensor
streams, or call Nav2 native Actions directly. The two capability Action nodes
cancel their active project Action during `halt()` and fail closed on rejected,
unavailable, timed-out, aborted, or unconfirmed-cancelled operations.

The V25-05 smoke runner loads the allowlisted installed smoke tree and executes
no motion capability. It is not part of default bringup. Groot2 monitoring, if
available in the deployment, is diagnostic only and disabled by default.

## V25-06 first mission status

The first production-style tree is implemented behind the `behavior_tree`
backend of `agt_mission_manager`. It accepts exactly one `WAYPOINT_TASK` step,
uses a per-execution safe id for both internal BT and waypoint requests, and
requires a non-empty canonical task content hash. Readiness blockers are
returned as structured Mission status evidence; internal BT failures are not
guessed to be localization failures. Parent cancellation has bounded goal,
result, and cancel waits and propagates through `haltTree()` to project Action
nodes.

Status: `SYSTEM-INTEGRATED` for the fake capability full-chain acceptance. The
`agt_bringup` launch test starts the real `agt_mission_manager` and
`agt_bt_executor`, loads `v25_06_waypoint_mission.xml`, and fakes only
`EvaluateTaskReadiness`, `Relocalize`, and `ExecuteWaypointTask`. It covers
success, readiness/relocalization/waypoint failures, feedback, and parent
cancellation. Vehicle validation remains pending and is a separate acceptance
stage.
The default backend remains `sequential`.

Run the focused test with:

```bash
colcon test --packages-select agt_bringup --ctest-args -R p0_bt_mission
```
