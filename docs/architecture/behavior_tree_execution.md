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

Status: `IMPLEMENTED`; fake integration validated; vehicle validation pending.
The default backend remains `sequential`.
