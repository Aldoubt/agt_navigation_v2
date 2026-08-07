# Behavior-tree execution boundary

V25-05 adds `agt_bt_executor` as a capability layer, not a second Mission
manager. The authoritative business boundary remains:

```text
/agt/missions/execute
        -> agt_mission_manager
        -> future controlled BT backend (V25-06)
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
