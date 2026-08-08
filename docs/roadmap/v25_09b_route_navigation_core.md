# V25-09B Robust Route Navigation Core

## Goal

Execute a frozen Route Asset without invoking the global planner for every goal. A persistent Route Asset remains in `map`; only the active segment is projected into `odom` at controlled boundaries and handed to a vehicle-specific tracker adapter.

## Runtime chain

```text
Mission / ExecuteWaypointTask
  -> Navigation Capability
  -> Route Asset Resolver
  -> Active RouteSegment (map)
  -> frozen map->odom snapshot
  -> RuntimePath (odom)
  -> Vehicle Tracker Adapter
  -> controller backend
  -> existing safety / chassis path
```

## V25-09B core invariants

1. ROUTE mode does not call the Global Planner during route execution
2. Route Asset is persistent `map`-frame intent
3. Active RuntimePath is controller geometry in `odom`
4. Once an active segment has been projected, later `map->odom` corrections do not move that active path
5. A newer alignment snapshot is consumed at the next segment boundary
6. F/R changes require a segment boundary, allowing an explicit stop before direction change
7. `event_ref` is surfaced as segment-completion metadata only; Navigation does not execute harvesting, spraying or capture capabilities
8. Route runtime does not publish TF and does not own velocity or Mission state

## First software gate

`agt_navigation.route_runtime` provides:

- immutable READY Route Asset loading and `route.csv` hash verification
- segment grouping and F/R semantics
- explicit map-to-odom planar projection
- `RuntimePath(frame_id=odom)`
- `VehicleTrackerAdapter` protocol
- segment-level state machine and bounded cancel/failure handling
- explicit `global_planner_requests == 0` integration metric

The first package test uses a fake tracker and two Route segments. It verifies that a correction arriving during segment `s000` does not change the active RuntimePath, while segment `s001` is projected using the newer snapshot.

## Intentionally deferred

- ROS/Nav2 FollowPath tracker adapter
- real vehicle tracking
- sparse global correction producer
- Hybrid-A*/State-Lattice/Reeds-Shepp offline connector
- wheel/GNSS/GTSAM fusion
- public `ExecuteRouteTask` Action

The public project boundary remains `ExecuteWaypointTask` until a real cross-package need proves otherwise.
