# V25-09B Robust Route Navigation Core

## Goal

Execute a frozen Route Asset without invoking the global planner for every goal. A persistent Route Asset remains in `map`; only the active segment is projected into `odom` at controlled boundaries and handed to a vehicle-specific tracker adapter.

The public project boundary remains `ExecuteWaypointTask`; V25-09B adds an internal MAP/ROUTE backend selector rather than a new public Route Action.

## Runtime chain

```text
Mission / ExecuteWaypointTask
  -> Navigation Capability
  -> exact TaskGroup revision
  -> optional TaskGroup -> Route execution binding
       no binding -> MAP backend / FollowWaypoints
       binding    -> ROUTE backend
                      -> READY Route Asset Resolver
                      -> Active RouteSegment (map)
                      -> frozen map->odom snapshot
                      -> RuntimePath (odom)
                      -> Vehicle Tracker Adapter
                      -> Nav2 FollowPath
  -> existing safety / chassis path
```

## Core invariants

1. ROUTE mode does not call the Global Planner during route execution
2. Route Asset is persistent `map`-frame intent
3. Active RuntimePath is controller geometry in `odom`
4. Once an active segment has been projected, later `map->odom` corrections do not move that active path
5. A newer alignment snapshot is consumed at the next segment boundary
6. F/R changes require a segment boundary and may select different controller IDs
7. `event_ref` is surfaced as segment-completion metadata only; Navigation does not execute harvesting, spraying or capture capabilities
8. Route runtime does not publish TF and does not own velocity or Mission state
9. MAP tasks with no Route binding preserve the existing FollowWaypoints behavior
10. A stale/invalid Route binding fails closed and never silently falls back to MAP
11. Formal ROUTE execution requires an exact execution-vehicle profile SHA and `route_acceptance.enabled=true`

## Implemented software layers

### Route runtime core

`agt_navigation.route_runtime` provides:

- immutable READY Route Asset loading and `route.csv` hash verification
- segment grouping and F/R semantics
- explicit map-to-odom planar projection
- `RuntimePath(frame_id=odom)`
- `VehicleTrackerAdapter` protocol
- segment-level state machine and bounded cancel/failure handling
- explicit `global_planner_requests == 0` metric

### Nav2 tracker adapter

`agt_navigation.nav2_follow_path_adapter` provides:

- `RuntimePath(odom)` -> `nav_msgs/Path(odom)`
- Nav2 `FollowPath` only
- forward/reverse controller selection
- serialized FIFO feedback dispatch so transport timing cannot reverse segment-completion order
- child cancellation and terminal failure propagation

### TaskGroup -> Route binding

`agt_navigation.route_task_binding` resolves the optional:

```text
tasks/<task_group_id>.route.yaml
```

The binding freezes the exact TaskGroup revision/content hash and Route manifest hash. The resolver also validates READY map content identity and execution-vehicle profile SHA.

See `docs/interfaces/task_route_execution_binding.md`.

### Navigation capability integration

`navigation_capability_server.py` subclasses the existing waypoint server so MAP logic remains unchanged while formal tasks with a valid execution binding use ROUTE.

The system launch now passes the selected `platform_profile` as `execution_vehicle_profile` to the capability server.

Parent cancel, safety loss, localization loss and TaskReadiness loss cancel/fail the active ROUTE tracker as well as update the parent NavigationSession.

## System integration tests

The package contains two complementary Action-level tests:

### ROUTE selector

- public `/agt/navigation/execute_waypoint_task`
- exact TaskGroup + READY Route binding
- fake `FollowPath` server
- intentionally no `FollowWaypoints` server
- F segment then R segment
- expected success with odom paths and forward/reverse controller IDs

Success proves ROUTE does not depend on the old MAP / FollowWaypoints path.

### MAP fallback

- same public `/agt/navigation/execute_waypoint_task`
- TaskGroup with no `.route.yaml`
- fake `FollowWaypoints` server
- intentionally no `FollowPath` server

Success proves existing MAP behavior remains backward-compatible.

## Current acceptance boundary

Software system integration can be accepted from unit/Action tests without a field vehicle.

Vehicle acceptance remains separate. In particular, a profile with unverified steering/minimum-turning-radius geometry may keep `route_acceptance.enabled=false`; such a vehicle must be rejected by ROUTE even when the software tests are green.

## Next gates

After this system-integration gate is green:

1. add explicit parent-cancel -> FollowPath cancel Action test
2. add safety/localization/TaskReadiness loss -> FollowPath cancel tests
3. validate against a real Nav2 controller_server with a synthetic/frozen READY Route
4. freeze vehicle-specific controller IDs and kinematic profile after measurement
5. run real bag -> READY Map -> READY Route -> simulated/vehicle Route tracking when dataset and field hardware are available

## Intentionally deferred

- real vehicle acceptance
- sparse global correction producer
- Hybrid-A*/State-Lattice/Reeds-Shepp offline connector
- wheel/GNSS/GTSAM fusion
- public `ExecuteRouteTask` Action
