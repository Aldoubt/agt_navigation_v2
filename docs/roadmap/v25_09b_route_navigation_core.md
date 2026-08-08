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
  -> collision monitor
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
12. Parent cancel and runtime-gate loss must cancel the active FollowPath child rather than only terminating the parent Action

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

The system launch passes the selected `platform_profile` as `execution_vehicle_profile` to the capability server.

Parent cancel, safety loss, localization loss and TaskReadiness loss cancel/fail the active ROUTE tracker as well as update the parent NavigationSession.

`RouteBackendExecutor` waits on an `rclpy.task.Future`; it does not depend on a Python asyncio event loop. FollowPath terminal feedback, parent cancel and runtime-gate failure all resolve the same completion Future.

## System integration tests

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

### Runtime cancellation gates

`test_navigation_capability_runtime_gates.py` starts a FollowPath child that deliberately does not finish by itself. The test only passes if the child actually receives cancellation.

Covered gates:

- parent `ExecuteWaypointTask` cancel -> FollowPath cancel -> parent CANCELED
- safety ready -> not ready -> FollowPath cancel -> parent FAILED
- localization accepted -> invalid -> FollowPath cancel -> parent FAILED
- TaskReadiness ready -> blocked -> FollowPath cancel -> parent FAILED

The structured final `NavigationSessionStatus` preserves the runtime blocker identity used to stop motion.

## Real controller software smoke

`route_controller_smoke.launch.py` and `route_controller_smoke.py` provide the first real Nav2 Controller Server gate:

```text
synthetic odom Path
  -> real controller_server / FollowPath
  -> collision_monitor
  -> agt_safety
  -> differential-drive simulator
  -> odom / TF
```

The smoke deliberately starts no map server, planner server or BT navigator. It validates controller plumbing only and keeps `global_planner_requests == 0` by construction.

See `docs/workflows/route_controller_software_smoke.md`.

## Current acceptance boundary

Software system integration can be accepted from package/Action tests plus the controller-only smoke without a field vehicle.

Vehicle acceptance remains separate. In particular, a profile with unverified steering/minimum-turning-radius geometry may keep `route_acceptance.enabled=false`; such a vehicle must be rejected by ROUTE even when the software tests are green.

The controller-only smoke uses the existing differential simulator and BUNKER-style software controller parameters. It must not be cited as MK-mini Ackermann or BUNKER field acceptance.

## Next gates

After the runtime-gate tests and controller-only smoke are green:

1. connect the formal `ExecuteWaypointTask -> ROUTE backend` to the real controller_server in one synthetic full-chain launch/test
2. freeze vehicle-specific forward/reverse controller IDs after controller experiments
3. measure and freeze MK-mini Ackermann minimum turning radius / steering limits before enabling `route_acceptance`
4. run real bag -> READY Map -> READY Route -> simulated Route tracking when the dataset is available
5. run the same READY Route chain on the execution vehicle with safety, localization and tracking metrics

## Intentionally deferred

- real vehicle acceptance
- sparse global correction producer
- Hybrid-A*/State-Lattice/Reeds-Shepp offline connector
- wheel/GNSS/GTSAM fusion
- public `ExecuteRouteTask` Action
