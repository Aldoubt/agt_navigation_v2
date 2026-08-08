# TaskGroup -> Route Execution Binding

V25-09B keeps the public navigation Action and the TaskGroup schema unchanged while allowing one exact TaskGroup revision to opt into the ROUTE backend

The binding is an execution-selection artifact, not a second Route Asset and not a Mission definition

## 1. Selection rule

```text
formal ExecuteWaypointTask
  -> resolve exact TaskGroup revision/content hash
  -> tasks/<task_group_id>.route.yaml exists?
       no  -> MAP backend / existing FollowWaypoints path
       yes -> validate binding fail-closed
              -> READY Route Asset
              -> RouteNavigationCore
              -> FollowPath
```

A malformed, stale or hash-mismatched binding MUST fail the request and MUST NOT silently fall back to MAP

Legacy task files and direct pose debug inputs never select ROUTE through this mechanism

## 2. Location

```text
runtime/maps/<map_id>/versions/<map_version_id>/tasks/
  <task_group_id>.json
  <task_group_id>.route.yaml    # optional
```

The binding is colocated with the robot-side Task Registry identity but does not modify the TaskGroup JSON schema

## 3. Schema

```yaml
schema_version: 1
status: READY
backend: ROUTE

task_binding:
  task_group_id: greenhouse_inspection
  task_revision: 3
  task_content_sha256: sha256:<64 hex>

route_binding:
  route_id: greenhouse_main_route
  revision: 5
  route_manifest_sha256: sha256:<64 hex>
```

The runtime additionally validates the selected Route Asset against

```text
active map version
map_content_sha256
selected execution vehicle profile SHA256
Route route.csv SHA256
Route status == READY
```

Thus a TaskGroup revision, Route revision, map-content identity or execution-vehicle change cannot silently reuse stale execution geometry

## 4. Backend boundary

MAP retains the existing behavior

```text
ExecuteWaypointTask
  -> TaskGroup points(map)
  -> FollowWaypoints
  -> Nav2 planning/controller stack
```

ROUTE uses frozen offline geometry

```text
ExecuteWaypointTask
  -> exact TaskGroup/Route binding
  -> RouteSegment(map)
  -> authoritative map->odom snapshot
  -> RuntimePath(odom)
  -> FollowPath
```

ROUTE MUST NOT request `ComputePathToPose` or another Global Planner during normal segment execution

The active RuntimePath is frozen for the current segment; a newer global alignment snapshot is consumed only at the next segment boundary

## 5. Vehicle gate

The navigation capability receives one explicit `execution_vehicle_profile`

ROUTE execution requires

```yaml
platform:
  route_acceptance:
    enabled: true
```

and the profile file SHA256 must equal the Route Asset vehicle binding

This is intentionally stricter than MAP operation. A software-complete profile with unverified steering geometry may remain usable for development while formal ROUTE execution stays fail-closed

## 6. Safety / cancellation

The parent `ExecuteWaypointTask` remains the lifecycle owner visible to Mission

Parent cancellation, safety loss, localization loss or TaskReadiness loss must cancel the active FollowPath child and transition the Route execution to a terminal failed/canceled state

`event_ref` values are emitted as segment-completion metadata only; Navigation does not execute harvesting, spraying, capture or other business capabilities

## 7. Compatibility

This contract intentionally adds no

```text
ExecuteRouteTask.action
ExecuteNavigationTask.action
TaskGroup schema field
TF publisher
velocity publisher
```

A TaskGroup with no `.route.yaml` binding behaves exactly as before through the MAP backend
