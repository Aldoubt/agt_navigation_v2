# Paper I Fixed Route to Nav2 RPP Validation Design

## Status

DESIGN ONLY / IMPLEMENT AFTER MAP-SEMANTICS FREEZE

## Purpose

Provide one minimal runtime proof that a frozen Paper I semantic map can produce an executable robot path. This is a downstream validation, not a planner contribution.

## Critical interface rule

CSV is an AGT offline interchange format, **not** a Nav2-native path format.

Runtime flow:

```text
frozen route asset
  -> route.csv + route.yaml
  -> AGT CSV route loader
  -> nav_msgs/Path (frame_id=map)
  -> nav2_msgs/action/FollowPath
  -> controller_id=RPP
  -> Nav2 controller_server
  -> cmd_vel / chassis chain
```

## Route asset contract

### `route.csv`

Required columns:

```text
seq,x_m,y_m,yaw_rad,segment_id
```

Optional future columns such as speed limits must not be required for Paper I.

Rules:

- `seq` is zero-based, unique, strictly increasing.
- coordinates are metres in `map` frame.
- `yaw_rad` follows ROS REP-103 planar yaw convention.
- if a generating tool derives yaw from neighboring points, it must write the resulting yaw explicitly before freeze.
- points are ordered in execution order.

### `route.yaml`

Must record:

```yaml
schema: agt_fixed_route/v1
route_id: greenhouse_01_validation_route_v01
frame_id: map
csv: route.csv
source:
  map_bundle_id: greenhouse_01_map_assets_v01
  map_bundle_manifest_sha256: ...
  accepted_map_yaml_sha256: ...
  accepted_map_pgm_sha256: ...
  semantic_map_sha256: ...
route_csv_sha256: ...
```

No runtime node may silently transform an unverified route from another frame.

## Runtime adapter

Implement in a later branch named:

`feat/paper1-route-rpp-validation`

Preferred ownership: `src/agt_navigation` unless an existing route-runtime package is discovered during implementation that already owns `FollowPath` submission.

Responsibilities:

1. parse and validate `route.yaml` + `route.csv`;
2. verify route SHA and source map identities against selected site assets;
3. convert to `nav_msgs/msg/Path` with one `PoseStamped` per CSV row;
4. publish an optional preview path for RViz;
5. submit `nav2_msgs/action/FollowPath` with explicit controller id for Regulated Pure Pursuit;
6. expose completion/failure and tracking feedback for logging.

It must not invoke Planner Server and must not alter route geometry at runtime.

## Controller role

Use Nav2 Regulated Pure Pursuit as a fixed tracking baseline. The route must already be geometrically feasible for the Ackermann platform; RPP is not responsible for repairing an infeasible global route.

Pin the final parameter names to ROS 2 Humble / installed Nav2 version on the target machine before implementation.

## Validation metrics

Record at minimum:

- route completion success
- path completion ratio
- cross-track error: mean / P95 / max
- heading error: mean / P95 / max
- elapsed time
- stop / controller failure reason
- safety intervention count if the existing safety chain emits it

One fixed RPP configuration is used for Paper I. Parameter sweeps and controller comparisons are out of scope.

## Non-goals

- planner comparison
- route optimization
- online replanning
- dynamic obstacle avoidance research
- maximum coverage
- Fields2Cover comparison
- Ackermann turn optimizer

## Branch gate

Create `feat/paper1-route-rpp-validation` only from the commit where:

- Paper I site identity is frozen;
- accepted map hash is fixed;
- semantic map is frozen;
- fixed route asset is frozen and verified.
