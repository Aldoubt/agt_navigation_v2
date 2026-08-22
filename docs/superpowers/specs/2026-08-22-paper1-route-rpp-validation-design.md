# Paper I Fixed Route to Nav2 RPP Validation Design

## Status

DESIGN ONLY / IMPLEMENT AFTER MAP-SEMANTICS FREEZE

## Purpose

Provide one minimal runtime proof that a frozen Paper I semantic map can produce an executable robot path. This is a downstream validation, not a planner contribution.

## Existing runtime assets to reuse

Do **not** create a second CSV-to-Nav2 stack. `src/agt_navigation` already owns the required runtime boundary:

- `route_runtime.py` loads an immutable READY `route.yaml + route.csv`, validates the frozen CSV hash and map/vehicle binding, and projects persistent `map` geometry to active `odom RuntimePath` segments.
- `nav2_follow_path_adapter.py` converts `RuntimePath` to `nav_msgs/Path` and submits `nav2_msgs/action/FollowPath` without requesting a global planner.

The Paper I work therefore only needs to export the offline route into the existing canonical Route Asset contract, bind it to the frozen Paper I map/vehicle identity, configure RPP as the FollowPath controller, and add paper-facing logging/validation where missing.

## Critical interface rule

CSV is an AGT persistent Route Asset format, **not** a Nav2-native path format.

Runtime flow:

```text
frozen Paper I route geometry
  -> canonical READY route.yaml + route.csv (frame_id=map)
  -> existing load_route_asset()
  -> existing map->odom segment projection
  -> RuntimePath
  -> existing Nav2FollowPathTrackerAdapter
  -> nav_msgs/Path (frame_id=odom)
  -> nav2_msgs/action/FollowPath
  -> controller_id=<Paper I RPP plugin id>
  -> Nav2 controller_server
  -> existing safety / chassis chain
```

## Route asset contract

Paper I must conform to the current `route_runtime.py` canonical CSV schema instead of introducing a reduced schema.

### `route.csv`

Required columns:

```text
seq,segment_id,x,y,yaw,direction,v_ref,curvature,clearance,semantic_ref,event_ref
```

Rules inherited from the current runtime contract:

- `seq` is strictly increasing.
- coordinates are metres in persistent `map` frame.
- `yaw` is finite planar ROS yaw.
- `direction` is `F` or `R`.
- every sample has a non-empty `segment_id`.
- `v_ref`, `curvature`, and `clearance` are finite.
- `semantic_ref` and `event_ref` may be empty but the columns remain present.
- route points are frozen in execution order before runtime.

Paper I offline export may derive yaw / curvature / clearance from the accepted route geometry, but the resulting values must be written explicitly into the frozen CSV rather than recomputed with hidden runtime heuristics.

### `route.yaml`

Reuse the existing READY Route manifest semantics. At minimum Paper I must preserve:

```yaml
status: READY
frame_id: map
route_id: greenhouse_01_validation_route_v01
revision: 1
route_csv_sha256: sha256:<...>
map_binding:
  map_id: greenhouse_01
  map_version_id: <frozen-paper-map-version>
  map_content_sha256: <accepted-map-content-identity>
vehicle_binding:
  platform_profile_sha256: <frozen-platform-profile-sha>
```

If Paper I needs additional bundle/semantic lineage, add it as compatible manifest metadata rather than weakening or replacing the runtime keys. Record at least the map-resource manifest SHA and formal semantic-map SHA in the paper freeze report.

No runtime path may silently accept an unverified map-frame route or mismatched map/vehicle identity.

## Implementation branch

Create later:

`feat/paper1-route-rpp-validation`

Base it on the exact Paper I map-semantics freeze commit, not today's moving branch head.

Preferred ownership remains `src/agt_navigation` because the necessary Route runtime and FollowPath adapter already exist there.

Likely implementation work is intentionally small:

1. add/reuse an offline exporter that writes the existing canonical Route Asset from the frozen Paper I route geometry;
2. verify route map/vehicle binding against the selected Paper I site;
3. configure the existing `Nav2FollowPathTrackerAdapter` with the RPP controller id;
4. add a thin launch/CLI path for one fixed-route run if the current runtime entry point is not already sufficient;
5. add paper-facing tracking metric capture without changing Route runtime semantics.

Do not add another generic CSV parser, another `nav_msgs/Path` conversion implementation, or another FollowPath client.

## Controller role

Use Nav2 Regulated Pure Pursuit as a fixed tracking baseline. The route must already be geometrically feasible for the Ackermann platform; RPP is not responsible for repairing an infeasible global route.

Pin parameter names and plugin configuration to the actual ROS 2 Humble / installed Nav2 version on the target robot before implementation.

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
- canonical READY Route Asset is frozen and verified.
