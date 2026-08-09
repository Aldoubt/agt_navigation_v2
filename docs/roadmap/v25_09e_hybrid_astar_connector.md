# V25-09E Hybrid A* Agricultural Headland Connector

V25-09E adds an offline, local obstacle-aware connector backend behind the
existing `ConnectorPlannerBackend` contract. It is not a ROS/Nav2 global
planner, does not own TF or Localization, and does not add a public Action.

```text
Rows -> lane sequence -> ConnectorPlannerBackend
  straight | reeds_shepp | hybrid_astar
        -> Route Asset -> existing feasibility -> READY Route
```

The first implementation uses SE(2) yaw bins, constant-curvature forward and
reverse motion primitives, a deterministic priority queue, occupancy/unknown/
semantic collision checks, a 2D grid lower-bound heuristic and the existing
Reeds-Shepp solver as an analytic kinematic lower bound/goal expansion. Missing
planning context, invalid grids, collision endpoints, timeout, iteration limit
and reconstruction failure are fail-closed results.

Hybrid A* remains a local connector. Final full-footprint feasibility remains
the authoritative Route gate; obstacle search does not replace semantic or
Route Asset validation.
