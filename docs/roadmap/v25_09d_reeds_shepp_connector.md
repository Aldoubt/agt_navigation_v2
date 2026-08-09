# V25-09D Reeds-Shepp Agricultural Headland Connector

V25-09D adds a local, curvature-bounded Reeds-Shepp connector backend to the
existing offline Route generation pipeline:

```text
semantic rows -> lane ordering -> boustrophedon -> connector backend
  straight | reeds_shepp -> Route Asset -> existing feasibility -> READY Route
```

The solver operates only between the actual previous lane end pose and next
lane start pose. It does not search obstacles, keepouts, dynamic objects or a
global map. Existing feasibility remains the final occupancy, footprint,
semantic and clearance judge. Hybrid A* is a later stage.

## Solver Contract

The solver uses normalized SE(2) poses and `rho = min_turning_radius_m`, with
signed `L`, `R` and `S` primitive lengths. Positive lengths are forward and
negative lengths are reverse. All canonical families are generated through
reflection and time reversal, then selected deterministically by total length,
direction changes, primitive count and signature.

Samples are integrated from the vehicle heading, not from travel tangent. Every
sample carries `x`, `y`, `yaw` and `direction`; endpoints are explicitly snapped
to the requested poses. `allow_reverse: false` filters all negative primitives
and fails closed when no forward-only solution exists.

## Route Boundary

A direction cusp is a Route segment boundary. A connector with F/R/F samples is
materialized as homogeneous segments such as `connector_003_f00`,
`connector_003_r01`, and `connector_003_f02`. The existing ROUTE runtime and
tracker therefore keep their unchanged F/R segment semantics.

The selected backend and core parameters are recorded in the Route manifest.
Preview properties expose connector backend, length, reverse length and
direction-change metrics. No ROS Action, TF owner or online Nav2 planner is
added.

Reeds-Shepp agricultural penalties, obstacle-aware search, Hybrid A*, local
occupancy, ESDF, GNSS and graph fusion remain out of scope.
