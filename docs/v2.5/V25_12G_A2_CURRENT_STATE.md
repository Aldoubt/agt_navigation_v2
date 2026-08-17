# V25-12G-A2 Current State

Date: 2026-08-17
Branch: `feat/v25-12g-maximum-feasible-coverage`

## Status

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED
WITH CONNECTIVITY / MAP CAVEATS
DIAGNOSTIC ONLY
NOT ROUTE-READY
```

A2 core implementation, acceptance harness, real-data topology derivation, strict-load validation, and deterministic rewrite are in place.

The next stage is strictly:

```text
V25-12G-A3 Connector + Reverse Motion Feasibility
```

A4 optimization must not begin until A3 has produced executable service/connector evidence.

## Implemented A2 contract

A2 consumes:

```text
vehicle_feasible_segments.yaml
turn_zones.yaml
```

and derives:

```text
ServiceResource
ServiceState
ConnectorCandidate
CandidateTopologyComponent
VehicleFeasibleServiceGraph
```

Canonical status remains:

```text
TOPOLOGY_CANDIDATE_ONLY
```

A2 does not contain START_POSE reachability, executable-edge claims, route order, or optimizer decisions.

## Real-data checkpoint

Observed greenhouse A2 graph:

```text
14 physical resources
97.3566739352054 m unique coverage
28 ordinary directional service states
4 dead-end reverse-service candidates
32 total service states
40 directed headland connector candidates
10 candidate topology components
9 isolated components
22 externally-unproven states
0 interior cross-aisle connectors
0 cross-side connectors
```

A1 segment identity and total coverage are preserved exactly at the semantic level.

Strict-load / deterministic rewrite evidence:

```text
A2 strict-load: PASS
A2 frozen semantics: PASS
A2 byte-stable rewrite: PASS
```

Exact pre/post upstream-asset SHA-256 values were not transcribed into the chat record and remain one consolidated verification item before final 12G completion.

## Important interpretation

A2 has solved the representation problem, not the motion-feasibility problem.

The large isolated-state count is expected under the frozen safety semantics: an `INTERIOR_BLOCKED_END` remains coverage evidence but does not receive a false cross-row access edge. A3 is responsible for deciding which headland candidates and dead-end reverse motions are actually executable.

`aisle_020.segment_001` remains the clean complete through-segment control:

```text
LOW_U_HEADLAND -> HIGH_U_HEADLAND
23.730021 m
2 ordinary service states
no dead-end service candidate
```

## A3 entry contract

A3 must take A2 candidate topology and convert only independently validated motions into executable evidence.

The A3 design must reuse the existing connector chain rather than introduce a parallel planner:

```text
Forward Dubins-family connector
  -> full footprint / Navigation Grid / Site Boundary gates
  -> R6A reverse-fallback admission when appropriate
  -> R6B bounded deterministic reverse-aware connector backend
```

For one-headland dead-end states, A3 must independently validate reverse-out motion along the frozen A1 safe segment geometry.

Required edge/service metrics for A4 include:

```text
status
backend
path_length_m
forward_distance_m
reverse_distance_m
cusp_count
search_expansions
risk / evidence diagnostics
```

A3 must fail closed and must not increase search budgets simply to force graph connectivity.

## Deferred items

Not part of A2 and not yet route-ready:

```text
START_POSE anchoring
mission reachability
A4 maximum-feasible-coverage search
PROVEN_OPTIMAL / BEST_FOUND_WITHIN_BUDGET classification
route export / execution
Workbench A2/A3 route editing
manual safety overrides
```

## Local workspace constraint

Persistent operator-local files remain intentionally outside this feature work:

```text
M  tools/rosbag_sensor_trimmer
?? tools/map_tools/render_pcd_top_views.py
```

Do not clean, reset, stage, or modify them as part of 12G.
