# V25-12G-A3 Current State

Date: 2026-08-17

## Status

V25-12G-A3 is implemented through the Site-Boundary reverse-admission amendment and has completed operator-side automated verification plus real-data diagnostic acceptance.

Current branch:

```text
feat/v25-12g-maximum-feasible-coverage
```

Acceptance code HEAD:

```text
76939424  test(v25-12g): fix boundary admission error assertion
```

Real-data evidence is frozen in:

```text
docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md
```

## What A3 now proves

A3 provides local vehicle-motion evidence for:

- directional service actions along A1 vehicle-feasible segments;
- exact forward-in/reverse-out dead-end retrace;
- A2 headland connector candidates through the R5 forward gate/audit, R6A admission policy, and R6B bounded reverse-aware local search;
- deterministic motion-graph serialization and strict loading;
- explicit distinction between endpoint Site-Boundary conflict and endpoint-safe forward-path Site-Boundary conflict.

A3 does not prove:

- start-pose reachability;
- route readiness;
- global connector infeasibility;
- maximum-coverage optimality;
- field execution readiness.

Top-level asset status remains:

```text
MOTION_EVIDENCE_ONLY
```

## Frozen Site-Boundary semantics

The Site Boundary remains the hard vehicle-permitted inner boundary.

The amended connector evidence is:

```text
CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
```

for an illegal start/goal footprint, which is a hard rejection and must never enter R6B, and:

```text
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
```

for legal endpoints where all locally relevant forward Dubins paths touch/cross the boundary. This second class may enter R6B because R6B independently re-checks the hard Site Boundary for every preview footprint.

No Site-Boundary geometry, navigation footprint, 0.05 m preview padding, or R6B search budget was relaxed by the amendment.

## Real-data state

Service layer:

```text
32 / 32 service actions executable
14 locally validated segments
97.3566739352054 m unique locally validated coverage
4 / 4 dead-end exact-retrace actions executable
```

Transition layer:

```text
40 A2 connector candidates
40 A3 transition validations
0 executable transitions
40 rejected transitions
0 unresolved transitions
```

Boundary/admission split:

```text
0  endpoint boundary conflicts
33 endpoint-safe forward-path boundary conflicts
33 / 33 admitted to R6B
0 / 33 recovered by R6B
```

The other seven HIGH_U connectors are `LOCAL_FORWARD_OCCUPANCY_BLOCKED` and also enter R6B.

All 40 transitions end with:

```text
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

No search reaches the frozen `max_expansions=30000`; observed maximum is 16621. Therefore increasing that budget is not justified by current evidence.

## Verified safety invariants

The real-data acceptance observed:

```text
all_a2_service_states_preserved: True
all_a2_connector_candidates_preserved: True
dead_end_non_retrace_count: 0
endpoint_boundary_conflict_reverse_admission_count: 0
site_boundary_reverse_bypass_count: 0
forbidden_semantic_key_count: 0
```

The generated motion graph also passed strict loading and byte-stable deterministic rewrite, and protected upstream input hashes remained unchanged.

## Current blocker

A3 no longer has an admission-policy ambiguity: all former broad boundary conflicts were confirmed to have legal endpoints and were correctly handed to R6B.

The remaining blocker is now isolated to connector motion generation/search:

```text
A2 topology has 40 candidate headland transitions
A3 current connector backends validate 0 as executable
```

This means an A4 maximum-feasible-coverage optimizer would currently receive a graph with executable service actions but no executable inter-segment transition edges. It could optimize only within disconnected/isolated service components and therefore cannot yet realize the intended multi-segment Maximum Feasible Coverage objective.

## Recommended next gate before A4

Do not tune R6B budgets blindly and do not weaken Site Boundary, NO_GO, footprint collision, or Ackermann constraints.

Before implementing the A4 optimizer, perform one focused connector-backend diagnosis to determine why all 40 bounded searches terminate without a solution. The diagnosis should separate at least:

- primitive/search-envelope insufficiency;
- state discretization and goal-shot reachability;
- Turn-Zone/local search-domain restrictions;
- footprint/navigation-grid collision constraints;
- genuine geometric insufficiency of available headland space.

Only after that evidence should the project choose between improving the bounded connector backend, adding a second deterministic reverse-aware connector backend, or formally accepting disconnected components as the input to A4.

## Source-control hygiene

The runtime `vehicle_feasible_motion_graph.yaml` is evidence only and must not be committed as a canonical map/runtime asset.

The unrelated local files remain outside this work:

```text
 M tools/rosbag_sensor_trimmer
?? tools/map_tools/render_pcd_top_views.py
```
