# V25-12E Current State

Date: 2026-08-15

Purpose: short continuation checkpoint for future conversations / Codex tasks

Read this file first, then open only the focused documents needed for the next increment

## Scene ownership

```text
Greenhouse
→ MK-mini
→ Ackermann
→ Rmin 1.5 m
→ agricultural aisle-route production

GAAS open field
→ BUNKER
→ future GNSS / RTK truth benchmark
```

## Frozen real greenhouse inputs

```text
Aisle Graph
input/candidate aisles      19
MK-mini width accepted      18
width rejected               1  aisle_005
connector requests          17
```

## R5 forward result

```text
forward preview free         0

R5.6 classification
13 LOCAL_FORWARD_OCCUPANCY_BLOCKED
 1 LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
 3 LOCAL_FORWARD_MIXED_EVIDENCE
```

Held connectors:

```text
connector_001  map evidence insufficient
connector_007  mixed evidence
connector_009  mixed evidence
connector_013  mixed evidence
```

## R6A

Status: REAL-DATA PASS

```text
eligible reverse            13
hold map review              1
hold mixed evidence          3
keep forward                 0
```

Eligible IDs:

```text
002 003 004 005 006 008 010 011 012 014 015 016 017
```

Asset:

```text
reverse_fallback_admission.yaml
schema agt_reverse_fallback_admission/v1
```

## R6B first real smoke

Backend:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

Observed on the 13 admitted connectors:

```text
1  REVERSE_PRIMITIVE_PREVIEW_FREE
11 R6B_START_FOOTPRINT_NOT_FREE
1  NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

Solved regression baseline:

```text
connector_015
FORWARD → REVERSE → FORWARD
2 cusps
4.499 m total
3.899 m forward
0.600 m reverse
1490 expansions
0.154 m / 8.11 deg goal error
footprint FREE=1.000 OCC=0 UNKNOWN=0 coverage=1.000
```

Unsolved-after-search baseline:

```text
connector_017
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
67 expansions
```

The eleven start-footprint failures did not enter search

## Safe Connector Anchor smoke

The first anchor strategy walked inward only on the existing structural aisle
centerline for at most 1.50 m and required 0.30 m of stable preview-footprint
FREE support

Real greenhouse result:

```text
13 admitted connectors
2  READY_FOR_LOCAL_CONNECTOR
11 HOLD_ANCHOR_REVIEW
```

READY:

```text
connector_015
connector_017
```

Notable partial result:

```text
connector_014 start retreat 0.650 m
but goal had no stable FREE anchor within the bounded limit
```

Some raw poses were individually FREE (`connector_006` goal and
`connector_010` goal) but failed the stable inward-span requirement

This proves that increasing the retreat limit alone is not the correct repair

## Root cause frozen before vehicle-safe-lane smoke

```text
Aisle Graph structural safe evidence
!= Navigation Map FREE configuration space
```

`navigation_corridor.safe_common` does not require
`navigation.occupancy == FREE`

The final Navigation Map additionally includes obstacle padding and geometry-bad
cells, and FREE also requires sufficient ground support

Therefore:

```text
raw structural aisle centerline
!= guaranteed MK-mini vehicle-pose-free route lane
```

Do not relax R6B occupancy / footprint gates

## Vehicle-Safe Aisle Lane real smoke

Asset:

```text
vehicle_safe_aisles.yaml
schema agt_vehicle_safe_aisle_lane/v1
```

Real greenhouse result with 0.10 m longitudinal sampling, 0.05 m lateral search,
0.05 m preview footprint padding, and the canonical MK-mini profile:

```text
19 structural aisles
1  VEHICLE_SAFE_LANE_READY
7  VEHICLE_SAFE_LANE_PARTIAL
11 NO_VEHICLE_SAFE_LANE
```

Only `aisle_020` is READY:

```text
span        23.730 / 25.126 m
coverage     0.944
LOW_U retreat 1.396 m
HIGH_U retreat 0.000 m
max lateral shift used 0.050 m
```

Representative PARTIAL aisles:

```text
aisle_016 coverage 0.511, selected span 14.465 / 28.331 m
aisle_017 coverage 0.305, selected span  8.082 / 26.540 m
aisle_018 coverage 0.629, selected span 16.245 / 25.813 m
aisle_019 coverage 0.292, selected span  7.396 / 25.287 m
```

Many earlier aisles (`001`, `002`, `003`, `006`, `007`, `008`, `011`, `012`,
`014`, `015`) produced zero preview-footprint-free samples even with their
bounded lateral search. `aisle_005` remains correctly rejected by the structural
width gate because 0.658 m < 0.700 m preview required width

This is now a system-level route-evidence failure, not an endpoint-retreat or R6
search-budget problem

## Current diagnostic gate

Do not continue R6B or R7 yet

New diagnostic asset:

```text
vehicle_safe_lane_diagnostics.yaml
schema agt_vehicle_safe_lane_diagnostic/v1
```

Implementation:

```text
src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane_diagnostics.py
src/agt_offline_assets/test/test_vehicle_safe_lane_diagnostics.py
```

The diagnostic separates, for every aisle:

```text
structural reference point FREE / OCCUPIED / UNKNOWN / out-of-grid
zero-offset full-footprint FREE poses
any bounded-lateral full-footprint FREE poses
best-candidate OCCUPIED / UNKNOWN / out-of-grid blocker counts
```

The next decision is based on this evidence:

```text
center reference mostly OCCUPIED
→ Navigation Map / Corridor semantic disagreement

center reference mostly UNKNOWN
→ ground-support / map-coverage problem

center reference mostly FREE but full footprint OCCUPIED
→ vehicle-width / row-end / lateral clearance problem

many isolated lateral FREE poses but no continuous lane
→ continuity / lane-selection problem
```

Navigation Grid loading orientation has been reviewed: AGT writes PGM with one
Y flip and `load_navigation_grid()` flips it back into minimum-Y-first world-grid
order. There is currently no evidence for a PGM Y-axis inversion bug

## Current architecture boundary

```text
Structural Aisle Graph
↓
Vehicle-Safe Lane Evidence Diagnostic
↓
Vehicle-Safe Aisle Lane
↓ only if route evidence is coherent
Vehicle-safe Connector Anchor
↓
R6B bounded local F/R/F primitive search
├─ solved → later R8
└─ valid-lane / valid-anchor unsolved → R7 Smac Hybrid-A* / State Lattice
```

Held R6A connectors still do not enter R6B automatically

R6B remains preview-only because the real `base_footprint` reference / final mounted envelope are not yet physically measured

## Next gate

```text
1. local pytest lane diagnostic + frozen contracts
2. derive vehicle_safe_lane_diagnostics.yaml
3. inspect reference FREE/OCCUPIED/UNKNOWN distribution per aisle
4. inspect any-lateral FREE-pose fraction
5. inspect best-candidate blocker type
6. decide whether the repair belongs to Navigation Map semantics, corridor semantics, or vehicle clearance
7. do not rerun R6B until this upstream conflict is resolved
```

Do not widen occupancy, turn UNKNOWN into FREE, or hand invalid lanes to R7 merely
to make route generation succeed
