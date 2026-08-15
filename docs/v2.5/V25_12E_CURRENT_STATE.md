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
span          23.730 / 25.126 m
coverage       0.944
LOW_U retreat  1.396 m
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
`014`, `015`) produced zero preview-footprint-free samples even with bounded
lateral search. `aisle_005` remains correctly rejected by the structural width
gate because 0.658 m < 0.700 m preview required width

This is a system-level route-evidence failure, not an endpoint-retreat or R6
search-budget problem

## Vehicle-Safe Lane evidence diagnostic — DONE

Asset:

```text
vehicle_safe_lane_diagnostics.yaml
schema agt_vehicle_safe_lane_diagnostic/v1
```

Real classification:

```text
13 FOOTPRINT_OCCUPIED_DOMINANT
 2 MIXED_OCCUPIED_DOMINANT
 2 MOSTLY_CONFIGURATION_SPACE_FREE
 1 CENTER_REFERENCE_OCCUPIED_DOMINANT
 1 STRUCTURAL_WIDTH_BLOCKED
```

No aisle was UNKNOWN-dominant or map-coverage-limited

Most structural centerline reference points are frequently FREE, but the full
MK-mini preview footprint overlaps OCCUPIED cells. `aisle_003` is the only
center-reference-occupied-dominant aisle

Configuration-space positive controls:

```text
aisle_016 any lateral FREE 0.951
aisle_020 any lateral FREE 0.945
```

## OCCUPIED source audit — DONE

Asset:

```text
vehicle_safe_lane_occupancy_sources.yaml
schema agt_vehicle_safe_lane_occupancy_source_audit/v1
```

Source exactness:

```text
APPROX_GEOMETRY_THRESHOLD_SOURCE_NO_POINT_COUNT_GROUND_VALID
```

Global Navigation OCCUPIED:

```text
actual occupied  85468
raw obstacle     46981  0.550
geometry         15813  0.185
padding only     22674  0.265
unexplained          0  0.000
```

Route-pose occupied-source classification:

```text
17 PADDING_ONLY_DOMINANT
 1 RAW_OBSTACLE_DIRECT_DOMINANT   aisle_003
 1 STRUCTURAL_WIDTH_BLOCKED       aisle_005
```

Representative padding fractions among occupied cells hit by the selected best
bounded-lateral vehicle poses:

```text
aisle_001 0.495
aisle_011 0.534
aisle_013 0.761
aisle_015 0.700
aisle_019 0.786
aisle_020 0.781
```

This strongly elevates obstacle-padding / full-footprint double-margin as the
next hypothesis, but does not yet justify changing the frozen map

The current Navigation Map implementation default is:

```text
resolution_m       0.10 m
obstacle_padding_m 0.05 m
```

Metric obstacle padding is converted with `ceil(padding/resolution)` and then a
square maximum filter. Therefore a 0.05 m request would become a one-cell
8-neighborhood dilation at 0.10 m resolution. The exact frozen greenhouse
`obstacle_padding_m` must still be read from `derivation.yaml` rather than
assumed from the default

## Current diagnostic gate: counterfactual padding sensitivity

Do not continue R6B or R7 yet

New asset:

```text
vehicle_safe_lane_padding_sensitivity.yaml
schema agt_vehicle_safe_lane_padding_sensitivity/v1
```

Matrix:

```text
map padding radius  0 / 1 / 2 grid cells
footprint padding   0 / 0.05 m
```

The sensitivity audit reconstructs immutable counterfactual Navigation Grids
from the frozen direct OCCUPIED source mask plus ground-height / ground-support
sidecars. It does not overwrite `navigation_map.pgm/yaml`

It must report:

```text
current_requested_obstacle_padding_m
current_effective_padding_cells
per-case occupied/free/unknown counts
per-case zero-feasible aisle count
per-case mostly-configuration-space-free aisle count
per-case mean any-lateral FREE fraction
per-aisle any-lateral FREE fraction
```

Decision policy:

```text
0-cell map padding strongly restores aisle feasibility
while 1-cell destroys it
→ map/footprint margin duplication or coarse discrete dilation

0-cell still leaves early aisles blocked
→ raw obstacle / geometry source remains limiting

footprint 0.05 alone causes the drop
→ preview envelope margin is the primary issue
```

## Current architecture boundary

```text
Structural Aisle Graph
↓
Vehicle-Safe Lane Evidence Diagnostic       DONE
↓
OCCUPIED Source Audit                       DONE
↓
Counterfactual Padding Sensitivity          CURRENT
↓ only after upstream semantics are coherent
Vehicle-Safe Aisle Lane
↓
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
1. local pytest occupancy-source + padding-sensitivity contracts
2. derive vehicle_safe_lane_padding_sensitivity.yaml
3. confirm frozen current_requested_obstacle_padding_m and effective cell radius
4. compare six map-padding / footprint-padding cases
5. inspect aisle_003 / 011 / 013 / 015 / 016 / 020 as probes
6. choose map-padding semantics only from the sensitivity evidence
7. do not rerun R6B until the vehicle-safe aisle evidence is coherent
```

Do not turn UNKNOWN into FREE, erase raw obstacles, or hand invalid lanes to R7
merely to make route generation succeed
