# V25-12E Current State

Date: 2026-08-14

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

## Root cause now frozen

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

## Vehicle-Safe Aisle Lane

New derived asset:

```text
vehicle_safe_aisles.yaml
schema agt_vehicle_safe_aisle_lane/v1
```

Implementation:

```text
src/agt_offline_assets/agt_offline_assets/vehicle_safe_lane.py
src/agt_offline_assets/test/test_vehicle_safe_lane.py
```

Policy:

```text
immutable Aisle Graph structural centerline
+ frozen Navigation Grid
+ canonical MK-mini preview footprint
        ↓
resample structural aisle
        ↓
bounded lateral search in row frame
        ↓
preview footprint must be fully FREE
        ↓
bounded lateral continuity
        ↓
longest continuous vehicle-safe lane segment
```

The lane records:

```text
coverage fraction
LOW_U retreat
HIGH_U retreat
allowed lateral shift
maximum used lateral shift
continuous lane centerline
```

The allowed lateral shift is conservatively bounded by both configuration and
vehicle-width surplus inside the aisle

No lateral teleport is allowed: if adjacent samples would require a lateral
jump above the configured limit, the lane segment is explicitly broken

Aisle Graph and Navigation Map remain immutable evidence

This layer is still PREVIEW_ONLY_NOT_R8_VEHICLE_READY

## Current architecture boundary

```text
R5.6 Forward Candidate Audit
↓
R6A Admission 13 / 1 / 3 / 0
↓
Vehicle-Safe Aisle Lane
↓
Vehicle-safe Connector Anchors
↓
R6B bounded local F/R/F primitive search
├─ solved → later R8
└─ valid-lane / valid-anchor unsolved → R7 Smac Hybrid-A* / State Lattice
```

Held R6A connectors still do not enter R6B automatically

## Next gate

```text
1. local pytest vehicle-safe lane + R6 contracts
2. derive vehicle_safe_aisles.yaml from frozen aisle_graph.yaml + navigation_map.yaml + mk_mini.yaml
3. inspect READY / PARTIAL / UNAVAILABLE aisle counts
4. inspect coverage / LOW_U retreat / HIGH_U retreat / lateral shift for all aisles
5. only after real-data lane review, wire connector anchors to the vehicle-safe lane
6. rerun the 13 R6A-admitted connectors
7. preserve connector_015 as the solved R6B regression baseline
8. only valid-lane / valid-anchor unsolved connectors may move to R7
```

Do not reopen Ground / Row tuning unless the new vehicle-safe lane evidence shows
that the Navigation Map / corridor contracts themselves require upstream changes
