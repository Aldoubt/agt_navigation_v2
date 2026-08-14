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

## Connector anchor finding

Root cause:

```text
R4 ConnectorRequest
currently uses raw Aisle Graph endpoint pose

but

raw structural centerline endpoint
!= guaranteed full-vehicle-safe connector anchor
```

Do not relax R6B occupancy / footprint gates

New derived asset:

```text
connector_anchors.yaml
schema agt_connector_anchor_plan/v1
```

Implementation:

```text
src/agt_offline_assets/agt_offline_assets/connector_anchors.py
src/agt_offline_assets/test/test_connector_anchors.py
```

Policy:

```text
R6A admitted connector only
raw endpoint
→ walk inward on the same aisle centerline
→ nearest stable MK-mini preview-footprint FREE pose
→ freeze retreat distance
→ adjusted ConnectorRequest
→ R6B
```

Aisle Graph and Navigation Map remain immutable evidence

Later Route assembly must trim aisle traversal to the selected connector anchor
rather than driving through the unsafe raw endpoint tail

## Current architecture boundary

```text
R5.6 Forward Candidate Audit
↓
R6A Admission 13 / 1 / 3 / 0
↓
Safe Connector Anchor Preparation
↓
R6B bounded local F/R/F primitive search
├─ solved → later R8
└─ valid-anchor unsolved → R7 Smac Hybrid-A* / State Lattice
```

Held R6A connectors still do not enter anchor preparation or R6B automatically

R6B remains preview-only because the real `base_footprint` reference / final mounted envelope are not yet physically measured

## Next gate

```text
1. local pytest connector anchor + R6 contracts
2. derive connector_anchors.yaml from aisle_graph.yaml + R6A admission
3. inspect start/goal retreat distances for all 13 admitted connectors
4. apply READY anchor-adjusted ConnectorRequests
5. rerun R6B
6. preserve connector_015 regression
7. only valid-anchor unsolved connectors may move to R7
```

Do not reopen Ground / Row / Aisle tuning unless a new real-data failure directly points back to those layers
