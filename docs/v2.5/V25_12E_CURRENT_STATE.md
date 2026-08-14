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

## R6B

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Backend:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

Core files:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
src/agt_offline_assets/agt_offline_assets/reverse_route_io.py
src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Focused architecture:

```text
docs/architecture/agricultural_route_r6_reverse_fallback.md
docs/v2.5/V25_12E_R6_REVERSE_FALLBACK.md
```

R6B policy:

```text
R6A admitted IDs only
bounded local row-frame search
Ackermann curvature {-1/R, 0, +1/R}
FORWARD / REVERSE
explicit cusp
max 2 cusps by default
OCCUPIED / UNKNOWN / out-of-grid fail closed
MK-mini preview footprint + 0.05 m padding
```

R6B is preview-only because the real `base_footprint` reference / final mounted envelope are not yet physically measured

## Next gate

```text
1. local pytest R6A + R6B
2. load frozen coverage_order.yaml
3. load reverse_fallback_admission.yaml
4. run only 13 admitted connectors
5. report solved / unsolved
6. inspect reverse distance / cusp count / expansions / footprint evidence
7. R6B solved → later R8
8. R6B unsolved → R7 Smac Hybrid-A* / State Lattice
```

Do not reopen Ground / Row / Aisle tuning unless a new real-data failure directly points back to those layers
