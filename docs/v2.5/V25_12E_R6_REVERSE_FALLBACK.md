# V25-12E R6 Reverse Fallback

Status: R6A REAL-DATA PASS; R6B BOUNDED REVERSE PRIMITIVE CORE IMPLEMENTED / LOCAL ACCEPTANCE PENDING

Date: 2026-08-14

This file is the focused continuation ledger for reverse-aware agricultural
connector planning. Read it together with:

- `docs/v2.5/V25_12E_AGRICULTURAL_ROUTE_PRODUCTION.md`
- `docs/architecture/agricultural_route_production.md`
- `docs/experiments/v25_12e_r5_6_forward_candidate_audit_20260814.md`
- `profiles/platforms/mk_mini.yaml`

## 1. R5.6 handoff

Real greenhouse candidate audit:

```text
17 connector requests
0  FORWARD_PREVIEW_FREE
13 LOCAL_FORWARD_OCCUPANCY_BLOCKED
1  LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
3  LOCAL_FORWARD_MIXED_EVIDENCE
```

R6 must not treat all forward failures as equivalent

## 2. R6A — Reverse Fallback Admission

Schema:

```text
agt_reverse_fallback_admission/v1
```

Purpose:

```text
R5.6 candidate audit
        ↓
explicit evidence-policy gate
        ↓
R6B input connector IDs
```

Default decisions:

```text
LOCAL_FORWARD_OCCUPANCY_BLOCKED
→ ELIGIBLE_REVERSE_FALLBACK

LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
→ HOLD_MAP_REVIEW

LOCAL_FORWARD_MIXED_EVIDENCE
→ HOLD_MIXED_EVIDENCE
→ may enter R6B only after explicit operator approval

FORWARD_PREVIEW_FREE
→ KEEP_FORWARD
```

R6A does not generate a path and cannot promote Route READY

Current implementation:

```text
src/agt_offline_assets/agt_offline_assets/reverse_fallback_admission.py
src/agt_offline_assets/agt_offline_assets/reverse_route_io.py
src/agt_offline_assets/test/test_reverse_fallback_admission.py
```

Real greenhouse operator acceptance:

```text
connector count             17
eligible reverse fallback   13
hold map review              1
hold mixed evidence          3
keep forward                 0
```

Frozen automatically admitted IDs:

```text
connector_002
connector_003
connector_004
connector_005
connector_006
connector_008
connector_010
connector_011
connector_012
connector_014
connector_015
connector_016
connector_017
```

Held IDs:

```text
connector_001 → HOLD_MAP_REVIEW
connector_007 → HOLD_MIXED_EVIDENCE
connector_009 → HOLD_MIXED_EVIDENCE
connector_013 → HOLD_MIXED_EVIDENCE
```

R6A real-data acceptance PASS

## 3. R6B — Bounded Reverse-aware Connector Planner

Status: IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING

Current backend identity:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

Schema:

```text
agt_reverse_primitive_connector_plan/v1
```

Implementation:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

R6B consumes only connector IDs admitted by R6A

Current local primitive model:

```text
canonical MK-mini Rmin = 1.5 m
curvature primitives    = {-1/R, 0, +1/R}
motion direction        = FORWARD or REVERSE
explicit zero-distance cusp action
max cusps default       = 2
primary maneuver family = F → R → F
```

The search is bounded in the Agricultural Aisle Graph row frame:

```text
longitudinal extent = requested Turn Zone extent + small explicit padding
lateral extent      = current from/to aisle pair + small explicit padding
```

It does not search the whole map

Every motion sample is checked against:

```text
frozen Navigation Grid
+ canonical MK-mini preview navigation footprint
+ 0.05 m default preview footprint padding
```

`OCCUPIED`, `UNKNOWN`, and out-of-grid footprint evidence all fail closed

Output samples explicitly contain:

```text
x / y / z / yaw
motion_direction = FORWARD | REVERSE
curvature_per_m
segment_index
is_cusp
```

The current `z` value is only linear start/end interpolation for review; R6B is a planar route connector planner

R6B requirements remain:

1. respect canonical MK-mini `Rmin=1.5 m`
2. permit true `motion_direction=REVERSE` only inside connector segments
3. preserve forward aisle traversal semantics
4. evaluate each candidate against frozen Navigation Grid evidence
5. use canonical MK-mini preview navigation footprint
6. keep UNKNOWN fail-closed
7. never modify Navigation Map occupancy
8. remain preview evidence until the physical `base_footprint` reference is measured
9. expose direction-change / cusp poses explicitly
10. output deterministic path samples suitable for later 2D/3D review

## 4. R6B design boundary

Do not collapse R6B into R7

```text
R6B
= bounded reverse-aware connector solution for an already-known aisle pair
= finite local primitive search
= max cusp / path length / expansion budget

R7
= general search fallback when local connector families cannot solve the pair
= Smac Hybrid-A* / State Lattice class backend
```

The preferred long-term analytic backend may still be Reeds-Shepp / RS-CC, but
the current R6B implementation must never be called analytic Reeds-Shepp

Target fallback policy remains:

```text
R6B bounded reverse primitive
        ↓ fail
R7 Smac Hybrid-A* / State Lattice
```

## 5. Current next action

```text
local pytest for R6A + R6B
→ load frozen reverse_fallback_admission.yaml
→ run only the 13 admitted connector IDs
→ inspect solved / unsolved count
→ inspect path length / reverse distance / cusp count / search expansions
→ verify preview footprint OCCUPIED=0 / UNKNOWN=0 on every solved path
→ inspect whether any connector hits search budget
→ freeze R6B real-data result
→ send only unsolved connectors to R7
```
