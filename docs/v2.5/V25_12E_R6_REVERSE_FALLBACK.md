# V25-12E R6 Reverse Fallback

Status: R6A REAL-DATA PASS; R6B CORE IMPLEMENTED; SAFE CONNECTOR ANCHOR RETEST PENDING

Date: 2026-08-14

This file is the focused continuation ledger for reverse-aware agricultural
connector planning. Read it together with:

- `docs/v2.5/V25_12E_CURRENT_STATE.md`
- `docs/architecture/agricultural_route_r6_reverse_fallback.md`
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

Real greenhouse acceptance:

```text
connector count             17
eligible reverse fallback   13
hold map review              1
hold mixed evidence          3
keep forward                 0
```

Automatically admitted IDs:

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

## 3. R6B backend boundary

Long-term policy wording remains:

```text
Reeds-Shepp / reverse-aware local connector
        ↓ fail
Smac Hybrid-A* / State Lattice
```

Current implemented backend is deliberately named:

```text
BOUNDED_REVERSE_PRIMITIVE_SEARCH_NOT_ANALYTIC_REEDS_SHEPP
```

It is not analytic Reeds-Shepp. It is a bounded local primitive search over one
already-known aisle pair

Current model:

```text
canonical MK-mini Rmin = 1.5 m
curvature primitives    = {-1/R, 0, +1/R}
motion direction        = FORWARD or REVERSE
explicit cusp action
max cusps default       = 2
primary maneuver family = F → R → F
```

Every motion sample is checked against the frozen Navigation Grid using the
canonical MK-mini preview navigation footprint. `OCCUPIED`, `UNKNOWN`, and
out-of-grid evidence fail closed

## 4. First real R6B smoke

Input:

```text
13 R6A-admitted connectors
```

Observed:

```text
1  REVERSE_PRIMITIVE_PREVIEW_FREE
11 R6B_START_FOOTPRINT_NOT_FREE
1  NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
```

Solved baseline:

```text
connector_015
motion             FORWARD → REVERSE → FORWARD
cusps              2
path length        4.499 m
forward distance   3.899 m
reverse distance   0.600 m
search expansions  1490
goal error         0.154 m / 8.11 deg
footprint          FREE=1.000 OCCUPIED=0 UNKNOWN=0 coverage=1.000
```

Unsolved-after-search baseline:

```text
connector_017
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
search expansions 67
```

The other eleven connectors did not enter search at all because the preview
footprint was not FREE at the raw ConnectorRequest start pose

## 5. Root cause: raw aisle endpoint is not a vehicle-safe anchor

R4 currently forms ConnectorRequest poses directly from Aisle Graph endpoint
poses

```text
Aisle Graph raw endpoint
≠ guaranteed vehicle-safe connector anchor
```

The Aisle Graph endpoint is structural centerline evidence. It was never a
contract that placing the full 0.84 x 0.60 m MK-mini preview footprint exactly
at the extreme centerline cell remains FREE

Therefore the R6B start-footprint gate is kept strict. We do not relax occupancy
or footprint checks

## 6. Safe Connector Anchor Preparation

New schema:

```text
agt_connector_anchor_plan/v1
```

Implementation:

```text
src/agt_offline_assets/agt_offline_assets/connector_anchors.py
src/agt_offline_assets/test/test_connector_anchors.py
```

Policy:

```text
raw start/goal endpoint
↓
walk inward on that same aisle centerline
↓
find nearest pose where MK-mini preview footprint is stably FREE
↓
freeze retreat distance + adjusted pose
↓
adjust ConnectorRequest only
```

The default stable-free check also verifies a short inward span rather than one
isolated FREE pose

Important boundary:

```text
Aisle Graph is immutable evidence
Navigation Map occupancy is immutable evidence
Connector Anchor is a derived route-production asset
```

Later Route assembly must trim each aisle traversal to the selected safe anchor;
it must not drive through an unsafe raw endpoint and then start the connector

## 7. R6B / R7 / R8 boundary

```text
R6B
= safe-anchor-adjusted bounded reverse-aware connector solution

R7
= general search fallback for connectors that remain unsolved after valid safe anchors

R8
= formal vehicle footprint / kinematic feasibility and Route READY gate
```

A connector that has no stable FREE anchor within the bounded inward-retreat
limit is held for anchor/map review. It is not handed to R7 unchanged

R6B remains preview-only until the real vehicle `base_footprint` reference and
final mounted envelope are physically measured

## 8. Current next action

```text
local pytest connector-anchor + R6 contracts
→ derive connector_anchors.yaml for the 13 admitted connectors
→ inspect start/goal retreat distances
→ apply only READY anchors to ConnectorRequests
→ rerun R6B
→ preserve connector_015 as a solved regression baseline
→ only valid-anchor unsolved connectors may move to R7
```
