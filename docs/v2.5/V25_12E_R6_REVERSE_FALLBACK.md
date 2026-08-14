# V25-12E R6 Reverse Fallback

Status: R6A REAL-DATA PASS; R6B CORE IMPLEMENTED; VEHICLE-SAFE LANE PREPARATION REQUIRED

Date: 2026-08-14

This file is the focused continuation ledger for reverse-aware agricultural
connector planning. Read it together with:

- `docs/v2.5/V25_12E_CURRENT_STATE.md`
- `docs/architecture/agricultural_route_r6_reverse_fallback.md`
- `docs/architecture/agricultural_route_connector_anchors.md`
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
eligible reverse fallback 13
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

R6B shall consume only connector IDs admitted by R6A

## 3. R6B backend boundary

Long-term policy wording remains:

```text
Reeds-Shepp / reverse-aware local connector
        ↓ fail
Smac Hybrid-A* / State Lattice search fallback
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

R7 Smac Hybrid-A* / State Lattice remains the general search fallback after a
connector has a valid vehicle-safe lane/anchor but the bounded R6B solver still
cannot solve it

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

## 5. Root cause: raw aisle endpoint is not a vehicle-safe route lane

R4 currently forms ConnectorRequest poses directly from Aisle Graph endpoint
poses

```text
raw structural centerline endpoint
≠ guaranteed vehicle-safe connector anchor
≠ guaranteed vehicle-pose-free route lane
```

The Aisle Graph endpoint and centerline are structural corridor evidence. The
corridor `safe_common` contract uses ground validity/confidence, slope, raw
obstacle clearance, and row-structural exclusion, but it does not require the
final Navigation Map cell to be `FREE`

The frozen Navigation Map additionally applies obstacle padding and
`geometry_bad`, and its `FREE` state also requires sufficient ground support

Therefore the R6B start-footprint gate remains strict. We do not relax occupancy
or footprint checks

## 6. First Safe Connector Anchor smoke

The initial anchor implementation walked inward only on the existing structural
centerline for at most 1.50 m and required 0.30 m of stable FREE preview
footprint support

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
connector_014 start anchor retreat 0.650 m
but goal anchor was not found within the bounded limit
```

Some raw endpoints were individually FREE (`connector_006` goal and
`connector_010` goal) but still failed the short stable-FREE-span requirement,
which confirms that one isolated FREE pose is not enough to define a route lane

This result invalidates the assumption that increasing `maximum_retreat_m`
alone is the correct repair

## 7. Required upstream refinement: vehicle-pose-free lane

Next route-production layer:

```text
Aisle Graph structural centerline
+ frozen Navigation Grid
+ canonical MK-mini preview footprint
        ↓
vehicle-pose-free configuration-space mask at row heading
        ↓
continuous vehicle-safe aisle lane near the structural aisle
        ↓
vehicle-safe connector anchors
        ↓
R6B
```

The structural Aisle Graph remains immutable evidence

The new lane may shift laterally within the accepted aisle when the structural
centerline is not vehicle-pose-free. The shift must stay bounded by aisle
geometry and Navigation Grid evidence; it must never force OCCUPIED/UNKNOWN to
FREE

Later Route assembly must use the vehicle-safe lane and trim aisle traversal to
its connector anchors. It must not drive through the unsafe raw structural tail

## 8. R6B / R7 / R8 boundary

```text
R6B
= vehicle-safe-lane / anchor adjusted bounded reverse-aware connector solution

R7
= general search fallback for connectors that remain unsolved after valid vehicle-safe lane and anchors

R8
= formal vehicle footprint / kinematic feasibility and Route READY gate
```

A connector that has no valid vehicle-safe lane/anchor is held for upstream
route/map review. It is not handed to R7 unchanged

R6B remains preview-only until the real vehicle `base_footprint` reference and
final mounted envelope are physically measured

## 9. Current next action

```text
local pytest vehicle-safe-lane + R6 contracts
→ derive vehicle_safe_aisles.yaml on the real greenhouse
→ inspect READY/PARTIAL/UNAVAILABLE aisle counts
→ inspect coverage fraction and LOW_U/HIGH_U retreat
→ inspect whether lateral shift can recover configuration-space FREE lanes
→ preserve connector_015 as solved regression baseline
→ only valid-lane / valid-anchor unsolved connectors may move to R7
```
