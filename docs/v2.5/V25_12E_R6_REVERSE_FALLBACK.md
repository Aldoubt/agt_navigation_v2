# V25-12E R6 Reverse Fallback

Status: R6A IMPLEMENTED CORE / LOCAL ACCEPTANCE PENDING; R6B NOT STARTED

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
src/agt_offline_assets/test/test_reverse_fallback_admission.py
```

Frozen real greenhouse expectation before optional mixed approval:

```text
eligible reverse fallback 13
hold map review            1
hold mixed evidence        3
keep forward               0
```

## 3. R6B — Reverse-aware Connector Planner

Status: NOT STARTED

R6B shall consume only connector IDs admitted by R6A

Target backend policy remains:

```text
Reeds-Shepp / reverse-aware local connector
        ↓ fail
Smac Hybrid-A* / State Lattice search fallback
```

R6B requirements:

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

R7
= search fallback when analytic/local connector families cannot solve the pair
```

The preferred R6B implementation is a Reeds-Shepp-compatible backend or an
explicitly named reverse primitive backend with equivalent forward/reverse
curvature constraints. It must not be mislabeled as analytic Reeds-Shepp if the
implementation is only a primitive search

## 5. Current next action

```text
local pytest for R5.6 + R6A
→ generate reverse_fallback_admission.yaml from frozen forward_connector_candidate_audit.yaml
→ confirm real count 13 / 1 / 3 / 0
→ freeze R6A PASS
→ implement R6B reverse-aware connector backend only for admitted IDs
```
