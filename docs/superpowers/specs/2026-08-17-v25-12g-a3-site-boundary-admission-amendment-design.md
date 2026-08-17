# V25-12G-A3 Site-Boundary Reverse-Admission Amendment

Date: 2026-08-17

Branch:

```text
feat/v25-12g-maximum-feasible-coverage
```

Status:

```text
DESIGN APPROVED / FROZEN / IMPLEMENTATION NOT STARTED
```

## 1. Purpose

This amendment corrects one evidence-boundary problem discovered by the first A3 real-data run.

The original A3 design treated a locally relevant forward Dubins family that touched or crossed the hard Site Boundary as a rejection of the entire connector. Real-data evidence showed that this was too strong: a failed forward-only family does not prove that a reverse-aware local connector cannot remain strictly inside the same hard boundary.

The amendment keeps the Site Boundary fully hard and fail-closed. It changes only reverse-fallback admission semantics.

It does not change A1, A2, Turn Zones, Navigation Grid, vehicle geometry, footprint padding, or R6B search budgets.

## 2. Real-data trigger

The first read-only A3 real-data run preserved all 32 A2 service states and all 14 physical segments as locally executable service motion, with 97.3566739352054 m of locally validated unique service coverage.

All 40 A2 connector candidates were rejected:

```text
HIGH_U  LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT     29
HIGH_U  NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION     7
LOW_U   LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT       4
```

The seven R6B failures used 62, 162, 16273, or 16621 expansions and therefore did not exhaust the frozen `max_expansions = 30000` budget.

The 33 former Site-Boundary classifications had `search_expansions = 0`, so they were rejected before reverse-aware local planning was attempted.

This amendment must not assume that all 33 are recoverable. It only requires them to be reclassified using endpoint-vs-path evidence before deciding whether R6B may be attempted.

## 3. Superseded original A3 statements

This amendment supersedes the original A3 statements that broadly said:

```text
Site Boundary conflicts never enter reverse fallback.
```

The corrected rule is narrower:

```text
A connector whose START or GOAL full vehicle footprint touches or crosses
Site Boundary never enters reverse fallback.

A connector whose START and GOAL footprints are both strictly legal, but whose
locally relevant FORWARD Dubins paths all violate Site Boundary, may enter R6B.
```

All other original A3 design requirements remain frozen unless explicitly changed below.

## 4. Safety invariant remains unchanged

`SiteBoundary` remains the vehicle-permitted inner boundary.

Every accepted vehicle footprint must be strictly inside it. Touching the boundary is a conflict.

The amendment never:

```text
expands Site Boundary
uses covers() instead of strict contains semantics
shrinks vehicle footprint
reduces preview_footprint_padding_m
turns Site Boundary into a soft cost
allows R6B to leave and re-enter the boundary
```

R6B must continue validating its start pose, every primitive sample, every cusp transition, and every accepted goal-shot footprint against the hard Site Boundary.

## 5. New forward-audit evidence split

The previous broad audit status:

```text
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT
```

is replaced for A3 admission purposes by two explicit statuses:

```text
CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
```

### 5.1 CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT

This status means at least one connector endpoint is not a legal vehicle pose under the frozen A3 footprint semantics.

The audit must evaluate the full padded navigation footprint at:

```text
ConnectorRequest.start_pose
ConnectorRequest.goal_pose
```

before classifying forward path families.

If either endpoint footprint touches or crosses Site Boundary:

```text
status = CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
```

This is a hard connector-level rejection because every valid transition must begin and end at those frozen A2 poses.

It must never be sent to R6B.

### 5.2 LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT

This status is legal only when both endpoint footprints are strictly inside Site Boundary.

It means:

```text
start footprint is boundary-safe
goal footprint is boundary-safe
there is at least one locally relevant forward Dubins candidate
none of the locally relevant forward Dubins candidates is boundary-safe
```

This proves only that the locally relevant forward-only family cannot provide an accepted path under the hard boundary.

It does not prove connector-level infeasibility.

Therefore it is eligible for reverse-aware local fallback.

## 6. Forward audit classification order

The forward audit classification order is frozen as follows.

```text
1. Validate Turn Zone metadata and vehicle/profile prerequisites.
2. Validate start and goal full padded footprints against Site Boundary.
3. If either endpoint conflicts:
      CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
      stop connector audit classification.
4. Otherwise enumerate and evaluate forward Dubins candidates.
5. If any candidate is full-footprint preview-free:
      FORWARD_PREVIEW_FREE
6. Otherwise evaluate the locally relevant candidate set.
7. If local candidates exist and none is Site-Boundary-safe:
      LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
8. Otherwise classify the boundary-safe local subset using the existing
   OCCUPIED / UNKNOWN / mixed / policy-review evidence rules.
```

Boundary-conflicting forward alternatives must not contaminate occupancy or map-sufficiency statistics for the boundary-safe local subset.

## 7. R6A admission policy amendment

R6A remains the only automatic policy boundary between forward evidence and R6B.

The amended automatic decisions are:

```text
FORWARD_PREVIEW_FREE
    -> KEEP_FORWARD

LOCAL_FORWARD_OCCUPANCY_BLOCKED
    -> ELIGIBLE_REVERSE_FALLBACK

LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
    -> ELIGIBLE_REVERSE_FALLBACK

CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
    -> REJECT_HARD_CONSTRAINT

LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT
    -> HOLD_MAP_REVIEW

LOCAL_FORWARD_MIXED_EVIDENCE
    -> HOLD_MIXED_EVIDENCE
       unless explicitly operator-approved by the existing mechanism

other policy states
    -> HOLD_POLICY_REVIEW
```

The first A3 implementation continues to use:

```text
operator_approved_mixed_connector_ids = ()
```

No mixed or map-insufficient case becomes automatically admissible because of this amendment.

## 8. A3 transition-state mapping

A3 normalizes the amended R5/R6A/R6B evidence as follows.

### Endpoint hard conflict

```text
forward audit:
    CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
R6A:
    REJECT_HARD_CONSTRAINT
A3:
    status      = REJECTED
    proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
```

R6B must not be invoked.

### Forward-path boundary conflict, R6B solved

```text
forward audit:
    LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
R6A:
    ELIGIBLE_REVERSE_FALLBACK
R6B:
    REVERSE_PRIMITIVE_PREVIEW_FREE
    or FORWARD_PRIMITIVE_PREVIEW_FREE
A3:
    status      = EXECUTABLE
    proof_scope = LOCAL_MOTION_EXECUTABLE
    backend     = BOUNDED_REVERSE_PRIMITIVE_SEARCH
```

The accepted R6B path must itself remain strictly inside Site Boundary at every checked footprint.

### Forward-path boundary conflict, R6B bounded no solution

```text
forward audit:
    LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
R6A:
    ELIGIBLE_REVERSE_FALLBACK
R6B:
    NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
A3:
    status      = REJECTED
    proof_scope = BOUNDED_SEARCH_NO_SOLUTION
```

This remains bounded-search evidence only and must never be described as global kinematic infeasibility.

## 9. R6B budgets remain frozen

This amendment must not change any R6B search parameter merely to improve connectivity.

In particular, the implementation must preserve the current defaults unless a separate approved design changes them:

```text
primitive_length_m              = 0.30
collision_sample_step_m         = 0.10
state_xy_resolution_m           = 0.15
state_yaw_resolution_deg        = 15.0
goal_position_tolerance_m       = 0.18
goal_yaw_tolerance_deg          = 12.0
goal_shot_distance_m            = 3.0
max_cusps                       = 2
max_expansions                  = 30000
max_path_length_m               = 18.0
longitudinal_zone_padding_m     = 0.80
lateral_pair_padding_m          = 1.25
preview_footprint_padding_m     = 0.05
```

The seven previously observed R6B no-solution transitions remain valid bounded-search evidence unless the amended admission flow changes their upstream evidence classification. Their failure is not a reason to enlarge the search budget in this amendment.

## 10. Evidence retained in TransitionValidation

No A3 schema version change is required.

`TransitionValidation.forward_evidence` must retain enough information to distinguish at least:

```text
forward audit status
endpoint boundary result
forward-path boundary result / audit reason
```

`reverse_admission_evidence` must retain the R6A decision.

For a transition whose forward-only path family conflicts with Site Boundary but whose R6B result is executable, the record must make both facts visible:

```text
forward evidence: LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
reverse admission: ELIGIBLE_REVERSE_FALLBACK
backend: BOUNDED_REVERSE_PRIMITIVE_SEARCH
final A3 status: EXECUTABLE
```

This is not a boundary bypass. It is an alternative motion family that independently satisfies the same hard boundary.

## 11. Acceptance-harness amendment

The previous broad diagnostic concept:

```text
site_boundary_reverse_bypass_count
```

must no longer interpret every forward-path boundary conflict followed by R6B as a violation.

The acceptance harness must instead check the actual forbidden case:

```text
endpoint_boundary_conflict_reverse_admission_count == 0
```

It should additionally report diagnostic funnel counts including:

```text
endpoint_boundary_conflict_count
forward_path_boundary_conflict_count
forward_path_boundary_reverse_admitted_count
forward_path_boundary_reverse_executable_count
forward_path_boundary_reverse_bounded_no_solution_count
```

These are diagnostic counts, not frozen expected values for the real greenhouse run.

The harness must continue to enforce:

```text
all A2 service states preserved
all A2 connector candidates preserved
dead-end exact retrace semantics preserved
no forbidden route-ready / reachable-from-start / optimal semantics
A1/A2/maps/Turn Zones/Site Boundary remain unchanged
```

## 12. Real-data rerun expectations

The real-data rerun must not hard-code an expected number of executable transitions.

The previously observed 33 broad Site-Boundary failures must be redistributed by evidence into:

```text
true endpoint hard conflicts
+
endpoint-safe forward-path boundary conflicts
```

Only the second group may enter R6B.

The rerun must report separately for LOW_U and HIGH_U:

```text
endpoint hard conflicts
forward-path boundary conflicts
R6A admitted connectors
R6B executable connectors
R6B bounded no-solution connectors
```

The purpose of the rerun is to measure how much of the original `0 / 40` transition result came from premature admission semantics versus actual hard endpoint geometry or bounded local-planner failure.

## 13. TDD requirements

Implementation must follow strict RED -> GREEN evidence.

Required regression cases include at least:

```text
1. start footprint crosses Site Boundary
   -> CONNECTOR_ENDPOINT_SITE_BOUNDARY_CONFLICT
   -> R6A REJECT_HARD_CONSTRAINT
   -> R6B not invoked

2. goal footprint crosses Site Boundary
   -> same hard rejection

3. endpoints safe, all locally relevant forward paths cross Site Boundary
   -> LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT
   -> R6A ELIGIBLE_REVERSE_FALLBACK

4. path-boundary conflict + R6B accepted path
   -> A3 EXECUTABLE
   -> accepted samples all boundary-safe

5. path-boundary conflict + R6B bounded no solution
   -> A3 REJECTED / BOUNDED_SEARCH_NO_SOLUTION

6. occupancy-blocked behavior unchanged
7. map-insufficient behavior unchanged
8. mixed-evidence behavior unchanged
9. existing forward-preview-free behavior unchanged
10. A3 service-motion behavior unchanged
11. acceptance harness rejects endpoint-boundary reverse admission
```

The RED gate must prove the old broad boundary behavior fails these new contracts before production changes are made.

## 14. Scope exclusions

This amendment does not include:

```text
A4 optimizer work
R7 implementation
new Reeds-Shepp analytic planner
R6B budget tuning
Turn Zone regeneration
Site Boundary editing
Navigation Grid editing
vehicle footprint tuning
A1/A2 re-derivation
START_POSE reachability
manual operator overrides
```

Any of those requires separate evidence and, where behavior changes, a separate approved design.

## 15. Completion criteria

The amendment is complete only when all of the following are demonstrated:

```text
forward audit distinguishes endpoint vs forward-path boundary conflict
endpoint conflict never reaches R6B
endpoint-safe forward-path conflict may pass through R6A to R6B
R6B keeps strict Site Boundary checks
R6B search budgets are unchanged
A3 maps solved reverse fallback to EXECUTABLE
A3 maps bounded no-solution to BOUNDED_SEARCH_NO_SOLUTION
existing occupancy/map/mixed semantics remain unchanged
all focused and package tests pass
real-data rerun preserves all A2 records
real-data rerun reports the amended boundary-admission funnel
upstream runtime assets remain byte-unchanged
```

Only after the amended A3 real-data result is frozen may A4 Maximum Feasible Coverage optimization resume.
