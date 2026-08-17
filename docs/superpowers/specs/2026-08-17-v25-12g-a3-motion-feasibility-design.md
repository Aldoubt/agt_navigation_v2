# V25-12G-A3 Vehicle-Feasible Motion Graph Design

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

V25-12G-A3 converts the static topology candidates produced by A2 into vehicle-motion evidence that can be consumed directly by A4 without reinterpreting R5, R6A, R6B, A1 centerline semantics, or local collision evidence.

A3 answers:

> Which directional service actions and headland transition candidates are locally executable under the frozen vehicle profile, Navigation Grid, Site Boundary, and bounded connector planners?

A3 does not answer start reachability, route ordering, final coverage, or global optimality.

```text
A1 Vehicle-Feasible Segments
        |
        v
A2 Static Service Topology Candidates
        |
        v
A3 Vehicle-Feasible Motion Graph
        |
        v
A4 START_POSE-aware Maximum Feasible Coverage Search
```

## 2. Frozen output contract

A3 produces a new sibling asset and never upgrades or rewrites the A2 graph in place.

```text
filename: vehicle_feasible_motion_graph.yaml
schema:   agt_vehicle_feasible_motion_graph/v1
status:   MOTION_EVIDENCE_ONLY
```

A2 remains immutable.

A3 must not modify:

```text
vehicle_feasible_segments.yaml
vehicle_feasible_service_graph.yaml
turn_zones.yaml
navigation_map.yaml
navigation_map.pgm
site_boundary.yaml
aisle_graph.yaml
```

A3 also must not change existing R5, R6A, or R6B planning budgets merely to improve graph connectivity.

## 3. Inputs and identity tolerances

The primary A3 derivation consumes:

```text
VehicleFeasibleServiceGraph
TurnZoneSet
NavigationGridEvidence
CanonicalVehicleProfile
optional SiteBoundary
```

A3 normally does not re-read the A1 runtime asset because A2 already embeds the physical service resource geometry copied from A1.

The implementation must fail closed unless:

```text
A2.frame_id == NavigationGrid.frame_id
A2.frame_id == TurnZoneSet.frame_id
A2.frame_id == SiteBoundary.frame_id when supplied
A2.platform_id == vehicle.profile_id
A2.platform_profile_sha256 == vehicle.profile_sha256
row_direction_xy is finite, non-zero, and orientation-consistent
```

Pose identity checks in the A3 contract use the following frozen tolerances:

```text
position tolerance: 1.0e-6 m per x/y/z component
wrapped yaw tolerance: 1.0e-6 rad
```

These tolerances are for verifying asset identity and cross references only. They are not planning goal tolerances and must not be substituted for R5/R6B planner tolerances.

## 4. Architecture

A3 has two validation domains:

```text
Service Motion Validation
    ordinary LOW -> HIGH forward service
    ordinary HIGH -> LOW forward service
    dead-end forward-in + exact reverse retrace

Connector Motion Validation
    A2 ConnectorCandidate adapter
    existing forward connector chain
    full-footprint Navigation Grid / Site Boundary gate
    existing forward evidence audit
    R6A reverse-fallback admission
    R6B bounded reverse-aware local connector search when admitted
```

A3 is an orchestration and evidence-normalization layer. It must not duplicate the existing forward or reverse connector planners.

## 5. A2 ConnectorCandidate adapter

Every A2 connector candidate maps one-to-one to the existing `ConnectorRequest` contract.

```text
ConnectorRequest.connector_id  = ConnectorCandidate.connector_candidate_id
from_aisle_id                  = source ServiceState.aisle_id
to_aisle_id                    = target ServiceState.aisle_id
turn_zone_id                   = ConnectorCandidate.turn_zone_id
side                           = ConnectorCandidate.side
start_pose                     = ConnectorCandidate.start_pose
goal_pose                      = ConnectorCandidate.goal_pose
```

A3 preserves the 12G identity outside the legacy connector interface:

```text
connector_candidate_id
from_service_state_id
to_service_state_id
from_segment_id
to_segment_id
```

Before planner invocation the adapter must verify, using the frozen identity tolerances where poses are involved:

```text
candidate.start_pose == source ServiceState.exit_pose
candidate.goal_pose  == target ServiceState.entry_pose
candidate.side matches referenced Turn Zone side
referenced service states exist
referenced service resources exist
```

Any mismatch is an input-contract error and raises `ValueError`.

## 6. Connector validation state machine

A3 must reuse the current planner evidence chain instead of implementing a new policy.

```text
A2 topology candidate
        |
        v
ConnectorRequest
        |
        v
forward candidate generation / Navigation gate
        |
        v
existing forward candidate audit
        |
        v
R6A admission
        |
        +--> KEEP_FORWARD ----------------------> EXECUTABLE
        |
        +--> ELIGIBLE_REVERSE_FALLBACK
        |             |
        |             v
        |            R6B
        |             |
        |             +--> solved -------------> EXECUTABLE
        |             |
        |             +--> bounded no solution -> REJECTED
        |
        +--> HOLD_MAP_REVIEW -------------------> UNRESOLVED
        +--> HOLD_MIXED_EVIDENCE ---------------> UNRESOLVED
        +--> HOLD_POLICY_REVIEW ----------------> UNRESOLVED
```

A3 must not treat a centerline-only R5 success as executable vehicle evidence.

A forward transition becomes executable only when the existing full-footprint Navigation Grid / Site Boundary validation path supplies accepted preview evidence.

## 7. Reverse fallback policy

A3 preserves the current R6A conservative admission boundary.

Automatic reverse admission is permitted only when existing forward evidence classifies the transition as sufficiently mapped but occupancy-blocked.

Map-insufficient evidence must not be hidden by reverse planning.

The first A3 implementation uses:

```text
operator_approved_mixed_connector_ids = ()
```

Therefore mixed-evidence candidates remain `UNRESOLVED` in the automatic A3 asset.

Site Boundary conflicts never enter reverse fallback.

A3 must not automatically increase:

```text
R6B max_expansions
R6B max_path_length_m
R6B max_cusps
or other frozen search budgets
```

## 8. Unified result classes

Every legal A3 validation record uses one of:

```text
EXECUTABLE
REJECTED
UNRESOLVED
```

`EXECUTABLE` means the local action has accepted vehicle-motion evidence under the frozen preview assumptions.

`REJECTED` means a valid candidate encountered a hard conflict or a bounded planner failed to find an accepted motion under its frozen search budget.

`UNRESOLVED` means evidence is insufficient or policy intentionally withholds automatic promotion.

The following distinction is mandatory:

```text
lack of proof of feasibility != proof of infeasibility
```

Each record includes `proof_scope`. Frozen initial proof-scope values are:

```text
LOCAL_MOTION_EXECUTABLE
PROVEN_HARD_CONSTRAINT_REJECTION
BOUNDED_SEARCH_NO_SOLUTION
MAP_EVIDENCE_INSUFFICIENT
MIXED_EVIDENCE_REQUIRES_REVIEW
POLICY_REVIEW_REQUIRED
```

A bounded-search failure must never be described as global kinematic infeasibility.

## 9. Directional service validation

A3 validates every A2 `ServiceState` independently and does not re-plan a service centerline.

It consumes the A2 `ServiceResource.centerline_xyz`, assigns the actual directional heading implied by the service state, builds ordered motion samples, and revalidates the full preview vehicle footprint.

This is necessary because the vehicle reference point and footprint need not be symmetric under a yaw change of pi. A1 local feasibility must not be blindly promoted to both A2 travel directions.

## 10. Ordinary LOW -> HIGH service

For `SERVICE_LOW_TO_HIGH`, A3 uses the resource centerline in stored order:

```text
P0, P1, ..., PN
```

Every sample uses:

```text
position         = resource.centerline_xyz[i]
yaw              = canonical row-direction yaw
motion_direction = FORWARD
is_cusp          = false
```

The first and last sample poses must agree with the A2 state entry and exit poses under the frozen A3 identity tolerances. A mismatch is an input-contract error, not a local motion rejection.

## 11. Ordinary HIGH -> LOW service

For `SERVICE_HIGH_TO_LOW`, A3 reverses the position order:

```text
PN, ..., P1, P0
```

The vehicle still drives forward:

```text
motion_direction = FORWARD
yaw              = row yaw + pi, normalized
```

Driving against the graph row direction is not reverse gear.

The first and last sample poses again must agree with the A2 state entry and exit poses under the frozen identity tolerances.

## 12. Dead-end forward-in / exact reverse-out

The user-approved A3 policy is exact retrace.

`DEAD_END_FORWARD_IN_REVERSE_OUT` must never invoke R6B.

For a LOW_U headland service, define the forward sample sequence as:

```text
F = [P0, P1, ..., PN]
```

The serialized dead-end motion sequence is frozen as:

```text
P0, P1, ..., PN                      FORWARD
PN                                   REVERSE, is_cusp=true, zero-distance marker
P(N-1), ..., P1, P0                  REVERSE
```

The cusp marker is a second sample at the same pose as the final forward sample. It changes motion direction, adds zero path length, increments `cusp_count` by exactly one, and starts a new `segment_index`.

When checking exact retrace, the zero-distance cusp marker is excluded from the geometry comparison. The reverse travel positions after the marker must equal `reversed(F[:-1])` exactly after deterministic float serialization.

For a HIGH_U headland, the forward-in sequence is first oriented HIGH_U -> LOW_U and then the same rule is applied.

The reverse-out segment uses the same vehicle yaw as the forward-in segment. Only `motion_direction` changes to `REVERSE`.

A reverse-out yaw rotation by pi is forbidden because that would represent turning around and driving forward rather than backing out.

## 13. MotionSample contract

A3 uses one ordered sample representation for service actions and connector transitions:

```text
MotionSample
    x
    y
    z
    yaw
    motion_direction   # FORWARD | REVERSE
    segment_index
    is_cusp
```

`segment_index` starts at zero and increments only at a motion-direction cusp. Ordinary services have only segment zero. Exact dead-end retrace has segment zero for forward-in and segment one beginning at the cusp marker.

Connector-specific curvature may remain available as backend evidence, but it is not mandatory on the unified A3 sample contract because A1-derived service geometry is not generated by R6B motion primitives.

## 14. Coverage reward vs travel metrics

A3 must keep coverage reward separate from actual travel distance.

The A1/A2 coverage reward is the longitudinal useful segment length. The A3 path length is the actual polyline arc length:

```text
L_path = sum(norm(P[i] - P[i-1]))
```

For an ordinary service action:

```text
coverage_reward_length_m = A2 coverage reward
path_length_m            = L_path
forward_distance_m       = L_path
reverse_distance_m       = 0
cusp_count               = 0
```

For a dead-end action:

```text
coverage_reward_length_m = A2 coverage reward, counted once
path_length_m            = 2 * L_path
forward_distance_m       = L_path
reverse_distance_m       = L_path
cusp_count               = 1
```

The zero-distance cusp marker contributes no travel distance.

## 15. Service footprint validation

Service samples use the same canonical navigation-footprint and padding semantics as connector validation. A3 must not create an inconsistent second vehicle geometry model.

A shared footprint-validation primitive may be extracted where useful, provided regression tests prove existing connector behavior remains unchanged.

Service result classification is:

```text
all sampled preview footprints FREE
and strictly inside Site Boundary when supplied
    -> EXECUTABLE

OCCUPIED conflict
or Site Boundary touch/cross
    -> REJECTED

UNKNOWN / insufficient grid evidence / out-of-grid evidence
    -> UNRESOLVED
```

Site Boundary remains a hard invariant and can never be bypassed by reverse fallback.

## 16. ServiceActionValidation contract

Each A2 service state produces exactly one record:

```text
ServiceActionValidation
    service_state_id
    segment_id
    aisle_id
    service_type
    entry_pose
    exit_pose

    status
    proof_scope
    backend
    backend_status
    reason

    coverage_segment_id
    coverage_reward_length_m

    path_length_m
    forward_distance_m
    reverse_distance_m
    cusp_count

    samples[]
    footprint_evidence
```

Frozen service backends are:

```text
A1_CENTERLINE_DIRECTIONAL_REVALIDATION
A1_CENTERLINE_EXACT_REVERSE_RETRACE
```

A3 must not claim that a dead-end service used Reeds-Shepp or R6B.

## 17. TransitionValidation contract

Each A2 connector candidate produces exactly one record, successful or not:

```text
TransitionValidation
    connector_candidate_id
    from_service_state_id
    to_service_state_id
    from_segment_id
    to_segment_id
    side
    turn_zone_id
    start_pose
    goal_pose

    status
    proof_scope
    backend
    backend_status

    path_length_m
    forward_distance_m
    reverse_distance_m
    cusp_count
    search_expansions

    samples[]
    forward_evidence
    reverse_admission_evidence
    reason
```

A3 must retain rejected and unresolved records for auditability. It must not emit only successful edges.

## 18. VehicleFeasibleMotionGraph contract

The complete top-level object is:

```text
VehicleFeasibleMotionGraph
    schema
    status
    frame_id
    platform_id
    platform_profile_sha256
    row_direction_xy
    source

    service_actions[]
    transition_validations[]

    executable_service_action_ids[]
    executable_transition_ids[]

    diagnostics
```

The motion graph does not duplicate the full A2 `ServiceResource` array. A4 is expected to consume the A3 graph directly rather than reinterpreting A1 or connector planner internals.

## 19. Diagnostics

A3 diagnostics include at least:

```text
a2_service_state_count
service_validation_count
executable_service_action_count
rejected_service_action_count
unresolved_service_action_count
ordinary_service_action_count
dead_end_service_action_count
executable_dead_end_service_action_count

a2_connector_candidate_count
transition_validation_count
executable_transition_count
forward_executable_transition_count
reverse_executable_transition_count
rejected_transition_count
unresolved_transition_count

locally_validated_segment_count
locally_validated_unique_coverage_length_m
distinct_locally_validated_aisle_count
```

A physical segment contributes locally validated coverage once when at least one service action for that `coverage_segment_id` is executable.

A3 uses `locally_validated` terminology rather than `reachable`, `achievable`, `route`, or `covered`, because START_POSE reachability is not yet known.

## 20. Determinism and serialization

A3 must be deterministic under semantically identical inputs.

Stable ordering is frozen as:

```text
service_actions                by service_state_id
transition_validations         by connector_candidate_id
executable_service_action_ids  lexical
executable_transition_ids      lexical
```

The serialized source metadata excludes non-deterministic runtime values such as wall-clock timestamps, hostnames, random UUIDs, and temporary paths.

The strict loader validates:

```text
schema
status
finite numeric fields
enums
unique IDs
cross references
sample structure
summary/count consistency
```

It must additionally verify:

```text
executable_service_action_ids are unique and reference only EXECUTABLE service records
executable_transition_ids are unique and reference only EXECUTABLE transition records
all service records reference a known A3 segment identity carried by that record set
all transition endpoints reference known A3 service records
diagnostics equal recomputed counts and deduplicated coverage totals
```

The round trip:

```text
load -> write
```

must reproduce byte-identical YAML.

## 21. Fail-closed input-contract behavior

The following conditions abort the A3 derivation with explicit validation errors:

```text
missing referenced service state
missing referenced service resource
duplicate IDs
non-finite poses
invalid centerline
entry or exit pose inconsistent with service resource
connector start pose inconsistent with source state exit
connector goal pose inconsistent with target state entry
Turn Zone side mismatch
frame mismatch
platform mismatch
vehicle profile hash mismatch
invalid enum or status
```

These are data-contract failures, not `REJECTED` motion results.

## 22. Safety invariants

A3 inherits the V25-12F safety invariants without weakening them.

The Site Boundary remains a hard vehicle-permitted inner perimeter:

```text
strictly inside -> continue
touch boundary  -> hard conflict
cross boundary  -> hard conflict
```

Navigation evidence remains three-valued:

```text
FREE
OCCUPIED
UNKNOWN
```

A3 must never silently convert UNKNOWN into FREE. UNKNOWN and insufficient-grid evidence remain unresolved evidence rather than proven obstacle truth.

## 23. Real-greenhouse acceptance dataset

The first A3 real-data checkpoint uses:

```text
/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run
```

The frozen A2 checkpoint contains:

```text
frame_id                     = map
platform_id                  = mk_mini
A2 service states            = 32
A2 connector candidates      = 40
A2 physical resources        = 14
A2 unique coverage length_m  = 97.3566739352054
```

Therefore structural A3 gates are:

```text
service_validation_count    = 32
transition_validation_count = 40
```

A3 does not pre-assume real-data counts for executable services, executable transitions, reverse admissions, reverse solutions, rejected records, or unresolved records.

## 24. Real-data service sanity controls

`aisle_020.segment_001` is the clean through-service sanity control because it spans:

```text
LOW_U_HEADLAND -> HIGH_U_HEADLAND
```

A3 reports both directional ordinary service results. The design does not require both directions to pass; real directional asymmetry must be reported faithfully.

The current four one-headland resources expected to produce dead-end validation records are:

```text
aisle_016.segment_001
aisle_016.segment_003
aisle_017.segment_005
aisle_019.segment_003
```

For every valid dead-end record A3 verifies:

```text
reverse travel geometry equals forward travel geometry reversed
reverse_distance_m approximately equals forward_distance_m
cusp_count == 1
coverage reward is counted once
backend == A1_CENTERLINE_EXACT_REVERSE_RETRACE
```

## 25. Real-data connector funnel

The A3 report exposes:

```text
A2 topology connector candidates
        -> forward-preview executable
        -> sufficiently mapped forward blocked
        -> R6A reverse admitted
        -> R6B reverse solved
        -> final executable transitions
```

It separately counts at least:

```text
map-insufficient candidates
mixed-evidence candidates
hard Site Boundary conflicts
bounded-search no-solution results
```

## 26. A3 -> A4 boundary

A3 supplies:

```text
locally executable service actions
locally executable transition edges
actual local motion metrics
audit records for rejected candidates
audit records for unresolved candidates
```

A3 does not supply:

```text
START_POSE reachability
selected coverage segments
execution order
repeated-transit policy decisions
terminal endpoint choice
global route coverage
global optimality
```

A4 may place only A3 `EXECUTABLE` actions and transitions in the executable search graph. A3 `REJECTED` and `UNRESOLVED` records remain diagnostic and cannot be traversed by the first A4 implementation.

A4 then applies the frozen objective hierarchy:

```text
1. maximize first-time covered feasible segment length
2. maximize distinct aisle count
3. minimize total travel distance
4. minimize reverse distance
5. minimize cusp count
6. minimize route risk
```

## 27. Acceptance criteria

A3 is accepted only when:

```text
1. A2 remains immutable
2. every A2 service state has exactly one A3 validation record
3. every A2 connector candidate has exactly one A3 validation record
4. ordinary services are directionally revalidated
5. dead-end service uses exact reverse retrace only
6. dead-end service never invokes R6B
7. coverage reward and actual travel distance remain independent
8. Site Boundary remains a hard constraint
9. UNKNOWN never becomes FREE
10. mixed evidence is not auto-approved in A3 v1
11. connector validation reuses existing R5/R6A/R6B infrastructure
12. R6B search budgets remain frozen
13. bounded-search failure is not mislabeled global infeasibility
14. IDs, order, and YAML are deterministic
15. the strict loader rejects malformed cross references
16. strict load/write is byte-stable
17. A3 writes only vehicle_feasible_motion_graph.yaml as its runtime sibling
18. upstream input hashes remain byte-identical
19. A3 never emits route_ready, reachable_from_start, or optimal claims
20. top-level status remains MOTION_EVIDENCE_ONLY
```

## 28. Explicit non-goals

A3 does not include:

```text
START_POSE anchoring
global route search
coverage optimization
learned planning
manual collision override
automatic map promotion
R6B budget tuning
arbitrary interior cross-row motion
dead-end local replanning
A4 risk scalarization
```

These exclusions preserve the separation between local motion evidence and global coverage optimization.
