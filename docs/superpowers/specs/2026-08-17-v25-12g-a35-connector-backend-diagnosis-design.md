# V25-12G-A3.5 Connector Backend Diagnosis Design

Date: 2026-08-17

Status: DESIGN APPROVED / FROZEN / IMPLEMENTATION NOT STARTED

## 1. Purpose

A3.5 exists to recover a minimally useful inter-segment motion subgraph before A4 Maximum Feasible Coverage optimization begins.

A3 is already accepted as local motion evidence, but the real greenhouse run currently has:

```text
32 / 32 service actions executable
40 A2 connector candidates
40 A3 transition validations
0 executable transitions
40 rejected transitions
```

The Site-Boundary admission amendment resolved the previous semantic ambiguity: all 33 former broad boundary conflicts have legal start/goal footprints, are correctly classified as `LOCAL_FORWARD_PATH_SITE_BOUNDARY_CONFLICT`, and are admitted to R6B. None are recovered by the current backend. The other seven HIGH_U connectors are `LOCAL_FORWARD_OCCUPANCY_BLOCKED` and also enter R6B. All 40 end with `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`.

A3.5 therefore targets the connector backend itself, not admission policy, map derivation, service-segment extraction, or the A4 optimizer.

## 2. Mandatory A4 Entry Gate

A4 must not begin until the real greenhouse map contains one continuous service chain satisfying all of the following:

```text
>= 3 distinct vehicle-feasible segment_ids
>= 2 distinct aisle_ids
>= 2 EXECUTABLE inter-segment transitions
```

The chain must have the form:

```text
segment A
  -> executable transition 1
segment B
  -> executable transition 2
segment C
```

The three service segments must span at least two different aisles. Multiple segments cut from only one aisle do not satisfy this gate.

The chain is a minimum engineering validation gate, not a claim of optimality or field readiness.

## 3. Hard Constraints That Remain Frozen

A3.5 must not obtain connectivity by weakening the vehicle or map safety contract.

The following remain unchanged unless a later evidence-backed design amendment is explicitly approved:

```text
canonical platform                mk_mini
kinematics                        Ackermann
verified minimum turning radius   1.5 m
canonical navigation footprint    unchanged
preview footprint padding         0.05 m
Site Boundary semantics           hard vehicle-permitted inner boundary
Navigation Grid policy            FREE-only preview footprint gate
NO_GO / occupied semantics        unchanged
max_cusps                         2
max_expansions                    30000
max_path_length_m                 18.0 m
```

The current real-data maximum observed search expansion count is 16621, so there is no present evidence that `max_expansions=30000` is the limiting factor.

A3.5 must not shrink the vehicle footprint, reduce padding, allow leave-and-reenter Site Boundary behavior, ignore occupied cells, relax Ackermann curvature, or silently alter A1/A2 assets.

## 4. Chosen Strategy

The chosen approach is:

```text
Diagnosis first
  -> failure funnel
  -> evidence-backed minimal connector-backend amendment
  -> real-map rerun
  -> minimum connected subgraph gate
```

A3.5 does not begin by tuning search parameters or replacing R6B wholesale.

The first implementation increment must be diagnostic-only: it must expose why each R6B request fails without changing the search result for the same frozen inputs and configuration.

Only after the real-data diagnostic report identifies the dominant blocker may a production behavior amendment be designed and implemented.

## 5. Current R6B Search Model

R6B is a bounded deterministic reverse-aware local Ackermann connector search. Its current search model includes:

- motion primitive length `0.30 m`;
- collision sampling step `0.10 m`;
- primitive curvatures `{-1/R, 0, +1/R}`;
- state XY resolution `0.15 m`;
- state yaw resolution `15 deg`;
- goal position tolerance `0.18 m`;
- goal yaw tolerance `12 deg`;
- forward goal-shot distance `3.0 m`;
- maximum two cusps;
- local row-frame search envelope;
- strict Site-Boundary footprint checking;
- FREE-only Navigation Grid footprint checking;
- bounded path length and expansion count;
- forward-only analytic Dubins goal shot from a FORWARD search state.

This is intentionally not labelled analytic Reeds-Shepp and must not be treated as a proof of global connector infeasibility.

## 6. Diagnostic Architecture

The R6B diagnostic layer must preserve the existing planner decision while exposing the search funnel for each connector.

For each connector, record at least:

```text
connector_candidate_id
side
turn_zone_id
forward_audit_status
R6A admission decision
search_expansions
queue_exhausted
expansion_budget_reached
best_goal_position_error_m
best_goal_yaw_error_rad
best_goal_distance_state_direction
best_goal_distance_cusp_count
```

The search-generation funnel must include counters for at least:

```text
nodes_popped
stale_nodes_skipped
cusp_switches_considered
cusp_switches_enqueued
primitive_edges_considered
primitive_edges_rejected_search_envelope
primitive_edges_rejected_site_boundary
primitive_edges_rejected_navigation_grid
primitive_edges_rejected_path_length
primitive_edges_rejected_state_dominance
primitive_edges_enqueued
goal_tolerance_checks
goal_tolerance_successes
goal_shot_attempts
goal_shot_candidates_considered
goal_shot_rejected_search_envelope
goal_shot_rejected_site_boundary
goal_shot_rejected_navigation_grid
goal_shot_successes
```

If one collision check currently returns only a Boolean, the diagnostic implementation may add an internal reason-returning helper, but the original Boolean behavior must remain equivalent.

The report must also distinguish whether the queue exhausted naturally before `max_expansions`.

## 7. Failure Classification

A3.5 diagnostic classification is evidence-oriented, not an infeasibility proof.

Each failed connector must receive one primary diagnostic class from:

```text
SEARCH_ENVELOPE_LIMITED
SITE_BOUNDARY_LIMITED
FOOTPRINT_GRID_LIMITED
STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED
GOAL_CONNECTION_LIMITED
MOTION_PRIMITIVE_LIMITED
PATH_OR_CUSP_ENVELOPE_LIMITED
MIXED_LIMITATION
INCONCLUSIVE
```

Classification must be computed from explicit counters/thresholds frozen in the implementation plan and tested with synthetic fixtures. It must not be based on arbitrary connector IDs or hard-coded real-map counts.

Examples of intended interpretation:

- many states reach the goal neighborhood but no goal connection is accepted -> `GOAL_CONNECTION_LIMITED`;
- most generated edges die at the row-frame bounds -> `SEARCH_ENVELOPE_LIMITED`;
- most candidates die on hard Site Boundary -> `SITE_BOUNDARY_LIMITED`;
- most candidates die on FREE-only footprint checks -> `FOOTPRINT_GRID_LIMITED`;
- a large generated set collapses through coarse state dominance while near-goal progress stalls -> `STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED`;
- search remains far from the goal even though collision/envelope rejection is modest -> candidate for `MOTION_PRIMITIVE_LIMITED` or `INCONCLUSIVE`.

These classes are diagnostic evidence only and must not promote a failed connector to executable.

## 8. Real-Data Diagnostic Acceptance

The first A3.5 real-data run must cover all 40 currently admitted R6B transitions and report:

- classification count by LOW_U / HIGH_U;
- classification count by original forward audit status;
- per-connector search funnel;
- expansion histogram;
- minimum observed goal position/yaw error;
- queue exhaustion versus expansion-budget termination;
- top rejection causes;
- candidate connectors that came closest to satisfying the target pose.

This diagnostic run must preserve exactly the same final transition statuses as the pre-A3.5 A3 motion graph for the same inputs.

Therefore the initial diagnostic-only acceptance invariant is:

```text
A3.5 diagnostic instrumentation changes evidence only, not executable/rejected/unresolved outcomes.
```

## 9. Evidence-Driven Backend Amendment

After the diagnostic run, only the dominant evidenced blocker should be changed in the next design amendment.

Allowed amendment families include:

1. goal connection behavior, if near-goal states exist but current forward-only goal-shot prevents completion;
2. motion primitive family or steering discretization, if the reachable search lattice is demonstrably insufficient;
3. state discretization/dominance semantics, if meaningful continuous states are being incorrectly collapsed;
4. local search-envelope formulation, if the envelope rather than the hard Site Boundary prevents valid headland motion;
5. a second deterministic reverse-aware connector backend, such as a separately validated analytic Reeds-Shepp candidate generator, if the current primitive backend is structurally inadequate.

A full Hybrid-A* local planner is not the default A3.5 action. It requires a separate design decision because it materially expands project scope.

No backend amendment may bypass the frozen hard footprint, Navigation Grid, Site Boundary, NO_GO, or Ackermann constraints.

## 10. Connectivity Acceptance After Amendment

After an evidence-backed backend change, the real greenhouse acceptance must derive the A3 motion graph again and search the executable service/transition graph for the minimum A4 entry chain.

A3.5 passes only if there exists a chain with:

```text
3 distinct segment_ids
2 or more distinct aisle_ids
2 or more executable inter-segment transitions
```

Every transition in that chain must retain full A3 local motion evidence, including backend, path length, forward/reverse distance, cusp count, samples, and hard-constraint validation.

A stronger non-gating quality preference is that at least one transition represents a normal cross-aisle headland maneuver rather than achieving connectivity only through an unusually long reverse maneuver.

## 11. If the Connectivity Gate Still Fails

A3.5 failure after one evidence-backed backend amendment does not authorize silent constraint relaxation.

The next decision must be explicit and based on the new evidence:

- add a second deterministic connector backend;
- change the local connector architecture through a new approved spec;
- or formally conclude that the current real-map/headland/vehicle geometry provides no demonstrated three-segment/two-aisle local connected subgraph under the frozen constraints.

A4 remains blocked until either the mandatory connectivity gate passes or the user explicitly approves changing the A4 research question to operate on disconnected components.

## 12. Testing Strategy

Implementation follows strict TDD.

The diagnostic-only stage must include RED/GREEN tests proving:

- all funnel counters are deterministic;
- Site Boundary, grid and envelope rejection reasons are separated correctly;
- queue exhaustion is distinguished from max-expansion termination;
- best-goal-distance evidence is updated correctly;
- instrumentation leaves planner outputs unchanged;
- serializer/loader preserves the new diagnostic evidence if persisted;
- no route-ready, reachable-from-start, or optimality semantics are introduced.

The later backend-amendment stage gets a new RED/GREEN cycle based on the diagnosed blocker. Production behavior must not be changed in the same commit that first introduces diagnostic instrumentation.

Real-data operator gates should remain batched to minimize repeated manual test interruptions.

## 13. Scope Exclusions

A3.5 does not include:

- A4 Maximum Feasible Coverage optimization;
- START_POSE anchoring or global reachability;
- Nav2 controller execution;
- RPP field tracking;
- map regeneration;
- A1 segment re-extraction;
- A2 topology regeneration;
- vehicle geometry tuning;
- Site Boundary editing;
- footprint/padding relaxation;
- blind R6B budget increases;
- RL, MILP, VLA, or human-in-the-loop planning.

## 14. Completion Criteria

A3.5 is complete only when:

1. diagnostic instrumentation has been verified not to change existing R6B outcomes;
2. all 40 real connectors have a frozen diagnostic failure classification;
3. one evidence-backed backend amendment has been implemented and tested if required;
4. the real greenhouse A3 graph contains the mandatory 3-segment / >=2-aisle / >=2-transition continuous service chain;
5. all hard safety invariants and upstream input hashes remain unchanged;
6. the resulting A3 real-data state is documented before A4 begins.
