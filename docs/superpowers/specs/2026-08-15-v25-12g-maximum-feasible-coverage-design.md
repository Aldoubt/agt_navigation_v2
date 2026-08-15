# V25-12G Maximum Feasible Coverage Design

Date: 2026-08-15

Branch:

```text
feat/v25-12g-maximum-feasible-coverage
```

Base checkpoint:

```text
feat/v25-12f-traversability-hard-boundary
bfcf6bac77955f015c6088088f1b6de95116f608
```

Status:

```text
DESIGN FROZEN / USER-APPROVED IN DISCUSSION / WRITTEN SPEC PENDING FINAL REVIEW / IMPLEMENTATION NOT STARTED
```

## 1. Problem statement

The current agricultural route chain is organized around whole-aisle primitives and deterministic boustrophedon ordering

That model is too coarse for the real greenhouse data observed in V25-12E and V25-12F

A structural aisle may contain multiple disjoint regions in which the complete preview vehicle footprint is feasible even when the whole aisle is not continuously feasible

The current Vehicle-Safe Lane layer already samples vehicle-pose feasibility along each structural aisle, but it keeps only the single longest contiguous feasible region as the lane result

V25-12G changes the planning unit from:

```text
whole structural aisle
```

into:

```text
vehicle-feasible aisle segment
```

The long-term goal is to generate the executable route that covers as much truly feasible agricultural workspace as possible instead of requiring every aisle to be traversable end-to-end

## 2. Frozen objective hierarchy

V25-12G uses a lexicographic objective

The primary goal is not shortest travel and not maximum aisle count

The objective order is frozen as:

```text
1. maximize first-time covered feasible segment length
2. maximize number of distinct aisles with useful coverage
3. minimize total travel distance
4. minimize reverse distance
5. minimize cusp count
6. minimize route risk
```

A route with greater covered length wins even if another route touches more aisles or has lower travel cost

Coverage reward is counted only once per segment

A segment may later be reused as transit, but repeated traversal does not earn repeated coverage reward

## 3. Safety invariants inherited from V25-12F

V25-12G must not weaken V25-12F safety semantics

All executable candidates remain subject to:

```text
Site Boundary
semantic NO_GO
continuous preview vehicle footprint
Navigation Grid / selected traversability source
structural width gate
Ackermann kinematics
minimum turning radius
connector-local validation
```

The Site Boundary remains the vehicle-permitted inner perimeter

```text
strictly inside boundary -> candidate may continue to other gates
touch boundary           -> SITE_BOUNDARY_CONFLICT
cross boundary           -> SITE_BOUNDARY_CONFLICT
```

No manual path editing, automatic optimizer, reverse fallback, future Hybrid A*, or future learned planner may override this invariant

The V25-12F canonical map non-promotion decision also remains unchanged

V25-12G must not silently promote `navigation_map_12f.*` to the canonical Navigation Map

## 4. Architecture

The target architecture is:

```text
Agricultural Aisle Graph
        |
        v
Vehicle footprint feasibility sampling
        |
        v
Vehicle-Feasible Aisle Segments
        |
        v
Segment service states
        |
        v
Headland connector feasibility
        |
        +--> Forward connector
        |
        +--> Reverse fallback connector
        |
        v
Vehicle-Feasible Segment Graph
        |
        v
Maximum Feasible Coverage optimizer
        |
        v
Route candidate
        |
        v
Independent route validation
        |
        v
Review / export
```

V25-12G must reuse existing R5 / R6A / R6B connector capabilities rather than introducing a parallel local motion planner in the first implementation

## 5. Phase decomposition

V25-12G is intentionally split into four implementation phases

```text
12G-A1  Vehicle-Feasible Segment Extraction
12G-A2  Segment Service States and Reachability Graph
12G-A3  Forward / Reverse Connector Feasibility Integration
12G-A4  Maximum Feasible Coverage Search
```

The first implementation plan must cover only 12G-A1

A2-A4 are architectural consumers of the A1 contract and must not be pulled into A1 implementation merely for convenience

## 6. V25-12G-A1: Vehicle-Feasible Segment Extraction

### 6.1 Purpose

A1 answers one question only:

> When the full preview vehicle footprint is evaluated along every structural aisle, what are all deterministic contiguous feasible longitudinal segments worth exposing to later coverage planning?

A1 does not decide route order and does not generate inter-aisle connectors

### 6.2 Inputs

A1 consumes the same truth sources already used by Vehicle-Safe Lane evaluation:

```text
AgriculturalAisleGraph
NavigationGridEvidence
CanonicalVehicleProfile
VehicleSafeLaneConfig
optional SiteBoundary
```

The existing structural aisle graph remains immutable

The existing Navigation Grid remains immutable

The existing V25-12F Site Boundary remains immutable

### 6.3 Compatibility rule

The public V25-12E / V25-12F `VehicleSafeLanePlan` contract must remain backward compatible

A1 must introduce an independent output contract instead of changing the existing lane plan into a multi-segment schema

Target relationship:

```text
existing stable API
VehicleSafeLanePlan
        |
        +--> unchanged public behavior

new 12G API
VehicleFeasibleSegmentPlan
        |
        +--> all qualifying contiguous feasible segments
```

Implementation may share a private pose-feasibility sampling primitive between the two layers to avoid duplicated safety logic

That shared primitive must not change the serialized behavior of `VehicleSafeLanePlan`

## 7. A1 data contract

The new schema name is frozen as:

```text
agt_vehicle_feasible_segment_plan/v1
```

The core structures are conceptually:

```text
VehicleFeasibleSegmentPlan
    frame_id
    platform_id
    platform_profile_sha256
    row_direction_xy
    aisles
    source
    schema
    status
```

Each aisle result contains active segments plus rejected short fragments and diagnostics

A qualifying segment contains at minimum:

```text
segment_id
aisle_id
ordinal_in_aisle
start_distance_m
end_distance_m
length_m
coverage_fraction_of_aisle
low_endpoint_type
high_endpoint_type
centerline_xyz
lateral_offsets_m
maximum_used_lateral_shift_m
low_endpoint_pose
high_endpoint_pose
status
reason
```

A rejected short fragment contains at minimum:

```text
fragment_id
___id
start_distance_m
end_distance_m
length_m
reason = BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH
```

Segment identifiers must be deterministic under identical inputs

Example:

```text
aisle_016.segment_001
aisle_016.segment_002
aisle_016.fragment_001
```

## 8. Longitudinal segmentation semantics

The pose-feasibility samples are processed in increasing aisle longitudinal distance

Every maximal contiguous run of feasible sampled poses forms one raw feasible fragment

A fragment qualifies as an active coverage segment if:

```text
segment length >= 1.0 m
```

The 1.0 m threshold is frozen for the first implementation and matches the existing `VehicleSafeLaneConfig.minimum_contiguous_span_m` default

Fragments shorter than 1.0 m:

```text
remain visible as diagnostics
must not enter the global coverage graph
must not earn coverage reward
```

A1 must preserve every qualifying maximal contiguous segment

It must not keep only the longest segment

Segments from the same aisle must:

```text
not overlap
be ordered by start_distance_m
use deterministic identifiers
retain the exact sampled vehicle-safe geometry selected by the feasibility evaluator
```

## 9. Endpoint classification

Each active segment has two longitudinal endpoint classifications

Allowed values are:

```text
LOW_U_HEADLAND
HIGH_U_HEADLAND
INTERIOR_BLOCKED_END
```

Only the first feasible segment in longitudinal order can receive `LOW_U_HEADLAND`

Only the last feasible segment in longitudinal order can receive `HIGH_U_HEADLAND`

A segment low endpoint is classified as `LOW_U_HEADLAND` when its low-side retreat from the structural aisle low endpoint is within the existing configured `maximum_endpoint_retreat_m`

Otherwise it is `INTERIOR_BLOCKED_END`

A segment high endpoint is classified as `HIGH_U_HEADLAND` when its high-side retreat from the structural aisle high endpoint is within the existing configured `maximum_endpoint_retreat_m`

Otherwise it is `INTERIOR_BLOCKED_END`

Endpoint classification is structural metadata only

It does not itself prove an inter-aisle connector feasible

Later A3 connector validation must use the actual segment endpoint pose

## 10. Segment reachability semantics for later phases

The following rules are frozen now so A1 exposes sufficient metadata for A2-A4

Cross-aisle transitions may be created only through compatible headland endpoints

```text
LOW_U_HEADLAND  <-> LOW_U headland connector domain
HIGH_U_HEADLAND <-> HIGH_U headland connector domain
```

An `INTERIOR_BLOCKED_END` must not create an arbitrary lateral cross-row edge

V25-12G-A does not permit the optimizer to cut through crop rows or create arbitrary 2D inter-aisle transitions from interior blocked endpoints

A segment with both endpoints classified `INTERIOR_BLOCKED_END` remains valid evidence but may later be unreachable from the route graph

This is acceptable and must be reported explicitly instead of forcing a false connector

## 11. Segment service modes for later phases

The physical segment geometry and the way the vehicle services the segment are separate concepts

Future A2 service modes include:

```text
THROUGH_FORWARD_LOW_TO_HIGH
THROUGH_FORWARD_HIGH_TO_LOW
DEAD_END_FORWARD_IN_REVERSE_OUT
DEAD_END_REVERSE_IN_FORWARD_OUT
```

Traversing an aisle against the graph canonical row direction is still ordinary forward vehicle motion when the vehicle heading is reversed accordingly

Reverse gear is represented explicitly only when the vehicle actually backs along the lane

A one-headland segment is a valid dead-end coverage target

It may be entered along its already validated lane and later backed out along the same validated lane

The reverse exit distance contributes to travel and reverse cost but does not create a second coverage reward

## 12. Start and terminal conditions for later phases

The first complete 12G route optimizer will use:

```text
fixed START_POSE
free terminal endpoint
```

The optimizer is not required to return to the start pose

Future support may add an optional fixed `GOAL_POSE` / `HOME_POSE`

That future option must not change the default fixed-start/free-end behavior of 12G-A

## 13. Connector integration policy for A3

A3 must reuse the existing connector chain

The first connector candidate remains forward-only where appropriate

Forward connector feasibility is evaluated with the existing Dubins-family generator and full preview-footprint Navigation Grid / Site Boundary gate

Reverse fallback continues to pass through the explicit R6A admission boundary

R6A map-insufficient evidence must not be hidden by reverse planning

R6B remains the bounded deterministic Ackermann reverse-aware local connector backend

V25-12G must consume the resulting metrics rather than increasing R6B search budgets in order to force graph connectivity

Useful connector edge metadata for A4 includes:

```text
status
backend
path_length_m
forward_distance_m
reverse_distance_m
cusp_count
search_expansions
risk / evidence diagnostics
```

## 14. Maximum Feasible Coverage optimizer for A4

A4 operates over an executable segment/service graph

The search state conceptually contains:

```text
current service endpoint / vehicle state
covered segment set
current motion state needed for connector/service accounting
accumulated route metrics
```

The first optimizer backend is deterministic and budget bounded

Recommended techniques are:

```text
branch-and-bound
dominance pruning
deterministic expansion ordering
beam or explicit expansion budget where required
```

The optimizer must not collapse the frozen objective hierarchy into an opaque single weighted score

Route comparison uses the lexicographic hierarchy in Section 2

Result status must distinguish proof from budget-limited search

Allowed conceptual statuses are:

```text
PROVEN_OPTIMAL
BEST_FOUND_WITHIN_BUDGET
NO_EXECUTABLE_ROUTE
```

`PROVEN_OPTIMAL` may be emitted only when the relevant finite search space is exhausted or otherwise formally bounded without an unresolved budget cutoff

A budget-limited search must never claim global optimality

## 15. Repeated traversal semantics

An already covered segment may be traversed again as transit if required for reachability

The accounting rule is frozen as:

```text
first service traversal
    -> earns segment coverage reward

later traversal of same segment
    -> zero new coverage reward
    -> adds travel distance
    -> adds reverse distance when applicable
    -> adds cusp cost when applicable
```

This prevents the optimizer from increasing apparent coverage by repeatedly driving the same lane

## 16. Route risk

Risk is the final lexicographic objective and therefore cannot compensate for lost primary coverage by itself

The initial design treats risk as an auditable metric assembled from existing planning evidence rather than a learned probability

Examples of later risk evidence may include:

```text
small structural width surplus
large lateral offset usage
boundary-limited samples
high proximity to occupancy / traversability uncertainty
mixed-evidence connector approval
large reverse share
```

The exact A4 risk scalarization is not part of A1 and must be frozen before A4 implementation

A1 must preserve sufficient diagnostics to support later risk accounting

## 17. A1 real-data experiment

The first A1 experiment uses the existing frozen greenhouse run directory and the current MK-mini canonical vehicle profile

The main diagnostic aisles are:

```text
aisle_016
aisle_017
aisle_018
aisle_019
aisle_020
```

These are useful because the V25-12F real-data rerun already showed non-zero partial or ready Vehicle-Safe Lane coverage on them

A1 must report for each aisle:

```text
structural length
raw feasible fragment count
active >= 1.0 m segment count
rejected < 1.0 m fragment count
sum active segment length
current longest-only selected span
recoverable additional segment length
headland endpoint classifications
```

Primary A1 experimental metric:

```text
recoverable_additional_length_m
= sum(all active segment lengths)
- current longest-only selected span
```

Aggregate metric:

```text
segment_recovery_fraction
= sum(all active segment lengths)
/ sum(structural aisle lengths)
```

These metrics are diagnostic only

A larger segment total does not prove an executable global route until A2-A4 establish reachability and connector feasibility

## 18. A1 acceptance criteria

A1 is accepted only when all of the following are demonstrated by tests and operator-machine verification

```text
1. every emitted segment consists only of sampled preview-footprint-free poses
2. Site Boundary is enforced at every emitted pose when supplied
3. structural width rejection semantics are preserved
4. segment length threshold is exactly the configured minimum contiguous span, default 1.0 m
5. qualifying segments from the same aisle do not overlap
6. qualifying segments are deterministically ordered by longitudinal distance
7. segment / fragment identifiers are deterministic
8. short fragments remain diagnosable but never become active coverage segments
9. endpoint classifications follow the frozen retreat rule
10. the existing VehicleSafeLanePlan serialized output remains behaviorally unchanged
11. repeated execution on identical inputs produces identical segment output
12. real greenhouse data can expose additional active segments when more than one qualifying feasible run exists
13. no canonical Navigation Map files are modified
```

If the real dataset happens not to contain multiple >=1.0 m runs in a specific aisle, that aisle must be reported faithfully rather than artificially split

The experiment succeeds at the software-contract level if the extractor is correct and deterministic; the real-data report must separately state whether additional recoverable length was actually observed

## 19. Error handling and fail-closed behavior

A1 must fail closed for invalid truth inputs

Examples:

```text
frame mismatch
invalid vehicle profile
invalid Site Boundary
invalid aisle geometry
non-finite geometry
missing planning-preview vehicle truth
```

These conditions must raise explicit validation errors instead of silently emitting empty feasible segments

An aisle that is valid structurally but has no feasible sampled run is not a system error

It must produce an aisle result with zero active segments and a clear reason

A short feasible run below 1.0 m is also not a system error

It must be retained as a rejected fragment diagnostic

## 20. Serialization and artifacts

A1 should produce a dedicated review asset with a deterministic writer

Frozen proposed filename:

```text
vehicle_feasible_segments.yaml
```

The writer must not overwrite existing V25-12E / V25-12F route, map, lane, or traversability assets

The YAML must include:

```text
schema
status
frame_id
platform identity and profile hash
source hashes / source metadata where already available
configuration used for extraction
summary counts and lengths
per-aisle segments
rejected fragments
diagnostics
```

A compact binary geometry artifact is not required for A1 unless profiling proves YAML geometry size materially problematic

## 21. Workbench visualization scope

A1 should expose review-only visualization sufficient to inspect the extraction result

The desired first review layer is:

```text
Vehicle-Feasible Segments
```

Each segment should be visually distinguishable from gaps and rejected fragments

The visualization is not an editor in A1

It must not change the segment extractor output by dragging graphics

Interactive route editing is intentionally deferred as described in Section 23

## 22. Testing strategy

A1 requires focused synthetic tests before the real-data run

Required synthetic cases include:

```text
single full-length feasible aisle
single partial feasible run
multiple disjoint >=1.0 m feasible runs
short fragment between blocked regions
short fragment adjacent to headland
no feasible poses
structural-width-blocked aisle
Site-Boundary-clipped endpoint
Site-Boundary-fully-blocked aisle
lateral-shifted feasible runs
stable ordering / deterministic serialization
backward compatibility with existing VehicleSafeLanePlan
```

At least one regression fixture must contain:

```text
headland -> segment A -> blocked gap -> segment B -> headland
```

and assert that both A and B are emitted when both satisfy the minimum useful length

A second fixture must contain:

```text
headland -> active segment -> short fragment -> active segment -> headland
```

and assert that the short fragment remains diagnostic only

Package-level verification must include the registered `agt_offline_assets` suite and any Workbench tests added for visualization

## 23. Manual route adjustment: recorded future feature

The project should preserve and later extend manual route adjustment capability

The intended interaction model is:

```text
automatic route candidate
        |
        v
operator selects connector / segment
        |
        v
display MK-mini preview footprint
        |
        v
operator drags vehicle pose or route control points
        |
        v
continuous safety / kinematic revalidation
```

The operator may place a preview into an invalid state for diagnosis

However:

```text
previewable
!= executable
```

A manual path may become an executable candidate only if it passes the same hard constraints as automatic planning

A future accepted manual path should be represented as an explicit candidate such as:

```text
MANUAL_FEASIBLE_CANDIDATE
```

It should be usable either as a replacement connector candidate or as an additional graph edge

Manual route editing must not silently modify the underlying map

Responsibility is separated as:

```text
map editing
    -> correct whether space should be considered traversable

route editing
    -> choose how the vehicle moves within accepted traversable space
```

If the operator believes an OCCUPIED region is actually traversable, that correction belongs in the map/evidence authoring workflow and must then be revalidated

## 24. Learning-based planning: recorded future research

Reinforcement learning is explicitly outside V25-12G-A implementation scope

It remains a future research direction built on the deterministic 12G environment and metrics

A future training environment may expose:

```text
map / traversability state
agricultural structure prior
vehicle state
coverage history
segment graph state
connector state
```

Hard environment constraints remain:

```text
Ackermann kinematics
vehicle footprint
Site Boundary
semantic NO_GO
validated collision semantics
```

A possible reward family is:

```text
+ useful first-time coverage
+ distinct aisle coverage
- travel distance
- reverse distance
- cusp actions
- risk
```

The research contribution should not be framed merely as "use RL to find the optimal path"

More promising research questions include:

```text
agricultural structural priors under partial terrain observation
constraint-aware learned coverage planning
few-shot human route correction / demonstration
human-assisted candidate generation
transfer from simulated greenhouse structure to real occluded greenhouse evidence
```

Future experimental comparisons may include:

```text
Fixed Boustrophedon
Greedy Maximum Coverage
12G Deterministic Search
Hybrid A* / Reeds-Shepp-style search
MILP / graph optimization backend
RL
Human-assisted deterministic planning
Human-demonstration + learning
```

The deterministic V25-12G route system is therefore intentionally designed to become a reproducible baseline for later research rather than being replaced by learning in the first implementation

## 25. Explicit non-goals for A1

V25-12G-A1 must not implement:

```text
global coverage ordering
segment graph search
forward connector generation changes
R6A policy changes
R6B search-budget increases
Hybrid A*
analytic Reeds-Shepp replacement
MILP
reinforcement learning
manual vehicle dragging
manual route control-point editing
canonical map promotion
global UNKNOWN -> FREE conversion
recovery of OCCUPIED merely because it is padding-dominant
```

These exclusions are deliberate

A1 must isolate and validate the new planning primitive before later optimization layers are added

## 26. A1 implementation boundary

The first implementation plan should be organized around these responsibilities:

```text
1. expose a reusable private aisle pose-feasibility sampling result without changing stable lane output
2. derive all maximal contiguous feasible fragments
3. classify fragments into active segments vs rejected short fragments
4. classify segment endpoints
5. define `VehicleFeasibleSegmentPlan` serialization
6. add deterministic unit / regression tests
7. add minimal review visualization
8. run focused and package-level verification
9. run the frozen greenhouse A1 experiment
10. record the A1 current-state / real-data result
```

No later optimizer work begins until this boundary passes

## 27. Design invariant

The V25-12G design is summarized by the following invariant:

> Maximize useful executable agricultural coverage by planning over vehicle-feasible segments, while preserving hard physical and semantic safety constraints and keeping every skipped or unreachable region explainable

A second research-facing invariant is:

> Human assistance and learned policies may propose better candidates, but they do not own or bypass the safety truth
