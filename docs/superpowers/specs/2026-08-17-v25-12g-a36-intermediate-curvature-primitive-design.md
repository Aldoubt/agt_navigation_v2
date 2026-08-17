# V25-12G-A3.6 Intermediate-Curvature Primitive Amendment Design

Date: 2026-08-17

Status: DESIGN APPROVED VERBALLY / WRITTEN SPEC PENDING REVIEW

## 1. Purpose

A3.6 is the first behavior-changing connector-backend amendment after the frozen A3.5 diagnosis.

A3.5 established on the real greenhouse dataset that:

```text
40 R6B connector candidates
0 EXECUTABLE
40 REJECTED
40 queue exhausted
0 expansion-budget reached
```

with failure classes:

```text
MIXED_LIMITATION          26
INCONCLUSIVE               6
FOOTPRINT_GRID_LIMITED     3
GOAL_CONNECTION_LIMITED    3
SITE_BOUNDARY_LIMITED      2
```

The diagnostic evidence does not support increasing `max_expansions`, relaxing Site Boundary, relaxing the FREE-only Navigation Grid gate, shrinking the footprint, reducing the 0.05 m preview padding, or reducing the verified 1.5 m Ackermann minimum turning radius.

A3.6 therefore tests one narrow, falsifiable backend hypothesis:

> The current saturated-left / straight / saturated-right three-curvature primitive lattice is too sparse to express some locally feasible Ackermann paths through the existing hard envelope, Site Boundary, footprint/grid, state-dominance and path-length constraints. Adding intermediate *lower-magnitude* curvatures may increase reachable path families without enlarging the physical or safety-feasible region.

This is a hypothesis to test, not a conclusion already established by A3.5.

## 2. Chosen Amendment

The only intended planner behavior change is the primitive curvature family.

Current family:

```text
{-1.0/R, 0.0, +1.0/R}
```

A3.6 family:

```text
{-1.0/R, -0.5/R, 0.0, +0.5/R, +1.0/R}
```

where `R` is the canonical verified Ackermann minimum turning radius from the vehicle profile.

The `0.5` fraction is deliberately fixed for this experiment. A3.6 must not turn into a curvature-fraction parameter sweep.

For the frozen `mk_mini` profile with `R = 1.50 m`, the added primitives have curvature magnitude:

```text
0.5 / 1.5 = 0.333333... 1/m
```

which corresponds to a turning radius of:

```text
3.0 m
```

Therefore the new primitives are physically *less* aggressive than the existing `1/R` saturated primitives. A3.6 must never generate `abs(curvature) > 1/R`.

## 3. Hard Constraints That Remain Frozen

A3.6 does not relax any physical, safety, map or bounded-search constraint.

The following remain exactly unchanged:

```text
canonical platform                mk_mini
kinematics                        Ackermann
verified minimum turning radius   1.50 m
canonical navigation footprint    unchanged
preview footprint padding         0.05 m
Site Boundary semantics           hard vehicle-permitted inner boundary
Navigation Grid policy            FREE-only preview footprint gate
NO_GO / occupied semantics        unchanged
primitive length                  0.30 m
collision sampling step           0.10 m
state XY resolution               0.15 m
state yaw resolution              15 deg
goal position tolerance           0.18 m
goal yaw tolerance                12 deg
goal-shot distance                3.0 m
goal-shot semantics               unchanged / forward-only Dubins
max cusps                         2
max expansions                    30000
max path length                   18.0 m
longitudinal zone padding         0.80 m
lateral pair padding              1.25 m
heuristic                         unchanged
cost model                        unchanged
state dominance rule              unchanged
queue ordering                    unchanged
A1 assets                         unchanged
A2 assets                         unchanged
```

A3.6 must not obtain connectivity by:

- shrinking the footprint;
- reducing preview padding;
- reducing the minimum turning radius;
- allowing any primitive with curvature magnitude above `1/R`;
- leaving and re-entering the Site Boundary;
- ignoring occupied, unknown or out-of-grid cells;
- increasing `max_expansions`;
- increasing `max_path_length_m`;
- increasing `max_cusps`;
- changing goal tolerances;
- changing the state-resolution or state-dominance contract;
- changing the search envelope;
- replacing the goal shot with reverse-aware or Reeds-Shepp behavior;
- modifying A1 or A2 assets.

## 4. Why This Amendment Is First

Three candidate amendment families were considered after A3.5.

### 4.1. Intermediate-curvature primitives — chosen

Advantages:

- changes one small search-generation surface;
- leaves every hard validity gate intact;
- does not weaken the 1.5 m turning-radius contract;
- can address mixed geometry/path/state interactions rather than one isolated terminal condition;
- is straightforward to falsify with one synthetic reachable-state fixture and one real-map rerun.

Main cost:

- each expanded search node considers five primitive edges instead of three, so runtime and queue size may increase.

### 4.2. Goal-connection amendment — deferred

A3.5 found three `GOAL_CONNECTION_LIMITED` connectors and near-goal states, but changing final connection behavior would directly alter terminal semantics and potentially the forward-only arrival contract. It is narrower in affected connector count and conceptually closer to acceptance semantics than to search expressiveness.

### 4.3. Analytic Reeds-Shepp / second backend — deferred

A second reverse-aware analytic backend would substantially increase scope, implementation surface and validation burden. It is not justified before the smaller primitive-family hypothesis is tested.

## 5. Production Architecture

A3.6 should keep the existing R6B backend and change only how the primitive curvature tuple is constructed.

Recommended internal representation:

```python
_PRIMITIVE_CURVATURE_FRACTIONS = (
    -1.0,
    -0.5,
     0.0,
     0.5,
     1.0,
)


def _primitive_curvature_values(radius: float) -> tuple[float, ...]:
    ...
```

The helper must:

- require finite `radius > 0`;
- preserve deterministic ascending fraction order;
- return exactly five values;
- return straight exactly at index/fraction `0.0`;
- guarantee `max(abs(kappa)) <= 1.0 / radius` within numerical tolerance.

The search loop should consume this tuple instead of the current hard-coded three-value tuple.

No second planner implementation should be created.

## 6. Steering Index / Cost Semantics

The search state currently includes `last_curvature_index`, and steering-change cost is charged when the next primitive index differs from the previous one.

A3.6 keeps that cost semantics unchanged.

Recommended deterministic indexing is:

```text
-2  -> -1.0/R
-1  -> -0.5/R
 0  ->  0.0
+1  -> +0.5/R
+2  -> +1.0/R
```

The meaning remains categorical: if the chosen primitive curvature category changes, apply the existing steering-change penalty. A3.6 must not introduce curvature-distance-weighted steering cost, interpolation cost, or any new penalty coefficient.

## 7. Diagnostic Accounting Impact

A3.5 diagnostics remain valid and must continue to account for every considered primitive edge exactly once.

The accounting identity remains:

```text
primitive_edges_considered
=
primitive_edges_rejected_search_envelope
+ primitive_edges_rejected_site_boundary
+ primitive_edges_rejected_navigation_grid
+ primitive_edges_rejected_path_length
+ primitive_edges_rejected_state_dominance
+ primitive_edges_enqueued
```

The only expected count-level difference is that a non-terminal expanded node can now consider five curvature primitives instead of three.

For the existing path-length short-circuit:

```text
if node.travel_m + primitive_length_m > max_path_length_m:
```

A3.6 must account for **five** considered primitives and **five** path-length rejections, not the previous hard-coded three.

No diagnostic classifier thresholds are changed in A3.6.

## 8. Strict TDD RED Contract

Production changes must not be written until the target-machine RED gate demonstrates the intended missing A3.6 behavior.

At minimum, tests must cover the following.

### 8.1. Curvature-family contract

For a test radius `R`:

```text
values == (-1/R, -0.5/R, 0, +0.5/R, +1/R)
len(values) == 5
max(abs(values)) == 1/R
```

No returned primitive may violate the minimum-turning-radius bound.

### 8.2. Synthetic one-primitive reachability fixture

Construct an open FREE synthetic environment where:

- start pose is legal;
- Site Boundary is non-limiting but still enforced;
- Navigation Grid is FREE;
- `R = 1.5 m` or an equivalent deterministic test profile;
- primitive length remains `0.30 m`;
- goal is the endpoint and yaw of exactly one `+0.5/R` primitive from start;
- goal-shot radius is set below the start-to-goal distance so goal shot cannot solve the fixture;
- max path length allows only one primitive-length motion family before termination;
- goal tolerances are tight enough that `0`, `+1/R`, and `-1/R` do not accidentally satisfy the target.

Expected TDD behavior:

```text
pre-A3.6 three-curvature implementation -> NO SOLUTION
A3.6 five-curvature implementation      -> executable bounded primitive path
```

This test proves a genuinely new Ackermann-feasible reachable state rather than merely checking that a helper returns five constants.

### 8.3. Frozen-hard-gate regression tests

Existing tests must continue to prove:

- Site Boundary still rejects illegal footprint samples;
- Navigation Grid still rejects non-FREE footprint samples;
- 0.05 m preview padding remains unchanged;
- `max_expansions=30000` remains unchanged;
- max path length and cusp limits remain unchanged;
- goal-shot semantics remain unchanged;
- diagnostics remain strict and accounting-valid.

### 8.4. Five-primitive accounting

Synthetic tests must verify that:

- an expanded node considers five primitive edges;
- the path-length short-circuit accounts for five considered/path-rejected edges;
- every considered primitive lands in exactly one terminal accounting bucket.

## 9. Real-Data Acceptance Harness

A3.6 should add a read-only real-data acceptance harness, recommended path:

```text
tools/v25_12g_a36_intermediate_curvature_acceptance.py
```

The harness must not write or replace canonical runtime/map assets.

It should derive the amended A3 motion graph in memory against the same frozen real inputs and compare against the frozen A3.5 baseline.

The report must include at least:

```text
report schema / validation scope
diagnostic implementation/amendment identity
40 connector universe preserved
32 service-action behavior preserved
baseline transition status counts
amended transition status counts
newly executable connector IDs
still rejected connector IDs
unexpectedly regressed connector IDs
queue-exhausted count
expansion-budget-reached count
expansion histogram
runtime / CPU / max RSS
protected-input hashes before/after
forbidden semantic key count
```

The harness must also compute the A4 minimum-chain predicate without promoting it to a route-readiness claim.

## 10. A4 Minimum-Chain Predicate

A4 remains blocked unless the amended real motion graph contains a continuous chain:

```text
segment A
 -> EXECUTABLE transition 1
segment B
 -> EXECUTABLE transition 2
segment C
```

with:

```text
>= 3 distinct vehicle-feasible segment_ids
>= 2 distinct aisle_ids
>= 2 EXECUTABLE inter-segment transitions
```

The three service segments must span at least two aisles.

The acceptance harness may expose a diagnostic Boolean such as:

```text
a4_entry_gate_passed
```

but this Boolean means only that the frozen minimum engineering chain predicate is satisfied. It must not imply optimality, route readiness, field readiness or global connectivity.

## 11. Real-Data Experiment Semantics

The A3.6 real-data run is a hypothesis test, not a benchmark that must improve.

Therefore:

```text
A3.6 harness RC = 0
```

means:

- the experiment ran correctly;
- protected inputs were not modified;
- contracts and invariants were preserved;
- results were reported deterministically.

It does **not** require any connector to become executable.

Three valid result families exist.

### 11.1. Positive result

At least one previously rejected connector becomes executable while all frozen safety/geometry constraints remain enforced.

This supports the hypothesis that primitive-lattice expressiveness was a real limiting factor.

If the A4 minimum-chain predicate is also satisfied, A4 may become eligible for a separate entry decision after evidence freeze.

### 11.2. Partial result

Some connectors improve or become executable but the A4 minimum chain is still absent.

Freeze the evidence. Do not automatically add more curvature fractions or relax other parameters.

### 11.3. Negative result

All 40 remain rejected.

This is a valid, useful result. Freeze it and move to a separately approved next hypothesis. Do not append `±0.25/R`, increase path length, refine state resolution, alter goal tolerance, or otherwise stack unapproved amendments onto the same experiment.

## 12. Regression / Fail-Closed Rules

A3.6 must stop as a regression if any of the following occurs:

- a previously executable service action becomes non-executable;
- connector/service IDs are lost or duplicated;
- Site Boundary or Navigation Grid checks are bypassed;
- any primitive exceeds curvature magnitude `1/R`;
- protected inputs change on disk;
- forbidden semantic keys appear;
- a success path fails its existing final path-evidence validation;
- the acceptance harness writes canonical runtime assets;
- an unrelated parameter changes together with the primitive family.

Any such result invalidates the experiment even if more connectors become executable.

## 13. Expected Files

The implementation plan may refine exact locations, but the intended narrow surface is:

Modify:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Potentially modify only for test registration / contract coverage:

```text
src/agt_offline_assets/CMakeLists.txt
```

Create:

```text
tests/test_v25_12g_a36_intermediate_curvature_contract.py
tools/v25_12g_a36_intermediate_curvature_acceptance.py
docs/v2.5/V25_12G_A36_INTERMEDIATE_CURVATURE_2026-08-17.md
```

Do not modify `vehicle_feasible_motion_graph` schemas unless an implementation-plan review proves that the acceptance report requires a new evidence field. The default expectation is no motion-graph schema change for A3.6.

## 14. Protected Local Paths

The following unrelated local paths remain outside A3.6 and must never be modified, reset, deleted or staged:

```text
M  tools/rosbag_sensor_trimmer
?? src/agt_route_benchmark/
?? tools/map_tools/render_pcd_top_views.py
```

Do not use:

```text
git add .
git add -A
git clean
git reset --hard
git restore .
git checkout -- .
```

Use the original checkout; do not create an extra worktree.

## 15. Completion Boundary

A3.6 ends after:

```text
approved written spec
-> strict TDD implementation plan
-> tests-only RED
-> target-machine valid RED confirmation
-> minimal production amendment
-> focused/package GREEN
-> one frozen all-40 real-data acceptance run
-> A3.6 evidence freeze
```

A3.6 does not automatically continue into a second amendment.

If the five-curvature experiment does not satisfy the A4 minimum-chain predicate, the next backend change requires a new evidence-backed design decision.
