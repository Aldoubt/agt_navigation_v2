# V25-12G-A3.5 Connector Backend Diagnosis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add diagnostic-only instrumentation to the existing R6B bounded reverse-primitive connector backend, preserve every pre-A3.5 planner outcome, classify the complete 40-connector real-data failure funnel with deterministic evidence, freeze that diagnosis, and stop before any behavior-changing backend amendment.

**Architecture:** Keep the frozen R5 -> forward audit -> R6A -> R6B planning chain intact. Add a pure diagnostic contract/classifier module, instrument R6B with counters and reason-returning internal probes that are Boolean-equivalent to the current gates, pass the resulting evidence through A3 `TransitionValidation` and strict YAML IO without changing status/proof semantics, and add a read-only A3.5 diagnosis harness that compares the newly derived graph against the frozen pre-A3.5 runtime motion graph after stripping only the new diagnostic field. The harness then aggregates all 40 real connector failures by side, forward-audit source, termination mode, failure class, rejection funnel, and closest-to-goal evidence.

**Tech Stack:** Python 3.10, dataclasses, math, pathlib, collections, NumPy, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing A2/A3 graph contracts, existing forward connector gate/audit, R6A admission, R6B bounded reverse primitive search, and the existing `tools/v25_12g_a3_acceptance.py` conventions.

## Global Constraints

- Work only in the existing checkout on `feat/v25-12g-maximum-feasible-coverage`; do not create a worktree.
- Do not run `git reset --hard` or `git clean`.
- Never touch, stage, revert, rename, or delete:
  - `tools/rosbag_sensor_trimmer`
  - `tools/map_tools/render_pcd_top_views.py`
- Strict TDD is mandatory: one tests-only RED batch first, operator confirms the focused RED for the intended missing diagnostic behavior, then and only then production code may be changed.
- The operator runs ROS 2/package tests on the target machine. Do not claim RED/GREEN/build/real-data success without fresh operator output.
- Batch operator gates. This plan uses exactly three mandatory operator gates before the diagnosis is frozen: one focused RED gate, one focused+package GREEN gate, and one full 40-connector real-data diagnostic gate.
- The first A3.5 implementation stage is diagnostic-only. It must not alter planner branch selection, queue ordering, state keys, costs, heuristic, goal tolerances, path acceptance, R6A policy, A3 transition classification, or executable IDs.
- Keep the existing hard Site Boundary invariant. Touching or crossing the Site Boundary remains invalid.
- Keep the frozen Navigation Grid semantics. Do not reinterpret UNKNOWN/out-of-grid as FREE and do not weaken FREE-only footprint acceptance.
- Keep the canonical navigation footprint and `preview_footprint_padding_m=0.05` unchanged.
- Keep the canonical Ackermann minimum turning radius at `1.50 m`; do not reduce or bypass it.
- Keep the frozen R6B search configuration unchanged:
  - `primitive_length_m=0.30`
  - `collision_sample_step_m=0.10`
  - `state_xy_resolution_m=0.15`
  - `state_yaw_resolution_deg=15.0`
  - `goal_position_tolerance_m=0.18`
  - `goal_yaw_tolerance_deg=12.0`
  - `goal_shot_distance_m=3.0`
  - `max_cusps=2`
  - `max_expansions=30000`
  - `max_path_length_m=18.0`
  - `longitudinal_zone_padding_m=0.80`
  - `lateral_pair_padding_m=1.25`
  - `preview_footprint_padding_m=0.05`
- Do not increase `max_expansions=30000`. The frozen real-data evidence already shows a maximum of `16621`, so A3.5 must diagnose queue/search structure instead of assuming budget saturation.
- Do not modify A1/A2/A3 upstream source assets in `runtime/maps/agt_workbench_run`.
- The runtime `vehicle_feasible_motion_graph.yaml` is a frozen real-data baseline/evidence asset, not a source-controlled canonical map asset. The A3.5 harness may read it but must not overwrite it.
- Preserve schema `agt_vehicle_feasible_motion_graph/v1` and top-level status `MOTION_EVIDENCE_ONLY`.
- Do not serialize `route_ready`, `reachable_from_start`, `optimal`, or equivalent claims.
- Do not hard-code connector IDs or real-map class counts into diagnostic classification logic.
- Do not describe `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION` as proof of global kinematic infeasibility.

## Completion Boundary For This Plan

This plan intentionally stops after the diagnostic evidence is frozen. It does **not** implement the later backend amendment and it does **not** satisfy the A4 entry gate by itself.

A4 remains blocked until a later, separately approved behavior-changing stage produces at least one real continuous chain:

```text
segment A -> transition 1 -> segment B -> transition 2 -> segment C
```

with all of the following simultaneously true:

- at least `3` distinct `segment_id` values;
- at least `2` distinct `aisle_id` values;
- at least `2` `EXECUTABLE` inter-segment transitions.

The expected outcome of this diagnostic-only plan on the frozen baseline is still `0` executable transitions. Any change from the pre-A3.5 final transition behavior during this plan is a regression and must stop execution.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  reverse_primitive_diagnostics.py
  reverse_primitive_connector.py
  vehicle_feasible_motion_graph.py
  vehicle_feasible_transition_motion.py
  vehicle_feasible_motion_graph_io.py

src/agt_offline_assets/test/
  test_reverse_primitive_diagnostics.py
  test_reverse_primitive_connector.py
  test_vehicle_feasible_motion_graph.py

src/agt_offline_assets/CMakeLists.txt

tools/
  v25_12g_a35_connector_diagnosis.py

tests/
  test_v25_12g_a35_connector_diagnosis_contract.py

docs/v2.5/
  V25_12G_A35_DIAGNOSIS_2026-08-17.md
```

Create `reverse_primitive_diagnostics.py`, `test_reverse_primitive_diagnostics.py`, `v25_12g_a35_connector_diagnosis.py`, `test_v25_12g_a35_connector_diagnosis_contract.py`, and the final real-data diagnosis document. Modify the other listed files. Do not add a second planner or duplicate R6B implementation.

---

## Frozen A3.5 Diagnostic Contract

### Public failure classes

Use exactly:

```python
SEARCH_ENVELOPE_LIMITED = "SEARCH_ENVELOPE_LIMITED"
SITE_BOUNDARY_LIMITED = "SITE_BOUNDARY_LIMITED"
FOOTPRINT_GRID_LIMITED = "FOOTPRINT_GRID_LIMITED"
STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED = (
    "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED"
)
GOAL_CONNECTION_LIMITED = "GOAL_CONNECTION_LIMITED"
MOTION_PRIMITIVE_LIMITED = "MOTION_PRIMITIVE_LIMITED"
PATH_OR_CUSP_ENVELOPE_LIMITED = "PATH_OR_CUSP_ENVELOPE_LIMITED"
MIXED_LIMITATION = "MIXED_LIMITATION"
INCONCLUSIVE = "INCONCLUSIVE"
R6B_DIAGNOSTIC_SCHEMA = "agt_r6b_connector_diagnostics/v1"
```

### Public diagnostic dataclass

Create this exact public data shape in `reverse_primitive_diagnostics.py`:

```python
@dataclass(frozen=True)
class ReversePrimitiveSearchDiagnostics:
    failure_class: str | None = None
    search_started: bool = False
    search_expansions: int = 0
    queue_exhausted: bool = False
    expansion_budget_reached: bool = False

    start_goal_position_error_m: float | None = None
    best_goal_position_error_m: float | None = None
    best_goal_yaw_error_rad: float | None = None
    best_goal_distance_state_direction: str | None = None
    best_goal_distance_cusp_count: int | None = None

    nodes_popped: int = 0
    stale_nodes_skipped: int = 0

    cusp_switches_considered: int = 0
    cusp_switches_rejected_state_dominance: int = 0
    cusp_switches_enqueued: int = 0
    nodes_at_max_cusps: int = 0

    primitive_edges_considered: int = 0
    primitive_edges_rejected_search_envelope: int = 0
    primitive_edges_rejected_site_boundary: int = 0
    primitive_edges_rejected_navigation_grid: int = 0
    primitive_edges_rejected_path_length: int = 0
    primitive_edges_rejected_state_dominance: int = 0
    primitive_edges_enqueued: int = 0

    goal_tolerance_checks: int = 0
    goal_tolerance_successes: int = 0

    goal_shot_attempts: int = 0
    goal_shot_reverse_direction_blocked: int = 0
    goal_shot_candidates_considered: int = 0
    goal_shot_rejected_path_length: int = 0
    goal_shot_rejected_search_envelope: int = 0
    goal_shot_rejected_site_boundary: int = 0
    goal_shot_rejected_navigation_grid: int = 0
    goal_shot_successes: int = 0

    schema: str = R6B_DIAGNOSTIC_SCHEMA
```

Append a `diagnostics` field to `ReversePrimitiveConnectorResult` after the existing `reason` field:

```python
diagnostics: ReversePrimitiveSearchDiagnostics = field(
    default_factory=ReversePrimitiveSearchDiagnostics
)
```

Append one optional evidence field to `TransitionValidation` after the existing `reverse_admission_evidence` field:

```python
reverse_backend_diagnostics: Mapping[str, Any] = field(default_factory=dict)
```

Do not change any pre-existing field name/order/meaning except for appending those evidence fields.

### Required pure helper interfaces

Implement these exact public signatures in `reverse_primitive_diagnostics.py`:

```text
reverse_primitive_search_diagnostics_to_dict(
    diagnostics: ReversePrimitiveSearchDiagnostics,
) -> dict[str, Any]

reverse_primitive_search_diagnostics_from_dict(
    raw: Mapping[str, Any],
) -> ReversePrimitiveSearchDiagnostics

classify_reverse_primitive_failure(
    diagnostics: ReversePrimitiveSearchDiagnostics,
    *,
    max_cusps: int,
) -> str
```

`classify_reverse_primitive_failure` rejects `max_cusps < 1` and reads no connector ID, aisle ID, side, turn-zone ID, or real-map count.

### Counter accounting invariants

For every completed diagnostic record enforce:

```text
nodes_popped
  = search_expansions + stale_nodes_skipped

cusp_switches_considered
  = cusp_switches_rejected_state_dominance
  + cusp_switches_enqueued

primitive_edges_considered
  = primitive_edges_rejected_search_envelope
  + primitive_edges_rejected_site_boundary
  + primitive_edges_rejected_navigation_grid
  + primitive_edges_rejected_path_length
  + primitive_edges_rejected_state_dominance
  + primitive_edges_enqueued

goal_shot_candidates_considered
  = goal_shot_rejected_path_length
  + goal_shot_rejected_search_envelope
  + goal_shot_rejected_site_boundary
  + goal_shot_rejected_navigation_grid
  + goal_shot_successes
```

Also enforce:

- `queue_exhausted` and `expansion_budget_reached` are mutually exclusive.
- For an unsuccessful search, `queue_exhausted=True` iff the queue is empty after loop exit.
- For an unsuccessful search, `expansion_budget_reached=True` iff work remains and `search_expansions >= max_expansions`.
- If the queue is empty exactly when the count reaches the limit, queue exhaustion wins.
- Start hard failures set `search_started=False` and both termination flags false.
- `goal_tolerance_checks` increments once for every non-stale expanded state.
- Only an existing FORWARD goal-tolerance success increments `goal_tolerance_successes`.
- A REVERSE state within `goal_shot_distance_m` increments `goal_shot_reverse_direction_blocked` but never starts a new shot.
- Each goal-shot candidate lands in exactly one terminal counter.
- Each primitive curvature candidate lands in exactly one terminal counter.
- When `node.travel_m + primitive_length_m > max_path_length_m`, count all three frozen curvature candidates as path-length rejections before preserving the existing node-level `continue`.

### Edge rejection reason contract

Add these internal constants to `reverse_primitive_connector.py`:

```python
EDGE_FREE = "FREE"
EDGE_SEARCH_ENVELOPE = "SEARCH_ENVELOPE"
EDGE_SITE_BOUNDARY = "SITE_BOUNDARY"
EDGE_NAVIGATION_GRID = "NAVIGATION_GRID"
```

Add `_edge_rejection_reason` with the **same parameter list as the current `_edge_is_free`**. For each sample, evaluate in exactly this order:

1. `_inside_search_envelope`; if false return `EDGE_SEARCH_ENVELOPE`.
2. `_preview_pose_status`; if it returns `SITE_BOUNDARY_CONFLICT`, return `EDGE_SITE_BOUNDARY`.
3. If `_preview_pose_status` is not `FREE`, return `EDGE_NAVIGATION_GRID`.
4. If every sample passes, return `EDGE_FREE`.

Retain `_edge_is_free` with its current parameter list and make it return only:

```python
_edge_rejection_reason(
    samples,
    bounds,
    navigation,
    local_footprint,
    site_boundary,
) == EDGE_FREE
```

This preserves the existing acceptance set while exposing why an edge died.

### Best-goal state semantics

For every non-stale popped state compute position and yaw errors. Maintain one coherent best-state snapshot. Update when:

1. position error improves by more than `1e-12`; or
2. position errors tie within `1e-12` and yaw error improves by more than `1e-12`.

If both tie within `1e-12`, keep the earlier popped state. Store position error, yaw error, direction, and cusp count from that same state.

### Frozen diagnostic thresholds

These constants are diagnostic semantics only and must never affect planner control flow:

```python
DOMINANT_REJECTION_FRACTION = 0.50
MIXED_REJECTION_FRACTION = 0.25
DOMINANCE_REJECTION_FRACTION = 0.60
DOMINANCE_ENQUEUED_FRACTION_MAX = 0.15
CUSP_SATURATION_FRACTION = 0.50
LITTLE_PROGRESS_FRACTION = 0.20
```

For combined geometric/path fractions use:

```text
geometry_candidate_count
  = primitive_edges_considered
  + goal_shot_candidates_considered

search_envelope_count
  = primitive_edges_rejected_search_envelope
  + goal_shot_rejected_search_envelope

site_boundary_count
  = primitive_edges_rejected_site_boundary
  + goal_shot_rejected_site_boundary

navigation_grid_count
  = primitive_edges_rejected_navigation_grid
  + goal_shot_rejected_navigation_grid

path_length_count
  = primitive_edges_rejected_path_length
  + goal_shot_rejected_path_length
```

Divide each cause count by `max(geometry_candidate_count, 1)`.

State-dominance signal:

```text
primitive_edges_rejected_state_dominance / max(primitive_edges_considered, 1)
    >= 0.60
AND
primitive_edges_enqueued / max(primitive_edges_considered, 1)
    <= 0.15
```

Cusp-saturation signal:

```text
search_expansions > 0
AND nodes_at_max_cusps / search_expansions >= 0.50
AND best_goal_distance_cusp_count == max_cusps
```

Goal-connection signal:

```text
best_goal_position_error_m is not None
AND goal_tolerance_successes == 0
AND goal_shot_successes == 0
AND (goal_shot_attempts > 0 OR goal_shot_reverse_direction_blocked > 0)
```

Little-progress fraction:

```text
(start_goal_position_error_m - best_goal_position_error_m)
/ max(start_goal_position_error_m, 1e-12)
```

Clamp only the reported progress value to `[0.0, 1.0]`.

Motion-primitive signal:

```text
queue_exhausted
AND primitive_edges_considered > 0
AND every geometric/path rejection fraction < 0.25
AND primitive_edges_rejected_state_dominance
    / max(primitive_edges_considered, 1) < 0.25
AND primitive_edges_enqueued
    / max(primitive_edges_considered, 1) >= 0.25
AND little_progress_fraction <= 0.20
AND no goal-connection signal
AND no cusp-saturation signal
```

### Frozen classifier order

Apply the classifier only to `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`; all other R6B outcomes keep `failure_class=None`.

For bounded no-solution results:

1. If two or more geometric/path causes are each `>=0.50`, return `MIXED_LIMITATION`.
2. If exactly one geometric/path cause is `>=0.50`, map directly:
   - envelope -> `SEARCH_ENVELOPE_LIMITED`
   - Site Boundary -> `SITE_BOUNDARY_LIMITED`
   - Navigation Grid -> `FOOTPRINT_GRID_LIMITED`
   - path length -> `PATH_OR_CUSP_ENVELOPE_LIMITED`
3. If no direct cause reaches `0.50`, form a set of moderate/structural classes:
   - each geometric/path cause `>=0.25` contributes its mapped class;
   - state dominance contributes `STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED`;
   - cusp saturation contributes `PATH_OR_CUSP_ENVELOPE_LIMITED`;
   - goal connection contributes `GOAL_CONNECTION_LIMITED`.
4. If two or more distinct classes are present, return `MIXED_LIMITATION`.
5. If exactly one structural signal from state dominance, cusp saturation, or goal connection is present, return it.
6. A lone geometric/path fraction in `[0.25, 0.50)` is insufficient by itself and falls through.
7. If the conservative motion-primitive signal is true, return `MOTION_PRIMITIVE_LIMITED`.
8. Otherwise return `INCONCLUSIVE`.

Do **not** map `expansion_budget_reached` to a failure class. A budget hit is reported independently and remains `INCONCLUSIVE` unless another evidence rule classifies the failure.

---

## Operator Gate Batching

### Gate G1 — tests-only RED

Run after Task 1 creates all A3.5 tests and before any production implementation:

```bash
cd ~/agt_navigation_v2

PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py \
  -k a35
```

Accept G1 only when tests collect successfully and fail on missing A3.5 behavior/API. Import errors, syntax errors, or bad fixtures are not valid RED.

### Gate G2 — focused GREEN plus ROS 2 package regression

Run once Tasks 2-5 are implemented:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash

PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py \
  -k a35

colcon build \
  --packages-select agt_offline_assets \
  --symlink-install

source install/setup.bash

colcon test \
  --packages-select agt_offline_assets \
  --event-handlers console_direct+

colcon test-result --verbose

PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q tests/test_v25_12g_a35_connector_diagnosis_contract.py
```

Proceed only with zero focused failures and zero package errors/failures from fresh operator output.

### Gate G3 — complete 40-connector real-data diagnosis

Run only after G2:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/time -v \
python3 tools/v25_12g_a35_connector_diagnosis.py \
  --run-dir runtime/maps/agt_workbench_run \
  --vehicle-profile profiles/platforms/mk_mini.yaml \
  --baseline-motion-graph vehicle_feasible_motion_graph.yaml \
  --pretty \
  > /tmp/v25_12g_a35_connector_diagnosis.json \
  2> /tmp/v25_12g_a35_connector_diagnosis.time
```

Inspect the captured JSON without rerunning R6B:

```bash
python3 - <<'PY'
import json
from pathlib import Path

report = json.loads(
    Path('/tmp/v25_12g_a35_connector_diagnosis.json').read_text()
)
summary = report['summary']
print('schema:', report['schema'])
print('connector_count:', summary['r6b_connector_count'])
print('behavior_projection_equal:', summary['behavior_projection_equal'])
print('final_transition_statuses_unchanged:', summary['final_transition_statuses_unchanged'])
print('queue_exhausted_count:', summary['queue_exhausted_count'])
print('expansion_budget_reached_count:', summary['expansion_budget_reached_count'])
print('failure_class_counts:', summary['failure_class_counts'])
print('classification_by_side:', summary['classification_by_side'])
print('classification_by_forward_audit:', summary['classification_by_forward_audit'])
print('expansion_histogram:', summary['expansion_histogram'])
print('top_rejection_causes:', summary['top_rejection_causes'])
print('closest_connectors:', summary['closest_connectors'])
PY
```

G3 requires:

- `r6b_connector_count == 40`;
- exactly 40 distinct connector IDs;
- every failed R6B result has one frozen failure class;
- `behavior_projection_equal == True`;
- `final_transition_statuses_unchanged == True`;
- post-instrumentation transitions remain `0 EXECUTABLE / 40 REJECTED`;
- protected runtime input hashes remain unchanged;
- forbidden semantic key count is zero.

---

# Task 1: Create the complete tests-only RED batch

**Files:**
- Create: `src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py`
- Modify: `src/agt_offline_assets/test/test_reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Create: `tests/test_v25_12g_a35_connector_diagnosis_contract.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

- [ ] **Step 1: Add RED-safe API discovery to the new pure diagnostic test file.**

Use this complete helper so missing production code fails as an assertion inside a test, not during collection:

```python
import pytest
import agt_offline_assets.reverse_primitive_connector as r6b


def _a35_api():
    diagnostics_type = getattr(r6b, "ReversePrimitiveSearchDiagnostics", None)
    classifier = getattr(r6b, "classify_reverse_primitive_failure", None)
    assert diagnostics_type is not None
    assert classifier is not None
    return diagnostics_type, classifier


def _diag(**overrides):
    diagnostics_type, _ = _a35_api()
    return diagnostics_type(**overrides)
```

Add `test_a35_diagnostic_schema_and_failure_classes_exist` asserting all nine class constants and `agt_r6b_connector_diagnostics/v1` exactly.

- [ ] **Step 2: Add synthetic classifier fixtures for all nine classes.**

Use this table as the minimum classifier matrix:

```python
@pytest.mark.parametrize(
    "diagnostics,expected",
    [
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_search_envelope=60,
                primitive_edges_enqueued=40,
            ),
            "SEARCH_ENVELOPE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_site_boundary=60,
                primitive_edges_enqueued=40,
            ),
            "SITE_BOUNDARY_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_navigation_grid=60,
                primitive_edges_enqueued=40,
            ),
            "FOOTPRINT_GRID_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_path_length=60,
                primitive_edges_enqueued=40,
            ),
            "PATH_OR_CUSP_ENVELOPE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=75,
                primitive_edges_rejected_state_dominance=65,
                primitive_edges_enqueued=10,
            ),
            "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED",
        ),
        (
            dict(
                best_goal_position_error_m=0.50,
                best_goal_yaw_error_rad=0.30,
                goal_shot_reverse_direction_blocked=4,
            ),
            "GOAL_CONNECTION_LIMITED",
        ),
        (
            dict(
                search_started=True,
                queue_exhausted=True,
                start_goal_position_error_m=10.0,
                best_goal_position_error_m=8.5,
                primitive_edges_considered=100,
                primitive_edges_rejected_state_dominance=20,
                primitive_edges_enqueued=80,
            ),
            "MOTION_PRIMITIVE_LIMITED",
        ),
        (
            dict(
                primitive_edges_considered=100,
                primitive_edges_rejected_search_envelope=30,
                primitive_edges_rejected_site_boundary=30,
                primitive_edges_enqueued=40,
            ),
            "MIXED_LIMITATION",
        ),
        (
            dict(
                search_started=True,
                queue_exhausted=True,
                start_goal_position_error_m=10.0,
                best_goal_position_error_m=5.0,
                primitive_edges_considered=100,
                primitive_edges_rejected_state_dominance=50,
                primitive_edges_enqueued=50,
            ),
            "INCONCLUSIVE",
        ),
    ],
)
def test_a35_classifier_uses_frozen_failure_classes(diagnostics, expected):
    diagnostics_type, classifier = _a35_api()
    item = diagnostics_type(**diagnostics)
    assert classifier(item, max_cusps=2) == expected
```

Add separate tests for exact threshold boundaries `0.50`, `0.25`, `0.60`, `0.15`, cusp `0.50`, and progress `0.20` so later refactors cannot silently change inclusivity.

- [ ] **Step 3: Add strict diagnostic record validation tests.**

Tests must reject:

- negative counters;
- invalid schema;
- invalid failure class;
- invalid direction outside `None/FORWARD/REVERSE`;
- NaN/inf/negative goal errors;
- both termination flags true;
- each of the four accounting identities being broken independently.

Add one valid round-trip test asserting `from_dict(to_dict(item)) == item`.

- [ ] **Step 4: Extend `test_reverse_primitive_connector.py` with real gate instrumentation tests.**

Add tests prefixed `test_a35_` for:

1. `_edge_rejection_reason` returning search envelope, Site Boundary, and Navigation Grid causes on real synthetic geometry;
2. `_edge_is_free` returning exactly the same Boolean truth value as `reason == EDGE_FREE` for those cases and one FREE case;
3. queue exhaustion using `max_path_length_m=0.10`, `max_expansions=50`;
4. expansion-budget termination using a far goal and `max_expansions=1`;
5. deterministic complete diagnostics for two identical derivations;
6. coherent best-goal state evidence;
7. all accounting identities on both a bounded failure and an executable reverse fixture;
8. pre-existing R6B status/path/sample semantics remaining unchanged in those fixtures.

For the path-limited queue-exhaustion fixture require:

```text
status == NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
queue_exhausted == True
expansion_budget_reached == False
primitive_edges_rejected_path_length > 0
```

For the one-expansion fixture require:

```text
status == NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
queue_exhausted == False
expansion_budget_reached == True
search_expansions == 1
```

- [ ] **Step 5: Extend A3 transition tests without changing existing test meaning.**

Modify the `_reverse_plan` fake helper so it accepts one explicit `diagnostics` argument with default `None` and includes `diagnostics=diagnostics or SimpleNamespace(...)` only in A3.5-prefixed tests. Existing non-A3.5 tests must continue to create the same old semantic fake results.

Add `test_a35_transition_preserves_r6b_diagnostics_without_changing_result` using the existing bounded-failure orchestration. Assert all of these together:

```text
status == REJECTED
proof_scope == BOUNDED_SEARCH_NO_SOLUTION
backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
backend_status == NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION
search_expansions == 321
reverse_backend_diagnostics.queue_exhausted == True
reverse_backend_diagnostics.failure_class == SEARCH_ENVELOPE_LIMITED
```

Add a matching R6B success test proving diagnostics can be attached while status remains `EXECUTABLE` and `failure_class` is null.

- [ ] **Step 6: Add A3 strict IO tests.**

Using the existing `_io_api()` and motion-graph fixture pattern, add:

- a non-empty diagnostic mapping serialize -> strict load -> deterministic rewrite test;
- a backward-compatibility test that deletes `reverse_backend_diagnostics` from one serialized transition before loading and requires `{}` after load;
- a forbidden-key test placing `route_ready` under `reverse_backend_diagnostics` and requiring strict loader rejection;
- a malformed diagnostic mapping test that breaks an accounting identity and requires loader rejection.

Keep `agt_vehicle_feasible_motion_graph/v1` unchanged in every fixture.

- [ ] **Step 7: Create the root diagnosis-harness contract test with a complete dynamic loader.**

Use:

```python
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a35_connector_diagnosis.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.5 diagnosis harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a35_connector_diagnosis",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
```

Required harness tests:

- exact report schema `agt_v25_12g_a35_connector_backend_diagnosis/v1`;
- exact validation scope `A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF`;
- parser requires run-dir/profile and defaults baseline filename correctly;
- preflight rejects missing baseline before derivation;
- read-only execution writes nothing to run-dir;
- behavior projection accepts graphs differing only by `reverse_backend_diagnostics`;
- behavior projection rejects a change to each of: transition status, backend status, search expansions, path length, samples;
- aggregation sorts connector IDs deterministically;
- closest connectors sort by position error, yaw error, connector ID;
- missing best-goal errors sort behind measured errors;
- forbidden readiness semantics fail closed;
- expansion histogram uses exactly these bins:

```text
0-9
10-99
100-499
500-999
1000-4999
5000-9999
10000-19999
20000-30000
```

- [ ] **Step 8: Register only the new package-level diagnostic test in CMake.**

Append inside `BUILD_TESTING`:

```cmake
ament_add_pytest_test(
  test_reverse_primitive_diagnostics
  test/test_reverse_primitive_diagnostics.py
)
```

Do not register the repository-root harness test in package CMake; run it explicitly like the existing A3 contract test.

- [ ] **Step 9: Run Gate G1 and stop until operator supplies valid RED.**

- [ ] **Step 10: After valid RED, commit only the tests/CMake files.**

```bash
git status --short

git add \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py \
  src/agt_offline_assets/CMakeLists.txt

git diff --cached --name-only

git commit -m "test(v25-12g): define A3.5 R6B diagnostic contract"
```

The staged list must not include either protected user path.

---

# Task 2: Implement the pure diagnostic model and classifier

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py` only to import/re-export the new diagnostic public API in this task.

- [ ] **Step 1: Implement all constants and the exact dataclass from the Frozen A3.5 Diagnostic Contract.**

- [ ] **Step 2: Implement strict `to_dict`/`from_dict`.**

`to_dict` must return every dataclass field with native bool/int/float/string/null types and deterministic insertion order.

`from_dict` must require the exact schema for non-empty mappings, validate all counters/errors/classes/direction values, enforce the four accounting identities, and enforce mutually exclusive termination flags.

- [ ] **Step 3: Implement the classifier exactly in the frozen order.**

No branch may read real connector identity. Keep threshold constants in this pure module and export them for tests.

- [ ] **Step 4: Import and re-export the diagnostic dataclass, constants, serializer helpers, and classifier from `reverse_primitive_connector.py`.**

Do not alter R6B search control flow in this task.

Do not request a manual operator test yet; remain inside the confirmed RED batch.

---

# Task 3: Instrument R6B without changing search behavior

**File:** `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`

- [ ] **Step 1: Add one private mutable diagnostic accumulator.**

The accumulator owns mutable counters and one best-state snapshot. Its finalizer receives the existing R6B status and `max_cusps`, invokes the pure classifier only for `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`, and returns immutable `ReversePrimitiveSearchDiagnostics`.

Never read accumulator values to choose planner branches.

- [ ] **Step 2: Implement `_edge_rejection_reason` and make `_edge_is_free` a wrapper.**

Use the exact rejection order from the frozen contract and retain the current function arguments. No extra collision/boundary sampling is permitted.

- [ ] **Step 3: Instrument start validation.**

Compute `start_goal_position_error_m` from request geometry. Preserve current returns:

```text
SITE_BOUNDARY_CONFLICT -> SITE_BOUNDARY_CONFLICT
non-FREE start footprint -> R6B_START_FOOTPRINT_NOT_FREE
```

Attach non-started diagnostics; do not enter the queue for these paths.

- [ ] **Step 4: Instrument pop/stale/expansion in the existing order.**

Keep:

```text
heappop -> stale check -> expansions += 1 -> goal check -> goal shot -> cusp -> primitives
```

Counters:

```text
nodes_popped += 1 immediately after heappop
stale_nodes_skipped += 1 only on the current stale continue
search_expansions += 1 at the current expansions increment location
```

Update the coherent best-goal snapshot only on non-stale expanded nodes.

- [ ] **Step 5: Instrument current goal-tolerance logic.**

Increment `goal_tolerance_checks` once per non-stale expansion. Increment `goal_tolerance_successes` only when the current FORWARD position+yaw goal branch succeeds. Do not enable reverse goal tolerance.

- [ ] **Step 6: Instrument `_try_forward_goal_shot`.**

Add the accumulator as one private parameter. Preserve all current early returns, Dubins candidate ordering, sampling, and success ordering.

For REVERSE nodes inside `goal_shot_distance_m`, increment only `goal_shot_reverse_direction_blocked` before returning the same `None` result.

For FORWARD nodes inside shot distance:

- increment `goal_shot_attempts` once;
- increment `goal_shot_candidates_considered` per candidate;
- count path-length rejection before sampling;
- use `_edge_rejection_reason` to increment exactly one envelope/boundary/grid rejection counter;
- increment success only for the candidate the existing code returns.

- [ ] **Step 7: Instrument cusp switching.**

When `node.cusp_count < max_cusps`, increment considered and then exactly one of dominance-rejected/enqueued based on the existing `best_cost` comparison. When already at max cusps, increment `nodes_at_max_cusps` once for the expanded node and do not create a new branch.

- [ ] **Step 8: Instrument primitive edges.**

For the existing node-level max-path test, record 3 considered + 3 path-length-rejected candidates, then keep the same `continue`.

Otherwise, before each frozen curvature candidate, increment considered. After integration:

- envelope failure -> envelope rejection counter;
- Site Boundary failure -> Site Boundary counter;
- Navigation Grid failure -> grid counter;
- current `best_cost` dominance failure -> state-dominance counter;
- current heap push -> enqueued counter.

Keep the current curvature tuple, state key, costs, heuristic, and heap priority untouched.

- [ ] **Step 9: Record exact termination mode.**

On bounded no-solution loop exit:

```python
queue_exhausted = not queue
expansion_budget_reached = bool(queue) and expansions >= cfg.max_expansions
```

Attach frozen diagnostics to the existing result. Successful results and pre-search hard failures keep `failure_class=None`.

- [ ] **Step 10: Add nested diagnostics to `reverse_primitive_connector_plan_to_dict`.**

Do not rename or remove any existing serialized R6B key.

---

# Task 4: Pass diagnostic evidence through A3 and strict v1 IO

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py`

- [ ] **Step 1: Append `reverse_backend_diagnostics` to `TransitionValidation`.**

Use the exact field declaration from the frozen contract.

- [ ] **Step 2: Extend `_base_result` with `reverse_backend_diagnostics=None`.**

Store it as `dict(reverse_backend_diagnostics or {})`.

Forward-only paths remain empty mappings.

- [ ] **Step 3: Pass R6B diagnostics on every R6B-backed A3 return branch.**

Immediately after `reverse = reverse_by_id[connector_id]`, convert `reverse.diagnostics` through the pure serializer. Pass that mapping on:

- R6B success;
- R6B Site Boundary hard conflict;
- R6B bounded no-solution;
- R6B start footprint non-free handling;
- unknown/future R6B status fallback.

Do not change status/proof/backend/backend-status/reason mapping.

- [ ] **Step 4: Serialize the new mapping after `reverse_admission_evidence`.**

Use `_plain(item.reverse_backend_diagnostics)`.

- [ ] **Step 5: Strict-load the mapping while preserving old v1 files.**

Use missing key -> `{}`. If the mapping is non-empty, pass it through `reverse_primitive_search_diagnostics_from_dict`, then reserialize that validated object to the canonical mapping stored in `TransitionValidation`.

Do not bump the motion-graph schema.

- [ ] **Step 6: Keep recursive forbidden-key rejection active on the new mapping.**

A readiness/optimality key nested anywhere inside diagnostics remains an error.

---

# Task 5: Add the read-only complete-40 diagnosis harness

**File:** `tools/v25_12g_a35_connector_diagnosis.py`

### Frozen public harness constants

```python
REPORT_SCHEMA = "agt_v25_12g_a35_connector_backend_diagnosis/v1"
VALIDATION_SCOPE = (
    "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
)
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_REAL_R6B_CONNECTOR_COUNT = 40
```

- [ ] **Step 1: Implement parser/preflight.**

Required arguments and defaults:

```text
--run-dir              required
--vehicle-profile      required
--baseline-motion-graph vehicle_feasible_motion_graph.yaml
--service-graph         vehicle_feasible_service_graph.yaml
--turn-zones            turn_zones.yaml
--navigation-map        navigation_map.yaml
--site-boundary         site_boundary.yaml
--pretty                false by default
```

There is no option to write/overwrite the motion graph in A3.5 v1.

- [ ] **Step 2: Hash protected runtime inputs before derivation.**

Hash exactly:

```text
vehicle_feasible_motion_graph.yaml
vehicle_feasible_service_graph.yaml
vehicle_feasible_segments.yaml
turn_zones.yaml
navigation_map.yaml
navigation_map.pgm
site_boundary.yaml
derivation.yaml
aisle_graph.yaml
```

Repeat hashing after derivation and fail if any digest differs.

- [ ] **Step 3: Load the baseline and derive the new graph only in memory.**

Load the frozen runtime baseline via `load_vehicle_feasible_motion_graph`. Load A2/zones/navigation/boundary/profile through the same APIs as `v25_12g_a3_acceptance.py`, then call normal `derive_vehicle_feasible_motion_graph`.

Do not write the new graph into the run directory.

- [ ] **Step 4: Implement behavior projection equality.**

Compare all service actions exactly. Compare these transition fields exactly, excluding only `reverse_backend_diagnostics`:

```text
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
reason
path_length_m
forward_distance_m
reverse_distance_m
cusp_count
search_expansions
samples
forward_evidence
reverse_admission_evidence
```

Also compare exactly:

```text
frame_id
platform_id
platform_profile_sha256
row_direction_xy
executable_service_action_ids
executable_transition_ids
MotionGraphDiagnostics
```

Do not compare top-level `source`. Any other mismatch raises `ValueError` before a success report can be printed.

- [ ] **Step 5: Require complete R6B diagnostic coverage on the frozen real dataset.**

Select new transitions whose backend is `BOUNDED_REVERSE_PRIMITIVE_SEARCH`. Require exactly 40 and 40 unique connector IDs. Require every one to carry `agt_r6b_connector_diagnostics/v1`. Require each `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION` to have one frozen failure class.

Keep the number 40 only in this acceptance harness, never in classifier logic.

- [ ] **Step 6: Emit one deterministic record per connector.**

Each record contains context:

```text
connector_candidate_id
from_segment_id
to_segment_id
side
turn_zone_id
forward_audit_status
r6a_decision
final_status
backend_status
```

and every field of `ReversePrimitiveSearchDiagnostics`. Sort records by connector ID.

- [ ] **Step 7: Emit deterministic aggregate diagnosis.**

Summary keys must include:

```text
r6b_connector_count
behavior_projection_equal
final_transition_statuses_unchanged
queue_exhausted_count
expansion_budget_reached_count
failure_class_counts
classification_by_side
classification_by_forward_audit
expansion_histogram
top_rejection_causes
closest_connectors
forbidden_semantic_key_count
protected_input_hashes_unchanged
```

`classification_by_side` contains LOW_U/HIGH_U -> failure class -> count.

`classification_by_forward_audit` contains original `forward_evidence.audit_status` -> failure class -> count.

Use exactly these expansion bins:

```text
0-9
10-99
100-499
500-999
1000-4999
5000-9999
10000-19999
20000-30000
```

Aggregate rejection causes with these exact names:

```text
SEARCH_ENVELOPE
SITE_BOUNDARY
NAVIGATION_GRID
PATH_LENGTH
STATE_DOMINANCE
CUSP_STATE_DOMINANCE
```

The first four sum corresponding primitive and goal-shot counters. State dominance uses `primitive_edges_rejected_state_dominance`; cusp dominance uses `cusp_switches_rejected_state_dominance`. Sort by descending count, then cause name.

`closest_connectors` contains up to 10 failures sorted by `(best_goal_position_error_m, best_goal_yaw_error_rad, connector_candidate_id)`, with missing numeric values treated as positive infinity.

- [ ] **Step 8: Enforce first-stage diagnostic-only final outcome.**

Require the new graph to retain:

```text
transition_validation_count == 40
executable_transition_count == 0
rejected_transition_count == 40
```

This guard is specific to the first diagnostic run and must be deleted/revised only in a later approved behavior-changing stage.

- [ ] **Step 9: Keep recommendations out of the harness.**

The program reports evidence only. It must not auto-recommend higher budgets, smaller footprints, softer boundaries, smaller turning radius, wider envelopes, different primitives, or coarser/finer discretization.

---

# Task 6: Execute batched GREEN verification and commit diagnostic-only production code

**Files:** production files from Tasks 2-5 plus the already committed RED tests.

- [ ] **Step 1: Run Gate G2 and stop if any focused/package test fails.**

- [ ] **Step 2: Review planner diff for behavior changes before commits.**

```bash
git diff -- \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py \
  tools/v25_12g_a35_connector_diagnosis.py
```

Reject any diff changing:

- `_state_key` math/resolution;
- `_heuristic` formula;
- curvature family `(-1/R, 0, +1/R)`;
- heap priority/cost formula;
- motion/reverse/cusp/steering costs;
- goal tolerances;
- forward-only goal/goal-shot rules;
- search budgets/defaults;
- Site Boundary or Navigation Grid acceptance.

Only evidence counters, coherent best-state bookkeeping, equivalent reason extraction, result evidence fields, IO, and report code are permitted.

- [ ] **Step 3: Check user files remain untouched.**

```bash
git status --short
```

The pre-existing workspace may still show:

```text
 M tools/rosbag_sensor_trimmer
?? tools/map_tools/render_pcd_top_views.py
```

Do not stage either.

- [ ] **Step 4: Commit core R6B instrumentation after G2 is green.**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py

git diff --cached --name-only

git commit -m "feat(v25-12g): instrument R6B diagnostic funnel"
```

- [ ] **Step 5: Commit A3 diagnostic evidence passthrough/IO.**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py

git diff --cached --name-only

git commit -m "feat(v25-12g): persist A3.5 connector diagnostics"
```

- [ ] **Step 6: Commit the read-only diagnosis harness.**

```bash
git add tools/v25_12g_a35_connector_diagnosis.py

git diff --cached --name-only

git commit -m "feat(v25-12g): add A3.5 connector diagnosis harness"
```

Never use `git add .` or `git add -A`.

---

# Task 7: Run the full real-data diagnosis and freeze evidence

**Files:**
- Create only after G3 evidence exists: `docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md`
- Runtime files remain read-only and uncommitted.

- [ ] **Step 1: Run Gate G3 exactly once and keep its JSON/timing files in `/tmp`.**

Do not rerun the expensive all-40 search solely for formatting.

- [ ] **Step 2: Validate captured evidence without rerunning R6B.**

```bash
python3 - <<'PY'
import json
from pathlib import Path

report = json.loads(Path('/tmp/v25_12g_a35_connector_diagnosis.json').read_text())
items = report['connectors']
summary = report['summary']
assert len(items) == 40
assert len({item['connector_candidate_id'] for item in items}) == 40
assert all(item['failure_class'] for item in items)
assert all(
    not (item['queue_exhausted'] and item['expansion_budget_reached'])
    for item in items
)
assert summary['behavior_projection_equal'] is True
assert summary['final_transition_statuses_unchanged'] is True
assert summary['protected_input_hashes_unchanged'] is True
assert summary['forbidden_semantic_key_count'] == 0
print('A3.5 diagnostic invariants: PASS')
PY
```

- [ ] **Step 3: Freeze actual evidence in `V25_12G_A35_DIAGNOSIS_2026-08-17.md`.**

Populate from G2/G3 output only. Include:

1. exact branch and diagnostic HEAD;
2. focused/package test result from G2;
3. real run-dir and profile paths;
4. pre-A3.5 baseline transition funnel;
5. post-instrumentation transition funnel and behavior-projection equality;
6. queue-exhausted vs expansion-budget counts;
7. overall failure-class counts;
8. failure classes by LOW_U/HIGH_U;
9. failure classes by original forward audit;
10. expansion histogram;
11. top rejection causes;
12. closest 10 connectors with best position/yaw/direction/cusp evidence;
13. one 40-row table containing connector ID, side, forward audit, R6A decision, expansions, termination mode, failure class, best position error, best yaw error, best direction, and best cusp count;
14. protected-input hash result;
15. wall/user/system time and maximum RSS from `/tmp/v25_12g_a35_connector_diagnosis.time`;
16. explicit statement that no planner behavior was amended and A4 remains blocked.

Do not include a backend fix recommendation in this evidence-freeze document.

- [ ] **Step 4: Commit only the evidence document.**

```bash
git add docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md
git diff --cached --name-only
git commit -m "docs(v25-12g): freeze A3.5 connector diagnosis evidence"
```

- [ ] **Step 5: Final workspace check.**

```bash
git status --short
git log --oneline -8
```

Verify the two protected user paths retain their pre-A3.5 state/content.

---

# Task 8: Stop before backend amendment and hand evidence to brainstorming

**Files:** none.

- [ ] **Step 1: Summarize evidence only.**

Summarize dominant classes, side/audit differences, queue exhaustion vs budget saturation, strongest rejection causes, closest-to-goal subset, and whether evidence supports or rejects `max_expansions` as the first hypothesis.

- [ ] **Step 2: Only after Task 7 is frozen, invoke `superpowers:brainstorming` for the minimum backend amendment.**

That behavior-changing stage requires its own approved spec and strict RED -> GREEN plan.

- [ ] **Step 3: Carry forward the hard prohibitions.**

Do not obtain connectivity by:

- relaxing/removing Site Boundary;
- relaxing Navigation Grid FREE-only checks;
- shrinking the footprint;
- reducing `0.05 m` padding;
- reducing `1.50 m` minimum turning radius;
- blindly increasing `max_expansions=30000`.

Only backend structures directly implicated by frozen diagnostics may enter the next brainstorming stage, such as search-envelope geometry, primitive family, state discretization/dominance, goal connection, or path/cusp envelope.

---

## Plan Self-Review Checklist

- [ ] Diagnostic-only through real-data evidence freeze; no amendment code in this plan.
- [ ] Original checkout only; no worktree.
- [ ] Strict tests-only RED before production code.
- [ ] Only three mandatory operator gates: G1 RED, G2 GREEN/package, G3 real data.
- [ ] Every approved A3.5 minimum diagnostic field is represented.
- [ ] Queue exhaustion and expansion-budget termination are distinct.
- [ ] Envelope, Site Boundary, Navigation Grid, path length, state dominance, cusp, goal tolerance, and goal shot have separated evidence.
- [ ] Best-goal errors/direction/cusp come from one coherent state.
- [ ] Failure classes use explicit frozen thresholds and no real connector identities.
- [ ] `max_expansions=30000` is unchanged and not assumed causal.
- [ ] Site Boundary, Navigation Grid, footprint, `0.05 m` padding, and `1.50 m` turning radius remain hard/frozen.
- [ ] A3 `status/proof_scope/backend/backend_status/reason/search_expansions/samples` semantics remain unchanged.
- [ ] Motion-graph schema remains `agt_vehicle_feasible_motion_graph/v1` and old runtime YAML remains loadable.
- [ ] All-40 harness compares old/new behavior while excluding only the new diagnostic evidence field and top-level source metadata.
- [ ] Harness writes no runtime planner asset.
- [ ] No readiness/reachability/optimality/global-infeasibility claim is introduced.
- [ ] A4 remains blocked until a later stage obtains the required 3-segment/2-aisle/2-transition real chain.
- [ ] `tools/rosbag_sensor_trimmer` and `tools/map_tools/render_pcd_top_views.py` are never staged or modified.
