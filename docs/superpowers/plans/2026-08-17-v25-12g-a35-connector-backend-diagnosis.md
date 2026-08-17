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
  reverse_primitive_diagnostics.py              # CREATE: diagnostic schema, validation, classifier
  reverse_primitive_connector.py                # MODIFY: instrumentation only; planner behavior frozen
  vehicle_feasible_motion_graph.py              # MODIFY: append diagnostic evidence field
  vehicle_feasible_transition_motion.py         # MODIFY: pass R6B diagnostic evidence through A3
  vehicle_feasible_motion_graph_io.py            # MODIFY: strict optional round-trip of diagnostics

src/agt_offline_assets/test/
  test_reverse_primitive_diagnostics.py          # CREATE: pure classifier/contract RED tests
  test_reverse_primitive_connector.py            # MODIFY: funnel accounting + gate-equivalence tests
  test_vehicle_feasible_motion_graph.py          # MODIFY: A3 passthrough + IO/backward-compat tests

src/agt_offline_assets/CMakeLists.txt            # MODIFY: register diagnostic package test

tools/
  v25_12g_a35_connector_diagnosis.py             # CREATE: read-only all-40 diagnosis harness

tests/
  test_v25_12g_a35_connector_diagnosis_contract.py # CREATE: harness/behavior-projection contract

docs/v2.5/
  V25_12G_A35_DIAGNOSIS_2026-08-17.md            # CREATE only after real-data evidence exists
```

Do not add a second planner or a duplicate R6B implementation. Diagnostic logic belongs beside the existing backend and must observe the existing control flow.

---

## Frozen A3.5 Diagnostic Contract

### Failure classes

Use exactly these public values:

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
```

Diagnostic schema:

```python
R6B_DIAGNOSTIC_SCHEMA = "agt_r6b_connector_diagnostics/v1"
```

### Per-connector diagnostic fields

`ReversePrimitiveSearchDiagnostics` must contain at least the following fields and preserve deterministic types:

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

The R6B result receives one additional evidence field only:

```python
@dataclass(frozen=True)
class ReversePrimitiveConnectorResult:
    # existing fields unchanged and in the existing order
    ...
    reason: str = ""
    diagnostics: ReversePrimitiveSearchDiagnostics = field(
        default_factory=ReversePrimitiveSearchDiagnostics
    )
```

The A3 transition contract receives one appended optional mapping so existing call sites remain source-compatible:

```python
@dataclass(frozen=True)
class TransitionValidation:
    # existing fields unchanged
    ...
    forward_evidence: Mapping[str, Any]
    reverse_admission_evidence: Mapping[str, Any]
    reverse_backend_diagnostics: Mapping[str, Any] = field(default_factory=dict)
```

Do not change `status`, `proof_scope`, `backend`, `backend_status`, `reason`, `search_expansions`, `samples`, or executable-ID semantics to accommodate diagnostics.

### Counter accounting invariants

The implementation must make the following identities true for every completed R6B diagnostic record:

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

Additional invariants:

- `queue_exhausted` and `expansion_budget_reached` are mutually exclusive.
- For an unsuccessful search, set `queue_exhausted=True` when the queue is empty after loop exit.
- For an unsuccessful search, set `expansion_budget_reached=True` only when work remains in the queue and `search_expansions >= max_expansions`.
- If the queue becomes empty exactly when the expansion count reaches the configured limit, classify termination as queue exhaustion, not budget saturation.
- Start-footprint hard failures do not start the bounded search and therefore leave both termination flags false.
- `goal_tolerance_checks` increments once for every non-stale expanded state. Existing success semantics remain: only a FORWARD state satisfying both frozen position and yaw tolerances succeeds.
- A reverse state inside `goal_shot_distance_m` increments `goal_shot_reverse_direction_blocked`; it does not cause a new goal-shot attempt and does not change current planner behavior.
- `goal_shot_attempts` increments only for a FORWARD expanded state inside the frozen `3.0 m` shot radius.
- Each goal-shot Dubins candidate is counted exactly once in one terminal category: path length, search envelope, Site Boundary, Navigation Grid, or success.
- Each primitive curvature candidate is counted exactly once in one terminal category. When `node.travel_m + primitive_length_m > max_path_length_m`, count all three frozen curvature candidates as path-length rejections and preserve the existing branch that skips primitive integration for that node.

### Rejection precedence

Replace the internal Boolean-only edge probe with a reason-returning helper while retaining `_edge_is_free` as a compatibility wrapper:

```python
EDGE_FREE = "FREE"
EDGE_SEARCH_ENVELOPE = "SEARCH_ENVELOPE"
EDGE_SITE_BOUNDARY = "SITE_BOUNDARY"
EDGE_NAVIGATION_GRID = "NAVIGATION_GRID"


def _edge_rejection_reason(...):
    for sample in samples:
        if not _inside_search_envelope(...):
            return EDGE_SEARCH_ENVELOPE
        pose_status = _preview_pose_status(...)
        if pose_status == "SITE_BOUNDARY_CONFLICT":
            return EDGE_SITE_BOUNDARY
        if pose_status != "FREE":
            return EDGE_NAVIGATION_GRID
    return EDGE_FREE


def _edge_is_free(...):
    return _edge_rejection_reason(...) == EDGE_FREE
```

This precedence is frozen because it matches the current executable control flow: envelope first, then `_preview_pose_status`, whose Site Boundary check precedes Navigation Grid occupancy/free-space checks.

### Best-goal state semantics

Track one coherent best state, not separate minima from unrelated states.

For each non-stale popped state, compute `(position_error_m, yaw_error_rad)`. Update the best-state snapshot when:

1. position error is smaller by more than `1e-12`; or
2. position error differs by at most `1e-12` and yaw error is smaller by more than `1e-12`.

If both are tied within `1e-12`, retain the earlier popped state. The existing heap counter already makes pop order deterministic. Store the chosen state's direction and cusp count together with the two errors.

### Frozen diagnostic classification thresholds

These values are diagnostic semantics only. They must never feed planner costs, acceptance, pruning, or queue ordering.

```python
DOMINANT_REJECTION_FRACTION = 0.50
MIXED_REJECTION_FRACTION = 0.25
DOMINANCE_REJECTION_FRACTION = 0.60
DOMINANCE_ENQUEUED_FRACTION_MAX = 0.15
CUSP_SATURATION_FRACTION = 0.50
LITTLE_PROGRESS_FRACTION = 0.20
```

Compute combined generated-geometry denominator:

```text
geometry_candidate_count
  = primitive_edges_considered
  + goal_shot_candidates_considered
```

For the four geometric/path causes, use:

```text
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

and divide each by `max(geometry_candidate_count, 1)`.

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

Clamp the value to `[0.0, 1.0]` for reporting only.

Motion-primitive signal is intentionally conservative:

```text
queue_exhausted
AND primitive_edges_considered > 0
AND each geometric/path rejection fraction < 0.25
AND primitive_edges_rejected_state_dominance
    / max(primitive_edges_considered, 1) < 0.25
AND primitive_edges_enqueued
    / max(primitive_edges_considered, 1) >= 0.25
AND little_progress_fraction <= 0.20
AND no goal-connection signal
AND no cusp-saturation signal
```

Classification order is frozen as follows:

1. Apply the classifier only to `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`. Other R6B outcomes keep `failure_class=None`.
2. If two or more geometric/path causes are each `>=0.50`, return `MIXED_LIMITATION`.
3. If exactly one geometric/path cause is `>=0.50`, return its direct class:
   - search envelope -> `SEARCH_ENVELOPE_LIMITED`
   - Site Boundary -> `SITE_BOUNDARY_LIMITED`
   - Navigation Grid -> `FOOTPRINT_GRID_LIMITED`
   - path length -> `PATH_OR_CUSP_ENVELOPE_LIMITED`
4. When no direct cause reaches `0.50`, build a set of moderate/structural signals:
   - each geometric/path cause `>=0.25` contributes its mapped class;
   - state-dominance contributes `STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED`;
   - cusp saturation contributes `PATH_OR_CUSP_ENVELOPE_LIMITED`;
   - goal connection contributes `GOAL_CONNECTION_LIMITED`.
5. If two or more distinct classes are present in that signal set, return `MIXED_LIMITATION`.
6. If exactly one structural signal from state dominance, cusp saturation, or goal connection is present, return that class. A lone `0.25 <= geometric/path fraction < 0.50` is not strong enough by itself and falls through.
7. If the conservative motion-primitive signal is true, return `MOTION_PRIMITIVE_LIMITED`.
8. Otherwise return `INCONCLUSIVE`.

`expansion_budget_reached` is deliberately **not** mapped to a failure class. It is reported independently. A budget hit without another dominant cause remains `INCONCLUSIVE`; this prevents the diagnostic layer from turning a budget observation into an unsupported recommendation to raise `max_expansions`.

---

## Operator Gate Batching

Use only these mandatory manual gates during execution:

### Gate G1 — focused tests-only RED

Run after all A3.5 diagnostic tests exist but before any production implementation exists.

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

Expected RED characteristics:

- tests are collected successfully;
- failures are assertions about missing A3.5 diagnostic API/field/harness behavior;
- there is no import/collection failure caused by a typo;
- existing non-A3.5 tests are not expected to fail.

Do not edit production code until the operator supplies this RED output.

### Gate G2 — focused GREEN + ROS 2 package regression

Run once all diagnostic-only production changes are implemented.

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

Required result before proceeding: focused A3.5 tests green and the package suite reports zero errors/failures. Do not infer success from build completion alone.

### Gate G3 — complete 40-connector real-data diagnosis

Run only after G2 is green.

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

Then inspect the machine-readable invariants without rerunning R6B:

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

G3 passes only if:

- `r6b_connector_count == 40`;
- all 40 have non-empty valid diagnostic records;
- all 40 failed R6B records have one of the nine frozen failure classes;
- `behavior_projection_equal == True`;
- `final_transition_statuses_unchanged == True`;
- the pre-A3.5 baseline remains `40` rejected, `0` executable transitions;
- no protected upstream asset hash changes;
- no forbidden readiness/optimality semantic key appears;
- `expansion_budget_reached_count` is reported from evidence rather than assumed.

---

# Task 1: Freeze the A3.5 diagnostic contract in a tests-only RED batch

**Files:**
- Create: `src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py`
- Modify: `src/agt_offline_assets/test/test_reverse_primitive_connector.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Create: `tests/test_v25_12g_a35_connector_diagnosis_contract.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces under test:**
- `agt_offline_assets.reverse_primitive_connector.ReversePrimitiveSearchDiagnostics`
- `agt_offline_assets.reverse_primitive_connector.classify_reverse_primitive_failure`
- `agt_offline_assets.reverse_primitive_connector._edge_rejection_reason`
- `ReversePrimitiveConnectorResult.diagnostics`
- `TransitionValidation.reverse_backend_diagnostics`
- `tools/v25_12g_a35_connector_diagnosis.py`

- [ ] **Step 1: Add RED-safe diagnostic API discovery tests.**

Do not import a not-yet-created module directly at collection time. Import the existing R6B module and assert the new API exists inside the test body so RED is an assertion failure, not a collection error:

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


def test_a35_diagnostic_schema_and_failure_classes_exist():
    diagnostics_type, _ = _a35_api()
    item = diagnostics_type()
    assert item.schema == "agt_r6b_connector_diagnostics/v1"
    assert r6b.SEARCH_ENVELOPE_LIMITED == "SEARCH_ENVELOPE_LIMITED"
    assert r6b.SITE_BOUNDARY_LIMITED == "SITE_BOUNDARY_LIMITED"
    assert r6b.FOOTPRINT_GRID_LIMITED == "FOOTPRINT_GRID_LIMITED"
    assert (
        r6b.STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED
        == "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED"
    )
    assert r6b.GOAL_CONNECTION_LIMITED == "GOAL_CONNECTION_LIMITED"
    assert r6b.MOTION_PRIMITIVE_LIMITED == "MOTION_PRIMITIVE_LIMITED"
    assert (
        r6b.PATH_OR_CUSP_ENVELOPE_LIMITED
        == "PATH_OR_CUSP_ENVELOPE_LIMITED"
    )
    assert r6b.MIXED_LIMITATION == "MIXED_LIMITATION"
    assert r6b.INCONCLUSIVE == "INCONCLUSIVE"
```

- [ ] **Step 2: Add synthetic classifier tests for all nine classes and the frozen thresholds.**

Use direct synthetic counters; no real connector IDs and no real-map counts. Every test must validate one explicit threshold boundary. Required cases:

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
def test_a35_classifier_uses_frozen_thresholds(diagnostics, expected):
    diagnostics_type, classifier = _a35_api()
    item = diagnostics_type(**diagnostics)
    assert classifier(item, max_cusps=2) == expected
```

Also add exact-boundary tests proving `0.50`, `0.25`, `0.60`, `0.15`, `0.50`, and `0.20` use the intended inclusive/exclusive comparisons.

- [ ] **Step 3: Extend R6B integration tests with reason separation and accounting invariants.**

Add `test_a35_edge_reason_preserves_boolean_gate_equivalence` to the existing R6B fixture file. Construct three real geometric cases using the existing `_request`, `_zones`, `_grid`, and `_vehicle` helpers:

1. sample outside the row-frame search envelope -> `SEARCH_ENVELOPE`;
2. sample inside the envelope but footprint crossing a supplied `SiteBoundary` -> `SITE_BOUNDARY`;
3. sample inside envelope/boundary over OCCUPIED Navigation Grid -> `NAVIGATION_GRID`.

For each case assert:

```python
assert r6b._edge_is_free(...) == (
    r6b._edge_rejection_reason(...) == r6b.EDGE_FREE
)
```

Add `test_a35_queue_exhaustion_and_path_length_funnel_are_distinct` using `max_path_length_m=0.10`, `max_expansions=50`; require `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`, `queue_exhausted=True`, `expansion_budget_reached=False`, non-zero path-length rejection, and all accounting identities.

Add `test_a35_expansion_budget_termination_is_reported_without_changing_status` using a far goal and `max_expansions=1`; require the same final R6B failure status, `queue_exhausted=False`, `expansion_budget_reached=True`, and `search_expansions==1`.

Add `test_a35_best_goal_snapshot_and_diagnostics_are_deterministic`; derive the same synthetic request twice and assert the complete `diagnostics` objects are equal and that direction/cusp/error fields belong to one coherent best state.

- [ ] **Step 4: Add A3 passthrough and strict IO RED tests.**

Extend `_reverse_plan` in `test_vehicle_feasible_motion_graph.py` so its fake R6B result can carry a synthetic diagnostic mapping/object. Add:

```python
def test_a35_transition_preserves_r6b_diagnostics_without_changing_result(...):
    ...
    assert result.status == REJECTED
    assert result.proof_scope == BOUNDED_SEARCH_NO_SOLUTION
    assert result.backend_status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert result.search_expansions == 321
    assert result.reverse_backend_diagnostics["queue_exhausted"] is True
    assert result.reverse_backend_diagnostics["failure_class"] == "SEARCH_ENVELOPE_LIMITED"
```

Add an IO test that serializes, strict-loads, and rewrites a graph with non-empty `reverse_backend_diagnostics`, asserting byte-stability and exact mapping preservation. Add a backward-compatibility test that removes `reverse_backend_diagnostics` from serialized transition YAML before loading and requires the loaded transition to expose `{}`. This keeps the existing v1 runtime baseline readable.

- [ ] **Step 5: Add the A3.5 harness RED contract.**

Follow the dynamic-loader pattern already used by `tests/test_v25_12g_a3_contract.py`:

```python
ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a35_connector_diagnosis.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.5 diagnosis harness: {HARNESS}"
    ...
```

Required contract tests:

- parser requires `--run-dir` and `--vehicle-profile`;
- default baseline filename is `vehicle_feasible_motion_graph.yaml`;
- report schema is exactly `agt_v25_12g_a35_connector_backend_diagnosis/v1`;
- validation scope contains diagnostic-only semantics and no route-ready semantics;
- read-only execution does not write or mutate run-dir inputs;
- behavior projection ignores only `reverse_backend_diagnostics` and accepts an otherwise identical baseline/new graph;
- changing one transition `status`, `backend_status`, `search_expansions`, path metric, or sample causes a `ValueError` before report success;
- aggregation sorts connector IDs deterministically;
- closest-connector ranking is deterministic by `(best_goal_position_error_m, best_goal_yaw_error_rad, connector_candidate_id)`;
- expansion histogram uses the exact frozen bins below;
- forbidden readiness/optimality keys fail closed.

Frozen expansion histogram bins:

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

- [ ] **Step 6: Register only the new package-level diagnostic test.**

Append to `src/agt_offline_assets/CMakeLists.txt` inside `BUILD_TESTING`:

```cmake
ament_add_pytest_test(
  test_reverse_primitive_diagnostics
  test/test_reverse_primitive_diagnostics.py
)
```

Do not add the repository-root harness contract to the package CMake test list; keep it explicitly runnable like the existing A3 root contract.

- [ ] **Step 7: Run operator Gate G1 and stop for RED evidence.**

Use the exact G1 command above. Verify failures are feature-absence assertions, not collection errors.

- [ ] **Step 8: After G1 is confirmed, commit the tests-only RED contract explicitly.**

First inspect workspace state:

```bash
git status --short
```

Then stage only the named test/CMake paths:

```bash
git add \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py \
  src/agt_offline_assets/CMakeLists.txt

git diff --cached --name-only
```

The staged file list must not contain either protected user path.

Commit:

```bash
git commit -m "test(v25-12g): define A3.5 R6B diagnostic contract"
```

---

# Task 2: Implement the pure diagnostic model, validation, and classifier

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py` only to import/re-export the public diagnostic symbols; do not instrument search yet.

- [ ] **Step 1: Implement the exact schema, class constants, threshold constants, and frozen dataclass from this plan.**

Keep the diagnostic module free of Navigation Grid, Site Boundary, planner queue, and A3 graph dependencies. It must be pure data/validation/classification logic.

- [ ] **Step 2: Add strict diagnostic serialization helpers.**

Provide:

```python
def reverse_primitive_search_diagnostics_to_dict(
    diagnostics: ReversePrimitiveSearchDiagnostics,
) -> dict[str, Any]: ...


def reverse_primitive_search_diagnostics_from_dict(
    raw: Mapping[str, Any],
) -> ReversePrimitiveSearchDiagnostics: ...
```

The loader must:

- require the exact schema when the mapping is non-empty;
- reject negative counters;
- reject invalid direction values outside `{None, "FORWARD", "REVERSE"}`;
- reject an unknown non-null failure class;
- validate finite/non-negative goal distances and yaw errors;
- enforce the four accounting identities;
- enforce mutually exclusive termination flags.

An empty mapping is handled by the A3 IO layer as “pre-A3.5 evidence absent” and must not be passed to the strict diagnostic loader.

- [ ] **Step 3: Implement `classify_reverse_primitive_failure` exactly from the frozen threshold/order section.**

Signature:

```python
def classify_reverse_primitive_failure(
    diagnostics: ReversePrimitiveSearchDiagnostics,
    *,
    max_cusps: int,
) -> str:
    ...
```

Reject `max_cusps < 1`. Do not read connector IDs, side, aisle IDs, or map-specific counts.

- [ ] **Step 4: Re-export the diagnostic API from `reverse_primitive_connector.py`.**

Import the constants, dataclass, classifier, and serialization helper into the existing module namespace. This satisfies the RED-safe tests without changing planner behavior.

Do not run a separate operator gate here; this task remains inside the already-confirmed RED batch.

---

# Task 3: Instrument R6B without changing search behavior

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`

- [ ] **Step 1: Add a private mutable accumulator that freezes to the public immutable diagnostic dataclass.**

The accumulator is internal implementation state. It may use mutable integer fields and one best-state snapshot. It must expose one `freeze(...)` method that computes the final `failure_class` only after search termination and returns `ReversePrimitiveSearchDiagnostics`.

Do not expose accumulator state to the planner heuristic or branch conditions.

- [ ] **Step 2: Replace Boolean-only edge inspection with `_edge_rejection_reason`, retaining `_edge_is_free` as an exact wrapper.**

Use the frozen precedence above. The new reason helper is allowed to explain a failure; it is not allowed to decide a different set of legal samples.

- [ ] **Step 3: Instrument the start gate without changing the existing returned status.**

Compute `start_goal_position_error_m` from request geometry. If start status is `SITE_BOUNDARY_CONFLICT`, preserve the current `SITE_BOUNDARY_CONFLICT` result and attach a diagnostic record with `search_started=False`. If start status is non-FREE Navigation Grid evidence, preserve `R6B_START_FOOTPRINT_NOT_FREE` and attach the same non-started diagnostic shape.

- [ ] **Step 4: Instrument queue pop/stale/expansion and best-goal evidence.**

Preserve this existing order:

```text
heappop -> stale check -> expansions += 1 -> goal check -> goal shot -> cusp -> primitives
```

Counter semantics:

```text
nodes_popped += 1 immediately after heappop
stale_nodes_skipped += 1 only on the existing stale continue
search_expansions += 1 exactly where the existing expansions counter increments
best-goal snapshot updates only for non-stale expanded states
```

Keep the existing local `expansions` value or replace it with the accumulator's identical count, but do not increment in a different location.

- [ ] **Step 5: Instrument goal tolerance checks.**

Increment `goal_tolerance_checks` for every non-stale expansion before the current FORWARD/tolerance condition. Increment `goal_tolerance_successes` only when the current success branch fires. Do not permit reverse tolerance success.

- [ ] **Step 6: Instrument forward goal-shot attempts and candidate failures.**

Change the private `_try_forward_goal_shot` signature to accept the accumulator. Preserve candidate generation and iteration order.

Before returning early for reverse direction, if the state is within `goal_shot_distance_m`, increment `goal_shot_reverse_direction_blocked`.

For a FORWARD state within the shot radius:

- increment `goal_shot_attempts` once;
- increment `goal_shot_candidates_considered` once per `_dubins_candidates` item;
- count path-length overflow before sampling acceptance;
- use `_edge_rejection_reason` and map the first failure reason to its exact counter;
- increment `goal_shot_successes` only for the candidate currently returned as success.

No diagnostic counter may cause the loop to continue/break differently from the current code.

- [ ] **Step 7: Instrument cusp transitions.**

When `node.cusp_count < cfg.max_cusps`, increment `cusp_switches_considered`. Keep the exact existing state key and cost comparison. Increment exactly one of:

```text
cusp_switches_rejected_state_dominance
cusp_switches_enqueued
```

When a non-stale expanded node already has `cusp_count >= cfg.max_cusps`, increment `nodes_at_max_cusps` once for that node. Do not add a new cusp branch.

- [ ] **Step 8: Instrument primitive candidates.**

For every non-stale expanded node:

- if `node.travel_m + cfg.primitive_length_m > cfg.max_path_length_m`, add `3` to `primitive_edges_considered` and `3` to `primitive_edges_rejected_path_length`, then execute the existing `continue`;
- otherwise increment `primitive_edges_considered` once before each of the three frozen curvatures;
- use `_edge_rejection_reason` instead of `_edge_is_free` to select the matching rejection counter;
- preserve the current state discretization key and best-cost comparison;
- if the child fails that comparison, increment `primitive_edges_rejected_state_dominance`;
- if the child is pushed, increment `primitive_edges_enqueued`.

- [ ] **Step 9: Freeze termination flags and attach diagnostics to every R6B result.**

For `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`:

```python
queue_exhausted = not queue
expansion_budget_reached = bool(queue) and expansions >= cfg.max_expansions
```

Call the pure classifier only for this status. For successful paths and pre-search hard failures, keep `failure_class=None`.

The existing result fields remain the source of planner behavior; diagnostics are additional evidence only.

- [ ] **Step 10: Extend `reverse_primitive_connector_plan_to_dict` with one nested `diagnostics` mapping.**

Do not rename or remove any existing R6B serialized key. The diagnostic mapping is deterministic and follows dataclass field order.

Do not run an operator gate yet; continue to Task 4 against the same RED batch.

---

# Task 4: Pass R6B diagnostics through A3 and strict v1 IO

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py`

- [ ] **Step 1: Append `reverse_backend_diagnostics` to `TransitionValidation`.**

Use:

```python
reverse_backend_diagnostics: Mapping[str, Any] = field(default_factory=dict)
```

Append it after existing fields so existing constructors that do not provide it remain valid.

- [ ] **Step 2: Extend `_base_result` with an optional diagnostic mapping.**

Add:

```python
reverse_backend_diagnostics=None
```

and construct:

```python
reverse_backend_diagnostics=dict(reverse_backend_diagnostics or {})
```

Forward-only transitions retain `{}`.

- [ ] **Step 3: Pass diagnostic evidence for every R6B-backed branch.**

Immediately after retrieving `reverse = reverse_by_id[connector_id]`, convert its diagnostic dataclass with the pure serialization helper. Pass the resulting mapping into `_base_result` for:

- R6B success;
- `SITE_BOUNDARY_CONFLICT`;
- `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`;
- `R6B_START_FOOTPRINT_NOT_FREE`;
- unknown/future R6B status fallback.

Do not change any branch's existing A3 `status`, `proof_scope`, `backend`, `backend_status`, or `reason` logic.

- [ ] **Step 4: Extend A3 serializer with the nested diagnostic mapping.**

In `_transition_to_dict` add:

```python
"reverse_backend_diagnostics": _plain(item.reverse_backend_diagnostics),
```

Keep existing field ordering stable; add this key after `reverse_admission_evidence`.

- [ ] **Step 5: Extend strict loader while remaining backward compatible with the frozen pre-A3.5 v1 runtime asset.**

In `_load_transition`:

```python
raw_diagnostics = dict(
    _mapping(
        data.get("reverse_backend_diagnostics", {}),
        "reverse_backend_diagnostics",
    )
)
```

If non-empty, validate it through `reverse_primitive_search_diagnostics_from_dict` and then store the canonical serialized mapping. If empty/missing, store `{}`.

Do not bump `agt_vehicle_feasible_motion_graph/v1`; A3.5 adds optional diagnostic evidence, not a new planner contract.

- [ ] **Step 6: Keep recursive forbidden-key rejection active inside diagnostics.**

Do not exempt the new mapping from `_reject_forbidden`. A forbidden readiness/optimality key inside diagnostic evidence must still fail strict loading.

Continue to Task 5 before requesting the GREEN gate.

---

# Task 5: Add the read-only all-40 A3.5 diagnosis harness

**Files:**
- Create: `tools/v25_12g_a35_connector_diagnosis.py`

**Public constants:**

```python
REPORT_SCHEMA = "agt_v25_12g_a35_connector_backend_diagnosis/v1"
VALIDATION_SCOPE = (
    "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
)
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_REAL_R6B_CONNECTOR_COUNT = 40
```

- [ ] **Step 1: Implement preflight and read-only loading.**

Required parser arguments:

```text
--run-dir
--vehicle-profile
--baseline-motion-graph   default vehicle_feasible_motion_graph.yaml
--service-graph           default vehicle_feasible_service_graph.yaml
--turn-zones              default turn_zones.yaml
--navigation-map          default navigation_map.yaml
--site-boundary           default site_boundary.yaml
--pretty
```

Preflight requires all listed files to exist before derivation. There is no write-motion-graph option in A3.5 v1.

- [ ] **Step 2: Hash frozen runtime inputs before and after derivation.**

Use SHA-256 on at least:

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

Fail if any hash changes during a diagnostic run.

- [ ] **Step 3: Load the frozen pre-A3.5 baseline and derive a new graph in memory.**

Use `load_vehicle_feasible_motion_graph` for the baseline. Because Task 4 keeps missing diagnostic fields backward compatible, the old runtime baseline must load with empty `reverse_backend_diagnostics`.

Derive the new graph through the normal `derive_vehicle_feasible_motion_graph` path with the same frozen input assets and canonical vehicle profile. Do not write the newly derived graph to runtime.

- [ ] **Step 4: Implement strict behavior projection comparison.**

The projection must compare all service actions exactly and, for each transition, all pre-A3.5 behavioral fields exactly while excluding only `reverse_backend_diagnostics`.

Transition behavior projection must include:

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

Also compare:

```text
frame_id
platform_id
platform_profile_sha256
row_direction_xy
executable_service_action_ids
executable_transition_ids
MotionGraphDiagnostics
```

Do not compare top-level `source`, because diagnostic invocation metadata is allowed to differ. Any other projection mismatch raises `ValueError` and the harness exits before reporting diagnosis success.

- [ ] **Step 5: Require complete R6B diagnostic coverage.**

From the newly derived transitions select `backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH`. On the frozen real dataset require exactly `40` records. Require every one to carry `schema == agt_r6b_connector_diagnostics/v1`.

For every transition whose `backend_status == NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`, require a non-null failure class from the frozen set.

This real-data cardinality check belongs only in the acceptance harness. The classifier remains map-agnostic.

- [ ] **Step 6: Produce deterministic per-connector records.**

Each connector report record must contain:

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
search_expansions
failure_class
queue_exhausted
expansion_budget_reached
start_goal_position_error_m
best_goal_position_error_m
best_goal_yaw_error_rad
best_goal_distance_state_direction
best_goal_distance_cusp_count
all diagnostic funnel counters
```

Sort by `connector_candidate_id`.

- [ ] **Step 7: Produce deterministic aggregate sections.**

`summary` must include:

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

`classification_by_side` keys are `LOW_U` and `HIGH_U`, each mapping failure class -> count.

`classification_by_forward_audit` is keyed by the existing `forward_evidence.audit_status`, then failure class -> count.

`top_rejection_causes` aggregates and sorts descending by count, then lexicographically by cause name, using exactly:

```text
SEARCH_ENVELOPE
SITE_BOUNDARY
NAVIGATION_GRID
PATH_LENGTH
STATE_DOMINANCE
CUSP_STATE_DOMINANCE
```

where the first four combine primitive and goal-shot counters, `STATE_DOMINANCE` is the primitive dominance counter, and `CUSP_STATE_DOMINANCE` is the cusp-switch dominance counter.

`closest_connectors` contains the first 10 failed connectors sorted by:

```python
(
    best_goal_position_error_m,
    best_goal_yaw_error_rad,
    connector_candidate_id,
)
```

Treat missing errors as positive infinity so measured evidence ranks before missing evidence.

- [ ] **Step 8: Assert the diagnostic-only baseline outcome before returning success.**

For this frozen first A3.5 real-data run require:

```text
transition_validation_count == 40
executable_transition_count == 0
rejected_transition_count == 40
```

This is a regression guard for the diagnostic-only stage, not a permanent planner requirement.

- [ ] **Step 9: Do not add an amendment recommendation to the harness.**

The harness reports evidence only. It must not automatically propose increasing search budget, widening envelopes, reducing footprint, changing Site Boundary, reducing turning radius, or altering primitive/discretization parameters.

---

# Task 6: Run the batched GREEN gate and commit the diagnostic-only implementation

**Files:** all files from Tasks 2-5 plus the already committed RED tests.

- [ ] **Step 1: Run operator Gate G2 exactly.**

Do not proceed to real data until both focused tests and the package regression are green on the operator machine.

- [ ] **Step 2: Review the diff specifically for accidental behavior changes.**

Run:

```bash
git diff -- \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py \
  tools/v25_12g_a35_connector_diagnosis.py
```

Reject the implementation if the diff changes any of these frozen planner expressions:

- `_state_key` resolution/math;
- `_heuristic` formula;
- curvature tuple `(-1/R, 0, +1/R)`;
- queue priority formula;
- child motion/reverse/cusp/steering costs;
- `max_cusps`, `max_path_length_m`, `max_expansions` defaults;
- goal position/yaw thresholds;
- forward-only goal acceptance or forward-only goal-shot behavior;
- Site Boundary/Navigation Grid acceptance semantics.

The only permitted search-code changes are observational counters, best-state bookkeeping, reason extraction equivalent to the old Boolean gate, and attaching diagnostics to results.

- [ ] **Step 3: Inspect workspace protection.**

```bash
git status --short
```

The pre-existing lines may still be:

```text
 M tools/rosbag_sensor_trimmer
?? tools/map_tools/render_pcd_top_views.py
```

No A3.5 command may alter their status/content.

- [ ] **Step 4: Commit production changes in explicit logical groups after GREEN is confirmed.**

Core instrumentation:

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py

git diff --cached --name-only
git commit -m "feat(v25-12g): instrument R6B diagnostic funnel"
```

A3 passthrough/IO:

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py

git diff --cached --name-only
git commit -m "feat(v25-12g): persist A3.5 connector diagnostics"
```

Diagnosis harness:

```bash
git add tools/v25_12g_a35_connector_diagnosis.py

git diff --cached --name-only
git commit -m "feat(v25-12g): add A3.5 connector diagnosis harness"
```

The tests/CMake paths were already committed in the RED contract commit. Never use `git add .` or `git add -A` in this stage.

---

# Task 7: Run the complete real-data funnel, freeze diagnosis, and verify no behavior drift

**Files:**
- Create after evidence exists: `docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md`
- Runtime assets: read-only; do not commit generated runtime YAML/JSON.

- [ ] **Step 1: Run operator Gate G3 exactly once for the complete 40-connector diagnosis.**

Capture both JSON and `/usr/bin/time -v` output under `/tmp` using the G3 command. Do not rerun the 19-minute-scale search merely to reformat the report.

- [ ] **Step 2: Validate the machine-readable invariants from the captured JSON.**

Use the exact inspection command from G3. Additionally run:

```bash
python3 - <<'PY'
import json
from pathlib import Path

report = json.loads(Path('/tmp/v25_12g_a35_connector_diagnosis.json').read_text())
items = report['connectors']
assert len(items) == 40
assert len({item['connector_candidate_id'] for item in items}) == 40
assert all(item['failure_class'] for item in items)
assert all(
    not (item['queue_exhausted'] and item['expansion_budget_reached'])
    for item in items
)
assert report['summary']['behavior_projection_equal'] is True
assert report['summary']['final_transition_statuses_unchanged'] is True
assert report['summary']['protected_input_hashes_unchanged'] is True
assert report['summary']['forbidden_semantic_key_count'] == 0
print('A3.5 diagnostic invariants: PASS')
PY
```

- [ ] **Step 3: Freeze the complete evidence in `V25_12G_A35_DIAGNOSIS_2026-08-17.md`.**

Populate the document only from the actual G2/G3 outputs. It must include:

1. branch and exact diagnostic HEAD;
2. G2 focused/package result;
3. real runtime/profile paths;
4. pre-A3.5 baseline transition funnel;
5. post-instrumentation final transition funnel and explicit equality statement;
6. `queue_exhausted_count` vs `expansion_budget_reached_count`;
7. failure-class counts overall;
8. failure-class counts split by `LOW_U` / `HIGH_U`;
9. failure-class counts split by original forward audit status;
10. frozen expansion histogram;
11. top rejection causes;
12. closest 10 connectors with best-goal position/yaw/direction/cusp evidence;
13. a 40-row connector table containing connector ID, side, forward audit, R6A decision, expansions, termination mode, failure class, best position error, best yaw error, best direction, and best cusp count;
14. protected-input hash invariant;
15. wall/user/system time and maximum RSS from the captured timing file;
16. explicit statement that no planner behavior was amended and A4 remains blocked.

Do not editorialize a backend fix inside this evidence document. The evidence should be sufficient for the next brainstorming stage to reason from.

- [ ] **Step 4: Commit only the frozen diagnosis document.**

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

Verify the two protected user paths remain exactly as they were before A3.5.

---

# Task 8: Stop at the evidence-backed amendment boundary

**Files:** none.

- [ ] **Step 1: Summarize the frozen diagnostic evidence without choosing a fix yet.**

Report:

- dominant failure classes;
- side/forward-audit differences;
- queue exhaustion vs budget saturation;
- whether Site Boundary, Navigation Grid, search envelope, path/cusp envelope, state dominance, goal connection, or motion primitives dominate;
- closest-to-goal connector subset;
- whether the existing evidence supports or rejects “increase max_expansions” as the first hypothesis.

- [ ] **Step 2: Invoke `superpowers:brainstorming` only now for the next backend amendment.**

The next stage must select the minimum behavior change supported by the frozen A3.5 evidence. It receives a new spec/approval boundary and a new strict RED -> GREEN implementation plan.

- [ ] **Step 3: Keep the following options explicitly forbidden unless new evidence independently justifies changing the product requirement itself.**

Do not use the next brainstorming stage to obtain connectivity by:

- relaxing/removing Site Boundary;
- relaxing Navigation Grid FREE-only footprint checks;
- shrinking the canonical footprint;
- reducing `0.05 m` preview padding;
- reducing the verified `1.50 m` Ackermann minimum turning radius;
- blindly increasing `max_expansions=30000`.

A behavior amendment may instead investigate backend structure evidenced by A3.5, such as search-envelope shape, primitive family, discretization/dominance, goal connection, or path/cusp limits, but only after the corresponding diagnostic class/funnel demonstrates that mechanism.

---

## Self-Review Checklist Before Execution

- [ ] The plan is diagnostic-only through Task 7 and contains no backend behavior amendment.
- [ ] Every required diagnostic field from the approved A3.5 design is represented.
- [ ] Queue exhaustion and expansion-budget termination are disambiguated.
- [ ] Search-envelope, Site Boundary, Navigation Grid, path length, state dominance, cusp, goal tolerance, and goal-shot rejection paths have separate counters.
- [ ] Best-goal position/yaw/direction/cusp values come from one coherent state.
- [ ] Failure classes use explicit frozen thresholds and no real connector IDs/counts.
- [ ] The 30000-expansion value is not increased or treated as the assumed cause.
- [ ] `_edge_rejection_reason` preserves current envelope -> Site Boundary -> Navigation Grid precedence.
- [ ] A3 transition result semantics remain unchanged; diagnostics are appended evidence only.
- [ ] `agt_vehicle_feasible_motion_graph/v1` remains backward compatible with the frozen pre-A3.5 runtime YAML.
- [ ] The all-40 harness compares the new derivation to the frozen old baseline while ignoring only the new diagnostic mapping.
- [ ] Real-data harness writes no runtime planner asset.
- [ ] A4 is explicitly blocked at the end of this plan.
- [ ] The two protected user paths are never staged.
- [ ] No `route_ready`, `reachable_from_start`, `optimal`, global infeasibility, or equivalent semantic claim is introduced.
- [ ] Mandatory operator interruptions are limited to G1 RED, G2 GREEN/package, and G3 real-data diagnosis.
