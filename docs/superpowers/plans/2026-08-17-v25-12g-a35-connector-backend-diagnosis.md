# V25-12G-A3.5 Connector Backend Diagnosis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add diagnostic-only instrumentation to the existing R6B bounded reverse-primitive connector backend, preserve every pre-A3.5 planner outcome, classify the complete 40-connector real-data failure funnel with deterministic evidence, freeze that diagnosis, and stop before any behavior-changing backend amendment.

**Architecture:** Keep the frozen R5 -> forward audit -> R6A -> R6B chain intact. Add a pure diagnostic schema/classifier, instrument R6B with observational counters and reason-returning probes that are Boolean-equivalent to the current gates, pass the evidence through A3 `TransitionValidation` and strict YAML IO, and add a read-only A3.5 diagnosis harness that compares the newly derived motion graph against the frozen pre-A3.5 runtime graph after excluding only the new diagnostic evidence field and top-level source metadata.

**Tech Stack:** Python 3.10, dataclasses, NumPy, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing A2/A3 motion-graph contracts, R6A reverse admission, R6B bounded reverse primitive search.

## 1. Frozen Constraints

- Work in the existing checkout on `feat/v25-12g-maximum-feasible-coverage`; do not create a worktree.
- Strict TDD: tests-only RED first, operator confirms intended failure, then production code, then GREEN.
- The operator runs ROS 2/package tests and the real-map run on the target machine.
- Batch operator gates; do not introduce per-file or per-counter manual test interruptions.
- First A3.5 stage is diagnostic-only. Do not alter planner branch selection, queue ordering, state keys, heuristic, cost formulas, goal tolerances, primitive family, path acceptance, R6A policy, A3 transition classification, or executable IDs.
- Do not relax Site Boundary.
- Do not relax Navigation Grid FREE-only footprint checks or reinterpret UNKNOWN/out-of-grid as FREE.
- Do not shrink the canonical footprint.
- Keep `preview_footprint_padding_m=0.05`.
- Keep Ackermann minimum turning radius `1.50 m`.
- Keep `max_expansions=30000`; do not increase it as a diagnostic shortcut.
- Keep the frozen R6B settings unchanged: primitive `0.30 m`, collision sampling `0.10 m`, XY state resolution `0.15 m`, yaw resolution `15 deg`, goal position tolerance `0.18 m`, goal yaw tolerance `12 deg`, goal-shot distance `3.0 m`, `max_cusps=2`, `max_path_length_m=18.0`, longitudinal zone padding `0.80 m`, lateral pair padding `1.25 m`.
- Preserve motion-graph schema `agt_vehicle_feasible_motion_graph/v1` and top-level status `MOTION_EVIDENCE_ONLY`.
- Do not serialize `route_ready`, `reachable_from_start`, `optimal`, or equivalent readiness/optimality claims.
- Do not describe `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION` as proof of global infeasibility.
- Never touch, stage, revert, rename, or delete:
  - `tools/rosbag_sensor_trimmer`
  - `tools/map_tools/render_pcd_top_views.py`
- Do not use `git add .` or `git add -A` in this stage.

## 2. Completion Boundary

This plan ends after diagnostic evidence is frozen. It does not implement a backend amendment.

A4 remains blocked until a later approved stage produces one real continuous chain:

```text
segment A -> transition 1 -> segment B -> transition 2 -> segment C
```

with all conditions true:

- at least `3` distinct `segment_id` values;
- at least `2` distinct `aisle_id` values;
- at least `2` `EXECUTABLE` inter-segment transitions.

For this diagnostic-only stage, the frozen real-map result must remain `0 EXECUTABLE / 40 REJECTED`. Any final transition behavior change is a regression and stops execution.

## 3. Files

Create:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py
src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py
tools/v25_12g_a35_connector_diagnosis.py
tests/test_v25_12g_a35_connector_diagnosis_contract.py
docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md
```

Modify:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py
src/agt_offline_assets/test/test_reverse_primitive_connector.py
src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py
src/agt_offline_assets/CMakeLists.txt
```

Do not add a second planner or duplicate the R6B search implementation.

## 4. Frozen Diagnostic Contract

### 4.1. Public failure classes

Use exactly:

```python
SEARCH_ENVELOPE_LIMITED = "SEARCH_ENVELOPE_LIMITED"
SITE_BOUNDARY_LIMITED = "SITE_BOUNDARY_LIMITED"
FOOTPRINT_GRID_LIMITED = "FOOTPRINT_GRID_LIMITED"
STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED = "STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED"
GOAL_CONNECTION_LIMITED = "GOAL_CONNECTION_LIMITED"
MOTION_PRIMITIVE_LIMITED = "MOTION_PRIMITIVE_LIMITED"
PATH_OR_CUSP_ENVELOPE_LIMITED = "PATH_OR_CUSP_ENVELOPE_LIMITED"
MIXED_LIMITATION = "MIXED_LIMITATION"
INCONCLUSIVE = "INCONCLUSIVE"
R6B_DIAGNOSTIC_SCHEMA = "agt_r6b_connector_diagnostics/v1"
```

### 4.2. Public diagnostic dataclass

Create exactly this evidence shape:

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

Append to `ReversePrimitiveConnectorResult` after existing fields:

```python
diagnostics: ReversePrimitiveSearchDiagnostics = field(
    default_factory=ReversePrimitiveSearchDiagnostics
)
```

Append to `TransitionValidation` after `reverse_admission_evidence`:

```python
reverse_backend_diagnostics: Mapping[str, Any] = field(default_factory=dict)
```

Existing semantic fields remain unchanged.

### 4.3. Pure helper interfaces

Implement in `reverse_primitive_diagnostics.py`:

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

The classifier must not read connector ID, side, aisle ID, turn-zone ID, or any real-map count.

### 4.4. Accounting invariants

Every completed diagnostic record must satisfy:

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
- Bounded failure sets `queue_exhausted=True` iff the queue is empty after loop exit.
- Bounded failure sets `expansion_budget_reached=True` iff work remains and `search_expansions >= max_expansions`.
- If queue becomes empty exactly at the limit, queue exhaustion wins.
- Start hard failures keep `search_started=False` and both termination flags false.
- `goal_tolerance_checks` increments once per non-stale expanded state.
- Only the existing FORWARD position+yaw goal success increments `goal_tolerance_successes`.
- A REVERSE state within `goal_shot_distance_m` increments `goal_shot_reverse_direction_blocked` and still returns no goal shot.
- Every goal-shot candidate lands in exactly one terminal candidate counter.
- Every primitive curvature candidate lands in exactly one terminal primitive counter.
- If current node travel plus primitive length exceeds max path length, count all three frozen curvature candidates as path-length rejections and preserve the existing node-level `continue`.

### 4.5. Edge rejection reason contract

Add:

```python
EDGE_FREE = "FREE"
EDGE_SEARCH_ENVELOPE = "SEARCH_ENVELOPE"
EDGE_SITE_BOUNDARY = "SITE_BOUNDARY"
EDGE_NAVIGATION_GRID = "NAVIGATION_GRID"
```

`_edge_rejection_reason` must use the same arguments and sample sequence as current `_edge_is_free`, with this exact precedence:

1. search envelope;
2. Site Boundary conflict from `_preview_pose_status`;
3. any remaining non-FREE Navigation Grid/footprint result;
4. FREE.

Retain `_edge_is_free` as a compatibility wrapper returning only whether `_edge_rejection_reason` equals `EDGE_FREE`.

### 4.6. Best-goal state

For each non-stale popped state compute position and yaw error. Keep one coherent best state. Update when position error improves by more than `1e-12`, or when position errors tie within `1e-12` and yaw error improves by more than `1e-12`. On a full tie keep the earlier popped state. Position, yaw, direction, and cusp count must all come from that one state.

### 4.7. Frozen diagnostic thresholds

```python
DOMINANT_REJECTION_FRACTION = 0.50
MIXED_REJECTION_FRACTION = 0.25
DOMINANCE_REJECTION_FRACTION = 0.60
DOMINANCE_ENQUEUED_FRACTION_MAX = 0.15
CUSP_SATURATION_FRACTION = 0.50
LITTLE_PROGRESS_FRACTION = 0.20
```

Combined candidate denominator:

```text
geometry_candidate_count
  = primitive_edges_considered
  + goal_shot_candidates_considered
```

Combined cause counts:

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

Divide each by `max(geometry_candidate_count, 1)`.

State-dominance signal:

```text
primitive_edges_rejected_state_dominance / max(primitive_edges_considered, 1) >= 0.60
AND primitive_edges_enqueued / max(primitive_edges_considered, 1) <= 0.15
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

Motion-primitive signal:

```text
queue_exhausted
AND primitive_edges_considered > 0
AND every geometric/path rejection fraction < 0.25
AND primitive state-dominance rejection fraction < 0.25
AND primitive enqueued fraction >= 0.25
AND little-progress fraction <= 0.20
AND no goal-connection signal
AND no cusp-saturation signal
```

### 4.8. Frozen classifier order

Apply failure classification only to `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`; other R6B statuses use `failure_class=None`.

For bounded no-solution:

1. two or more geometric/path causes each `>=0.50` -> `MIXED_LIMITATION`;
2. exactly one geometric/path cause `>=0.50` -> direct class;
3. otherwise collect moderate geometric/path causes `>=0.25` and structural state-dominance/cusp/goal signals;
4. two or more distinct classes in that set -> `MIXED_LIMITATION`;
5. exactly one structural class -> that structural class;
6. a lone geometric/path fraction in `[0.25, 0.50)` is insufficient;
7. conservative motion-primitive signal -> `MOTION_PRIMITIVE_LIMITED`;
8. otherwise -> `INCONCLUSIVE`.

Direct mappings:

```text
search envelope -> SEARCH_ENVELOPE_LIMITED
Site Boundary -> SITE_BOUNDARY_LIMITED
Navigation Grid -> FOOTPRINT_GRID_LIMITED
path length -> PATH_OR_CUSP_ENVELOPE_LIMITED
state dominance -> STATE_DISCRETIZATION_OR_DOMINANCE_LIMITED
cusp saturation -> PATH_OR_CUSP_ENVELOPE_LIMITED
goal connection -> GOAL_CONNECTION_LIMITED
```

`expansion_budget_reached` is reported independently and is not itself a failure class.

## 5. Operator Gates

Exactly three mandatory operator gates are used.

### G1 — tests-only RED

Run after Task 1, before any production implementation:

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

Valid RED means tests collect successfully and fail on missing A3.5 API/behavior. Syntax, import, fixture, or collection failures are not valid RED.

### G2 — focused GREEN plus ROS 2 package regression

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

Proceed only when focused tests are green and package test-result reports zero errors/failures.

### G3 — complete 40-connector real-data diagnosis

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

Then inspect the captured JSON without rerunning R6B:

```bash
python3 - <<'PY'
import json
from pathlib import Path

report = json.loads(Path('/tmp/v25_12g_a35_connector_diagnosis.json').read_text())
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

G3 requires exactly 40 distinct R6B connector records, valid diagnostics for all, behavior projection equality, unchanged final transition statuses, `0 EXECUTABLE / 40 REJECTED`, unchanged protected input hashes, and zero forbidden semantic keys.

## 6. Task 1 — Tests-Only RED Contract

**Files:** create/modify only test files and CMake listed in Section 3.

- [ ] Add RED-safe API discovery in `test_reverse_primitive_diagnostics.py` by importing the existing `reverse_primitive_connector` module and using `getattr` inside test functions. Missing diagnostics/classifier APIs must fail assertions during test execution, not test collection.
- [ ] Add exact schema and all nine failure-class constant tests.
- [ ] Add synthetic classifier fixtures covering all nine classes.
- [ ] Add exact threshold boundary tests for `0.50`, `0.25`, state dominance `0.60`, enqueued maximum `0.15`, cusp saturation `0.50`, and little progress `0.20`.
- [ ] Add strict diagnostic `to_dict`/`from_dict` round-trip and rejection tests for negative counters, bad schema, bad failure class, bad direction, non-finite/negative errors, mutually true termination flags, and each accounting identity independently.
- [ ] Extend `test_reverse_primitive_connector.py` with `test_a35_` cases for edge rejection reason separation, Boolean gate equivalence, queue exhaustion, expansion budget termination, deterministic diagnostics, coherent best-goal state, accounting identities, and unchanged R6B result semantics.
- [ ] For path-limit queue exhaustion use `max_path_length_m=0.10` and `max_expansions=50`; require bounded no-solution, queue exhausted, no budget hit, and non-zero path-length rejection.
- [ ] For explicit budget termination use a far goal and `max_expansions=1`; require bounded no-solution, queue not exhausted, budget reached, and one expansion.
- [ ] Extend A3 tests so A3.5-only test fixtures receive an actual `ReversePrimitiveSearchDiagnostics` instance obtained through the RED-safe diagnostic API helper. Do not use a placeholder object. Existing non-A3.5 fake R6B results remain unchanged.
- [ ] Add A3 bounded-failure passthrough test requiring unchanged `REJECTED`, `BOUNDED_SEARCH_NO_SOLUTION`, backend status, and expansion count plus exact diagnostic mapping.
- [ ] Add A3 R6B-success passthrough test requiring unchanged `EXECUTABLE` and `failure_class=None`.
- [ ] Add strict IO tests for non-empty diagnostics round-trip, old v1 YAML missing diagnostics -> `{}`, forbidden readiness key under diagnostics, and malformed diagnostic accounting rejection.
- [ ] Create root harness contract tests using `importlib.util.spec_from_file_location`; require exact report schema, validation scope, parser behavior, read-only semantics, behavior projection equality, deterministic sort/ranking/histogram, and fail-closed semantic checks.
- [ ] Register only `test_reverse_primitive_diagnostics.py` in package CMake with `ament_add_pytest_test`; keep the root harness contract explicitly invoked.
- [ ] Run G1 and stop until the operator supplies valid RED output.
- [ ] After valid RED, stage only the named tests/CMake paths and commit:

```bash
git add \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py \
  src/agt_offline_assets/CMakeLists.txt

git diff --cached --name-only
git commit -m "test(v25-12g): define A3.5 R6B diagnostic contract"
```

## 7. Task 2 — Pure Diagnostic Model and Classifier

**Files:** create `reverse_primitive_diagnostics.py`; modify `reverse_primitive_connector.py` only for import/re-export in this task.

- [ ] Implement the exact constants, dataclass, thresholds, serialization helpers, strict validation, and classifier from Section 4.
- [ ] `to_dict` returns all fields in deterministic dataclass field order using native scalar/null types.
- [ ] `from_dict` validates exact schema, finite/non-negative errors, counter types/ranges, directions, classes, accounting identities, and termination exclusivity.
- [ ] Keep this module pure; it must not depend on Navigation Grid, Site Boundary, R6B queue internals, A3 graph derivation, connector IDs, or real-map counts.
- [ ] Re-export diagnostics public symbols from `reverse_primitive_connector.py` for stable test access.
- [ ] Do not request a separate operator gate; continue within the confirmed RED batch.

## 8. Task 3 — R6B Diagnostic Instrumentation Without Behavior Change

**File:** `reverse_primitive_connector.py`.

- [ ] Add one private mutable accumulator whose only output is the frozen public diagnostics object.
- [ ] Never read diagnostic counters to choose planner branches.
- [ ] Implement `_edge_rejection_reason` and make `_edge_is_free` the exact Boolean wrapper described in Section 4.5.
- [ ] Instrument start validation and attach non-started diagnostics to existing Site Boundary/start-footprint hard failures without changing status or control flow.
- [ ] Preserve queue loop order exactly: heap pop -> stale check -> expansion increment -> goal tolerance -> goal shot -> cusp transition -> primitive expansion.
- [ ] Increment `nodes_popped` immediately after heap pop, stale counter only on current stale continue, and expansion counter exactly where current `expansions` increments.
- [ ] Track coherent best-goal state only for non-stale expanded states.
- [ ] Instrument existing FORWARD goal tolerance without enabling reverse tolerance success.
- [ ] Add accumulator parameter to `_try_forward_goal_shot`; preserve all current early returns, candidate generation order, path-length checks, sampling order, and first-success behavior.
- [ ] Count reverse-direction shot blockage only when within the existing shot distance.
- [ ] Count each FORWARD Dubins shot candidate once and assign exactly one terminal category.
- [ ] Instrument cusp switch considered/dominance/enqueued counts using the existing state key and cost comparison; `nodes_at_max_cusps` is observational only.
- [ ] Instrument all three frozen curvature candidates. Preserve the existing node-level max-path continue and state-dominance comparison.
- [ ] On bounded failure set:

```python
queue_exhausted = not queue
expansion_budget_reached = bool(queue) and expansions >= cfg.max_expansions
```

- [ ] Classify only bounded no-solution results; other statuses keep `failure_class=None`.
- [ ] Add deterministic nested diagnostics to `reverse_primitive_connector_plan_to_dict` without renaming/removing existing keys.

## 9. Task 4 — A3 Passthrough and Strict v1 IO

**Files:** `vehicle_feasible_motion_graph.py`, `vehicle_feasible_transition_motion.py`, `vehicle_feasible_motion_graph_io.py`.

- [ ] Append `reverse_backend_diagnostics` to `TransitionValidation` with default empty mapping.
- [ ] Extend `_base_result` with optional diagnostic mapping stored as a copied dictionary.
- [ ] Serialize R6B diagnostics immediately after retrieving each R6B result and pass them through every R6B-backed A3 branch: success, Site Boundary hard conflict, bounded no-solution, start footprint failure, and unknown future status fallback.
- [ ] Do not change A3 status/proof/backend/backend-status/reason mappings.
- [ ] Serialize diagnostics after `reverse_admission_evidence`.
- [ ] Strict loader accepts missing diagnostics as `{}` for the frozen pre-A3.5 v1 runtime YAML.
- [ ] Non-empty diagnostics must pass strict diagnostic `from_dict` validation before being stored canonically.
- [ ] Keep recursive forbidden-key rejection active inside diagnostics.
- [ ] Do not bump `agt_vehicle_feasible_motion_graph/v1`.

## 10. Task 5 — Read-Only All-40 Diagnosis Harness

**File:** `tools/v25_12g_a35_connector_diagnosis.py`.

Use exact public constants:

```python
REPORT_SCHEMA = "agt_v25_12g_a35_connector_backend_diagnosis/v1"
VALIDATION_SCOPE = "A35_R6B_DIAGNOSTIC_ONLY_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_REAL_R6B_CONNECTOR_COUNT = 40
```

- [ ] Parser requires `--run-dir` and `--vehicle-profile`; defaults baseline graph to `vehicle_feasible_motion_graph.yaml`, service graph to `vehicle_feasible_service_graph.yaml`, zones to `turn_zones.yaml`, navigation map to `navigation_map.yaml`, Site Boundary to `site_boundary.yaml`, and `--pretty` false.
- [ ] There is no write-motion-graph option.
- [ ] Before derivation SHA-256 hash these runtime inputs: baseline motion graph, service graph, segments, turn zones, navigation YAML, navigation PGM, Site Boundary, derivation YAML, aisle graph.
- [ ] Load baseline via strict motion-graph loader.
- [ ] Load A2/zones/navigation/boundary/profile through the same normal APIs used by A3 acceptance and derive the new graph only in memory.
- [ ] Hash the same runtime inputs after derivation and fail if any digest changed.
- [ ] Compare all service actions exactly.
- [ ] Compare transition behavior fields exactly except `reverse_backend_diagnostics`: connector/service/segment IDs, side, zone, start/goal pose, status, proof scope, backend, backend status, reason, path/forward/reverse distances, cusp count, search expansions, samples, forward evidence, R6A evidence.
- [ ] Compare frame, platform, profile SHA, row direction, executable IDs, and `MotionGraphDiagnostics` exactly.
- [ ] Exclude only top-level source metadata and the new transition diagnostic mapping from behavior projection equality.
- [ ] Require exactly 40 distinct transitions with backend `BOUNDED_REVERSE_PRIMITIVE_SEARCH` and valid diagnostic schema.
- [ ] Require every bounded no-solution record to have one frozen failure class.
- [ ] Emit one deterministic record per connector with connector/segment/side/zone context, forward audit, R6A decision, final status/backend status, and every diagnostic field; sort by connector ID.
- [ ] Emit summary keys:

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

- [ ] `classification_by_side` separates `LOW_U` and `HIGH_U`.
- [ ] `classification_by_forward_audit` uses original `forward_evidence.audit_status`.
- [ ] Expansion histogram uses exact bins: `0-9`, `10-99`, `100-499`, `500-999`, `1000-4999`, `5000-9999`, `10000-19999`, `20000-30000`.
- [ ] Aggregate rejection causes exactly as `SEARCH_ENVELOPE`, `SITE_BOUNDARY`, `NAVIGATION_GRID`, `PATH_LENGTH`, `STATE_DOMINANCE`, `CUSP_STATE_DOMINANCE`; sort by descending count then cause name.
- [ ] `closest_connectors` contains at most 10 failed connectors sorted by best position error, best yaw error, connector ID; missing errors sort last.
- [ ] Require the diagnostic-only real graph to remain 40 transitions, 0 executable, 40 rejected.
- [ ] Do not emit backend amendment recommendations from this harness.

## 11. Task 6 — G2 GREEN and Diagnostic-Only Commits

- [ ] Run G2 exactly.
- [ ] Reject any implementation diff that changes `_state_key`, `_heuristic`, curvature tuple `(-1/R, 0, +1/R)`, queue priority, motion/reverse/cusp/steering costs, goal tolerances, forward-only goal semantics, search defaults, Site Boundary acceptance, or Navigation Grid acceptance.
- [ ] Run `git status --short` and verify the two protected user paths remain untouched.
- [ ] After G2 is green, commit core R6B instrumentation:

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py \
  src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py

git diff --cached --name-only
git commit -m "feat(v25-12g): instrument R6B diagnostic funnel"
```

- [ ] Commit A3 passthrough/IO:

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py

git diff --cached --name-only
git commit -m "feat(v25-12g): persist A3.5 connector diagnostics"
```

- [ ] Commit harness:

```bash
git add tools/v25_12g_a35_connector_diagnosis.py
git diff --cached --name-only
git commit -m "feat(v25-12g): add A3.5 connector diagnosis harness"
```

## 12. Task 7 — G3 Real-Data Funnel and Evidence Freeze

- [ ] Run G3 once and retain JSON/time output in `/tmp`; do not rerun the expensive search for formatting.
- [ ] Validate captured evidence without rerunning R6B:

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
assert all(not (item['queue_exhausted'] and item['expansion_budget_reached']) for item in items)
assert summary['behavior_projection_equal'] is True
assert summary['final_transition_statuses_unchanged'] is True
assert summary['protected_input_hashes_unchanged'] is True
assert summary['forbidden_semantic_key_count'] == 0
print('A3.5 diagnostic invariants: PASS')
PY
```

- [ ] Create `docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md` only from actual G2/G3 outputs.
- [ ] Include exact branch/HEAD, G2 results, run-dir/profile, baseline and post-instrumentation transition funnel, behavior equality, queue-vs-budget counts, overall failure classes, side split, forward-audit split, expansion histogram, top rejection causes, closest 10, one 40-row connector table, protected hashes, wall/user/system time, max RSS, and explicit A4-blocked statement.
- [ ] Do not include a backend fix recommendation in the frozen diagnosis document.
- [ ] Commit only that document:

```bash
git add docs/v2.5/V25_12G_A35_DIAGNOSIS_2026-08-17.md
git diff --cached --name-only
git commit -m "docs(v25-12g): freeze A3.5 connector diagnosis evidence"
```

- [ ] Final `git status --short` must still preserve the two protected user paths exactly as before A3.5.

## 13. Task 8 — Stop Before Backend Amendment

- [ ] Summarize evidence only: dominant failure classes, LOW_U/HIGH_U differences, forward-audit differences, queue exhaustion vs budget saturation, strongest rejection causes, closest-to-goal subset, and whether evidence supports or rejects raising `max_expansions` as the first hypothesis.
- [ ] Only after the diagnosis document is frozen invoke `superpowers:brainstorming` for the minimum behavior-changing backend amendment.
- [ ] That later stage requires a new approved spec and a new strict RED -> GREEN plan.
- [ ] Carry forward hard prohibitions: no relaxed Site Boundary, no relaxed Navigation Grid, no smaller footprint, no reduced `0.05 m` padding, no reduced `1.50 m` turning radius, no blind `max_expansions` increase.

## 14. Self-Review Checklist

- [ ] Diagnostic-only until evidence freeze; no amendment code is mixed in.
- [ ] Original checkout only; no worktree.
- [ ] Tests-only RED precedes production implementation.
- [ ] Only three mandatory operator gates: G1, G2, G3.
- [ ] Every approved minimum diagnostic field is represented.
- [ ] Queue exhaustion and budget termination are distinct.
- [ ] Envelope, Site Boundary, Navigation Grid, path length, state dominance, cusp, goal tolerance, and goal shot have separate evidence.
- [ ] Best-goal metrics come from one coherent state.
- [ ] Failure classification uses explicit frozen thresholds and no connector identity.
- [ ] `max_expansions=30000` remains unchanged and is not assumed causal.
- [ ] Site Boundary, Navigation Grid, footprint, `0.05 m` padding, and `1.50 m` turning radius remain frozen.
- [ ] A3 status/proof/backend/backend-status/reason/search-expansion/sample semantics remain unchanged.
- [ ] Motion-graph v1 remains backward compatible with pre-A3.5 runtime YAML.
- [ ] All-40 harness is read-only and compares old/new behavior excluding only diagnostic evidence and top-level source metadata.
- [ ] No readiness, reachability, optimality, or global-infeasibility claim is introduced.
- [ ] A4 remains blocked until the later real 3-segment / 2-aisle / 2-transition chain exists.
- [ ] `tools/rosbag_sensor_trimmer` and `tools/map_tools/render_pcd_top_views.py` are never staged or modified.
