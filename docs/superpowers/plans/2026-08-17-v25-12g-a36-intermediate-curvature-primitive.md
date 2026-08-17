# V25-12G-A3.6 Intermediate-Curvature Primitive Amendment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Test whether adding fixed intermediate Ackermann curvature primitives `±0.5/R` to the existing bounded R6B search recovers real inter-segment connector motion without changing any frozen physical, map, safety, terminal, search-budget, or A4 semantics.

**Architecture:** Keep the existing R5 -> forward audit -> R6A -> R6B -> A3 motion-graph chain and all A3.5 diagnostics intact. Change only R6B primitive generation from three to five deterministic curvature categories, update diagnostic accounting from three to five where it depends on primitive cardinality, and add a read-only A3.6 acceptance harness that compares the amended in-memory motion graph with the frozen A3.5 baseline while allowing only transition outcomes to improve or remain rejected. No motion-graph schema change is planned.

**Tech Stack:** Python 3.10, dataclasses, NumPy, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing R6A/R6B/A3 motion-graph contracts.

## Global Constraints

- Work in the existing checkout on `feat/v25-12g-maximum-feasible-coverage`; do not create another worktree.
- Strict TDD: tests-only RED first; the operator must confirm a valid target-machine RED before any A3.6 production behavior is written.
- The operator owns ROS 2/package and real-map execution. Never claim RED/GREEN/build/real-data success without operator-supplied output.
- The only intended planner behavior change is:

```text
{-1.0/R, 0.0, +1.0/R}
->
{-1.0/R, -0.5/R, 0.0, +0.5/R, +1.0/R}
```

- The `0.5` fraction is fixed. Do not add `±0.25/R`, expose a curvature-fraction tuning option, or run a parameter sweep.
- Every generated primitive must satisfy `abs(curvature) <= 1/R`.
- Keep `minimum_turning_radius_m = 1.50 m` from the canonical `mk_mini` profile.
- Keep the canonical footprint unchanged.
- Keep `preview_footprint_padding_m = 0.05`.
- Keep Site Boundary semantics unchanged and fail-closed.
- Keep Navigation Grid `FREE`-only full-footprint validation unchanged; UNKNOWN, occupied and out-of-grid remain invalid.
- Keep `primitive_length_m = 0.30`.
- Keep `collision_sample_step_m = 0.10`.
- Keep state resolution `0.15 m / 15 deg`.
- Keep goal tolerances `0.18 m / 12 deg` in production defaults.
- Keep forward-only Dubins goal-shot behavior and `goal_shot_distance_m = 3.0` in production defaults.
- Keep `max_cusps = 2`.
- Keep `max_expansions = 30000`.
- Keep `max_path_length_m = 18.0`.
- Keep search-envelope padding `0.80 m / 1.25 m`.
- Keep heuristic, queue ordering, reverse-cost multiplier, cusp penalty, steering-change penalty and state-dominance rule unchanged.
- Keep A1 and A2 assets unchanged.
- Keep motion-graph schema `agt_vehicle_feasible_motion_graph/v1` and status `MOTION_EVIDENCE_ONLY` unchanged.
- Do not add route-ready, reachable-from-start, optimal, field-ready or global-infeasibility semantics.
- A negative `0/40` A3.6 result is valid evidence and must not trigger an unapproved second amendment.
- Never touch, stage, reset, rename or delete:

```text
M  tools/rosbag_sensor_trimmer
?? src/agt_route_benchmark/
?? tools/map_tools/render_pcd_top_views.py
```

- Never use `git add .`, `git add -A`, `git clean`, `git reset --hard`, `git restore .`, or `git checkout -- .`.

---

## File Structure

Modify:

```text
src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
src/agt_offline_assets/test/test_reverse_primitive_connector.py
```

Create:

```text
tests/test_v25_12g_a36_intermediate_curvature_contract.py
tools/v25_12g_a36_intermediate_curvature_acceptance.py
docs/v2.5/V25_12G_A36_INTERMEDIATE_CURVATURE_2026-08-17.md
```

Do not modify:

```text
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py
src/agt_offline_assets/agt_offline_assets/reverse_primitive_diagnostics.py
src/agt_offline_assets/CMakeLists.txt
```

unless a target-machine RED/GREEN failure demonstrates a contract defect that cannot be addressed inside the planned files. Any such expansion requires stopping and reporting the reason before editing those files.

---

### Task 1: Land the tests-only A3.6 RED contract

**Files:**
- Modify: `src/agt_offline_assets/test/test_reverse_primitive_connector.py`
- Create: `tests/test_v25_12g_a36_intermediate_curvature_contract.py`

**Interfaces:**
- Consumes: existing `ReversePrimitiveConnectorConfig`, `derive_reverse_primitive_connector_plan`, A3.5 diagnostic fields, existing synthetic `_vehicle()`, `_grid()`, `_zones()`, `_admission()` fixtures.
- Produces: RED requirements for `_PRIMITIVE_CURVATURE_FRACTIONS`, `_primitive_curvature_values(radius)`, five-primitive search behavior, five-edge diagnostic accounting, and the A3.6 read-only acceptance harness.

- [ ] **Step 1: Add a focused A3.6 helper API contract to `test_reverse_primitive_connector.py`**

Append:

```python
def _a36_curvature_api():
    fractions = getattr(r6b, "_PRIMITIVE_CURVATURE_FRACTIONS", None)
    values = getattr(r6b, "_primitive_curvature_values", None)
    assert fractions is not None, "missing A3.6 curvature fraction contract"
    assert values is not None, "missing A3.6 primitive curvature helper"
    return fractions, values


def test_a36_curvature_family_is_fixed_five_level_and_respects_minimum_radius():
    fractions, values_fn = _a36_curvature_api()
    radius = 1.5
    assert tuple(fractions) == (-1.0, -0.5, 0.0, 0.5, 1.0)
    values = tuple(values_fn(radius))
    expected = tuple(fraction / radius for fraction in fractions)
    assert values == expected
    assert len(values) == 5
    assert values[2] == 0.0
    assert max(abs(value) for value in values) <= (1.0 / radius) + 1.0e-12


@pytest.mark.parametrize("radius", [0.0, -1.0, float("inf"), float("-inf"), float("nan")])
def test_a36_curvature_helper_rejects_invalid_radius(radius):
    _fractions, values_fn = _a36_curvature_api()
    with pytest.raises(ValueError, match="radius"):
        values_fn(radius)
```

Also add `import pytest` at the top of the file because this test file currently imports `math` and NumPy but not pytest.

- [ ] **Step 2: Add a public-behavior synthetic fixture that only `+0.5/R` can hit in one primitive**

Append:

```python
def _a36_intermediate_goal_request(connector_id="connector_a36_half_curvature"):
    radius = 1.5
    length = 0.30
    curvature = 0.5 / radius
    yaw = curvature * length
    x = math.sin(yaw) / curvature
    y = (1.0 - math.cos(yaw)) / curvature
    return ConnectorRequest(
        connector_id=connector_id,
        from_aisle_id="aisle_002",
        to_aisle_id="aisle_003",
        turn_zone_id="turn_low_u",
        side="LOW_U",
        start_pose=(0.0, 0.0, -1.6, 0.0),
        goal_pose=(x, y, -1.6, yaw),
    )


def test_a36_intermediate_curvature_recovers_one_primitive_reachable_state():
    request = _a36_intermediate_goal_request()
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)),
    )
    cfg = ReversePrimitiveConnectorConfig(
        primitive_length_m=0.30,
        collision_sample_step_m=0.10,
        goal_position_tolerance_m=0.005,
        goal_yaw_tolerance_deg=1.0,
        goal_shot_distance_m=0.01,
        max_cusps=1,
        max_expansions=100,
        max_path_length_m=0.31,
    )
    result = derive_reverse_primitive_connector_plan(
        (request,),
        _admission(request.connector_id),
        _zones(),
        _grid(),
        _vehicle(),
        cfg,
        site_boundary=boundary,
    ).connectors[0]

    assert result.status == "FORWARD_PRIMITIVE_PREVIEW_FREE"
    assert result.path_length_m is not None
    assert result.path_length_m <= 0.31 + 1.0e-9
    assert result.reverse_distance_m == pytest.approx(0.0, abs=1.0e-9)
    assert result.goal_position_error_m is not None
    assert result.goal_position_error_m <= cfg.goal_position_tolerance_m
    assert result.goal_yaw_error_rad is not None
    assert result.goal_yaw_error_rad <= math.radians(cfg.goal_yaw_tolerance_deg)
    assert any(
        abs(sample.curvature_per_m - (0.5 / _vehicle().minimum_turning_radius_m))
        <= 1.0e-12
        for sample in result.samples
    )
```

The old three-curvature implementation must fail this assertion with `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION`; the fixture must not fail due to an illegal start, Site Boundary, occupied grid, missing admission, or goal-shot shortcut.

- [ ] **Step 3: Lock five-edge path-length accounting**

Append:

```python
def test_a36_path_length_short_circuit_accounts_for_five_primitive_categories():
    request = _request("connector_a36_path_limit")
    result = derive_reverse_primitive_connector_plan(
        (request,),
        _admission(request.connector_id),
        _zones(),
        _grid(),
        _vehicle(),
        ReversePrimitiveConnectorConfig(
            max_path_length_m=0.10,
            max_expansions=50,
        ),
    ).connectors[0]
    diagnostics = result.diagnostics
    assert result.status == "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
    assert diagnostics.search_expansions > 0
    assert diagnostics.primitive_edges_considered == 5 * diagnostics.search_expansions
    assert diagnostics.primitive_edges_rejected_path_length == diagnostics.primitive_edges_considered
    _assert_a35_accounting(diagnostics)
```

The current implementation should expose the intended RED because it hard-codes three considered/path-rejected primitive edges on this branch.

- [ ] **Step 4: Create root acceptance-harness contract tests**

Create `tests/test_v25_12g_a36_intermediate_curvature_contract.py` with:

```python
import copy
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tools" / "v25_12g_a36_intermediate_curvature_acceptance.py"


def _load_harness():
    assert HARNESS.is_file(), f"missing A3.6 acceptance harness: {HARNESS}"
    spec = importlib.util.spec_from_file_location(
        "v25_12g_a36_intermediate_curvature_acceptance",
        HARNESS,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _service(service_id, segment_id, aisle_id):
    return {
        "service_state_id": service_id,
        "segment_id": segment_id,
        "aisle_id": aisle_id,
        "status": "EXECUTABLE",
    }


def _transition(connector_id, from_state, to_state, from_segment, to_segment, status):
    return {
        "connector_candidate_id": connector_id,
        "from_service_state_id": from_state,
        "to_service_state_id": to_state,
        "from_segment_id": from_segment,
        "to_segment_id": to_segment,
        "status": status,
        "backend_status": (
            "REVERSE_PRIMITIVE_PREVIEW_FREE"
            if status == "EXECUTABLE"
            else "NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION"
        ),
        "search_expansions": 10,
        "reverse_backend_diagnostics": {},
    }


def _payload():
    return {
        "schema": "agt_vehicle_feasible_motion_graph/v1",
        "status": "MOTION_EVIDENCE_ONLY",
        "source": {"fixture": True},
        "service_actions": [
            _service("service.a", "segment.a", "aisle.1"),
            _service("service.b", "segment.b", "aisle.2"),
            _service("service.c", "segment.c", "aisle.2"),
        ],
        "executable_service_action_ids": ["service.a", "service.b", "service.c"],
        "transition_validations": [
            _transition("connector.ab", "service.a", "service.b", "segment.a", "segment.b", "REJECTED"),
            _transition("connector.bc", "service.b", "service.c", "segment.b", "segment.c", "REJECTED"),
        ],
        "executable_transition_ids": [],
    }


def test_a36_harness_constants_and_parser_are_read_only():
    module = _load_harness()
    assert module.REPORT_SCHEMA == "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
    assert module.VALIDATION_SCOPE == (
        "A36_INTERMEDIATE_CURVATURE_EXPERIMENT_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
    )
    assert module.EXPECTED_CONNECTOR_COUNT == 40
    assert module.EXPECTED_SERVICE_ACTION_COUNT == 32
    args = module.build_parser().parse_args(
        ["--run-dir", "/tmp/run", "--vehicle-profile", "/tmp/mk_mini.yaml"]
    )
    assert args.baseline_motion_graph == "vehicle_feasible_motion_graph.yaml"
    assert args.pretty is False
    assert not hasattr(args, "write_motion_graph")
    assert not hasattr(args, "overwrite_motion_graph")


def test_a36_service_behavior_projection_must_remain_exact():
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    current["source"] = {"fixture": "amended"}
    current["transition_validations"][0]["status"] = "EXECUTABLE"
    current["executable_transition_ids"] = ["connector.ab"]
    module.assert_service_behavior_equal(baseline, current)

    current["service_actions"][0]["status"] = "REJECTED"
    with pytest.raises(ValueError, match="service"):
        module.assert_service_behavior_equal(baseline, current)


def test_a36_transition_comparison_is_deterministic_and_detects_changes():
    module = _load_harness()
    baseline = _payload()
    current = copy.deepcopy(baseline)
    current["transition_validations"][0]["status"] = "EXECUTABLE"
    current["transition_validations"][0]["backend_status"] = "REVERSE_PRIMITIVE_PREVIEW_FREE"
    current["executable_transition_ids"] = ["connector.ab"]
    result = module.compare_transition_outcomes(baseline, current)
    assert result["newly_executable_connector_ids"] == ["connector.ab"]
    assert result["still_rejected_connector_ids"] == ["connector.bc"]
    assert result["unexpectedly_regressed_connector_ids"] == []


def test_a36_a4_chain_requires_two_continuous_transitions_three_segments_two_aisles():
    module = _load_harness()
    payload = _payload()
    for transition in payload["transition_validations"]:
        transition["status"] = "EXECUTABLE"
        transition["backend_status"] = "REVERSE_PRIMITIVE_PREVIEW_FREE"
    payload["executable_transition_ids"] = ["connector.ab", "connector.bc"]
    result = module.find_a4_minimum_chain(payload)
    assert result["a4_entry_gate_passed"] is True
    assert result["connector_candidate_ids"] == ["connector.ab", "connector.bc"]
    assert result["segment_ids"] == ["segment.a", "segment.b", "segment.c"]
    assert set(result["aisle_ids"]) == {"aisle.1", "aisle.2"}


def test_a36_a4_chain_rejects_disconnected_or_single_aisle_pairs():
    module = _load_harness()
    disconnected = _payload()
    disconnected["transition_validations"][0]["status"] = "EXECUTABLE"
    disconnected["transition_validations"][1]["status"] = "EXECUTABLE"
    disconnected["transition_validations"][1]["from_service_state_id"] = "service.other"
    assert module.find_a4_minimum_chain(disconnected)["a4_entry_gate_passed"] is False

    one_aisle = _payload()
    for service in one_aisle["service_actions"]:
        service["aisle_id"] = "aisle.1"
    for transition in one_aisle["transition_validations"]:
        transition["status"] = "EXECUTABLE"
    assert module.find_a4_minimum_chain(one_aisle)["a4_entry_gate_passed"] is False


@pytest.mark.parametrize("key", ["route_ready", "reachable_from_start", "optimal"])
def test_a36_forbidden_semantic_keys_fail_closed_recursively(key):
    module = _load_harness()
    payload = _payload()
    payload["transition_validations"][0]["reverse_backend_diagnostics"][key] = True
    with pytest.raises(ValueError, match=key):
        module.assert_no_forbidden_semantic_keys(payload)
```

- [ ] **Step 5: Commit tests only**

```bash
git add \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a36_intermediate_curvature_contract.py

git diff --cached --check
git commit -m "test(v25-12g): define A3.6 intermediate-curvature RED contract"
```

Do not stage any production file.

- [ ] **Step 6: Target-machine RED gate — stop here until operator output**

Run exactly:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a36_intermediate_curvature_contract.py \
  -k a36
```

Expected valid RED characteristics:

```text
no SyntaxError
no import/fixture construction error
curvature API tests fail because A3.6 helper/constants do not exist
one-primitive reachability test fails because current three-curvature search cannot solve it
five-edge accounting test fails because current short-circuit accounts for 3 categories
root harness tests fail because A3.6 harness does not exist yet
```

If the synthetic reachability fixture fails for Site Boundary, Navigation Grid, invalid config, missing R6A admission, or goal-shot behavior instead of the intended missing `+0.5/R` reachability, classify RED as invalid and fix the test before production code.

**Manual Gate G1:** operator must report collected/passed/failed/error counts and the failure families. No A3.6 production code before a valid G1 RED.

---

### Task 2: Implement the minimal five-curvature production amendment

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py`

**Interfaces:**
- Consumes: `_PRIMITIVE_CURVATURE_FRACTIONS`, `_primitive_curvature_values(radius)` required by Task 1 tests.
- Produces: deterministic five-category primitive generation with categorical indices `-2,-1,0,1,2`; existing planner results and diagnostics remain the same except for the larger search lattice and resulting connector outcomes/counters.

- [ ] **Step 1: Add the fixed curvature family and validator helper**

Near the existing edge constants add:

```python
_PRIMITIVE_CURVATURE_FRACTIONS = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _primitive_curvature_values(radius: float) -> tuple[float, ...]:
    radius = float(radius)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("primitive curvature radius must be finite and > 0")
    return tuple(fraction / radius for fraction in _PRIMITIVE_CURVATURE_FRACTIONS)
```

Do not add a config field for these fractions.

- [ ] **Step 2: Replace the three-value search tuple**

Replace:

```python
curvature_values = (-1.0 / radius, 0.0, 1.0 / radius)
```

with:

```python
curvature_values = _primitive_curvature_values(radius)
```

- [ ] **Step 3: Preserve categorical steering indexing with straight at zero**

Replace:

```python
for curvature_index, curvature in enumerate(curvature_values, start=-1):
```

with:

```python
for curvature_index, curvature in enumerate(curvature_values, start=-2):
```

Do not change the steering penalty formula:

```python
steering_change = (
    cfg.steering_change_penalty_m
    if curvature_index != node.last_curvature_index
    else 0.0
)
```

The initial `_SearchNode.last_curvature_index = 0` therefore continues to mean straight.

- [ ] **Step 4: Remove the hard-coded three-edge diagnostic assumption**

Replace:

```python
if node.travel_m + cfg.primitive_length_m > cfg.max_path_length_m:
    diagnostics.primitive_edges_considered += 3
    diagnostics.primitive_edges_rejected_path_length += 3
    continue
```

with:

```python
if node.travel_m + cfg.primitive_length_m > cfg.max_path_length_m:
    primitive_count = len(curvature_values)
    diagnostics.primitive_edges_considered += primitive_count
    diagnostics.primitive_edges_rejected_path_length += primitive_count
    continue
```

This must remain derived from the actual family cardinality rather than hard-coding `5`.

- [ ] **Step 5: Run only the A3.6 connector-focused tests**

```bash
PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  -k a36
```

Expected after implementation: all A3.6 connector tests PASS.

Do not claim this result unless the operator supplies the output.

- [ ] **Step 6: Commit the minimal production change only after GREEN evidence**

```bash
git add src/agt_offline_assets/agt_offline_assets/reverse_primitive_connector.py
git diff --cached --check
git commit -m "feat(v25-12g): add intermediate R6B curvature primitives"
```

---

### Task 3: Implement the read-only A3.6 real-data acceptance harness

**Files:**
- Create: `tools/v25_12g_a36_intermediate_curvature_acceptance.py`
- Test: `tests/test_v25_12g_a36_intermediate_curvature_contract.py`

**Interfaces:**
- Consumes: existing strict motion-graph IO, `derive_vehicle_feasible_motion_graph`, the same real run assets used by A3.5, and the amended R6B behavior from Task 2.
- Produces: structured JSON report with behavior-preservation gates, transition delta, A4 minimum-chain diagnostic predicate and process resource metrics. It writes no runtime/map assets.

- [ ] **Step 1: Define frozen public harness constants and parser**

The file begins with:

```python
#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import resource
import time
from pathlib import Path
from typing import Any, Mapping

REPORT_SCHEMA = "agt_v25_12g_a36_intermediate_curvature_acceptance/v1"
VALIDATION_SCOPE = (
    "A36_INTERMEDIATE_CURVATURE_EXPERIMENT_NOT_ROUTE_READY_NOT_GLOBAL_INFEASIBILITY_PROOF"
)
BASELINE_MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
EXPECTED_CONNECTOR_COUNT = 40
EXPECTED_SERVICE_ACTION_COUNT = 32
FORBIDDEN_SEMANTIC_KEYS = frozenset(
    {"route_ready", "reachable_from_start", "optimal"}
)
```

`build_parser()` must require only:

```text
--run-dir
--vehicle-profile
```

and default the same run assets as A3.5:

```text
--baseline-motion-graph vehicle_feasible_motion_graph.yaml
--service-graph vehicle_feasible_service_graph.yaml
--turn-zones turn_zones.yaml
--navigation-map navigation_map.yaml
--navigation-pgm navigation_map.pgm
--site-boundary site_boundary.yaml
--segments vehicle_feasible_segments.yaml
--derivation derivation.yaml
--aisle-graph aisle_graph.yaml
--pretty
```

Do not expose any write/overwrite option.

- [ ] **Step 2: Reuse A3.5-style preflight/hash/forbidden-key logic without importing the A3.5 script**

Implement local helpers:

```python
def _require_file(path: Path) -> Path: ...
def _resolve_run_asset(run_dir: Path, value: str) -> Path: ...
def _preflight(args: argparse.Namespace) -> dict[str, Path]: ...
def _sha256(path: Path) -> str: ...
def _input_hashes(paths: Mapping[str, Path]) -> dict[str, str]: ...
def assert_no_forbidden_semantic_keys(value: Any, path: str = "root") -> None: ...
```

Preserve the same recursive fail-closed semantics as A3.5. Do not import `tools/v25_12g_a35_connector_diagnosis.py` as a runtime dependency.

- [ ] **Step 3: Implement service behavior preservation**

```python
def service_behavior_projection(payload: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "service_actions": copy.deepcopy(payload.get("service_actions", [])),
        "executable_service_action_ids": list(payload.get("executable_service_action_ids", [])),
    }


def assert_service_behavior_equal(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
) -> None:
    if service_behavior_projection(baseline) != service_behavior_projection(current):
        raise ValueError("A3.6 service behavior drift")
```

A3.6 intentionally allows transition status/path/counter changes, so do not reuse the A3.5 full behavior projection equality check.

- [ ] **Step 4: Implement deterministic transition outcome comparison**

```python
def _transition_by_id(payload: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    output = {}
    for raw in payload.get("transition_validations", []):
        item = dict(raw)
        connector_id = str(item.get("connector_candidate_id", ""))
        if not connector_id or connector_id in output:
            raise ValueError(f"duplicate or empty connector id: {connector_id}")
        output[connector_id] = item
    return output


def compare_transition_outcomes(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
) -> dict[str, Any]:
    before = _transition_by_id(baseline)
    after = _transition_by_id(current)
    if set(before) != set(after):
        raise ValueError("A3.6 connector universe drift")

    newly_executable = []
    still_rejected = []
    regressed = []
    for connector_id in sorted(before):
        old_status = str(before[connector_id].get("status", ""))
        new_status = str(after[connector_id].get("status", ""))
        if old_status != "EXECUTABLE" and new_status == "EXECUTABLE":
            newly_executable.append(connector_id)
        if old_status == "REJECTED" and new_status == "REJECTED":
            still_rejected.append(connector_id)
        if old_status == "EXECUTABLE" and new_status != "EXECUTABLE":
            regressed.append(connector_id)
        if old_status == "REJECTED" and new_status == "UNRESOLVED":
            regressed.append(connector_id)

    return {
        "newly_executable_connector_ids": newly_executable,
        "still_rejected_connector_ids": still_rejected,
        "unexpectedly_regressed_connector_ids": sorted(set(regressed)),
    }
```

The real A3.5 baseline contains `0 EXECUTABLE / 40 REJECTED`, but keep the helper fail-closed for future reuse.

- [ ] **Step 5: Implement the exact A4 minimum continuous-chain predicate**

```python
def find_a4_minimum_chain(payload: Mapping[str, Any]) -> dict[str, Any]:
    services = {
        str(item["service_state_id"]): dict(item)
        for item in payload.get("service_actions", [])
    }
    transitions = sorted(
        (
            dict(item)
            for item in payload.get("transition_validations", [])
            if item.get("status") == "EXECUTABLE"
        ),
        key=lambda item: str(item.get("connector_candidate_id", "")),
    )

    for first in transitions:
        for second in transitions:
            if first is second:
                continue
            if first.get("to_service_state_id") != second.get("from_service_state_id"):
                continue
            state_ids = [
                str(first.get("from_service_state_id", "")),
                str(first.get("to_service_state_id", "")),
                str(second.get("to_service_state_id", "")),
            ]
            if any(state_id not in services for state_id in state_ids):
                continue
            segment_ids = [
                str(first.get("from_segment_id", "")),
                str(first.get("to_segment_id", "")),
                str(second.get("to_segment_id", "")),
            ]
            aisle_ids = [str(services[state_id].get("aisle_id", "")) for state_id in state_ids]
            if len(set(segment_ids)) < 3 or len(set(aisle_ids)) < 2:
                continue
            return {
                "a4_entry_gate_passed": True,
                "connector_candidate_ids": [
                    str(first["connector_candidate_id"]),
                    str(second["connector_candidate_id"]),
                ],
                "service_state_ids": state_ids,
                "segment_ids": segment_ids,
                "aisle_ids": aisle_ids,
            }

    return {
        "a4_entry_gate_passed": False,
        "connector_candidate_ids": [],
        "service_state_ids": [],
        "segment_ids": [],
        "aisle_ids": [],
    }
```

This is only the frozen engineering entry predicate; it must not be serialized as route readiness or global connectivity.

- [ ] **Step 6: Derive the amended graph in memory and enforce experiment integrity**

Load inputs with existing package loaders, then:

```python
baseline_graph = load_vehicle_feasible_motion_graph(paths["baseline_motion_graph"])
baseline_payload = vehicle_feasible_motion_graph_to_dict(baseline_graph)

current_graph = derive_vehicle_feasible_motion_graph(
    service_graph,
    turn_zones,
    navigation,
    vehicle,
    site_boundary=boundary,
    source={
        "acceptance_stage": "v25_12g_a36_intermediate_curvature",
        "experiment_only": True,
    },
)
current_payload = vehicle_feasible_motion_graph_to_dict(current_graph)
```

Then require:

```text
len(service_actions) == 32
len(transition_validations) == 40
service behavior exact equality
connector ID universe exact equality
no unexpected regressions
no forbidden semantic keys
protected input hashes unchanged
```

Do not require any connector to become executable.

- [ ] **Step 7: Report deterministic transition summaries and search counters**

Report at least:

```text
baseline transition status counts
amended transition status counts
newly executable connector IDs
still rejected connector IDs
unexpectedly regressed connector IDs
queue-exhausted count among amended R6B failures
expansion-budget-reached count among amended R6B failures
expansion histogram using the A3.5 frozen bins
A4 minimum-chain predicate/result
```

For an amended executable connector, do not require `failure_class`; successful R6B diagnostics legitimately have `failure_class=None`.

- [ ] **Step 8: Add process resource metrics without changing acceptance semantics**

At `main()` entry capture:

```python
wall_start = time.perf_counter()
usage_start = resource.getrusage(resource.RUSAGE_SELF)
```

Before output capture end values and emit:

```text
wall_seconds
user_seconds
system_seconds
cpu_percent_approx
max_rss_kb
```

These fields are observational and must not influence RC except for serialization validity.

- [ ] **Step 9: Run the root harness contract test**

```bash
PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q tests/test_v25_12g_a36_intermediate_curvature_contract.py
```

Expected: all PASS after harness implementation.

- [ ] **Step 10: Commit the harness**

```bash
git add \
  tools/v25_12g_a36_intermediate_curvature_acceptance.py

git diff --cached --check
git commit -m "feat(v25-12g): add A3.6 intermediate-curvature acceptance harness"
```

The root contract test was already committed in the RED batch; do not recommit it here unless a genuine test-contract correction was required and documented.

---

### Task 4: Target-machine GREEN and regression gate

**Files:**
- No new files expected.

**Interfaces:**
- Consumes: Task 1 tests, Task 2 production change, Task 3 harness.
- Produces: operator-supplied evidence that the focused A3.6 behavior, package build/tests and existing A3.5 diagnostic contracts remain valid.

- [ ] **Step 1: Focused A3.6 pytest**

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a36_intermediate_curvature_contract.py \
  -k a36
```

Expected: `0 failed`, `0 errors`.

- [ ] **Step 2: Existing A3.5 diagnostic regression subset**

```bash
PYTHONPATH="$PWD/src/agt_offline_assets:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/agt_offline_assets/test/test_reverse_primitive_diagnostics.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  tests/test_v25_12g_a35_connector_diagnosis_contract.py
```

Do not require old real-data behavior equality here; this is unit/contract regression only.

- [ ] **Step 3: Package build and package tests**

```bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --verbose
```

Interpret `agt_offline_assets` package results separately from unrelated historical third-party workspace failures.

- [ ] **Step 4: Freeze GREEN code HEAD**

After all intended A3.6 focused/package checks pass, record:

```bash
git status --short
git rev-parse HEAD
git log --oneline -8
```

Confirm protected paths remain unstaged and unchanged by A3.6.

**Manual Gate G2:** stop and report exact focused counts, A3.5 regression counts, build result, package CTest result, HEAD and `git status --short`. Do not execute the real-map A3.6 acceptance until G2 is valid.

---

### Task 5: Execute one all-40 A3.6 real-data experiment

**Files:**
- Generated temporary evidence only under `/tmp`; do not write runtime/map assets.

**Interfaces:**
- Consumes: frozen A3.5 baseline `vehicle_feasible_motion_graph.yaml`, canonical `mk_mini` profile, current real run directory.
- Produces: exactly one all-40 A3.6 JSON experiment result plus external timing log.

- [ ] **Step 1: Run the A3.6 harness exactly once**

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

rm -f \
  /tmp/v25_12g_a36_intermediate_curvature.json \
  /tmp/v25_12g_a36_intermediate_curvature.time

/usr/bin/time -v \
python3 tools/v25_12g_a36_intermediate_curvature_acceptance.py \
  --run-dir runtime/maps/agt_workbench_run \
  --vehicle-profile profiles/platforms/mk_mini.yaml \
  --baseline-motion-graph vehicle_feasible_motion_graph.yaml \
  --pretty \
  > /tmp/v25_12g_a36_intermediate_curvature.json \
  2> /tmp/v25_12g_a36_intermediate_curvature.time

A36_RC=$?
echo "A36_RC=$A36_RC"
```

Do not rerun merely because connectivity did not improve.

- [ ] **Step 2: If RC is non-zero, stop without tuning**

Print the timing/error stream and preserve any partial JSON. Classify whether failure is:

```text
preflight/input
service behavior drift
connector universe drift
unexpected regression
forbidden semantics
protected input hash mutation
serialization/runtime error
```

Do not change a planner parameter automatically.

- [ ] **Step 3: If RC is zero, inspect only the saved JSON**

Use a short read-only summary script; do not rederive the motion graph. Report:

```text
baseline transition counts
amended transition counts
newly executable connector IDs
still rejected connector count/IDs
unexpectedly regressed connector IDs
queue exhausted count
expansion budget reached count
expansion histogram
A4 entry gate passed
A4 witness chain if present
protected hashes unchanged
forbidden semantic key count
```

- [ ] **Step 4: Record external `/usr/bin/time -v` evidence**

```bash
grep -E \
'Elapsed|User time|System time|Maximum resident set size|Percent of CPU' \
/tmp/v25_12g_a36_intermediate_curvature.time
```

**Manual Gate G3:** operator supplies the saved result summary. A result of `0 newly executable` is a valid negative experiment if all integrity gates pass.

---

### Task 6: Freeze A3.6 evidence and decide only the A4 gate

**Files:**
- Create: `docs/v2.5/V25_12G_A36_INTERMEDIATE_CURVATURE_2026-08-17.md`
- Modify: `docs/v2.5/V25_12G_A36_HANDOFF_2026-08-17.md`

**Interfaces:**
- Consumes: operator-supplied G1/G2/G3 evidence and exact frozen code HEAD.
- Produces: immutable positive/partial/negative A3.6 evidence; no second backend amendment.

- [ ] **Step 1: Write evidence provenance exactly as observed**

Record:

```text
RED commit/HEAD and intended failure families
GREEN implementation HEAD
focused/package verification counts
single real-data run RC and resource usage
protected local paths
```

Never invent unsupplied per-connector values.

- [ ] **Step 2: Record the experiment result family**

Classify only as:

```text
POSITIVE: >=1 newly executable connector
PARTIAL: connectivity improves but A4 chain is absent
NEGATIVE: 0 newly executable connectors
```

`POSITIVE` does not itself mean A4 may start; A4 requires the exact minimum-chain predicate.

- [ ] **Step 3: Record A4 eligibility separately**

Only if the saved A3.6 JSON contains a witness with:

```text
2 continuous EXECUTABLE transitions
3 distinct segment IDs
>=2 aisle IDs
```

may the evidence document state:

```text
A4 minimum engineering entry predicate satisfied
```

It must still avoid route-ready/optimal/global-connectivity claims.

- [ ] **Step 4: Stop before any second amendment**

If A4 predicate is false, do not add more curvature fractions or change terminal/search parameters in A3.6. A new hypothesis requires a new brainstorm/spec cycle.

- [ ] **Step 5: Commit evidence docs only**

```bash
git add \
  docs/v2.5/V25_12G_A36_INTERMEDIATE_CURVATURE_2026-08-17.md \
  docs/v2.5/V25_12G_A36_HANDOFF_2026-08-17.md

git diff --cached --check
git commit -m "docs(v25-12g): freeze A3.6 intermediate-curvature evidence"
```

---

## Plan Self-Review

### Spec coverage

- Fixed five-curvature family: Task 1 + Task 2.
- `abs(curvature) <= 1/R`: Task 1 helper contract + Task 2 helper.
- No curvature parameter sweep: Global Constraints + Task 2.
- Synthetic newly reachable `+0.5/R` state: Task 1.
- Five-edge path-length diagnostic accounting: Task 1 + Task 2.
- Site Boundary / Navigation Grid / footprint / padding / turning radius frozen: Global Constraints + Task 4 regression gate.
- Goal shot, state resolution, costs, cusp/path/expansion budgets frozen: Global Constraints + narrow Task 2 file surface.
- No motion-graph schema change: File Structure.
- Read-only real-data harness: Task 3.
- Exact service behavior and connector universe preservation: Task 3.
- A4 continuous-chain predicate: Task 3 + Task 5/6.
- Negative real-data result accepted: Task 5 + Task 6.
- No automatic second amendment: Global Constraints + Task 6.
- Protected local paths: Global Constraints and all explicit `git add` commands.

### Placeholder scan

No `TBD`, `TODO`, “implement later”, generic error-handling placeholders, or unnamed test steps are permitted by this plan. Helper bodies that are abbreviated with `...` are limited to preflight/hash functions whose exact A3.5 behavior is already frozen in `tools/v25_12g_a35_connector_diagnosis.py`; during execution they must be copied semantically, not left incomplete in production.

### Type / naming consistency

The plan consistently uses:

```text
_PRIMITIVE_CURVATURE_FRACTIONS
_primitive_curvature_values(radius: float) -> tuple[float, ...]
assert_service_behavior_equal(baseline, current)
compare_transition_outcomes(baseline, current)
find_a4_minimum_chain(payload)
REPORT_SCHEMA = agt_v25_12g_a36_intermediate_curvature_acceptance/v1
```

No task introduces a motion-graph schema or dataclass field change.

## Execution Handoff

The user already requested progression up to the point where target-machine Codex is required, so execution should proceed inline only through **Task 1 tests-only RED landing** and then stop at **Manual Gate G1**. Do not ask the user to choose subagent-driven execution because no subagent runtime is available in this session and the requested handoff target is explicitly the local Codex RED gate.
