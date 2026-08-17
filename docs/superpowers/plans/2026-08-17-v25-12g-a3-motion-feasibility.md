# V25-12G-A3 Vehicle-Feasible Motion Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the frozen A2 static service topology into one deterministic `agt_vehicle_feasible_motion_graph/v1` asset whose service actions and headland transitions carry local vehicle-motion evidence, without claiming START_POSE reachability, route readiness, or optimality.

**Architecture:** Keep A2 immutable. Add a focused A3 core model, a service-motion validator, a connector-motion orchestrator that adapts A2 candidates into the existing R5/R6A/R6B chain, and a strict IO module. Ordinary service actions revalidate A1-derived centerlines directionally; dead-end actions use exact forward-in/reverse-out retrace; connector actions reuse existing forward gate/audit/admission/reverse search. A diagnostic acceptance harness may write only `vehicle_feasible_motion_graph.yaml`.

**Tech Stack:** Python 3.10, dataclasses, math, pathlib, PyYAML, NumPy, ROS 2 Humble, `ament_cmake_pytest`, existing `VehicleFeasibleServiceGraph`, `TurnZoneSet`, `NavigationGridEvidence`, `CanonicalVehicleProfile`, `SiteBoundary`, `ForwardConnectorNavigationPlan`, `ForwardConnectorCandidateAuditPlan`, `ReverseFallbackAdmissionPlan`, and `ReversePrimitiveConnectorPlan`.

## Global Constraints

- Work only on `feat/v25-12g-maximum-feasible-coverage` in the existing checkout; do not create another worktree.
- Do not use `git reset --hard` or `git clean`.
- Never touch or stage `tools/rosbag_sensor_trimmer` or `tools/map_tools/render_pcd_top_views.py`.
- Frozen output filename: `vehicle_feasible_motion_graph.yaml`.
- Frozen schema: `agt_vehicle_feasible_motion_graph/v1`.
- Frozen top-level status: `MOTION_EVIDENCE_ONLY`.
- A2 is immutable. A3 must not rewrite `vehicle_feasible_service_graph.yaml` or any A1/A2 upstream asset.
- A3 may write only its own new sibling runtime asset.
- A3 must produce exactly one service validation per A2 service state and exactly one transition validation per A2 connector candidate.
- Result classes are exactly `EXECUTABLE`, `REJECTED`, and `UNRESOLVED`.
- A bounded reverse-search failure is not global infeasibility; use `BOUNDED_SEARCH_NO_SOLUTION`.
- Site Boundary touch/cross is a hard rejection and never enters reverse fallback.
- UNKNOWN or insufficient grid coverage never becomes FREE and remains unresolved evidence unless a separate OCCUPIED hard conflict already proves rejection.
- `operator_approved_mixed_connector_ids` is frozen to `()` for A3 v1.
- Do not increase R6B `max_expansions`, `max_path_length_m`, `max_cusps`, or other default search budgets to improve connectivity.
- Ordinary LOW->HIGH and HIGH->LOW services both use forward gear; HIGH->LOW changes yaw by pi and reverses point order.
- `DEAD_END_FORWARD_IN_REVERSE_OUT` never calls R6B; it uses exact reverse retrace of the forward-in geometry.
- Dead-end reverse samples keep the forward-in yaw; only `motion_direction` becomes `REVERSE`.
- Dead-end cusp is one zero-distance marker at the terminal pose, then reverse resumes from the preceding geometric sample so travel length is not double-counted.
- Pose cross-reference tolerance is `1e-6 m` for position and `1e-6 rad` for yaw.
- Coverage reward is A2 useful segment length; actual travel distance is the 3D polyline arc length of the validated motion samples.
- A physical segment contributes locally validated coverage once if at least one service action for its `coverage_segment_id` is `EXECUTABLE`.
- A3 must never emit or serialize `route_ready`, `reachable_from_start`, `optimal`, or equivalent claims.
- Semantic-identical inputs must produce byte-identical YAML.
- Strict TDD is mandatory. No behavior implementation before the operator has shown the corresponding RED evidence.
- Operator test gates are intentionally batched to reduce interruptions; implementation commits remain small between gates.
- Never claim tests/build/real-data acceptance pass until operator-machine output proves it.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_feasible_motion_graph.py          # A3 public dataclasses/constants + top-level derivation/diagnostics
  vehicle_feasible_service_motion.py        # directional service samples, dead-end retrace, footprint classification
  vehicle_feasible_transition_motion.py     # A2->ConnectorRequest adapter + R5/R6A/R6B orchestration
  vehicle_feasible_motion_graph_io.py       # deterministic serializer + strict loader
  forward_connector_candidate_audit.py      # optional Site Boundary-aware audit evidence, default behavior unchanged
  __init__.py

src/agt_offline_assets/test/
  test_vehicle_feasible_motion_graph.py
  test_forward_connector_candidate_audit.py

src/agt_offline_assets/CMakeLists.txt

tools/
  v25_12g_a3_acceptance.py

tests/
  test_v25_12g_a3_contract.py

docs/v2.5/
  V25_12G_A3_CURRENT_STATE.md
  V25_12G_A3_REAL_DATA_2026-08-17.md
```

Do not modify A1/A2 derivation modules. Existing R5/R6A/R6B algorithms remain the planning backends; only the forward candidate audit receives optional structured Site Boundary evidence needed to stop boundary-conflicting candidates before R6A admission.

---

## Frozen Public Contract

### Core constants

```python
VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA = "agt_vehicle_feasible_motion_graph/v1"
MOTION_EVIDENCE_ONLY = "MOTION_EVIDENCE_ONLY"

EXECUTABLE = "EXECUTABLE"
REJECTED = "REJECTED"
UNRESOLVED = "UNRESOLVED"

LOCAL_MOTION_EXECUTABLE = "LOCAL_MOTION_EXECUTABLE"
PROVEN_HARD_CONSTRAINT_REJECTION = "PROVEN_HARD_CONSTRAINT_REJECTION"
BOUNDED_SEARCH_NO_SOLUTION = "BOUNDED_SEARCH_NO_SOLUTION"
MAP_EVIDENCE_INSUFFICIENT = "MAP_EVIDENCE_INSUFFICIENT"
MIXED_EVIDENCE_REQUIRES_REVIEW = "MIXED_EVIDENCE_REQUIRES_REVIEW"
POLICY_REVIEW_REQUIRED = "POLICY_REVIEW_REQUIRED"

A1_CENTERLINE_DIRECTIONAL_REVALIDATION = "A1_CENTERLINE_DIRECTIONAL_REVALIDATION"
A1_CENTERLINE_EXACT_REVERSE_RETRACE = "A1_CENTERLINE_EXACT_REVERSE_RETRACE"
FORWARD_DUBINS_NAVIGATION_GATE = "FORWARD_DUBINS_NAVIGATION_GATE"
BOUNDED_REVERSE_PRIMITIVE_SEARCH = "BOUNDED_REVERSE_PRIMITIVE_SEARCH"
```

### Configuration

```python
@dataclass(frozen=True)
class VehicleFeasibleMotionGraphConfig:
    preview_footprint_padding_m: float = 0.05
    pose_position_tolerance_m: float = 1.0e-6
    pose_yaw_tolerance_rad: float = 1.0e-6

    def validate(self) -> None:
        if self.preview_footprint_padding_m < 0.0:
            raise ValueError("preview_footprint_padding_m must be >= 0")
        if self.pose_position_tolerance_m <= 0.0:
            raise ValueError("pose_position_tolerance_m must be > 0")
        if self.pose_yaw_tolerance_rad <= 0.0:
            raise ValueError("pose_yaw_tolerance_rad must be > 0")
```

### Core dataclasses

```python
@dataclass(frozen=True)
class MotionSample:
    x: float
    y: float
    z: float
    yaw: float
    motion_direction: str
    segment_index: int
    is_cusp: bool = False


@dataclass(frozen=True)
class ServiceActionValidation:
    service_state_id: str
    segment_id: str
    aisle_id: str
    service_type: str
    entry_pose: tuple[float, float, float, float]
    exit_pose: tuple[float, float, float, float]
    status: str
    proof_scope: str
    backend: str
    backend_status: str
    reason: str
    coverage_segment_id: str
    coverage_reward_length_m: float
    path_length_m: float
    forward_distance_m: float
    reverse_distance_m: float
    cusp_count: int
    samples: tuple[MotionSample, ...]
    footprint_evidence: GridPathEvidence


@dataclass(frozen=True)
class TransitionValidation:
    connector_candidate_id: str
    from_service_state_id: str
    to_service_state_id: str
    from_segment_id: str
    to_segment_id: str
    side: str
    turn_zone_id: str
    start_pose: tuple[float, float, float, float]
    goal_pose: tuple[float, float, float, float]
    status: str
    proof_scope: str
    backend: str
    backend_status: str
    reason: str
    path_length_m: float
    forward_distance_m: float
    reverse_distance_m: float
    cusp_count: int
    search_expansions: int
    samples: tuple[MotionSample, ...]
    forward_evidence: Mapping[str, Any]
    reverse_admission_evidence: Mapping[str, Any]


@dataclass(frozen=True)
class MotionGraphDiagnostics:
    a2_service_state_count: int
    service_validation_count: int
    executable_service_action_count: int
    rejected_service_action_count: int
    unresolved_service_action_count: int
    ordinary_service_action_count: int
    dead_end_service_action_count: int
    executable_dead_end_service_action_count: int
    a2_connector_candidate_count: int
    transition_validation_count: int
    executable_transition_count: int
    forward_executable_transition_count: int
    reverse_executable_transition_count: int
    rejected_transition_count: int
    unresolved_transition_count: int
    locally_validated_segment_count: int
    locally_validated_unique_coverage_length_m: float
    distinct_locally_validated_aisle_count: int


@dataclass(frozen=True)
class VehicleFeasibleMotionGraph:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    row_direction_xy: tuple[float, float]
    service_actions: tuple[ServiceActionValidation, ...]
    transition_validations: tuple[TransitionValidation, ...]
    executable_service_action_ids: tuple[str, ...]
    executable_transition_ids: tuple[str, ...]
    diagnostics: MotionGraphDiagnostics
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA
    status: str = MOTION_EVIDENCE_ONLY
```

### Exact top-level signatures

```python
def derive_vehicle_feasible_motion_graph(
    service_graph: VehicleFeasibleServiceGraph,
    turn_zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleMotionGraph:
    """Validate every A2 service state and connector candidate into local motion evidence."""


def write_vehicle_feasible_motion_graph(
    graph: VehicleFeasibleMotionGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    """Write only the A3 sibling asset, refusing overwrite by default."""


def load_vehicle_feasible_motion_graph(
    path: str | Path,
) -> VehicleFeasibleMotionGraph:
    """Strict-load one v1 A3 graph and reject malformed motion evidence."""
```

---

## Batched TDD Execution Policy

The operator asked for fewer interruptions. Use three explicit operator gates only:

```text
Gate R0  -> prove the new A3 module is absent before any A3 production file exists
Gate R1  -> after skeleton-only import surface exists, run the complete behavior/contract suite and capture specific RED failures
Gate G1  -> after Tasks 3-7 implementation, run all focused/package/contract tests in one GREEN batch
```

No behavior implementation is allowed between R0 and R1. After R1 has shown specific behavior failures, Tasks 3-7 may be implemented and committed without another operator interruption until G1.

---

### Task 1: R0 module-absence test and import skeleton

**Files:**
- Create test-only first: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Then, only after operator RED: create `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`

**Interfaces:** This task produces only the importable public constants/config/dataclass names above; no derivation logic and no serializer.

- [ ] **Step 1: Add the first failing test without creating the production module**

```python
import importlib


def test_a3_public_module_exists_with_frozen_schema():
    module = importlib.import_module(
        "agt_offline_assets.vehicle_feasible_motion_graph"
    )
    assert module.VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA == (
        "agt_vehicle_feasible_motion_graph/v1"
    )
    assert module.MOTION_EVIDENCE_ONLY == "MOTION_EVIDENCE_ONLY"
```

- [ ] **Step 2: Commit the test only**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py
git commit -m "test(v25-12g): require A3 motion graph contract"
```

- [ ] **Step 3: Operator Gate R0**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py::test_a3_public_module_exists_with_frozen_schema
```

Expected RED: `ModuleNotFoundError: No module named 'agt_offline_assets.vehicle_feasible_motion_graph'`.

- [ ] **Step 4: After R0 evidence, add import skeleton only**

The new module must contain the frozen constants, `VehicleFeasibleMotionGraphConfig`, and dataclass declarations from the public contract. The only callable body at this point is `VehicleFeasibleMotionGraphConfig.validate()`; do not add derivation or validation behavior yet.

- [ ] **Step 5: Commit skeleton**

```bash
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py
git commit -m "feat(v25-12g): define A3 motion graph contract"
```

Do not ask the operator to run GREEN here. Move directly to Task 2 and prepare the complete behavior RED batch.

---

### Task 2: Write the complete A3 behavior and acceptance tests, then Gate R1

**Files:**
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Modify: `src/agt_offline_assets/test/test_forward_connector_candidate_audit.py`
- Create: `tests/test_v25_12g_a3_contract.py`

**Interfaces:** Tests freeze `validate_service_actions()`, `adapt_connector_candidates()`, `validate_transition_candidates()`, top-level derivation, deterministic IO, acceptance harness CLI, and the Site Boundary-aware audit extension before any of those behaviors are implemented.

- [ ] **Step 1: Add focused fixtures**

Use small A2 fixtures built directly from the existing dataclasses so tests do not depend on real runtime files:

```python
import math
import pytest
import numpy as np

from agt_offline_assets.forward_connector_navigation_gate import GridPathEvidence
from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_feasible_service_graph import (
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    ConnectorCandidate,
    ServiceGraphDiagnostics,
    ServiceResource,
    ServiceState,
    VehicleFeasibleServiceGraph,
)


def _resource(
    segment_id="aisle_001.segment_001",
    *,
    y=0.0,
    low_type="LOW_U_HEADLAND",
    high_type="HIGH_U_HEADLAND",
):
    return ServiceResource(
        segment_id=segment_id,
        aisle_id=segment_id.split(".")[0],
        ordinal_in_aisle=1,
        coverage_length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=low_type,
        high_endpoint_type=high_type,
        low_endpoint_pose=(0.0, y, 0.0, 0.0),
        high_endpoint_pose=(4.0, y, 0.0, 0.0),
        centerline_xyz=((0.0, y, 0.0), (2.0, y, 0.0), (4.0, y, 0.0)),
    )
```

Add helper constructors for ordinary LOW->HIGH, ordinary HIGH->LOW, one-headland dead-end states, a two-resource connector graph, a FREE navigation grid, an UNKNOWN navigation grid, an OCCUPIED-cell grid, and a simple rectangular `SiteBoundary` using the exact existing constructor signature from `site_boundary.py`.

- [ ] **Step 2: Freeze ordinary directional service behavior**

```python
def test_low_to_high_service_is_forward_and_uses_actual_polyline_length():
    action = _validate_single_service(_graph_with_low_to_high(), _free_navigation())
    assert action.status == EXECUTABLE
    assert [sample.motion_direction for sample in action.samples] == ["FORWARD"] * 3
    assert [sample.x for sample in action.samples] == [0.0, 2.0, 4.0]
    assert all(sample.yaw == pytest.approx(0.0) for sample in action.samples)
    assert action.path_length_m == pytest.approx(4.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(0.0)
    assert action.cusp_count == 0


def test_high_to_low_service_reverses_points_but_stays_forward_gear():
    action = _validate_single_service(_graph_with_high_to_low(), _free_navigation())
    assert action.status == EXECUTABLE
    assert [sample.x for sample in action.samples] == [4.0, 2.0, 0.0]
    assert all(sample.motion_direction == "FORWARD" for sample in action.samples)
    assert all(abs(abs(sample.yaw) - math.pi) <= 1.0e-9 for sample in action.samples)
```

- [ ] **Step 3: Freeze service evidence classification**

```python
def test_service_occupied_footprint_is_rejected():
    action = _validate_single_service(_graph_with_low_to_high(), _occupied_navigation())
    assert action.status == REJECTED
    assert action.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION


def test_service_unknown_footprint_is_unresolved():
    action = _validate_single_service(_graph_with_low_to_high(), _unknown_navigation())
    assert action.status == UNRESOLVED
    assert action.proof_scope == MAP_EVIDENCE_INSUFFICIENT


def test_service_site_boundary_conflict_is_hard_rejection():
    action = _validate_single_service(
        _graph_with_low_to_high(),
        _free_navigation(),
        site_boundary=_boundary_that_clips_high_endpoint(),
    )
    assert action.status == REJECTED
    assert action.backend_status == "SITE_BOUNDARY_CONFLICT"
```

- [ ] **Step 4: Freeze exact dead-end retrace semantics**

```python
def test_dead_end_is_forward_in_then_exact_reverse_retrace_with_one_cusp():
    action = _validate_single_service(_graph_with_dead_end(), _free_navigation())
    assert action.backend == A1_CENTERLINE_EXACT_REVERSE_RETRACE
    assert action.cusp_count == 1
    assert action.coverage_reward_length_m == pytest.approx(4.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(4.0)
    assert action.path_length_m == pytest.approx(8.0)

    cusp_indices = [i for i, sample in enumerate(action.samples) if sample.is_cusp]
    assert len(cusp_indices) == 1
    cusp = cusp_indices[0]
    assert action.samples[cusp - 1].x == pytest.approx(4.0)
    assert action.samples[cusp].x == pytest.approx(4.0)
    assert action.samples[cusp].motion_direction == "REVERSE"
    assert action.samples[cusp].yaw == pytest.approx(action.samples[cusp - 1].yaw)

    forward_xyz = [(s.x, s.y, s.z) for s in action.samples[:cusp]]
    reverse_xyz = [(s.x, s.y, s.z) for s in action.samples[cusp + 1 :]]
    assert reverse_xyz == list(reversed(forward_xyz[:-1]))
```

Also add the HIGH_U-headland symmetric test and a test proving `validate_service_actions()` never imports/calls `derive_reverse_primitive_connector_plan` for dead-end actions.

- [ ] **Step 5: Freeze pose cross-reference failures**

```python
def test_service_entry_pose_mismatch_fails_closed():
    graph = _graph_with_low_to_high(entry_x=0.01)
    with pytest.raises(ValueError, match="entry pose"):
        validate_service_actions(graph, _free_navigation(), _vehicle())
```

Add equivalent exit-pose, missing-resource, non-finite-centerline, duplicate-state-ID, platform/profile-hash, frame, and row-direction contract tests.

- [ ] **Step 6: Extend forward audit tests for Site Boundary-aware classification**

The new optional argument is:

```python
def derive_forward_connector_candidate_audit(
    connector_requests,
    zones,
    navigation,
    vehicle,
    config=None,
    *,
    site_boundary=None,
    source=None,
): ...
```

Freeze two behaviors:

```python
def test_candidate_audit_default_without_boundary_is_backward_compatible():
    baseline = derive_forward_connector_candidate_audit(
        requests, zones, navigation, vehicle
    )
    explicit_none = derive_forward_connector_candidate_audit(
        requests, zones, navigation, vehicle, site_boundary=None
    )
    assert baseline == explicit_none


def test_candidate_audit_reports_local_site_boundary_conflict_before_r6a():
    plan = derive_forward_connector_candidate_audit(
        requests,
        zones,
        navigation,
        vehicle,
        site_boundary=boundary,
    )
    assert plan.connectors[0].status == "LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT"
```

- [ ] **Step 7: Freeze A2 connector adapter invariants**

```python
def test_a2_connector_candidate_maps_one_to_one_to_connector_request():
    requests, bindings = adapt_connector_candidates(_two_resource_graph(), _zones())
    assert len(requests) == 1
    request = requests[0]
    binding = bindings[request.connector_id]
    assert request.connector_id == binding.connector_candidate_id
    assert request.start_pose == binding.start_pose
    assert request.goal_pose == binding.goal_pose


def test_connector_start_not_equal_source_exit_fails_closed():
    graph = _two_resource_graph(connector_start_x=0.1)
    with pytest.raises(ValueError, match="start pose"):
        adapt_connector_candidates(graph, _zones())
```

Also cover missing source/target state, wrong side, missing Turn Zone, and duplicate connector IDs.

- [ ] **Step 8: Freeze connector evidence funnel with monkeypatched backend plans**

Use monkeypatches at the A3 orchestration boundary so these unit tests do not depend on the numerical Dubins/R6B search:

```python
def test_forward_preview_free_transition_becomes_executable(monkeypatch):
    _patch_forward_gate(monkeypatch, status="PREVIEW_FOOTPRINT_FREE")
    transitions = validate_transition_candidates(
        _two_resource_graph(), _zones(), _free_navigation(), _vehicle()
    )
    assert transitions[0].status == EXECUTABLE
    assert transitions[0].backend == FORWARD_DUBINS_NAVIGATION_GATE
    assert transitions[0].reverse_distance_m == pytest.approx(0.0)


def test_map_insufficient_transition_is_unresolved_without_r6b(monkeypatch):
    _patch_forward_gate(monkeypatch, status="NO_FORWARD_PREVIEW_FREE_CANDIDATE")
    _patch_audit(monkeypatch, status="LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT")
    called = {"r6b": False}
    _patch_r6b_counter(monkeypatch, called)
    result = validate_transition_candidates(
        _two_resource_graph(), _zones(), _free_navigation(), _vehicle()
    )[0]
    assert result.status == UNRESOLVED
    assert result.proof_scope == MAP_EVIDENCE_INSUFFICIENT
    assert called["r6b"] is False


def test_occupancy_blocked_transition_calls_r6b_and_preserves_metrics(monkeypatch):
    _patch_forward_gate(monkeypatch, status="NO_FORWARD_PREVIEW_FREE_CANDIDATE")
    _patch_audit(monkeypatch, status="LOCAL_FORWARD_OCCUPANCY_BLOCKED")
    _patch_r6b_solution(
        monkeypatch,
        status="REVERSE_PRIMITIVE_PREVIEW_FREE",
        path_length_m=5.5,
        forward_distance_m=3.0,
        reverse_distance_m=2.5,
        cusp_count=2,
        search_expansions=321,
    )
    result = validate_transition_candidates(
        _two_resource_graph(), _zones(), _free_navigation(), _vehicle()
    )[0]
    assert result.status == EXECUTABLE
    assert result.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
    assert result.path_length_m == pytest.approx(5.5)
    assert result.reverse_distance_m == pytest.approx(2.5)
    assert result.cusp_count == 2
    assert result.search_expansions == 321
```

Also freeze: hard Site Boundary audit -> REJECTED without R6A/R6B; mixed evidence -> UNRESOLVED; `NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION` -> REJECTED + `BOUNDED_SEARCH_NO_SOLUTION`; R6B start footprint with OCCUPIED evidence -> REJECTED; R6B start footprint with UNKNOWN/out-of-grid evidence -> UNRESOLVED.

- [ ] **Step 9: Freeze top-level counts, coverage dedup, forbidden semantics, and deterministic IO**

```python
def test_graph_preserves_all_a2_records_and_deduplicates_coverage():
    graph = derive_vehicle_feasible_motion_graph(
        _graph_with_two_directions_same_resource(),
        _zones(),
        _free_navigation(),
        _vehicle(),
    )
    assert graph.diagnostics.a2_service_state_count == 2
    assert graph.diagnostics.service_validation_count == 2
    assert graph.diagnostics.locally_validated_segment_count == 1
    assert graph.diagnostics.locally_validated_unique_coverage_length_m == pytest.approx(4.0)


def test_serializer_contains_no_route_ready_reachable_or_optimal_keys(tmp_path):
    path = tmp_path / "vehicle_feasible_motion_graph.yaml"
    write_vehicle_feasible_motion_graph(_motion_graph_fixture(), path)
    text = path.read_text()
    assert "route_ready" not in text
    assert "reachable_from_start" not in text
    assert "optimal" not in text


def test_load_write_round_trip_is_byte_identical(tmp_path):
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    write_vehicle_feasible_motion_graph(_motion_graph_fixture(), first)
    loaded = load_vehicle_feasible_motion_graph(first)
    write_vehicle_feasible_motion_graph(loaded, second)
    assert first.read_bytes() == second.read_bytes()
```

Strict loader tests must reject duplicate IDs, missing cross-references, invalid status/proof scopes, non-finite metrics, inconsistent executable ID lists, inconsistent diagnostic counts, and malformed samples.

- [ ] **Step 10: Freeze root acceptance harness CLI contract**

`tests/test_v25_12g_a3_contract.py` loads `tools/v25_12g_a3_acceptance.py` using `importlib.util.spec_from_file_location` and asserts:

```python
REPORT_SCHEMA == "agt_v25_12g_a3_acceptance_report/v1"
VALIDATION_SCOPE == "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
MOTION_GRAPH_ASSET == "vehicle_feasible_motion_graph.yaml"
```

Parser defaults:

```text
--service-graph vehicle_feasible_service_graph.yaml
--turn-zones turn_zones.yaml
--navigation-map navigation_map.yaml
--site-boundary site_boundary.yaml
--write-motion-graph false
--overwrite-motion-graph false
--pretty false
--vehicle-profile required
```

The fixture contract must prove default read-only mode writes nothing, `--write-motion-graph` creates only the A3 sibling, overwrite is refused by default, upstream sentinels are preserved byte-for-byte, and the report recursively contains none of `route_ready`, `reachable_from_start`, or `optimal`.

- [ ] **Step 11: Commit the entire behavior test batch only**

```bash
git add \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  tests/test_v25_12g_a3_contract.py
git commit -m "test(v25-12g): batch A3 motion feasibility contracts"
```

- [ ] **Step 12: Operator Gate R1**

After the operator fast-forwards to the test commit, run:

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  tests/test_v25_12g_a3_contract.py
```

Expected RED is a collection of specific failures caused by missing `vehicle_feasible_service_motion`, `vehicle_feasible_transition_motion`, motion graph IO/derivation, Site Boundary-aware audit support, and missing A3 acceptance harness. Once this RED batch is shown, do not interrupt the operator again until Gate G1 unless a failure reveals a contradictory spec or an upstream defect.

---

### Task 3: Implement directional service validation and exact dead-end retrace

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_motion.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`

**Interfaces:**

```python
def validate_service_actions(
    service_graph: VehicleFeasibleServiceGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig,
    *,
    site_boundary: SiteBoundary | None = None,
) -> tuple[ServiceActionValidation, ...]: ...
```

- [ ] **Step 1: Validate A2/service resource identity before producing any result**

Implement unique indexes for service resources and states. Validate finite centerline points, finite entry/exit poses, service-state `segment_id` references, platform/profile/frame consistency, and the `1e-6` entry/exit pose contract.

Use wrapped yaw difference:

```python
def _angle_error(a: float, b: float) -> float:
    return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)
```

- [ ] **Step 2: Build ordinary directional samples**

```python
def _ordinary_samples(resource, state, row_yaw):
    if state.service_type == SERVICE_LOW_TO_HIGH:
        points = resource.centerline_xyz
        yaw = row_yaw
    elif state.service_type == SERVICE_HIGH_TO_LOW:
        points = tuple(reversed(resource.centerline_xyz))
        yaw = _wrap_pi(row_yaw + math.pi)
    else:
        raise ValueError(f"not an ordinary service type: {state.service_type}")

    return tuple(
        MotionSample(
            x=float(x), y=float(y), z=float(z), yaw=float(yaw),
            motion_direction="FORWARD", segment_index=0, is_cusp=False,
        )
        for x, y, z in points
    )
```

- [ ] **Step 3: Build exact dead-end samples without R6B**

Orient the forward-in points from the headland endpoint toward the interior. Build the forward samples, then append exactly one zero-distance cusp marker at the terminal pose with `motion_direction="REVERSE"`, `segment_index=1`, `is_cusp=True`, followed by `reversed(forward_samples[:-1])` converted to `REVERSE` with unchanged yaw.

- [ ] **Step 4: Compute actual path metrics from geometric movement only**

```python
def _distance(a: MotionSample, b: MotionSample) -> float:
    return math.sqrt((b.x-a.x)**2 + (b.y-a.y)**2 + (b.z-a.z)**2)


def _motion_metrics(samples):
    forward = reverse = 0.0
    for previous, current in zip(samples[:-1], samples[1:]):
        d = _distance(previous, current)
        if current.motion_direction == "REVERSE":
            reverse += d
        else:
            forward += d
    return forward + reverse, forward, reverse, sum(s.is_cusp for s in samples)
```

The cusp marker contributes zero distance because it repeats the terminal pose.

- [ ] **Step 5: Reuse existing footprint primitives for service evidence**

Convert `MotionSample` to `ForwardConnectorSample`, then call existing `_evaluate_candidate()` with `_preview_local_footprint(vehicle, config.preview_footprint_padding_m)`. Check Site Boundary with existing `_candidate_inside_site_boundary()`.

Classification order is hard boundary first, then OCCUPIED, then UNKNOWN/out-of-grid, then FREE:

```python
if not boundary_free:
    status = REJECTED
    proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
    backend_status = "SITE_BOUNDARY_CONFLICT"
elif footprint.occupied_count > 0:
    status = REJECTED
    proof_scope = PROVEN_HARD_CONSTRAINT_REJECTION
    backend_status = "OCCUPIED_FOOTPRINT_CONFLICT"
elif footprint.grid_coverage_fraction < 1.0 or footprint.unknown_count > 0:
    status = UNRESOLVED
    proof_scope = MAP_EVIDENCE_INSUFFICIENT
    backend_status = "MAP_EVIDENCE_INSUFFICIENT"
else:
    status = EXECUTABLE
    proof_scope = LOCAL_MOTION_EXECUTABLE
    backend_status = "PREVIEW_FOOTPRINT_FREE"
```

- [ ] **Step 6: Commit service implementation**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_motion.py
git commit -m "feat(v25-12g): validate A3 directional service motion"
```

Do not ask for operator tests yet; R1 already covers these behaviors.

---

### Task 4: Make forward candidate audit Site Boundary-aware without changing default behavior

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py`

**Interfaces:** Extend only the keyword-only API:

```python
def derive_forward_connector_candidate_audit(
    connector_requests,
    zones,
    navigation,
    vehicle,
    config=None,
    *,
    site_boundary: SiteBoundary | None = None,
    source=None,
) -> ForwardConnectorCandidateAuditPlan: ...
```

- [ ] **Step 1: Add structured boundary evidence per raw forward candidate**

Import `_candidate_inside_site_boundary` from `forward_connector_navigation_gate` and evaluate each sampled Dubins candidate with the same preview footprint. Do not alter behavior when `site_boundary is None`.

- [ ] **Step 2: Include boundary in preview-free classification**

A candidate is `preview_footprint_free` only when both the existing grid predicate and `boundary_free` are true.

- [ ] **Step 3: Classify hard local boundary conflict before occupancy/map-insufficient classes**

If every locally relevant candidate conflicts the supplied Site Boundary, emit:

```text
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT
```

with a reason explicitly stating this is a hard vehicle-permitted-boundary conflict and must not enter reverse fallback.

If at least one locally relevant candidate is boundary-safe, classify only those boundary-safe candidates through the existing occupancy/map-insufficient logic. A boundary-conflicting raster-free candidate must never force `FORWARD_PREVIEW_FREE` or `KEEP_FORWARD`.

- [ ] **Step 4: Preserve source determinism**

Add only:

```python
"site_boundary_enforced": site_boundary is not None
```

Do not add timestamps or machine paths.

- [ ] **Step 5: Commit audit extension**

```bash
git add src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py
git commit -m "feat(v25-12g): gate forward audit by site boundary"
```

---

### Task 5: Implement A2 connector adapter and R5/R6A/R6B transition orchestration

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class ConnectorBinding:
    connector_candidate_id: str
    from_service_state_id: str
    to_service_state_id: str
    from_segment_id: str
    to_segment_id: str
    side: str
    turn_zone_id: str
    start_pose: tuple[float, float, float, float]
    goal_pose: tuple[float, float, float, float]


def adapt_connector_candidates(
    service_graph: VehicleFeasibleServiceGraph,
    turn_zones: TurnZoneSet,
    config: VehicleFeasibleMotionGraphConfig,
) -> tuple[tuple[ConnectorRequest, ...], Mapping[str, ConnectorBinding]]: ...


def validate_transition_candidates(
    service_graph: VehicleFeasibleServiceGraph,
    turn_zones: TurnZoneSet,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleFeasibleMotionGraphConfig,
    *,
    site_boundary: SiteBoundary | None = None,
) -> tuple[TransitionValidation, ...]: ...
```

- [ ] **Step 1: Implement fail-closed one-to-one adapter**

Index A2 states and zones. Validate unique IDs, candidate source/target references, segment references, side, Turn Zone membership, and `start_pose == source.exit_pose`, `goal_pose == target.entry_pose` using the frozen tolerances. Return requests sorted by connector ID and a deterministic binding map.

- [ ] **Step 2: Run forward Navigation gate for every request**

Construct `ForwardConnectorNavigationGateConfig` using only the A3 padding override while retaining all existing strict defaults:

```python
forward_gate_cfg = ForwardConnectorNavigationGateConfig(
    preview_footprint_padding_m=config.preview_footprint_padding_m,
)
```

Call `derive_forward_connector_navigation_gate(..., site_boundary=site_boundary)`.

Any `PREVIEW_FOOTPRINT_FREE` result becomes `EXECUTABLE` with backend `FORWARD_DUBINS_NAVIGATION_GATE`, actual gate samples converted to `MotionSample`, `reverse_distance_m=0`, `cusp_count=0`, and the gate length as forward/path length.

- [ ] **Step 3: For remaining candidates run the Site Boundary-aware audit**

Use `ForwardConnectorCandidateAuditConfig(preview_footprint_padding_m=config.preview_footprint_padding_m)` and pass `site_boundary`.

Map audit status before R6A:

```text
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT       -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT    -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
LOCAL_FORWARD_MIXED_EVIDENCE               -> UNRESOLVED / MIXED_EVIDENCE_REQUIRES_REVIEW
NO_FORWARD_DUBINS_CANDIDATE                -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
TURN_ZONE_METADATA_INVALID                 -> impossible after adapter; raise ValueError if observed
LOCAL_FORWARD_OCCUPANCY_BLOCKED            -> pass to R6A
```

Do not call R6A/R6B for the first four result classes.

- [ ] **Step 4: Run R6A with no manual approvals**

```python
admission = derive_reverse_fallback_admission(
    audit_plan,
    ReverseFallbackAdmissionConfig(
        operator_approved_mixed_connector_ids=(),
    ),
    source={"acceptance_stage": "v25_12g_a3"},
)
```

Only `ELIGIBLE_REVERSE_FALLBACK` IDs may proceed.

- [ ] **Step 5: Run R6B only for R6A-admitted IDs with frozen budgets**

Instantiate `ReversePrimitiveConnectorConfig` without changing any search limit, overriding only the shared footprint padding:

```python
reverse_cfg = ReversePrimitiveConnectorConfig(
    preview_footprint_padding_m=config.preview_footprint_padding_m,
)
```

Call `derive_reverse_primitive_connector_plan(..., site_boundary=site_boundary)`.

Map results:

```text
REVERSE_PRIMITIVE_PREVIEW_FREE -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE
FORWARD_PRIMITIVE_PREVIEW_FREE -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE
SITE_BOUNDARY_CONFLICT          -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION -> REJECTED / BOUNDED_SEARCH_NO_SOLUTION
```

For `R6B_START_FOOTPRINT_NOT_FREE`, independently evaluate the start pose with `_evaluate_candidate` and the same footprint. If OCCUPIED cells are present, reject; if only UNKNOWN/out-of-grid evidence prevents FREE, classify unresolved.

- [ ] **Step 6: Preserve all backend evidence without inventing route semantics**

`forward_evidence` stores deterministic summaries from the gate/audit; `reverse_admission_evidence` stores the R6A decision/status/reason. Do not serialize Python object reprs or temporary paths.

- [ ] **Step 7: Commit transition orchestration**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
git commit -m "feat(v25-12g): validate A3 headland transitions"
```

---

### Task 6: Assemble diagnostics and deterministic strict YAML IO

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:** top-level `derive_vehicle_feasible_motion_graph`, `vehicle_feasible_motion_graph_to_dict`, `write_vehicle_feasible_motion_graph`, `load_vehicle_feasible_motion_graph`.

- [ ] **Step 1: Add top-level input validation and derivation**

Validate service graph schema/status, frame, platform ID, profile hash, row direction, Navigation Grid frame, Turn Zone frame, and optional Site Boundary frame before service or transition validation.

Call:

```python
service_actions = validate_service_actions(...)
transition_validations = validate_transition_candidates(...)
```

Then enforce:

```python
if len(service_actions) != len(service_graph.service_states):
    raise RuntimeError("A3 service validation cardinality mismatch")
if len(transition_validations) != len(service_graph.connector_candidates):
    raise RuntimeError("A3 transition validation cardinality mismatch")
```

- [ ] **Step 2: Compute deterministic executable ID lists and diagnostics**

Coverage dedup uses physical `coverage_segment_id` only. Build the resource length/aisle lookup from A2 `service_resources`; count each executable physical segment once.

- [ ] **Step 3: Serialize stable top-level key order and record order**

Sort service actions by `service_state_id` and transitions by `connector_candidate_id` before serialization. Use `yaml.safe_dump(..., sort_keys=False, allow_unicode=True)` after constructing dictionaries in frozen key order.

- [ ] **Step 4: Strict loader validates semantics, not only shape**

Reject:

```text
wrong schema/status
unknown status/proof_scope/backend enum
non-finite pose/sample/metric
negative distances or search_expansions
cusp count inconsistent with sample markers
EXECUTABLE ID list referencing non-executable record
missing or extra executable IDs
duplicate service or connector IDs
transition references unknown service IDs
service coverage_segment_id mismatch
summary/count mismatch
locally validated coverage mismatch
```

Also recursively reject forbidden keys `route_ready`, `reachable_from_start`, and `optimal`.

- [ ] **Step 5: Writer refuses overwrite by default**

```python
def write_vehicle_feasible_motion_graph(graph, path, *, overwrite=False):
    output = Path(path)
    if output.exists() and not overwrite:
        raise FileExistsError(str(output))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            vehicle_feasible_motion_graph_to_dict(graph),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
```

- [ ] **Step 6: Export the public A3 API and register pytest**

In `__init__.py`, export the schema/status/constants, dataclasses, derive/load/write functions. In CMake add:

```cmake
ament_add_pytest_test(
  test_vehicle_feasible_motion_graph
  test/test_vehicle_feasible_motion_graph.py
)
```

- [ ] **Step 7: Commit graph assembly and IO**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12g): serialize A3 motion evidence graph"
```

---

### Task 7: Acceptance harness and root contract

**Files:**
- Create: `tools/v25_12g_a3_acceptance.py`
- Test already created in Task 2: `tests/test_v25_12g_a3_contract.py`

**Interfaces:** CLI reads frozen A2/runtime inputs, derives one A3 report, writes only the A3 sibling when explicitly requested.

- [ ] **Step 1: Implement frozen parser**

```python
REPORT_SCHEMA = "agt_v25_12g_a3_acceptance_report/v1"
VALIDATION_SCOPE = "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Validate V25-12G-A3 local vehicle motion evidence"
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--service-graph", default="vehicle_feasible_service_graph.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--navigation-map", default="navigation_map.yaml")
    parser.add_argument("--site-boundary", default="site_boundary.yaml")
    parser.add_argument("--write-motion-graph", action="store_true")
    parser.add_argument("--overwrite-motion-graph", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser
```

- [ ] **Step 2: Preflight all required inputs before parsing heavy assets**

Require service graph, Turn Zones, Navigation Grid YAML, Site Boundary, and explicit vehicle profile. If write mode is requested and the output exists without overwrite, raise `FileExistsError` before loading assets.

- [ ] **Step 3: Load frozen inputs and derive one motion graph**

Use existing strict loaders:

```python
service_graph = load_vehicle_feasible_service_graph(service_graph_path)
turn_zones = load_turn_zones(turn_zone_path)
navigation = load_navigation_grid(navigation_path)
boundary = load_site_boundary(boundary_path)
vehicle = load_canonical_vehicle_profile(vehicle_profile_path)
```

Then call `derive_vehicle_feasible_motion_graph(..., site_boundary=boundary, source={...})`.

- [ ] **Step 4: Emit deterministic diagnostic report**

Top-level report keys:

```text
schema
validation_scope
run_dir
frame_id
platform_id
service_graph_asset
turn_zones_asset
navigation_map_asset
site_boundary_asset
service_actions
transitions
summary
```

`summary` includes every A3 diagnostic field plus the two cardinality checks:

```text
all_a2_service_states_preserved
all_a2_connector_candidates_preserved
```

and these invariant counters:

```text
dead_end_non_retrace_count
site_boundary_reverse_bypass_count
forbidden_semantic_key_count
```

All three must be zero or the harness raises `ValueError`.

- [ ] **Step 5: Write only the A3 sibling when requested**

```python
if args.write_motion_graph:
    write_vehicle_feasible_motion_graph(
        graph,
        output_path,
        overwrite=args.overwrite_motion_graph,
    )
```

Never write upstream assets.

- [ ] **Step 6: Commit acceptance harness**

```bash
git add tools/v25_12g_a3_acceptance.py
git commit -m "feat(v25-12g): add A3 motion acceptance harness"
```

---

### Task 8: Operator Gate G1 — focused, package, and contract GREEN batch

**Files:** no production edits unless failures reveal a defect covered by existing R1 tests.

- [ ] **Step 1: Operator fast-forwards and builds package**

```bash
cd ~/agt_navigation_v2
git pull --ff-only

source /opt/ros/humble/setup.bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
```

- [ ] **Step 2: Run the A3 and modified-backend focused suite**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a3_contract.py
```

Expected: all selected tests PASS. Do not state the pass count until operator output supplies it.

- [ ] **Step 3: Run package registration suite**

```bash
colcon test \
  --packages-select agt_offline_assets \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/agt_offline_assets \
  --verbose
```

Expected: package tests report zero failures. If unrelated legacy failures appear, isolate them before changing A3.

- [ ] **Step 4: If a failure occurs, stop and use systematic debugging**

Do not patch by intuition. Identify whether the failing layer is A3 contract, service validation, audit boundary extension, connector orchestration, IO, harness, or unrelated package regression. If production behavior needs to change and no existing test specifically captures it, add and verify a new RED test before the fix.

---

### Task 9: Frozen real-greenhouse A3 checkpoint

**Files:**
- Create after evidence: `docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md`
- Create/update after evidence: `docs/v2.5/V25_12G_A3_CURRENT_STATE.md`

**Interfaces:** real-data run produces diagnostic evidence only; it does not create A4 route output.

- [ ] **Step 1: Preflight frozen runtime inputs**

```bash
RUN=/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run

for f in \
  vehicle_feasible_service_graph.yaml \
  turn_zones.yaml \
  navigation_map.yaml \
  navigation_map.pgm \
  site_boundary.yaml \
  vehicle_feasible_segments.yaml \
  derivation.yaml \
  aisle_graph.yaml
do
  test -f "$RUN/$f" || { echo "MISSING: $RUN/$f"; exit 1; }
done
```

Resolve the same canonical `mk_mini` vehicle profile path used for the A1 experiment and pass it explicitly as `PROFILE=...`; do not guess a new profile file.

- [ ] **Step 2: Freeze upstream hashes before A3 write**

```bash
sha256sum \
  "$RUN/vehicle_feasible_service_graph.yaml" \
  "$RUN/vehicle_feasible_segments.yaml" \
  "$RUN/turn_zones.yaml" \
  "$RUN/navigation_map.yaml" \
  "$RUN/navigation_map.pgm" \
  "$RUN/site_boundary.yaml" \
  "$RUN/derivation.yaml" \
  "$RUN/aisle_graph.yaml" \
  | tee /tmp/v25_12g_a3_pre.sha256
```

- [ ] **Step 3: Run read-only A3 acceptance first**

```bash
python3 tools/v25_12g_a3_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --pretty \
  > /tmp/v25_12g_a3_readonly.json
```

Required structural gates:

```text
schema = agt_v25_12g_a3_acceptance_report/v1
validation_scope = A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY
frame_id = map
platform_id = mk_mini
service_validation_count = 32
transition_validation_count = 40
all_a2_service_states_preserved = true
all_a2_connector_candidates_preserved = true
dead_end_non_retrace_count = 0
site_boundary_reverse_bypass_count = 0
forbidden_semantic_key_count = 0
```

Do not hardcode or pre-approve executable/rejected/unresolved totals.

- [ ] **Step 4: Inspect the real evidence funnel**

Report at least:

```text
service: executable / rejected / unresolved
ordinary service count
dead-end service count and executable dead-end count
locally validated physical segments / unique coverage / distinct aisles
transition: forward executable / reverse executable / rejected / unresolved
forward occupancy-blocked count
R6A reverse-admitted count
R6B reverse-solved count
map-insufficient count
mixed-evidence count
hard Site Boundary conflict count
bounded-search-no-solution count
```

Sanity controls:

```text
aisle_020.segment_001.service_low_to_high
aisle_020.segment_001.service_high_to_low
```

must both have explicit validation records, but their status is observed rather than hardcoded.

The four known dead-end states from A2 must each have an explicit validation record and, regardless of EXECUTABLE/REJECTED/UNRESOLVED status, their geometry contract must show exact retrace with one cusp and single-count coverage reward.

- [ ] **Step 5: Narrow-write only after read-only semantics are accepted**

If `vehicle_feasible_motion_graph.yaml` already exists, stop and inspect it; do not silently overwrite.

```bash
test ! -e "$RUN/vehicle_feasible_motion_graph.yaml" || exit 2

python3 tools/v25_12g_a3_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --write-motion-graph \
  --pretty \
  > /tmp/v25_12g_a3_write.json
```

- [ ] **Step 6: Prove upstream assets are unchanged**

```bash
sha256sum \
  "$RUN/vehicle_feasible_service_graph.yaml" \
  "$RUN/vehicle_feasible_segments.yaml" \
  "$RUN/turn_zones.yaml" \
  "$RUN/navigation_map.yaml" \
  "$RUN/navigation_map.pgm" \
  "$RUN/site_boundary.yaml" \
  "$RUN/derivation.yaml" \
  "$RUN/aisle_graph.yaml" \
  | tee /tmp/v25_12g_a3_post.sha256

diff -u /tmp/v25_12g_a3_pre.sha256 /tmp/v25_12g_a3_post.sha256
```

Expected diff: empty.

- [ ] **Step 7: Strict-load and byte-stability gate**

```bash
python3 - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
from agt_offline_assets.vehicle_feasible_motion_graph import (
    load_vehicle_feasible_motion_graph,
    write_vehicle_feasible_motion_graph,
)

src = Path(
    "/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/"
    "vehicle_feasible_motion_graph.yaml"
)
graph = load_vehicle_feasible_motion_graph(src)
assert graph.schema == "agt_vehicle_feasible_motion_graph/v1"
assert graph.status == "MOTION_EVIDENCE_ONLY"
assert graph.diagnostics.service_validation_count == 32
assert graph.diagnostics.transition_validation_count == 40

with TemporaryDirectory() as tmp:
    dst = Path(tmp) / src.name
    write_vehicle_feasible_motion_graph(graph, dst)
    assert src.read_bytes() == dst.read_bytes()

print("A3 strict-load and byte-stable rewrite: PASS")
PY
```

- [ ] **Step 8: Record evidence without overstating readiness**

`docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md` records actual observed counts, the connector funnel, service classifications, dead-end evidence, unchanged upstream hashes, byte-stability result, and remaining caveats.

The strongest allowed classification is:

```text
A3 LOCAL VEHICLE-MOTION EVIDENCE SUPPORTED
REAL-DATA OBSERVED
MOTION_EVIDENCE_ONLY
NOT START-REACHABILITY VALIDATED
NOT ROUTE-READY
```

If the evidence shows material map/footprint caveats, append `WITH MAP / FOOTPRINT CAVEATS` rather than weakening safety gates.

`V25_12G_A3_CURRENT_STATE.md` must name the next phase only as:

```text
V25-12G-A4 START_POSE-Aware Maximum Feasible Coverage Optimizer
```

- [ ] **Step 9: Commit evidence docs only**

```bash
git add \
  docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md \
  docs/v2.5/V25_12G_A3_CURRENT_STATE.md
git commit -m "docs(v25-12g): record A3 real-data motion checkpoint"
```

Do not add runtime YAML assets to git unless repository policy is explicitly changed.

---

## Final Verification Before A3 Completion Claim

Before saying A3 is complete, invoke `superpowers:verification-before-completion` and require fresh operator evidence for:

```text
focused A3/backend tests: zero failures
agt_offline_assets package tests: zero A3-related failures
A3 contract tests: zero failures
real-data service_validation_count = 32
real-data transition_validation_count = 40
all A2 IDs preserved
no dead-end non-retrace result
no Site Boundary reverse bypass
upstream hash diff empty
strict loader PASS
byte-stable rewrite PASS
```

Only after those gates may the branch be handed to the finishing workflow and A4 design begin.
