# V25-12G-A3 Vehicle-Feasible Motion Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the frozen A2 static service topology into one deterministic `agt_vehicle_feasible_motion_graph/v1` asset carrying local service-motion and headland-transition evidence, without claiming START_POSE reachability, route readiness, or optimality.

**Architecture:** Keep A2 immutable. Add a core A3 model, a directional service validator, a connector orchestrator that adapts A2 candidates into the existing forward gate/audit/R6A/R6B chain, and a strict IO module. Ordinary services revalidate stored A1/A2 centerlines directionally; dead-end actions use exact forward-in/reverse-out retrace; connector actions reuse existing planners and preserve all failed/unresolved candidates for audit.

**Tech Stack:** Python 3.10, dataclasses, math, pathlib, NumPy, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing `VehicleFeasibleServiceGraph`, `TurnZoneSet`, `NavigationGridEvidence`, `CanonicalVehicleProfile`, `SiteBoundary`, forward connector gate/audit, R6A admission, and R6B bounded reverse primitive search.

## Global Constraints

- Work only on `feat/v25-12g-maximum-feasible-coverage` in the existing checkout; do not create another worktree.
- Do not use `git reset --hard` or `git clean`.
- Never touch or stage `tools/rosbag_sensor_trimmer` or `tools/map_tools/render_pcd_top_views.py`.
- Output filename is exactly `vehicle_feasible_motion_graph.yaml`.
- Schema is exactly `agt_vehicle_feasible_motion_graph/v1`.
- Top-level status is exactly `MOTION_EVIDENCE_ONLY`.
- A3 must not modify `vehicle_feasible_service_graph.yaml`, `vehicle_feasible_segments.yaml`, `turn_zones.yaml`, `navigation_map.yaml`, `navigation_map.pgm`, `site_boundary.yaml`, `derivation.yaml`, or `aisle_graph.yaml`.
- A3 writes only its own sibling runtime asset.
- Produce exactly one service validation for every A2 service state and one transition validation for every A2 connector candidate.
- Result classes are exactly `EXECUTABLE`, `REJECTED`, and `UNRESOLVED`.
- Site Boundary touch/cross is a hard conflict and must never enter reverse fallback.
- UNKNOWN/out-of-grid evidence is not obstacle truth; absent a separate OCCUPIED conflict, classify it `UNRESOLVED`.
- `operator_approved_mixed_connector_ids` is exactly `()` in A3 v1.
- Do not increase R6B `max_expansions`, `max_path_length_m`, `max_cusps`, or other default search budgets.
- LOW->HIGH and HIGH->LOW ordinary services both use forward gear. HIGH->LOW reverses point order and adds pi to yaw.
- `DEAD_END_FORWARD_IN_REVERSE_OUT` never calls R6B. Reverse-out uses the exact forward geometry in reverse sample order with unchanged yaw.
- Dead-end has one zero-distance cusp marker at the terminal pose. Reverse travel resumes from the previous geometric point so distance is not double counted.
- Pose cross-reference tolerance is `1e-6 m` position and `1e-6 rad` yaw.
- Coverage reward is A2 useful segment length. Travel distance is the actual 3D sample polyline length.
- A physical `coverage_segment_id` contributes locally validated coverage at most once.
- A3 must not serialize `route_ready`, `reachable_from_start`, `optimal`, or equivalent claims.
- Semantic-identical inputs must serialize to byte-identical YAML.
- Strict TDD is mandatory. No behavior implementation before the operator shows the corresponding RED evidence.
- Operator test gates are batched to reduce interruption. Implementation commits remain small.
- Never claim build/test/real-data success without fresh operator-machine output.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_feasible_motion_graph.py
  vehicle_feasible_service_motion.py
  vehicle_feasible_transition_motion.py
  vehicle_feasible_motion_graph_io.py
  forward_connector_candidate_audit.py
  __init__.py

src/agt_offline_assets/test/
  test_vehicle_feasible_motion_graph.py
  test_forward_connector_candidate_audit.py

src/agt_offline_assets/CMakeLists.txt

tools/v25_12g_a3_acceptance.py
tests/test_v25_12g_a3_contract.py

docs/v2.5/V25_12G_A3_CURRENT_STATE.md
docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md
```

`vehicle_feasible_motion_graph.py` owns public constants/dataclasses, top-level validation, graph assembly, and diagnostics. `vehicle_feasible_service_motion.py` owns service sample construction and footprint classification. `vehicle_feasible_transition_motion.py` owns A2 connector adaptation and R5/R6A/R6B orchestration. `vehicle_feasible_motion_graph_io.py` owns deterministic serializer/writer/strict loader.

---

## Frozen Public Contract

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

```python
@dataclass(frozen=True)
class VehicleFeasibleMotionGraphConfig:
    preview_footprint_padding_m: float = 0.05
    pose_position_tolerance_m: float = 1.0e-6
    pose_yaw_tolerance_rad: float = 1.0e-6
```

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

Exact top-level signatures:

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
) -> VehicleFeasibleMotionGraph: ...


def write_vehicle_feasible_motion_graph(
    graph: VehicleFeasibleMotionGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path: ...


def load_vehicle_feasible_motion_graph(
    path: str | Path,
) -> VehicleFeasibleMotionGraph: ...
```

---

## Shared Focused-Test Fixtures

Add these exact helpers at the top of `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py` after imports. They make the RED/GREEN tests self-contained.

```python
import math
import numpy as np
import pytest

from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED, UNKNOWN
from agt_offline_assets.site_boundary import SiteBoundary
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_profile import CanonicalVehicleProfile
from agt_offline_assets.vehicle_feasible_service_graph import (
    CANDIDATE_TOPOLOGY_COMPONENT,
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    HEADLAND_TOPOLOGY_CANDIDATE,
    REQUIRES_A3_CONNECTOR_VALIDATION,
    REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    TOPOLOGY_SERVICE_CANDIDATE,
    CandidateTopologyComponent,
    ConnectorCandidate,
    ServiceGraphDiagnostics,
    ServiceResource,
    ServiceState,
    VehicleFeasibleServiceGraph,
)


def _vehicle():
    footprint = ((0.30, 0.20), (0.30, -0.20), (-0.30, -0.20), (-0.30, 0.20))
    return CanonicalVehicleProfile(
        profile_id="mk_mini",
        profile_path="fixture.yaml",
        profile_sha256="fixture-sha256",
        kinematics="ackermann",
        footprint_frame="base_link",
        base_frame="base_link",
        physical_length_m=0.60,
        physical_width_m=0.40,
        footprint_xy=footprint,
        navigation_footprint_xy=footprint,
        navigation_width_m=0.40,
        navigation_length_m=0.60,
        wheel_base_m=0.60,
        track_width_m=0.40,
        wheel_diameter_m=0.24,
        ground_clearance_m=0.11,
        minimum_turning_radius_m=1.00,
        minimum_turning_radius_verified=True,
        maximum_steering_angle_rad=0.54,
        maximum_steering_angle_deg=31.0,
        allow_in_place_rotation=False,
        max_forward_velocity_mps=1.0,
        max_reverse_velocity_mps=0.5,
        max_angular_velocity_rps=1.0,
        manufacturer_maximum_speed_mps=1.0,
        route_acceptance_enabled=False,
        preview_planning_enabled=True,
        blocked_reason="",
    )


def _navigation(fill=FREE):
    return NavigationGridEvidence(
        resolution_m=0.25,
        origin_x_m=-2.0,
        origin_y_m=-4.0,
        width=40,
        height=32,
        occupancy=np.full((32, 40), fill, dtype=np.uint8),
        frame_id="map",
        source={},
    )


def _boundary(x_max=8.0):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((-1.5, -3.5), (x_max, -3.5), (x_max, 3.5), (-1.5, 3.5)),
        source={},
    )


def _resource(segment_id="aisle_001.segment_001", y=0.0, low="LOW_U_HEADLAND", high="HIGH_U_HEADLAND"):
    aisle_id = segment_id.split(".")[0]
    return ServiceResource(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=1,
        coverage_length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=low,
        high_endpoint_type=high,
        low_endpoint_pose=(0.0, y, 0.0, 0.0),
        high_endpoint_pose=(4.0, y, 0.0, 0.0),
        centerline_xyz=((0.0, y, 0.0), (2.0, y, 0.0), (4.0, y, 0.0)),
    )


def _ordinary_state(resource, service_type):
    if service_type == SERVICE_LOW_TO_HIGH:
        return ServiceState(
            service_state_id=f"{resource.segment_id}.service_low_to_high",
            segment_id=resource.segment_id,
            aisle_id=resource.aisle_id,
            service_type=SERVICE_LOW_TO_HIGH,
            entry_endpoint_type=resource.low_endpoint_type,
            exit_endpoint_type=resource.high_endpoint_type,
            entry_pose=resource.low_endpoint_pose,
            exit_pose=resource.high_endpoint_pose,
            service_motion_direction="FORWARD",
            forward_service_distance_m=resource.coverage_length_m,
            reverse_service_distance_m=0.0,
            coverage_segment_id=resource.segment_id,
            coverage_reward_length_m=resource.coverage_length_m,
            external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
            validation_status=TOPOLOGY_SERVICE_CANDIDATE,
        )
    return ServiceState(
        service_state_id=f"{resource.segment_id}.service_high_to_low",
        segment_id=resource.segment_id,
        aisle_id=resource.aisle_id,
        service_type=SERVICE_HIGH_TO_LOW,
        entry_endpoint_type=resource.high_endpoint_type,
        exit_endpoint_type=resource.low_endpoint_type,
        entry_pose=(resource.high_endpoint_pose[0], resource.high_endpoint_pose[1], resource.high_endpoint_pose[2], -math.pi),
        exit_pose=(resource.low_endpoint_pose[0], resource.low_endpoint_pose[1], resource.low_endpoint_pose[2], -math.pi),
        service_motion_direction="FORWARD",
        forward_service_distance_m=resource.coverage_length_m,
        reverse_service_distance_m=0.0,
        coverage_segment_id=resource.segment_id,
        coverage_reward_length_m=resource.coverage_length_m,
        external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
        validation_status=TOPOLOGY_SERVICE_CANDIDATE,
    )


def _dead_end_state(resource):
    return ServiceState(
        service_state_id=f"{resource.segment_id}.dead_end_forward_in_reverse_out",
        segment_id=resource.segment_id,
        aisle_id=resource.aisle_id,
        service_type=DEAD_END_FORWARD_IN_REVERSE_OUT,
        entry_endpoint_type=resource.low_endpoint_type,
        exit_endpoint_type=resource.low_endpoint_type,
        entry_pose=resource.low_endpoint_pose,
        exit_pose=resource.low_endpoint_pose,
        service_motion_direction="FORWARD",
        forward_service_distance_m=resource.coverage_length_m,
        reverse_service_distance_m=resource.coverage_length_m,
        coverage_segment_id=resource.segment_id,
        coverage_reward_length_m=resource.coverage_length_m,
        external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
        validation_status=REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    )


def _diagnostics(resources, states, connectors):
    return ServiceGraphDiagnostics(
        resource_count=len(resources),
        ordinary_service_state_count=sum(s.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT for s in states),
        dead_end_candidate_count=sum(s.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT for s in states),
        total_service_state_count=len(states),
        connector_candidate_count=len(connectors),
        low_u_connector_candidate_count=sum(c.side == "LOW_U" for c in connectors),
        high_u_connector_candidate_count=sum(c.side == "HIGH_U" for c in connectors),
        candidate_component_count=0,
        isolated_component_count=0,
        externally_unproven_state_count=0,
        unique_coverage_length_m=sum(r.coverage_length_m for r in resources),
        distinct_aisle_count=len({r.aisle_id for r in resources}),
    )


def _graph(resources, states, connectors=()):
    resources = tuple(resources)
    states = tuple(states)
    connectors = tuple(connectors)
    return VehicleFeasibleServiceGraph(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        service_resources=resources,
        service_states=states,
        connector_candidates=connectors,
        candidate_components=(),
        diagnostics=_diagnostics(resources, states, connectors),
        source={},
    )


def _zones():
    return TurnZoneSet(
        frame_id="map",
        row_direction_xy=(1.0, 0.0),
        zones=(
            TurnZone(
                zone_id="turn_high_u",
                side="HIGH_U",
                polygon_xy=((3.0, -2.5), (6.0, -2.5), (6.0, 2.5), (3.0, 2.5)),
                supported_aisle_ids=("aisle_001", "aisle_002"),
                endpoint_count=2,
                free_fraction=1.0,
                allow_turn=True,
            ),
        ),
        source={},
    )


def _two_resource_connector_graph():
    a = _resource("aisle_001.segment_001", y=0.0)
    b = _resource("aisle_002.segment_001", y=2.0)
    a_state = _ordinary_state(a, SERVICE_LOW_TO_HIGH)
    b_state = _ordinary_state(b, SERVICE_HIGH_TO_LOW)
    connector = ConnectorCandidate(
        connector_candidate_id="headland.turn_high_u.a_to_b",
        from_service_state_id=a_state.service_state_id,
        to_service_state_id=b_state.service_state_id,
        from_segment_id=a.segment_id,
        to_segment_id=b.segment_id,
        side="HIGH_U",
        turn_zone_id="turn_high_u",
        start_pose=a_state.exit_pose,
        goal_pose=b_state.entry_pose,
        validation_status=REQUIRES_A3_CONNECTOR_VALIDATION,
    )
    return _graph((a, b), (a_state, b_state), (connector,))
```

---

## Batched TDD Policy

Use only three operator gates:

```text
R0: prove the public A3 module is absent
R1: after import-only skeleton exists, run all detailed A3 behavior/contract tests and prove specific RED failures
G1: after implementation, run all focused + package + root contract tests in one GREEN batch
```

Between R1 and G1, commit implementation tasks without asking the operator to test again unless a contradiction or unrelated upstream defect blocks progress.

---

### Task 1: R0 module absence, then public import skeleton

**Files:**
- Create test first: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- After RED only: create `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`

**Interfaces:** public constants, config, and dataclass names only. No derivation behavior in the skeleton.

- [ ] **Step 1: Write failing module-contract test**

```python
import importlib


def test_a3_public_module_exists_with_frozen_schema():
    module = importlib.import_module("agt_offline_assets.vehicle_feasible_motion_graph")
    assert module.VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA == "agt_vehicle_feasible_motion_graph/v1"
    assert module.MOTION_EVIDENCE_ONLY == "MOTION_EVIDENCE_ONLY"
```

- [ ] **Step 2: Commit test only**

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

Expected RED: `ModuleNotFoundError` for `agt_offline_assets.vehicle_feasible_motion_graph`.

- [ ] **Step 4: After R0, add import skeleton only**

Create the constants/config/dataclasses from the Frozen Public Contract. `VehicleFeasibleMotionGraphConfig.validate()` may validate non-negative padding and positive tolerances. Do not create service/transition derivation functions yet.

- [ ] **Step 5: Commit skeleton**

```bash
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py
git commit -m "feat(v25-12g): define A3 motion graph contract"
```

Proceed immediately to Task 2 without requesting GREEN.

---

### Task 2: Write the complete behavior RED batch

**Files:**
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py`
- Modify: `src/agt_offline_assets/test/test_forward_connector_candidate_audit.py`
- Create: `tests/test_v25_12g_a3_contract.py`

**Interfaces frozen by tests:**

```python
def validate_service_actions(service_graph, navigation, vehicle, config, *, site_boundary=None): ...
def adapt_connector_candidates(service_graph, turn_zones, config): ...
def validate_transition_candidates(service_graph, turn_zones, navigation, vehicle, config, *, site_boundary=None): ...
def derive_vehicle_feasible_motion_graph(service_graph, turn_zones, navigation, vehicle, config=None, *, site_boundary=None, source=None): ...
def vehicle_feasible_motion_graph_to_dict(graph): ...
def write_vehicle_feasible_motion_graph(graph, path, *, overwrite=False): ...
def load_vehicle_feasible_motion_graph(path): ...
```

- [ ] **Step 1: Add ordinary service tests**

```python
def test_low_to_high_is_forward_and_uses_polyline_length():
    r = _resource()
    g = _graph((r,), (_ordinary_state(r, SERVICE_LOW_TO_HIGH),))
    action = validate_service_actions(g, _navigation(), _vehicle(), VehicleFeasibleMotionGraphConfig())[0]
    assert action.status == EXECUTABLE
    assert [s.x for s in action.samples] == [0.0, 2.0, 4.0]
    assert all(s.motion_direction == "FORWARD" for s in action.samples)
    assert action.path_length_m == pytest.approx(4.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(0.0)
    assert action.cusp_count == 0


def test_high_to_low_reverses_points_but_stays_forward_gear():
    r = _resource()
    g = _graph((r,), (_ordinary_state(r, SERVICE_HIGH_TO_LOW),))
    action = validate_service_actions(g, _navigation(), _vehicle(), VehicleFeasibleMotionGraphConfig())[0]
    assert [s.x for s in action.samples] == [4.0, 2.0, 0.0]
    assert all(s.motion_direction == "FORWARD" for s in action.samples)
    assert all(abs(abs(s.yaw) - math.pi) <= 1.0e-9 for s in action.samples)
```

- [ ] **Step 2: Add service evidence tests**

```python
def test_service_occupied_is_rejected_unknown_is_unresolved_and_boundary_is_hard():
    r = _resource()
    g = _graph((r,), (_ordinary_state(r, SERVICE_LOW_TO_HIGH),))
    cfg = VehicleFeasibleMotionGraphConfig()

    occupied = validate_service_actions(g, _navigation(OCCUPIED), _vehicle(), cfg)[0]
    assert occupied.status == REJECTED
    assert occupied.proof_scope == PROVEN_HARD_CONSTRAINT_REJECTION

    unknown = validate_service_actions(g, _navigation(UNKNOWN), _vehicle(), cfg)[0]
    assert unknown.status == UNRESOLVED
    assert unknown.proof_scope == MAP_EVIDENCE_INSUFFICIENT

    clipped = validate_service_actions(
        g, _navigation(), _vehicle(), cfg, site_boundary=_boundary(x_max=3.9)
    )[0]
    assert clipped.status == REJECTED
    assert clipped.backend_status == "SITE_BOUNDARY_CONFLICT"
```

- [ ] **Step 3: Add exact dead-end retrace test**

```python
def test_dead_end_exact_retrace_has_one_cusp_and_single_coverage_reward():
    r = _resource(high="INTERIOR_BLOCKED_END")
    g = _graph((r,), (_dead_end_state(r),))
    action = validate_service_actions(g, _navigation(), _vehicle(), VehicleFeasibleMotionGraphConfig())[0]
    assert action.backend == A1_CENTERLINE_EXACT_REVERSE_RETRACE
    assert action.coverage_reward_length_m == pytest.approx(4.0)
    assert action.path_length_m == pytest.approx(8.0)
    assert action.forward_distance_m == pytest.approx(4.0)
    assert action.reverse_distance_m == pytest.approx(4.0)
    assert action.cusp_count == 1
    cusp = next(i for i, sample in enumerate(action.samples) if sample.is_cusp)
    assert action.samples[cusp - 1].x == pytest.approx(4.0)
    assert action.samples[cusp].x == pytest.approx(4.0)
    assert action.samples[cusp].motion_direction == "REVERSE"
    assert action.samples[cusp].yaw == pytest.approx(action.samples[cusp - 1].yaw)
    forward_xyz = [(s.x, s.y, s.z) for s in action.samples[:cusp]]
    reverse_xyz = [(s.x, s.y, s.z) for s in action.samples[cusp + 1:]]
    assert reverse_xyz == list(reversed(forward_xyz[:-1]))
```

Add a second explicit HIGH_U dead-end fixture by creating a resource with `low="INTERIOR_BLOCKED_END", high="HIGH_U_HEADLAND"` and asserting the forward-in sequence is `[4.0, 2.0, 0.0]`, reverse samples preserve that forward-in yaw, and `cusp_count == 1`.

- [ ] **Step 4: Add fail-closed service identity tests**

Create a LOW->HIGH state, replace its `entry_pose` x coordinate with `0.01` using `dataclasses.replace`, and assert `validate_service_actions()` raises `ValueError` matching `entry pose`. Add equivalent explicit tests for exit pose mismatch, missing resource ID, non-finite centerline coordinate, duplicate state ID, frame mismatch, platform mismatch, and vehicle profile hash mismatch.

- [ ] **Step 5: Add Site Boundary-aware forward audit tests**

Extend `derive_forward_connector_candidate_audit` with this frozen keyword-only signature:

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

Add:

```python
def test_candidate_audit_explicit_none_boundary_matches_default():
    default = derive_forward_connector_candidate_audit(requests, zones, navigation, vehicle)
    explicit = derive_forward_connector_candidate_audit(
        requests, zones, navigation, vehicle, site_boundary=None
    )
    assert default == explicit
```

and one fixture where every locally relevant forward candidate crosses the permitted boundary:

```python
assert plan.connectors[0].status == "LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT"
```

- [ ] **Step 6: Add adapter tests**

```python
def test_connector_adapter_is_one_to_one_and_checks_exit_entry_identity():
    graph = _two_resource_connector_graph()
    requests, bindings = adapt_connector_candidates(
        graph, _zones(), VehicleFeasibleMotionGraphConfig()
    )
    assert len(requests) == 1
    request = requests[0]
    binding = bindings[request.connector_id]
    assert request.connector_id == "headland.turn_high_u.a_to_b"
    assert request.start_pose == graph.connector_candidates[0].start_pose
    assert request.goal_pose == graph.connector_candidates[0].goal_pose
    assert binding.from_service_state_id == graph.connector_candidates[0].from_service_state_id
```

Use `dataclasses.replace` to change connector `start_pose` by 0.01 m and assert `ValueError` matching `start pose`. Add explicit wrong-side, missing Turn Zone, missing source state, missing target state, and duplicate connector ID tests.

- [ ] **Step 7: Add transition orchestration tests using monkeypatch**

Patch functions inside `vehicle_feasible_transition_motion`, not the original backend modules. Freeze these cases with explicit assertions:

```text
PREVIEW_FOOTPRINT_FREE -> EXECUTABLE, backend FORWARD_DUBINS_NAVIGATION_GATE, reverse_distance=0
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT -> REJECTED, no R6A/R6B call
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT -> UNRESOLVED, no R6B call
LOCAL_FORWARD_MIXED_EVIDENCE -> UNRESOLVED, no R6B call
LOCAL_FORWARD_OCCUPANCY_BLOCKED + REVERSE_PRIMITIVE_PREVIEW_FREE -> EXECUTABLE with R6B metrics preserved
LOCAL_FORWARD_OCCUPANCY_BLOCKED + NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION -> REJECTED + BOUNDED_SEARCH_NO_SOLUTION
R6B_START_FOOTPRINT_NOT_FREE + OCCUPIED start evidence -> REJECTED
R6B_START_FOOTPRINT_NOT_FREE + UNKNOWN/out-of-grid start evidence -> UNRESOLVED
```

For the reverse-success test assert exactly:

```python
assert result.path_length_m == pytest.approx(5.5)
assert result.forward_distance_m == pytest.approx(3.0)
assert result.reverse_distance_m == pytest.approx(2.5)
assert result.cusp_count == 2
assert result.search_expansions == 321
```

- [ ] **Step 8: Add top-level graph/IO tests**

Freeze cardinality preservation, physical coverage dedup, deterministic ordering, overwrite refusal, strict loader cross-reference checks, and byte-stable roundtrip.

```python
def test_graph_deduplicates_two_executable_directions_to_one_physical_coverage():
    r = _resource()
    g = _graph((r,), (
        _ordinary_state(r, SERVICE_LOW_TO_HIGH),
        _ordinary_state(r, SERVICE_HIGH_TO_LOW),
    ))
    motion = derive_vehicle_feasible_motion_graph(
        g, _zones(), _navigation(), _vehicle()
    )
    assert motion.diagnostics.service_validation_count == 2
    assert motion.diagnostics.locally_validated_segment_count == 1
    assert motion.diagnostics.locally_validated_unique_coverage_length_m == pytest.approx(4.0)
```

Serializer test must recursively reject/omit exact forbidden keys `route_ready`, `reachable_from_start`, and `optimal`.

- [ ] **Step 9: Add root A3 harness contract tests**

Create `tests/test_v25_12g_a3_contract.py` using `importlib.util.spec_from_file_location` against `tools/v25_12g_a3_acceptance.py`. Freeze:

```python
REPORT_SCHEMA = "agt_v25_12g_a3_acceptance_report/v1"
VALIDATION_SCOPE = "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
```

Parser contract:

```text
--run-dir required
--vehicle-profile required
--service-graph default vehicle_feasible_service_graph.yaml
--turn-zones default turn_zones.yaml
--navigation-map default navigation_map.yaml
--site-boundary default site_boundary.yaml
--write-motion-graph false
--overwrite-motion-graph false
--pretty false
```

Fixture contract: read-only mode writes nothing; write mode creates only `vehicle_feasible_motion_graph.yaml`; existing output refuses overwrite by default; service graph/Turn Zones/navigation/site boundary sentinel bytes remain unchanged.

- [ ] **Step 10: Commit tests only**

```bash
git add \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  tests/test_v25_12g_a3_contract.py
git commit -m "test(v25-12g): batch A3 motion feasibility contracts"
```

- [ ] **Step 11: Operator Gate R1**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  tests/test_v25_12g_a3_contract.py
```

Expected RED: specific failures for missing service validator, connector adapter/orchestrator, Site Boundary-aware audit behavior, graph derivation/IO, and missing acceptance harness. After these specific RED failures are shown, implement Tasks 3-7 without another operator checkpoint.

---

### Task 3: Implement directional service motion and exact dead-end retrace

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

- [ ] **Step 1: Implement unique indexes and input validation**

Reject duplicate service/resource IDs, missing resource references, non-finite centerlines/poses, frame/platform/profile mismatch, invalid service type, and entry/exit mismatch using the frozen tolerances.

```python
def _angle_error(a, b):
    return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)
```

- [ ] **Step 2: Build ordinary samples**

LOW->HIGH uses stored point order and row yaw. HIGH->LOW uses reversed point order and normalized `row_yaw + pi`. Every sample is `motion_direction="FORWARD"`, `segment_index=0`, `is_cusp=False`.

- [ ] **Step 3: Build exact dead-end samples**

Orient the forward-in sequence from the actual headland endpoint toward the interior endpoint. Append one terminal cusp marker with identical XYZ/yaw, `motion_direction="REVERSE"`, `segment_index=1`, `is_cusp=True`. Then append `reversed(forward_samples[:-1])` as REVERSE samples with unchanged yaw and `is_cusp=False`.

- [ ] **Step 4: Compute path metrics from geometry**

```python
def _step_distance(a, b):
    return math.sqrt((b.x-a.x)**2 + (b.y-a.y)**2 + (b.z-a.z)**2)
```

Assign each non-zero step to forward or reverse distance according to the destination sample's `motion_direction`; cusp count is the number of `is_cusp=True` samples.

- [ ] **Step 5: Reuse the existing footprint evaluator**

Convert `MotionSample` to `ForwardConnectorSample`, call `_evaluate_candidate()` with `_preview_local_footprint(vehicle, config.preview_footprint_padding_m)`, and call `_candidate_inside_site_boundary()` when a boundary exists.

Classification order:

```text
boundary conflict -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION / SITE_BOUNDARY_CONFLICT
OCCUPIED footprint cells -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION / OCCUPIED_FOOTPRINT_CONFLICT
UNKNOWN or grid_coverage<1 -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
otherwise -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE / PREVIEW_FOOTPRINT_FREE
```

- [ ] **Step 6: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_motion.py
git commit -m "feat(v25-12g): validate A3 directional service motion"
```

---

### Task 4: Extend forward candidate audit with optional Site Boundary evidence

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py`

**Interfaces:** add `site_boundary: SiteBoundary | None = None` as a keyword-only parameter; all calls with no boundary remain byte/behavior compatible.

- [ ] **Step 1: Evaluate boundary per sampled forward candidate**

Import `_candidate_inside_site_boundary`. A candidate is preview-free only if both its existing grid predicate and boundary predicate are true.

- [ ] **Step 2: Prevent raster-free boundary-crossing candidates from influencing R6A**

When a boundary is supplied, classify occupancy/map evidence using only locally relevant candidates whose preview footprints are boundary-safe. If every locally relevant candidate crosses/touches the boundary, emit `LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT`.

- [ ] **Step 3: Preserve deterministic source metadata**

Add only `"site_boundary_enforced": site_boundary is not None`.

- [ ] **Step 4: Commit**

```bash
git add src/agt_offline_assets/agt_offline_assets/forward_connector_candidate_audit.py
git commit -m "feat(v25-12g): gate forward audit by site boundary"
```

---

### Task 5: Implement A2 connector adaptation and transition orchestration

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


def adapt_connector_candidates(service_graph, turn_zones, config): ...
def validate_transition_candidates(service_graph, turn_zones, navigation, vehicle, config, *, site_boundary=None): ...
```

- [ ] **Step 1: Implement fail-closed one-to-one adapter**

Check unique IDs, source/target state existence, source/target segment identity, Turn Zone existence/side/support, and exact `exit -> entry` pose binding. Produce `ConnectorRequest` sorted by candidate ID and a binding map keyed by the same ID.

- [ ] **Step 2: Run forward Navigation gate on every request**

Use:

```python
ForwardConnectorNavigationGateConfig(
    preview_footprint_padding_m=config.preview_footprint_padding_m,
)
```

with `site_boundary=site_boundary`. `PREVIEW_FOOTPRINT_FREE` maps directly to `EXECUTABLE`, backend `FORWARD_DUBINS_NAVIGATION_GATE`, with gate path length/samples and zero reverse/cusps.

- [ ] **Step 3: Run Site Boundary-aware audit for unresolved forward cases**

Use `ForwardConnectorCandidateAuditConfig(preview_footprint_padding_m=config.preview_footprint_padding_m)` and pass the same Site Boundary.

Map:

```text
LOCAL_FORWARD_SITE_BOUNDARY_CONFLICT -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
LOCAL_FORWARD_MAP_EVIDENCE_INSUFFICIENT -> UNRESOLVED / MAP_EVIDENCE_INSUFFICIENT
LOCAL_FORWARD_MIXED_EVIDENCE -> UNRESOLVED / MIXED_EVIDENCE_REQUIRES_REVIEW
NO_FORWARD_DUBINS_CANDIDATE -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
TURN_ZONE_METADATA_INVALID -> ValueError because adapter already validated metadata
LOCAL_FORWARD_OCCUPANCY_BLOCKED -> R6A
```

- [ ] **Step 4: Run R6A with no manual approval**

```python
ReverseFallbackAdmissionConfig(operator_approved_mixed_connector_ids=())
```

Only admitted IDs proceed.

- [ ] **Step 5: Run R6B with frozen budgets**

Use `ReversePrimitiveConnectorConfig(preview_footprint_padding_m=config.preview_footprint_padding_m)` and do not set any other field. Pass the same Site Boundary.

Map:

```text
REVERSE_PRIMITIVE_PREVIEW_FREE -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE
FORWARD_PRIMITIVE_PREVIEW_FREE -> EXECUTABLE / LOCAL_MOTION_EXECUTABLE
SITE_BOUNDARY_CONFLICT -> REJECTED / PROVEN_HARD_CONSTRAINT_REJECTION
NO_REVERSE_PRIMITIVE_PREVIEW_SOLUTION -> REJECTED / BOUNDED_SEARCH_NO_SOLUTION
```

For `R6B_START_FOOTPRINT_NOT_FREE`, re-evaluate one start-pose footprint with the shared evaluator. OCCUPIED => `REJECTED`; UNKNOWN/out-of-grid without OCCUPIED => `UNRESOLVED`.

- [ ] **Step 6: Convert R5/R6B samples to unified MotionSample and preserve metrics**

Do not infer or recompute R6B `forward_distance_m`, `reverse_distance_m`, `cusp_count`, or `search_expansions`; copy the backend values into the transition validation.

- [ ] **Step 7: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_transition_motion.py
git commit -m "feat(v25-12g): validate A3 headland transitions"
```

---

### Task 6: Assemble diagnostics and deterministic strict IO

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py`
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

- [ ] **Step 1: Implement top-level input checks and derivation**

Validate A2 schema/status, frames, platform ID, profile hash, row-direction finiteness/orientation, optional Site Boundary frame, then call service and transition validators. Raise if output cardinalities differ from A2 input cardinalities.

- [ ] **Step 2: Build executable ID lists and diagnostics**

Sort executable service IDs and transition IDs lexically. Deduplicate locally validated coverage by `coverage_segment_id`, looking up length and aisle in A2 service resources.

- [ ] **Step 3: Implement deterministic serializer**

Serialize service actions sorted by `service_state_id`, transitions by `connector_candidate_id`, executable ID lists lexically, fixed dictionary key order, and `yaml.safe_dump(sort_keys=False, allow_unicode=True)`.

- [ ] **Step 4: Implement strict loader**

Reject wrong schema/status, invalid enums, non-finite poses/samples/metrics, negative travel/search values, duplicate IDs, bad cross references, invalid motion directions, cusp count mismatch, inconsistent executable ID lists, inconsistent diagnostics, and forbidden keys `route_ready`, `reachable_from_start`, `optimal` anywhere recursively.

- [ ] **Step 5: Writer refuses overwrite by default**

```python
if output.exists() and not overwrite:
    raise FileExistsError(str(output))
```

- [ ] **Step 6: Export API and register test**

Add A3 public constants/dataclasses/derive/load/write to `__init__.py`. Add to CMake:

```cmake
ament_add_pytest_test(test_vehicle_feasible_motion_graph test/test_vehicle_feasible_motion_graph.py)
```

- [ ] **Step 7: Commit**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_motion_graph_io.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12g): serialize A3 motion evidence graph"
```

---

### Task 7: Implement the A3 acceptance harness

**Files:**
- Create: `tools/v25_12g_a3_acceptance.py`

**Frozen CLI:**

```python
REPORT_SCHEMA = "agt_v25_12g_a3_acceptance_report/v1"
VALIDATION_SCOPE = "A3_LOCAL_MOTION_EVIDENCE_DIAGNOSTIC_NOT_ROUTE_READY"
MOTION_GRAPH_ASSET = "vehicle_feasible_motion_graph.yaml"
```

```python
def build_parser():
    parser = argparse.ArgumentParser(description="Validate V25-12G-A3 local vehicle motion evidence")
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

- [ ] **Step 1: Preflight required files before heavy parsing**

Require service graph, Turn Zones, navigation map YAML, Site Boundary, and explicit vehicle profile. If write mode is requested and output exists without overwrite, raise before loading assets.

- [ ] **Step 2: Strict-load frozen inputs and derive graph**

Use `load_vehicle_feasible_service_graph`, `load_turn_zones`, `load_navigation_grid`, `load_site_boundary`, `load_canonical_vehicle_profile`, then call `derive_vehicle_feasible_motion_graph(..., site_boundary=boundary)`.

- [ ] **Step 3: Emit deterministic report**

Top-level keys in order:

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

`summary` includes every A3 diagnostic field plus:

```text
all_a2_service_states_preserved
all_a2_connector_candidates_preserved
dead_end_non_retrace_count
site_boundary_reverse_bypass_count
forbidden_semantic_key_count
```

Raise if any of the final three counts is nonzero.

- [ ] **Step 4: Write only A3 sibling when explicitly requested**

Call `write_vehicle_feasible_motion_graph(..., overwrite=args.overwrite_motion_graph)` and never write upstream assets.

- [ ] **Step 5: Commit**

```bash
git add tools/v25_12g_a3_acceptance.py
git commit -m "feat(v25-12g): add A3 motion acceptance harness"
```

---

### Task 8: Operator Gate G1 — batched focused/package verification

- [ ] **Step 1: Build**

```bash
cd ~/agt_navigation_v2
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
```

- [ ] **Step 2: Focused regression suite**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_motion_graph.py \
  src/agt_offline_assets/test/test_forward_connector_candidate_audit.py \
  src/agt_offline_assets/test/test_forward_connector_navigation_gate.py \
  src/agt_offline_assets/test/test_reverse_fallback_admission.py \
  src/agt_offline_assets/test/test_reverse_primitive_connector.py \
  tests/test_v25_12g_a3_contract.py
```

Expected: all selected tests PASS. Do not state a pass count until operator output supplies it.

- [ ] **Step 3: Registered package tests**

```bash
colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

If anything fails, stop and invoke systematic debugging. Do not patch by intuition. If the existing R1 tests do not specifically capture the defect, add and verify a new RED test before changing production behavior.

---

### Task 9: Frozen real-greenhouse A3 checkpoint

**Files after evidence only:**
- Create: `docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md`
- Create/update: `docs/v2.5/V25_12G_A3_CURRENT_STATE.md`

- [ ] **Step 1: Preflight frozen runtime inputs**

```bash
RUN=/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run
for f in \
  vehicle_feasible_service_graph.yaml \
  vehicle_feasible_segments.yaml \
  turn_zones.yaml \
  navigation_map.yaml \
  navigation_map.pgm \
  site_boundary.yaml \
  derivation.yaml \
  aisle_graph.yaml
do
  test -f "$RUN/$f" || { echo "MISSING: $RUN/$f"; exit 1; }
done
```

Resolve the same canonical `mk_mini` profile used for A1 and set `PROFILE` to that exact path. Do not substitute a new profile.

- [ ] **Step 2: Freeze upstream hashes**

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

- [ ] **Step 3: Run read-only acceptance first**

```bash
python3 tools/v25_12g_a3_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --pretty \
  > /tmp/v25_12g_a3_readonly.json
```

Required structural facts:

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

Do not hardcode executable/rejected/unresolved totals.

- [ ] **Step 4: Inspect real evidence funnel**

Record actual service executable/rejected/unresolved counts; ordinary/dead-end counts; locally validated segment count/unique length/distinct aisles; forward-executable/reverse-executable/rejected/unresolved transition counts; map-insufficient, mixed-evidence, boundary-conflict, and bounded-search-no-solution counts.

Require explicit records for:

```text
aisle_020.segment_001.service_low_to_high
aisle_020.segment_001.service_high_to_low
```

but do not predeclare their status.

Require one dead-end validation record for each A2 dead-end state and verify exact-retrace structure, one cusp, equal forward/reverse geometric distance, and single-count coverage reward regardless of top-level result class.

- [ ] **Step 5: Narrow-write only when no existing A3 asset exists**

```bash
test ! -e "$RUN/vehicle_feasible_motion_graph.yaml" || exit 2
python3 tools/v25_12g_a3_acceptance.py \
  --run-dir "$RUN" \
  --vehicle-profile "$PROFILE" \
  --write-motion-graph \
  --pretty \
  > /tmp/v25_12g_a3_write.json
```

- [ ] **Step 6: Prove upstream hashes unchanged**

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

Expected diff is empty.

- [ ] **Step 7: Strict-load and byte-stable rewrite**

```bash
python3 - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
from agt_offline_assets.vehicle_feasible_motion_graph import (
    load_vehicle_feasible_motion_graph,
    write_vehicle_feasible_motion_graph,
)

src = Path("/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_motion_graph.yaml")
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

Strongest allowed classification:

```text
A3 LOCAL VEHICLE-MOTION EVIDENCE SUPPORTED
REAL-DATA OBSERVED
MOTION_EVIDENCE_ONLY
NOT START-REACHABILITY VALIDATED
NOT ROUTE-READY
```

If real evidence exposes meaningful map/footprint limitations, add `WITH MAP / FOOTPRINT CAVEATS`.

`V25_12G_A3_CURRENT_STATE.md` names the next phase exactly:

```text
V25-12G-A4 START_POSE-Aware Maximum Feasible Coverage Optimizer
```

- [ ] **Step 9: Commit docs only**

```bash
git add \
  docs/v2.5/V25_12G_A3_REAL_DATA_2026-08-17.md \
  docs/v2.5/V25_12G_A3_CURRENT_STATE.md
git commit -m "docs(v25-12g): record A3 real-data motion checkpoint"
```

Do not add runtime YAML assets to git.

---

## Self-Review Checklist

Before execution, verify this plan against the approved spec:

- A2 remains immutable.
- Every A2 service state and connector candidate has one A3 record.
- Ordinary direction semantics are explicit.
- Dead-end exact retrace semantics and cusp representation are explicit.
- Coverage reward and travel metrics are separate.
- Site Boundary is hard and cannot be bypassed by R6A/R6B.
- UNKNOWN stays unresolved.
- Mixed evidence has no automatic operator approval.
- R6B budgets stay frozen.
- Bounded search failure is not global infeasibility.
- Strict deterministic IO and byte-stability are covered.
- START_POSE/reachability/optimality are excluded.
- Real-data gates preserve the frozen 32 service / 40 transition cardinalities.

No implementation task may weaken these constraints to improve apparent coverage.

## Final Verification Before Completion Claim

Before claiming A3 complete, invoke `superpowers:verification-before-completion` and require fresh operator evidence for:

```text
focused A3/backend tests: zero failures
agt_offline_assets package tests: zero A3-related failures
root A3 contract tests: zero failures
real-data service_validation_count = 32
real-data transition_validation_count = 40
all A2 IDs preserved
no dead-end non-retrace record
no Site Boundary reverse bypass
upstream hash diff empty
strict loader PASS
byte-stable rewrite PASS
```

Only then enter the branch-finishing workflow and begin A4 design.
