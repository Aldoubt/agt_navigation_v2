# V25-12G-A2 Service / Connectivity Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert frozen A1 vehicle-feasible physical segments plus Turn Zone evidence into one deterministic static directional A2 service graph, without claiming motion feasibility, robot reachability, route readiness, or optimality.

**Architecture:** Add one focused `vehicle_feasible_service_graph.py` module. A1 `segment_id` remains the single physical coverage identity. A2 expands service direction/state, adds one-headland `DEAD_END_FORWARD_IN_REVERSE_OUT` candidates, emits only same-side Turn-Zone-supported `exit -> entry` directed connector candidates, groups them into diagnostic candidate topology components, and serializes one strict sibling YAML. A diagnostic-only acceptance harness reads `vehicle_feasible_segments.yaml` plus `turn_zones.yaml` and may write only `vehicle_feasible_service_graph.yaml`.

**Tech Stack:** Python 3.10, dataclasses, math, pathlib, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing `VehicleFeasibleSegmentPlan`, `TurnZoneSet`, `load_turn_zones`, and the existing strict A1 loader/writer.

## Global Constraints

- Work only on `feat/v25-12g-maximum-feasible-coverage` in the existing checkout; do not create another worktree.
- Scope is A2 Core only: service resources, directional service states, one-headland dead-end topology candidates, directed headland connector candidates, candidate topology components, deterministic YAML IO, diagnostics, and real-data A2 acceptance.
- Do not implement A3 Dubins/reverse connector search, connector footprint/Navigation Grid/Site Boundary validation, minimum-turn-radius connector validation, executable components, START anchoring, A4 optimization, MILP, RL, or manual override.
- A2 is a static map-level asset; `START_POSE` does not belong in the A2 graph.
- A1 `VehicleFeasibleSegmentPlan` is physical coverage truth. A2 must not re-derive, shorten, merge, discard, or move A1 active segments.
- Coverage identity is exactly `segment_id`; multiple service states never create multiple coverage resources.
- A2 v1 service types are exactly `SERVICE_LOW_TO_HIGH`, `SERVICE_HIGH_TO_LOW`, and `DEAD_END_FORWARD_IN_REVERSE_OUT`.
- `DEAD_END_REVERSE_IN_FORWARD_OUT` is out of scope.
- Connector candidates are directed and strictly mean `from_service_state.exit -> to_service_state.entry`.
- Cross-aisle candidates require real same-side headland endpoints, one shared Turn Zone supporting both aisles, and `allow_turn == true`.
- LOW_U and HIGH_U domains never connect directly in A2.
- `INTERIOR_BLOCKED_END` never becomes a cross-aisle connector port.
- Candidate topology components are diagnostic only; they never mean `EXECUTABLE`, `A3_VALIDATED`, `REACHABLE_FROM_START`, or `ROUTE_READY`.
- Same-resource membership may group states diagnostically, but it must never be serialized as a fake motion edge.
- Dead-end reverse distance is an accounting expectation only; A3 must independently validate reverse motion.
- A2 may write only `vehicle_feasible_service_graph.yaml`; it must never modify `vehicle_feasible_segments.yaml`, `turn_zones.yaml`, `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml`, `aisle_graph.yaml`, or `site_boundary.yaml`.
- Workbench A2 visualization is deferred to a separate follow-up plan.
- Contract violations fail closed; do not skip malformed records and return a partial graph.
- Identical semantic inputs must serialize to byte-identical YAML.
- Do not use `git reset --hard` or `git clean`; do not touch `tools/rosbag_sensor_trimmer` or `tools/map_tools/render_pcd_top_views.py`.
- Follow strict TDD. The operator must show the expected RED output before the corresponding production change is made.
- Do not claim tests, package verification, or real-data acceptance passed until operator-machine output proves it.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_feasible_service_graph.py
  __init__.py
src/agt_offline_assets/test/
  test_vehicle_feasible_service_graph.py
src/agt_offline_assets/CMakeLists.txt
tools/v25_12g_a2_acceptance.py
tests/test_v25_12g_a2_contract.py
docs/v2.5/V25_12G_A2_CURRENT_STATE.md
docs/v2.5/V25_12G_A2_REAL_DATA_2026-08-16.md
```

Do not modify `vehicle_feasible_segment.py`, `turn_zones.py`, `agricultural_coverage_ordering.py`, or `forward_connector.py` unless a separately verified upstream defect makes A2 impossible.

---

## Frozen Public Contract

Constants:

```python
VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA = "agt_vehicle_feasible_service_graph/v1"
TOPOLOGY_CANDIDATE_ONLY = "TOPOLOGY_CANDIDATE_ONLY"
SERVICE_LOW_TO_HIGH = "SERVICE_LOW_TO_HIGH"
SERVICE_HIGH_TO_LOW = "SERVICE_HIGH_TO_LOW"
DEAD_END_FORWARD_IN_REVERSE_OUT = "DEAD_END_FORWARD_IN_REVERSE_OUT"
FORWARD = "FORWARD"
TOPOLOGY_SERVICE_CANDIDATE = "TOPOLOGY_SERVICE_CANDIDATE"
REQUIRES_A3_REVERSE_SERVICE_VALIDATION = "REQUIRES_A3_REVERSE_SERVICE_VALIDATION"
HEADLAND_TOPOLOGY_CANDIDATE = "HEADLAND_TOPOLOGY_CANDIDATE"
EXTERNAL_REACHABILITY_UNPROVEN = "EXTERNAL_REACHABILITY_UNPROVEN"
HEADLAND_CONNECTOR_CANDIDATE = "HEADLAND_CONNECTOR_CANDIDATE"
REQUIRES_A3_CONNECTOR_VALIDATION = "REQUIRES_A3_CONNECTOR_VALIDATION"
CANDIDATE_TOPOLOGY_COMPONENT = "CANDIDATE_TOPOLOGY_COMPONENT"
```

Dataclasses:

```python
@dataclass(frozen=True)
class ServiceResource:
    segment_id: str
    aisle_id: str
    ordinal_in_aisle: int
    coverage_length_m: float
    coverage_fraction_of_aisle: float
    low_endpoint_type: str
    high_endpoint_type: str
    low_endpoint_pose: tuple[float, float, float, float]
    high_endpoint_pose: tuple[float, float, float, float]
    centerline_xyz: tuple[tuple[float, float, float], ...]

@dataclass(frozen=True)
class ServiceState:
    service_state_id: str
    segment_id: str
    aisle_id: str
    service_type: str
    entry_endpoint_type: str
    exit_endpoint_type: str
    entry_pose: tuple[float, float, float, float]
    exit_pose: tuple[float, float, float, float]
    service_motion_direction: str
    forward_service_distance_m: float
    reverse_service_distance_m: float
    coverage_segment_id: str
    coverage_reward_length_m: float
    external_reachability_status: str
    validation_status: str

@dataclass(frozen=True)
class ConnectorCandidate:
    connector_candidate_id: str
    from_service_state_id: str
    to_service_state_id: str
    from_segment_id: str
    to_segment_id: str
    side: str
    turn_zone_id: str
    start_pose: tuple[float, float, float, float]
    goal_pose: tuple[float, float, float, float]
    candidate_kind: str = HEADLAND_CONNECTOR_CANDIDATE
    validation_status: str = REQUIRES_A3_CONNECTOR_VALIDATION

@dataclass(frozen=True)
class CandidateTopologyComponent:
    component_id: str
    classification: str
    service_state_ids: tuple[str, ...]
    segment_ids: tuple[str, ...]
    connector_candidate_ids: tuple[str, ...]
    total_unique_coverage_length_m: float
    distinct_aisle_count: int

@dataclass(frozen=True)
class ServiceGraphDiagnostics:
    resource_count: int
    ordinary_service_state_count: int
    dead_end_candidate_count: int
    total_service_state_count: int
    connector_candidate_count: int
    low_u_connector_candidate_count: int
    high_u_connector_candidate_count: int
    candidate_component_count: int
    isolated_component_count: int
    externally_unproven_state_count: int
    unique_coverage_length_m: float
    distinct_aisle_count: int

@dataclass(frozen=True)
class VehicleFeasibleServiceGraph:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    row_direction_xy: tuple[float, float]
    service_resources: tuple[ServiceResource, ...]
    service_states: tuple[ServiceState, ...]
    connector_candidates: tuple[ConnectorCandidate, ...]
    candidate_components: tuple[CandidateTopologyComponent, ...]
    diagnostics: ServiceGraphDiagnostics
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA
    status: str = TOPOLOGY_CANDIDATE_ONLY
```

Exact public signatures:

```python
def derive_vehicle_feasible_service_graph(
    segment_plan: VehicleFeasibleSegmentPlan,
    turn_zones: TurnZoneSet,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleServiceGraph:
    """Derive static A2 service topology from frozen A1 and Turn Zone evidence."""


def vehicle_feasible_service_graph_to_dict(
    graph: VehicleFeasibleServiceGraph,
) -> dict[str, Any]:
    """Serialize the frozen A2 graph contract deterministically."""


def write_vehicle_feasible_service_graph(
    graph: VehicleFeasibleServiceGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    """Write only the A2 sibling asset, refusing overwrite by default."""


def load_vehicle_feasible_service_graph(
    path: str | Path,
) -> VehicleFeasibleServiceGraph:
    """Strict-load one v1 A2 graph and reject inconsistent topology claims."""
```

These snippets freeze names/signatures only; production bodies are added after RED tests.

---

## Shared Test Fixtures

Put these helpers at the top of `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`. The A1 source configuration is intentionally complete so the same fixture can later be serialized by the A1 writer.

```python
_A1_SOURCE = {
    "vehicle_safe_lane_configuration": {
        "sample_spacing_m": 0.10,
        "lateral_search_step_m": 0.05,
        "maximum_lateral_shift_m": 0.50,
        "maximum_lateral_step_m": 0.15,
        "preview_footprint_padding_m": 0.05,
        "minimum_lane_coverage_fraction": 0.70,
        "maximum_endpoint_retreat_m": 2.00,
        "minimum_contiguous_span_m": 1.00,
    }
}


def _segment(
    segment_id="aisle_001.segment_001",
    aisle_id="aisle_001",
    ordinal=1,
    *,
    length=4.0,
    y=0.0,
    low_type=LOW_U_HEADLAND,
    high_type=HIGH_U_HEADLAND,
    yaw=0.0,
):
    return VehicleFeasibleSegment(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=ordinal,
        start_distance_m=0.0,
        end_distance_m=length,
        length_m=length,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=low_type,
        high_endpoint_type=high_type,
        centerline_xyz=((0.0, y, 0.0), (length, y, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, y, 0.0, yaw),
        high_endpoint_pose=(length, y, 0.0, yaw),
    )


def _plan(*segments, frame_id="map", row_direction=(1.0, 0.0)):
    grouped = {}
    for segment in segments:
        grouped.setdefault(segment.aisle_id, []).append(segment)
    aisles = []
    for aisle_id in sorted(grouped):
        items = tuple(sorted(grouped[aisle_id], key=lambda item: item.ordinal_in_aisle))
        aisles.append(
            AisleFeasibleSegmentResult(
                aisle_id=aisle_id,
                structural_length_m=sum(item.length_m for item in items),
                active_segments=items,
                rejected_fragments=(),
                raw_feasible_fragment_count=len(items),
                allowed_lateral_shift_m=0.5,
                site_boundary_rejected_pose_count=0,
                site_boundary_limited_sample_count=0,
                grid_rejected_pose_count=0,
                reason="test fixture",
            )
        )
    return VehicleFeasibleSegmentPlan(
        frame_id=frame_id,
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=row_direction,
        aisles=tuple(aisles),
        source=dict(_A1_SOURCE),
    )


def _zone(zone_id="turn_low_u", side="LOW_U", aisle_ids=("aisle_001",), allow_turn=True):
    aisle_ids = tuple(aisle_ids)
    return TurnZone(
        zone_id=zone_id,
        side=side,
        polygon_xy=((-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)),
        supported_aisle_ids=aisle_ids,
        endpoint_count=len(aisle_ids),
        free_fraction=1.0,
        allow_turn=allow_turn,
    )


def _zones(*zones, frame_id="map", row_direction=(1.0, 0.0)):
    return TurnZoneSet(
        frame_id=frame_id,
        row_direction_xy=row_direction,
        zones=tuple(zones),
        source={},
    )
```

Imports required by the fixture block:

```python
import math
import yaml
import pytest
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    AisleFeasibleSegmentResult,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
)
```

---

### Task 1: Physical resources and ordinary directional service states

**Files:** create A2 module and focused test file.

**Interfaces:** `_build_service_resources_and_states(segment_plan) -> (resources, states)`; at this task boundary it creates exactly two ordinary states per resource.

- [ ] **Step 1: Write failing ordinary-state test**

```python
def test_resource_expands_into_two_forward_directional_states():
    resources, states = _build_service_resources_and_states(_plan(_segment()))
    assert len(resources) == 1
    assert resources[0].segment_id == "aisle_001.segment_001"
    assert resources[0].coverage_length_m == pytest.approx(4.0)
    assert len(states) == 2
    low = next(state for state in states if state.service_type == SERVICE_LOW_TO_HIGH)
    high = next(state for state in states if state.service_type == SERVICE_HIGH_TO_LOW)
    assert low.entry_endpoint_type == LOW_U_HEADLAND
    assert low.exit_endpoint_type == HIGH_U_HEADLAND
    assert low.entry_pose == (0.0, 0.0, 0.0, 0.0)
    assert low.exit_pose == (4.0, 0.0, 0.0, 0.0)
    assert low.service_motion_direction == FORWARD
    assert low.coverage_segment_id == resources[0].segment_id
    assert low.coverage_reward_length_m == pytest.approx(4.0)
    assert low.validation_status == TOPOLOGY_SERVICE_CANDIDATE
    assert low.external_reachability_status == HEADLAND_TOPOLOGY_CANDIDATE
    assert high.entry_endpoint_type == HIGH_U_HEADLAND
    assert high.exit_endpoint_type == LOW_U_HEADLAND
    assert high.entry_pose[3] == pytest.approx(-math.pi)
    assert high.exit_pose[3] == pytest.approx(-math.pi)
```

- [ ] **Step 2: Run RED and commit test only**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_resource_expands_into_two_forward_directional_states
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 directional service resources"
```

Expected: import failure because the A2 module/API does not exist.

- [ ] **Step 3: Implement minimum ordinary-state model**

```python
def _normalize_angle(yaw: float) -> float:
    return (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi


def _reverse_heading(pose: tuple[float, float, float, float]):
    x, y, z, yaw = pose
    return (float(x), float(y), float(z), _normalize_angle(float(yaw) + math.pi))
```

Implement deterministic resource ordering `(aisle_id, ordinal_in_aisle, segment_id)`. Copy A1 geometry verbatim. LOW->HIGH uses A1 low/high poses. HIGH->LOW uses `_reverse_heading(high_pose)` then `_reverse_heading(low_pose)`. `external_reachability_status` depends only on the actual entry endpoint: headland entry => `HEADLAND_TOPOLOGY_CANDIDATE`, otherwise `EXTERNAL_REACHABILITY_UNPROVEN`.

- [ ] **Step 4: Run GREEN/regression and commit**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_resource_expands_into_two_forward_directional_states
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_segment.py
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): expand A1 segments into directional service states"
```

---

### Task 2: Dead-end expansion and fail-closed static-input validation

**Files:** modify A2 module/test.

**Interfaces:** add `_validate_a2_inputs(segment_plan, turn_zones) -> None`; add one dead-end state only for exactly-one-headland resources.

- [ ] **Step 1: Write dead-end RED tests**

```python
def test_exactly_one_low_headland_adds_dead_end_candidate():
    segment = _segment(high_type=INTERIOR_BLOCKED_END)
    _resources, states = _build_service_resources_and_states(_plan(segment))
    dead = [state for state in states if state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT]
    assert len(dead) == 1
    state = dead[0]
    assert state.entry_endpoint_type == LOW_U_HEADLAND
    assert state.exit_endpoint_type == LOW_U_HEADLAND
    assert state.entry_pose == segment.low_endpoint_pose
    assert state.exit_pose == segment.low_endpoint_pose
    assert state.forward_service_distance_m == pytest.approx(segment.length_m)
    assert state.reverse_service_distance_m == pytest.approx(segment.length_m)
    assert state.validation_status == REQUIRES_A3_REVERSE_SERVICE_VALIDATION


def test_exactly_one_high_headland_reverses_dead_end_heading():
    segment = _segment(low_type=INTERIOR_BLOCKED_END, high_type=HIGH_U_HEADLAND, yaw=0.25)
    _resources, states = _build_service_resources_and_states(_plan(segment))
    state = next(item for item in states if item.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT)
    assert state.entry_pose[:3] == segment.high_endpoint_pose[:3]
    assert state.entry_pose[3] == pytest.approx(_normalize_angle(0.25 + math.pi))
    assert state.exit_pose == state.entry_pose


def test_double_interior_stays_present_without_dead_end_candidate():
    segment = _segment(low_type=INTERIOR_BLOCKED_END, high_type=INTERIOR_BLOCKED_END)
    resources, states = _build_service_resources_and_states(_plan(segment))
    assert len(resources) == 1
    assert len(states) == 2
    assert all(state.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT for state in states)
    assert all(state.external_reachability_status == EXTERNAL_REACHABILITY_UNPROVEN for state in states)
```

- [ ] **Step 2: Run RED, commit tests, implement dead-end, run GREEN**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'dead_end or double_interior'
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 dead-end service candidates"
```

Implementation rule:

```python
low_headland = resource.low_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
high_headland = resource.high_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
if low_headland != high_headland:
    # create exactly one DEAD_END_FORWARD_IN_REVERSE_OUT state
    # LOW headland uses low pose; HIGH headland uses reverse_heading(high pose)
```

State ID is `<segment_id>.dead_end_forward_in_reverse_out`; forward/reverse accounting distances both equal resource length.

- [ ] **Step 3: Write validation RED tests**

```python
@pytest.mark.parametrize(("plan_frame", "zone_frame"), (("map", "odom"), ("odom", "map")))
def test_validate_a2_inputs_rejects_frame_mismatch(plan_frame, zone_frame):
    with pytest.raises(ValueError, match="frame"):
        _validate_a2_inputs(_plan(_segment(), frame_id=plan_frame), _zones(_zone(), frame_id=zone_frame))


def test_validate_a2_inputs_rejects_opposite_row_direction():
    with pytest.raises(ValueError, match="row_direction"):
        _validate_a2_inputs(
            _plan(_segment(), row_direction=(1.0, 0.0)),
            _zones(_zone(), row_direction=(-1.0, 0.0)),
        )


def test_validate_a2_inputs_rejects_duplicate_zone_ids():
    zone = _zone()
    with pytest.raises(ValueError, match="duplicate.*zone"):
        _validate_a2_inputs(_plan(_segment()), _zones(zone, zone))


def test_validate_a2_inputs_rejects_unknown_endpoint_type():
    with pytest.raises(ValueError, match="endpoint"):
        _validate_a2_inputs(_plan(_segment(low_type="UNKNOWN_ENDPOINT")), _zones(_zone()))
```

- [ ] **Step 4: Run validation RED and commit tests**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'validate_a2_inputs'
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 fail-closed topology inputs"
```

- [ ] **Step 5: Implement exact validation and commit GREEN**

Reject wrong A1 schema; empty frame/platform/profile hash; frame mismatch; non-finite/zero row direction; opposite row direction; normalized component difference above `1e-9`; empty/duplicate segment ID; non-finite/non-positive segment length; unknown endpoint type; non-finite endpoint pose; empty centerline; empty/duplicate zone ID; zone side outside `{LOW_U, HIGH_U}`; duplicate aisle ID inside one zone support list.

```python
a = _normalize_xy(segment_plan.row_direction_xy)
b = _normalize_xy(turn_zones.row_direction_xy)
if a[0] * b[0] + a[1] * b[1] <= 0.0:
    raise ValueError("A2 row_direction_xy orientation mismatch")
if max(abs(a[0] - b[0]), abs(a[1] - b[1])) > 1.0e-9:
    raise ValueError("A2 row_direction_xy mismatch")
```

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): add A2 dead-end states and input gates"
```

---

### Task 3: Directed same-side Turn-Zone connector candidates

**Files:** modify A2 module/test.

**Interfaces:** add `ConnectorCandidate`; add `_build_connector_candidates(states, turn_zones)`; uniqueness key `(from_service_state_id, to_service_state_id, turn_zone_id)`.

- [ ] **Step 1: Write directed-candidate RED test**

```python
def test_same_side_shared_turn_zone_emits_independent_directions():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001", y=0.0)
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    _resources, states = _build_service_resources_and_states(_plan(s1, s2))
    candidates = _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))))
    low = [candidate for candidate in candidates if candidate.side == "LOW_U"]
    assert len(low) == 2
    assert {(candidate.from_segment_id, candidate.to_segment_id) for candidate in low} == {
        ("aisle_001.segment_001", "aisle_002.segment_001"),
        ("aisle_002.segment_001", "aisle_001.segment_001"),
    }
    for candidate in low:
        from_state = next(state for state in states if state.service_state_id == candidate.from_service_state_id)
        to_state = next(state for state in states if state.service_state_id == candidate.to_service_state_id)
        assert candidate.start_pose == from_state.exit_pose
        assert candidate.goal_pose == to_state.entry_pose
        assert candidate.validation_status == REQUIRES_A3_CONNECTOR_VALIDATION
```

- [ ] **Step 2: Run RED/commit; implement minimum candidate builder**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_side_shared_turn_zone_emits_independent_directions
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 directed headland candidates"
```

Implementation filter:

```python
if (
    from_side == to_side == zone.side
    and from_aisle in zone.supported_aisle_ids
    and to_aisle in zone.supported_aisle_ids
    and zone.allow_turn
    and from_segment_id != to_segment_id
):
    candidate_id = f"headland.{zone.zone_id}.{from_state.service_state_id}.to.{to_state.service_state_id}"
```

Only service-state headland exits become outgoing ports; only headland entries become incoming ports. Sort by `(turn_zone_id, from_service_state_id, to_service_state_id)`.

- [ ] **Step 3: Add exact negative/filter tests**

```python
def test_connector_candidates_never_reference_double_interior_resource():
    interior = _segment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    through = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    _resources, states = _build_service_resources_and_states(_plan(interior, through))
    candidates = _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))))
    assert all(candidate.from_segment_id != interior.segment_id for candidate in candidates)
    assert all(candidate.to_segment_id != interior.segment_id for candidate in candidates)


def test_connector_candidates_keep_low_and_high_domains_separate():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    _resources, states = _build_service_resources_and_states(_plan(s1, s2))
    zones = _zones(
        _zone(zone_id="turn_low_u", side="LOW_U", aisle_ids=("aisle_001", "aisle_002")),
        _zone(zone_id="turn_high_u", side="HIGH_U", aisle_ids=("aisle_001", "aisle_002")),
    )
    by_id = {state.service_state_id: state for state in states}
    candidates = _build_connector_candidates(states, zones)
    assert {candidate.side for candidate in candidates} == {"LOW_U", "HIGH_U"}
    for candidate in candidates:
        expected = LOW_U_HEADLAND if candidate.side == "LOW_U" else HIGH_U_HEADLAND
        assert by_id[candidate.from_service_state_id].exit_endpoint_type == expected
        assert by_id[candidate.to_service_state_id].entry_endpoint_type == expected


def test_connector_candidates_respect_allow_turn_and_supported_aisles():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    _resources, states = _build_service_resources_and_states(_plan(s1, s2))
    assert _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001", "aisle_002"), allow_turn=False))) == ()
    assert _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001",)))) == ()


def test_connector_candidates_forbid_same_physical_segment():
    _resources, states = _build_service_resources_and_states(_plan(_segment()))
    assert _build_connector_candidates(states, _zones(_zone())) == ()


def test_same_aisle_different_segment_is_not_hard_forbidden():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001", ordinal=1)
    s2 = _segment(segment_id="aisle_001.segment_002", aisle_id="aisle_001", ordinal=2, y=0.4)
    _resources, states = _build_service_resources_and_states(_plan(s1, s2))
    candidates = _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001",))))
    assert any(candidate.from_segment_id == s1.segment_id and candidate.to_segment_id == s2.segment_id for candidate in candidates)
    assert any(candidate.from_segment_id == s2.segment_id and candidate.to_segment_id == s1.segment_id for candidate in candidates)
```

- [ ] **Step 4: Run filter tests, commit tests, finish missing rules, commit GREEN**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'connector_candidates'
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): lock A2 connector topology filters"
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'connector'
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): derive directed A2 headland connector candidates"
```

If the first filter run is already green because the minimal builder exactly satisfies all frozen rules, record that evidence rather than inventing a failure.

---

### Task 4: Candidate topology components, diagnostics, top-level derivation

**Files:** modify A2 module/test.

**Interfaces:** add `CandidateTopologyComponent`, `ServiceGraphDiagnostics`, `VehicleFeasibleServiceGraph`, `_build_candidate_components`, `_build_diagnostics`, and public `derive_vehicle_feasible_service_graph`.

- [ ] **Step 1: Write component RED test**

```python
def test_same_resource_states_share_component_without_fake_motion_candidate():
    segment = _segment(low_type=INTERIOR_BLOCKED_END, high_type=INTERIOR_BLOCKED_END)
    resources, states = _build_service_resources_and_states(_plan(segment))
    components = _build_candidate_components(resources, states, ())
    assert len(components) == 1
    component = components[0]
    assert component.classification == CANDIDATE_TOPOLOGY_COMPONENT
    assert component.service_state_ids == tuple(sorted(state.service_state_id for state in states))
    assert component.segment_ids == (segment.segment_id,)
    assert component.connector_candidate_ids == ()
    assert component.total_unique_coverage_length_m == pytest.approx(segment.length_m)
```

- [ ] **Step 2: Run RED/commit; implement diagnostic grouping**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_resource_states_share_component_without_fake_motion_candidate
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 candidate topology components"
```

Use service states as nodes. Undirected diagnostic relations are exactly: all states sharing one `segment_id`, and each directed candidate's source/destination pair. Do not create fake connector candidates for same-resource grouping. Sort components by minimum service-state ID and assign `candidate_component_001`, `candidate_component_002`, and so on.

- [ ] **Step 3: Add inter-resource coverage-dedup test**

```python
def test_connector_candidate_joins_resources_and_component_coverage_is_unique():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001", length=4.0)
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", length=6.0, y=2.0)
    resources, states = _build_service_resources_and_states(_plan(s1, s2))
    candidates = _build_connector_candidates(states, _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))))
    components = _build_candidate_components(resources, states, candidates)
    assert len(components) == 1
    assert components[0].segment_ids == ("aisle_001.segment_001", "aisle_002.segment_001")
    assert components[0].total_unique_coverage_length_m == pytest.approx(10.0)
```

Run it. If it fails, add the connector-induced relation; if it passes from Step 2's complete grouping, retain that evidence.

- [ ] **Step 4: Write top-level derive/diagnostics RED test**

```python
def test_derive_service_graph_reports_static_candidate_diagnostics():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0, high_type=INTERIOR_BLOCKED_END)
    plan = _plan(s1, s2)
    zones = _zones(_zone(aisle_ids=("aisle_001", "aisle_002")))
    graph = derive_vehicle_feasible_service_graph(
        plan,
        zones,
        source={"vehicle_feasible_segments_asset": "vehicle_feasible_segments.yaml", "turn_zones_asset": "turn_zones.yaml"},
    )
    assert graph.schema == VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA
    assert graph.status == TOPOLOGY_CANDIDATE_ONLY
    assert graph.diagnostics.resource_count == 2
    assert graph.diagnostics.ordinary_service_state_count == 4
    assert graph.diagnostics.dead_end_candidate_count == 1
    assert graph.diagnostics.total_service_state_count == 5
    assert graph.diagnostics.unique_coverage_length_m == pytest.approx(8.0)
    assert graph.source["derivation_kind"] == "V25_12G_A2_STATIC_SERVICE_TOPOLOGY"
    assert graph.source["vehicle_feasible_segment_schema"] == plan.schema
    assert graph.source["turn_zone_schema"] == zones.schema
```

- [ ] **Step 5: Run RED/commit; implement top-level graph and diagnostics**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_derive_service_graph_reports_static_candidate_diagnostics
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 graph diagnostics contract"
```

Implementation sequence is fixed: validate -> resources/states -> candidates -> components -> diagnostics -> invariant checks -> graph. Verify resource count equals A1 active-segment count and absolute A1/A2 coverage difference is at most `1e-9`. Fixed provenance keys override caller conflicts:

```python
fixed_source = {
    "vehicle_feasible_segment_schema": segment_plan.schema,
    "turn_zone_schema": turn_zones.schema,
    "derivation_kind": "V25_12G_A2_STATIC_SERVICE_TOPOLOGY",
}
```

`isolated_component_count` means components with zero connector candidate IDs. `externally_unproven_state_count` means states marked `EXTERNAL_REACHABILITY_UNPROVEN`.

- [ ] **Step 6: Verify GREEN and commit**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git add src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): assemble A2 static candidate topology graph"
```

---

### Task 5: Deterministic strict YAML IO, exports, ament registration

**Files:** modify A2 module, `__init__.py`, focused test, CMake.

**Interfaces:** add serializer/writer/strict loader; writer defaults to `overwrite=False`.

- [ ] **Step 1: Write round-trip RED tests**

```python
def _graph_fixture():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    return derive_vehicle_feasible_service_graph(_plan(s1, s2), _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))))


def test_service_graph_yaml_round_trip_is_equal_and_byte_stable(tmp_path):
    graph = _graph_fixture()
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    write_vehicle_feasible_service_graph(graph, first)
    loaded = load_vehicle_feasible_service_graph(first)
    write_vehicle_feasible_service_graph(loaded, second)
    assert loaded == graph
    assert first.read_bytes() == second.read_bytes()


def test_service_graph_writer_refuses_overwrite_by_default(tmp_path):
    graph = _graph_fixture()
    output = tmp_path / "vehicle_feasible_service_graph.yaml"
    write_vehicle_feasible_service_graph(graph, output)
    with pytest.raises(FileExistsError, match="vehicle_feasible_service_graph.yaml"):
        write_vehicle_feasible_service_graph(graph, output)
```

- [ ] **Step 2: Run RED/commit; implement deterministic writer/initial loader**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'yaml_round_trip or writer_refuses'
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 deterministic graph YAML"
```

Top-level YAML order is exactly `schema`, `status`, `frame_id`, `platform_id`, `platform_profile_sha256`, `row_direction_xy`, `source`, `diagnostics`, `service_resources`, `service_states`, `connector_candidates`, `candidate_components`. Sort resources by `segment_id`; states by `(segment_id, service-type rank)`; candidates by `(turn_zone_id, from_service_state_id, to_service_state_id)`; components by `component_id`; component ID lists lexically. Use `yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)`.

- [ ] **Step 3: Write tamper RED tests**

```python
def _write_tampered_graph(tmp_path, mutate):
    valid = tmp_path / "valid.yaml"
    tampered = tmp_path / "tampered.yaml"
    write_vehicle_feasible_service_graph(_graph_fixture(), valid)
    payload = yaml.safe_load(valid.read_text(encoding="utf-8"))
    mutate(payload)
    tampered.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return tampered


def test_loader_rejects_unknown_schema(tmp_path):
    path = _write_tampered_graph(tmp_path, lambda payload: payload.__setitem__("schema", "unknown/v1"))
    with pytest.raises(ValueError, match="schema"):
        load_vehicle_feasible_service_graph(path)


def test_loader_rejects_duplicate_service_state_id(tmp_path):
    def mutate(payload):
        payload["service_states"][1]["service_state_id"] = payload["service_states"][0]["service_state_id"]
    path = _write_tampered_graph(tmp_path, mutate)
    with pytest.raises(ValueError, match="duplicate.*service_state"):
        load_vehicle_feasible_service_graph(path)


def test_loader_rejects_connector_pose_not_matching_state_endpoint(tmp_path):
    def mutate(payload):
        payload["connector_candidates"][0]["start_pose"][0] += 0.25
    path = _write_tampered_graph(tmp_path, mutate)
    with pytest.raises(ValueError, match="start_pose"):
        load_vehicle_feasible_service_graph(path)


def test_loader_rejects_tampered_component_coverage(tmp_path):
    def mutate(payload):
        payload["candidate_components"][0]["total_unique_coverage_length_m"] += 1.0
    path = _write_tampered_graph(tmp_path, mutate)
    with pytest.raises(ValueError, match="component.*coverage"):
        load_vehicle_feasible_service_graph(path)


def test_loader_rejects_tampered_diagnostics(tmp_path):
    def mutate(payload):
        payload["diagnostics"]["resource_count"] += 1
    path = _write_tampered_graph(tmp_path, mutate)
    with pytest.raises(ValueError, match="diagnostics"):
        load_vehicle_feasible_service_graph(path)
```

- [ ] **Step 4: Run tamper RED/commit; complete strict loader**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'loader_rejects'
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): lock A2 strict graph loader"
```

Strict loader must reject missing required keys, wrong schema/status, non-finite values, unknown enums, duplicate IDs, unknown references, malformed poses/geometry, `coverage_segment_id != segment_id`, reward mismatch beyond `1e-9`, connector start/goal mismatch beyond `1e-9`, connector side/headland mismatch, component mismatch, and diagnostics mismatch. Recompute components/diagnostics after parsing and compare serialized claims to recomputed truth.

- [ ] **Step 5: Verify focused GREEN; export API; register ament test**

Add all public A2 constants/dataclasses/functions to `agt_offline_assets/__init__.py` and `__all__`; do not export private helpers.

Add to CMake beside the A1 test:

```cmake
ament_add_pytest_test(test_vehicle_feasible_service_graph test/test_vehicle_feasible_service_graph.py)
```

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

- [ ] **Step 6: Commit GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12g): serialize A2 static service graph"
```

---

### Task 6: Diagnostic-only A2 acceptance harness

**Files:** create `tools/v25_12g_a2_acceptance.py` and `tests/test_v25_12g_a2_contract.py`.

**Interfaces:** default inputs `vehicle_feasible_segments.yaml`, `turn_zones.yaml`; stdout JSON by default; optional output `vehicle_feasible_service_graph.yaml` only with `--write-graph`.

Constants:

```python
REPORT_SCHEMA = "agt_v25_12g_a2_acceptance_report/v1"
VALIDATION_SCOPE = "A2_STATIC_SERVICE_TOPOLOGY_DIAGNOSTIC_NOT_ROUTE_READY"
```

- [ ] **Step 1: Write CLI/preflight RED test**

```python
from importlib import util
import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "v25_12g_a2_acceptance.py"


def _load_tool():
    assert TOOL_PATH.is_file(), f"missing A2 acceptance harness: {TOOL_PATH}"
    spec = util.spec_from_file_location("v25_12g_a2_acceptance", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a2_acceptance_harness_freezes_contract_and_defaults():
    tool = _load_tool()
    assert tool.REPORT_SCHEMA == "agt_v25_12g_a2_acceptance_report/v1"
    assert tool.VALIDATION_SCOPE == "A2_STATIC_SERVICE_TOPOLOGY_DIAGNOSTIC_NOT_ROUTE_READY"
    args = tool.build_parser().parse_args(["--run-dir", "/tmp/agt-run"])
    assert args.segments == "vehicle_feasible_segments.yaml"
    assert args.turn_zones == "turn_zones.yaml"
    assert args.write_graph is False
    assert args.overwrite_graph is False
    assert args.pretty is False
```

- [ ] **Step 2: Run RED/commit; implement parser/preflight**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py::test_a2_acceptance_harness_freezes_contract_and_defaults
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): specify A2 acceptance harness contract"
```

Parser:

```python
def build_parser():
    parser = argparse.ArgumentParser(description="Inspect V25-12G-A2 static service topology from frozen A1 assets")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--segments", default="vehicle_feasible_segments.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--write-graph", action="store_true")
    parser.add_argument("--overwrite-graph", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser
```

Preflight requires exactly the two input files and reserves `run_dir / "vehicle_feasible_service_graph.yaml"`. Refuse an existing output before parsing inputs when write is requested without overwrite.

- [ ] **Step 3: Add required-input/overwrite RED tests**

```python
@pytest.mark.parametrize("missing_name", ("vehicle_feasible_segments.yaml", "turn_zones.yaml"))
def test_a2_harness_requires_both_frozen_inputs(tmp_path, missing_name):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    for name in ("vehicle_feasible_segments.yaml", "turn_zones.yaml"):
        if name != missing_name:
            (run_dir / name).write_text("placeholder\n", encoding="utf-8")
    with pytest.raises(FileNotFoundError, match=missing_name):
        tool.main(["--run-dir", str(run_dir)])


def test_a2_harness_refuses_existing_graph_without_overwrite(tmp_path):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "vehicle_feasible_segments.yaml").write_text("placeholder\n", encoding="utf-8")
    (run_dir / "turn_zones.yaml").write_text("placeholder\n", encoding="utf-8")
    output = run_dir / "vehicle_feasible_service_graph.yaml"
    output.write_text("sentinel\n", encoding="utf-8")
    with pytest.raises(FileExistsError, match="vehicle_feasible_service_graph.yaml"):
        tool.main(["--run-dir", str(run_dir), "--write-graph"])
    assert output.read_text(encoding="utf-8") == "sentinel\n"
```

- [ ] **Step 4: Run/commit RED; finish preflight**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py -k 'requires_both or refuses_existing'
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): lock A2 acceptance narrow-write preflight"
```

- [ ] **Step 5: Add exact serializable fixture helpers to root contract test**

Repeat these definitions in `tests/test_v25_12g_a2_contract.py`; do not import private helpers from another test file:

```python
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet, write_turn_zones
from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    AisleFeasibleSegmentResult,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
    write_vehicle_feasible_segment_plan,
)

_A1_SOURCE = {
    "vehicle_safe_lane_configuration": {
        "sample_spacing_m": 0.10,
        "lateral_search_step_m": 0.05,
        "maximum_lateral_shift_m": 0.50,
        "maximum_lateral_step_m": 0.15,
        "preview_footprint_padding_m": 0.05,
        "minimum_lane_coverage_fraction": 0.70,
        "maximum_endpoint_retreat_m": 2.00,
        "minimum_contiguous_span_m": 1.00,
    }
}


def _serializable_plan():
    segment = VehicleFeasibleSegment(
        segment_id="aisle_001.segment_001",
        aisle_id="aisle_001",
        ordinal_in_aisle=1,
        start_distance_m=0.0,
        end_distance_m=4.0,
        length_m=4.0,
        coverage_fraction_of_aisle=1.0,
        low_endpoint_type=LOW_U_HEADLAND,
        high_endpoint_type=INTERIOR_BLOCKED_END,
        centerline_xyz=((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)),
        lateral_offsets_m=(0.0, 0.0),
        maximum_used_lateral_shift_m=0.0,
        low_endpoint_pose=(0.0, 0.0, 0.0, 0.0),
        high_endpoint_pose=(4.0, 0.0, 0.0, 0.0),
    )
    aisle = AisleFeasibleSegmentResult(
        aisle_id="aisle_001",
        structural_length_m=4.0,
        active_segments=(segment,),
        rejected_fragments=(),
        raw_feasible_fragment_count=1,
        allowed_lateral_shift_m=0.5,
        site_boundary_rejected_pose_count=0,
        site_boundary_limited_sample_count=0,
        grid_rejected_pose_count=0,
        reason="test fixture",
    )
    return VehicleFeasibleSegmentPlan(
        frame_id="map",
        platform_id="mk_mini",
        platform_profile_sha256="fixture-sha256",
        row_direction_xy=(1.0, 0.0),
        aisles=(aisle,),
        source=dict(_A1_SOURCE),
    )


def _serializable_zones():
    zone = TurnZone(
        zone_id="turn_low_u",
        side="LOW_U",
        polygon_xy=((-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)),
        supported_aisle_ids=("aisle_001",),
        endpoint_count=1,
        free_fraction=1.0,
    )
    return TurnZoneSet(frame_id="map", row_direction_xy=(1.0, 0.0), zones=(zone,), source={})


def _write_valid_inputs(run_dir):
    write_vehicle_feasible_segment_plan(_serializable_plan(), run_dir / "vehicle_feasible_segments.yaml")
    write_turn_zones(_serializable_zones(), run_dir / "turn_zones.yaml")
```

- [ ] **Step 6: Add report RED test**

```python
def _contains_forbidden_key(value):
    forbidden = {"route_ready", "executable", "reachable_from_start", "optimal"}
    if isinstance(value, dict):
        return any(key in forbidden or _contains_forbidden_key(item) for key, item in value.items())
    if isinstance(value, list):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def test_a2_harness_reports_static_topology_without_route_claims(tmp_path, capsys):
    tool = _load_tool()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_valid_inputs(run_dir)
    assert tool.main(["--run-dir", str(run_dir)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["schema"] == tool.REPORT_SCHEMA
    assert report["validation_scope"] == tool.VALIDATION_SCOPE
    assert report["summary"]["coverage_preserved"] is True
    assert report["summary"]["all_a1_segment_ids_preserved"] is True
    assert report["summary"]["interior_cross_aisle_connector_count"] == 0
    assert report["summary"]["cross_side_connector_count"] == 0
    assert _contains_forbidden_key(report) is False
```

- [ ] **Step 7: Run RED/commit; implement load/report/write path**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py -k 'reports_static_topology'
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): specify A2 acceptance report evidence"
```

Use `load_vehicle_feasible_segment_plan` and `load_turn_zones`, derive A2 with source `{acceptance_stage: v25_12g_a2, vehicle_feasible_segments_asset: <arg>, turn_zones_asset: <arg>}`. Report top-level keys exactly: `schema`, `validation_scope`, `run_dir`, `frame_id`, `platform_id`, `segments_asset`, `turn_zones_asset`, `resources`, `components`, `summary`.

Each resource record: `segment_id`, `aisle_id`, `coverage_length_m`, `low_endpoint_type`, `high_endpoint_type`, `service_state_ids`, `dead_end_candidate_count`, `candidate_component_id`.

Each component record: `component_id`, `classification`, `service_state_count`, `unique_segment_count`, `connector_candidate_count`, `unique_coverage_length_m`, `distinct_aisle_count`.

Summary mirrors graph diagnostics and adds `a1_active_segment_count`, `a1_active_segment_length_m`, `coverage_preserved`, `all_a1_segment_ids_preserved`, `interior_cross_aisle_connector_count`, `cross_side_connector_count`. Recompute the two illegal-edge counters independently and raise `ValueError` if nonzero. Default mode prints JSON only; call writer only with `--write-graph`.

- [ ] **Step 8: Verify GREEN and commit harness**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git add tools/v25_12g_a2_acceptance.py tests/test_v25_12g_a2_contract.py
git commit -m "feat(v25-12g): add A2 static topology acceptance harness"
```

---

### Task 7: Frozen real-greenhouse A2 checkpoint

**Files:** create evidence docs only after operator output exists; runtime output is the A2 sibling YAML.

- [ ] **Step 1: Rebuild and verify automated gates**

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
python3 -m pytest -q tests/test_v25_12g_a2_contract.py
colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

- [ ] **Step 2: Record forbidden-input hashes**

```bash
RUN=/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run
sha256sum \
  "$RUN/vehicle_feasible_segments.yaml" \
  "$RUN/turn_zones.yaml" \
  "$RUN/navigation_map.yaml" \
  "$RUN/navigation_map.pgm" \
  "$RUN/derivation.yaml" \
  "$RUN/aisle_graph.yaml" \
  "$RUN/site_boundary.yaml"
```

- [ ] **Step 3: Run A2 acceptance read-only**

```bash
python3 tools/v25_12g_a2_acceptance.py --run-dir "$RUN" --pretty
```

Required A1-preservation checks from accepted evidence:

```text
resource_count = 14
ordinary_service_state_count = 28
unique_coverage_length_m = 97.35667393520538 within 1e-9
coverage_preserved = true
all_a1_segment_ids_preserved = true
interior_cross_aisle_connector_count = 0
cross_side_connector_count = 0
```

Do not hard-code expected total connector count/component count/isolated count into production code; record them as real observations. Verify exactly-one-headland resources each receive one dead-end candidate, rather than embedding a greenhouse-specific dead-end count constant.

- [ ] **Step 4: Review topology semantics before write**

Verify all A1 segments remain present; through resources have two ordinary states only; exactly-one-headland resources have one extra dead-end state; double-interior resources remain present; no interior endpoint is a connector port; LOW/HIGH domains remain separate; connector candidates remain `REQUIRES_A3_CONNECTOR_VALIDATION`; dead-end states remain `REQUIRES_A3_REVERSE_SERVICE_VALIDATION`.

- [ ] **Step 5: Narrow-write without overwrite**

```bash
python3 tools/v25_12g_a2_acceptance.py --run-dir "$RUN" --write-graph --pretty
```

If output exists, stop and inspect before considering overwrite.

- [ ] **Step 6: Repeat Step 2 hashes and require byte-for-byte equality for all forbidden inputs**

Any changed forbidden input is an A2 failure.

- [ ] **Step 7: Strict-load and byte-stable rewrite**

```bash
python3 - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
from agt_offline_assets.vehicle_feasible_service_graph import load_vehicle_feasible_service_graph, write_vehicle_feasible_service_graph

src = Path("/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_service_graph.yaml")
graph = load_vehicle_feasible_service_graph(src)
with TemporaryDirectory() as tmp:
    dst = Path(tmp) / src.name
    write_vehicle_feasible_service_graph(graph, dst)
    assert src.read_bytes() == dst.read_bytes()
print("A2 strict-load and byte-stable rewrite: PASS")
PY
```

- [ ] **Step 8: Record evidence docs and freeze A2 -> A3 gate**

`V25_12G_A2_REAL_DATA_2026-08-16.md` records run path, input assets, pre/post hashes, resource/state/dead-end counts, preserved coverage, LOW/HIGH connector counts, candidate/isolated component counts, per-resource states, per-component coverage, zero illegal edges, and strict-load/byte-stability evidence.

Strongest permitted classification:

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED
DIAGNOSTIC ONLY
NOT ROUTE-READY
```

If warranted by observations use `A2 STATIC SERVICE TOPOLOGY SUPPORTED WITH CONNECTIVITY / MAP CAVEATS` instead.

`V25_12G_A2_CURRENT_STATE.md` names only `V25-12G-A3 Connector + Reverse Motion Feasibility Design` as the next stage; A3 independently proves executability.

- [ ] **Step 9: Commit evidence docs only after operator evidence exists**

```bash
git add docs/v2.5/V25_12G_A2_REAL_DATA_2026-08-16.md docs/v2.5/V25_12G_A2_CURRENT_STATE.md
git commit -m "docs(v25-12g): record A2 real-data topology checkpoint"
```

Do not add runtime map assets to git unless repository policy explicitly requires it.

---

## Final Verification Gate

A2 is complete only after operator evidence proves:

```text
1. Focused A2 unit/IO tests PASS
2. A2 acceptance harness contract tests PASS
3. agt_offline_assets package-level colcon tests have no new A2 failure
4. Real greenhouse A1 -> A2 preserves all 14 physical resources
5. Unique coverage remains 97.35667393520538 m within 1e-9
6. No INTERIOR cross-aisle candidate exists
7. No LOW_U <-> HIGH_U direct candidate exists
8. Written A2 YAML strict-loads and rewrites byte-identically
9. Forbidden runtime input SHA256 values remain unchanged
10. Real-data docs classify A2 as diagnostic static topology only and NOT ROUTE-READY
```

Do not pull A3 behavior into A2. A2 is done when its static topology contract is correct, deterministic, auditable, and honestly bounded.
