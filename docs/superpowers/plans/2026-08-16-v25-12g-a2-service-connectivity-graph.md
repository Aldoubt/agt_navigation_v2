# V25-12G-A2 Service / Connectivity Graph Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the frozen A1 vehicle-feasible physical segments and Turn Zone evidence into one deterministic, static, directional A2 service graph containing coverage resources, directional service states, one-headland dead-end candidates, directed headland connector candidates, and diagnostic candidate topology components without claiming motion feasibility or route readiness.

**Architecture:** Add one focused offline-assets module that treats A1 `segment_id` as the unique coverage resource, expands directional service states, emits only same-side Turn-Zone-supported `exit -> entry` connector candidates, groups states into diagnostic candidate topology components, and serializes the result as `vehicle_feasible_service_graph.yaml`. A standalone A2 acceptance harness consumes the frozen A1 and Turn Zone sibling assets, reports topology evidence, and may write only the A2 sibling artifact. A3 motion feasibility, START anchoring, route optimization, and Workbench topology visualization remain outside this plan.

**Tech Stack:** Python 3.10, standard-library `dataclasses` / `math` / `pathlib`, PyYAML, ROS 2 Humble, `ament_cmake_pytest`, existing `VehicleFeasibleSegmentPlan`, `TurnZoneSet`, and agricultural-route YAML loaders.

## Global Constraints

- Work only on `feat/v25-12g-maximum-feasible-coverage` in the existing repository checkout; do not create an additional worktree for this execution.
- Scope is V25-12G-A2 Core only: service resources, directional service states, dead-end topology candidates, static directed headland connector candidates, candidate topology components, deterministic YAML IO, diagnostics, and real-data A2 acceptance.
- Do not implement A3 Dubins search, reverse connector search, connector footprint validation, Site Boundary connector collision gates, Navigation Grid connector collision gates, minimum-turn-radius connector validation, executable components, START anchoring, A4 optimization, MILP, RL, or manual route override.
- A2 is a reusable map-level static topology asset. `START_POSE` must not enter the canonical A2 graph.
- A1 `VehicleFeasibleSegmentPlan` is the physical coverage truth. A2 must not re-derive, shorten, merge, discard, or spatially move A1 active segments.
- Coverage identity is exactly `segment_id`; multiple service states for one physical segment never create multiple coverage resources.
- A2 v1 service types are exactly `SERVICE_LOW_TO_HIGH`, `SERVICE_HIGH_TO_LOW`, and `DEAD_END_FORWARD_IN_REVERSE_OUT`.
- `DEAD_END_REVERSE_IN_FORWARD_OUT` is out of scope for A2 v1.
- A2 connector candidates are directed and strictly connect `from_service_state.exit -> to_service_state.entry`.
- Cross-aisle connector candidates may be emitted only when both endpoints are real same-side headland endpoints supported by one shared Turn Zone whose `allow_turn` flag is true.
- LOW_U and HIGH_U domains must never be directly cross-connected in A2.
- `INTERIOR_BLOCKED_END` must never be used as a cross-aisle connector port.
- A2 candidate topology components are diagnostic only. They must never claim `EXECUTABLE`, `A3_VALIDATED`, `REACHABLE_FROM_START`, or `ROUTE_READY`.
- Component grouping may use same-resource membership as a diagnostic relation, but A2 must never serialize a fake motion edge between service states of the same resource.
- Reverse distance on `DEAD_END_FORWARD_IN_REVERSE_OUT` is accounting expectation only; A3 must validate the actual reverse service motion.
- A2 may write only `vehicle_feasible_service_graph.yaml`; it must never modify `vehicle_feasible_segments.yaml`, `turn_zones.yaml`, `navigation_map.yaml`, `navigation_map.pgm`, `derivation.yaml`, `aisle_graph.yaml`, or `site_boundary.yaml`.
- A2 Workbench visualization is explicitly deferred to a separate follow-up plan. Do not draw unvalidated connector geometry in this plan.
- Input contract violations fail closed. Do not skip malformed records and return a partial graph.
- Deterministic output is required: identical semantic inputs must serialize to byte-identical YAML.
- Do not use `git reset --hard` or `git clean` and do not touch unrelated local changes, especially `tools/rosbag_sensor_trimmer` or `tools/map_tools/render_pcd_top_views.py`.
- Follow strict TDD. No production implementation step begins until the operator has run the preceding test and shown the expected RED failure.
- Do not claim a test, package verification, or real-data acceptance passed until operator-machine output proves it.

---

## File Structure

```text
src/agt_offline_assets/agt_offline_assets/
  vehicle_feasible_service_graph.py     # new A2 domain model, derivation, components, diagnostics, YAML IO
  __init__.py                           # export frozen A2 public API

src/agt_offline_assets/test/
  test_vehicle_feasible_service_graph.py # focused A2 unit + IO contract tests

src/agt_offline_assets/CMakeLists.txt   # register focused A2 pytest with ament

tools/
  v25_12g_a2_acceptance.py              # diagnostic-only real-data harness

tests/
  test_v25_12g_a2_contract.py           # harness CLI / narrow-write / report contract

docs/v2.5/
  V25_12G_A2_CURRENT_STATE.md           # state gate after automated verification and real-data observation
  V25_12G_A2_REAL_DATA_2026-08-16.md    # evidence recorded only after operator output exists
```

Do not modify `vehicle_feasible_segment.py`, `turn_zones.py`, `agricultural_coverage_ordering.py`, or `forward_connector.py` unless a verified defect in an existing frozen contract makes A2 impossible. If such a defect appears, stop and debug it separately instead of silently changing upstream semantics.

---

## Frozen A2 Public Contract

The new module exports these constants:

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

The public dataclasses are:

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

The public functions are exactly:

```python
def derive_vehicle_feasible_service_graph(
    segment_plan: VehicleFeasibleSegmentPlan,
    turn_zones: TurnZoneSet,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleServiceGraph: ...


def vehicle_feasible_service_graph_to_dict(
    graph: VehicleFeasibleServiceGraph,
) -> dict[str, Any]: ...


def write_vehicle_feasible_service_graph(
    graph: VehicleFeasibleServiceGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path: ...


def load_vehicle_feasible_service_graph(
    path: str | Path,
) -> VehicleFeasibleServiceGraph: ...
```

Internal helpers may be named as this plan specifies. Do not create an additional public endpoint-node graph API in A2 v1.

---

### Task 1: Build physical resources and ordinary directional service states

**Files:**
- Create: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py`
- Create: `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`

**Interfaces:**
- Consumes: `VehicleFeasibleSegmentPlan`, `VehicleFeasibleSegment`, `LOW_U_HEADLAND`, `HIGH_U_HEADLAND`, `INTERIOR_BLOCKED_END` from `vehicle_feasible_segment.py`.
- Produces internally: `_build_service_resources_and_states(segment_plan) -> tuple[tuple[ServiceResource, ...], tuple[ServiceState, ...]]`.
- Produces stable types: `ServiceResource`, `ServiceState` and the service/status constants in the Frozen A2 Public Contract.

- [ ] **Step 1: Write the fixture helpers and the first failing ordinary-state test**

Create the test file with these helpers first:

```python
import math

import pytest

from agt_offline_assets.vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    AisleFeasibleSegmentResult,
    VehicleFeasibleSegment,
    VehicleFeasibleSegmentPlan,
)
from agt_offline_assets.vehicle_feasible_service_graph import (
    EXTERNAL_REACHABILITY_UNPROVEN,
    FORWARD,
    HEADLAND_TOPOLOGY_CANDIDATE,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    TOPOLOGY_SERVICE_CANDIDATE,
    _build_service_resources_and_states,
)


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
        items = tuple(sorted(grouped[aisle_id], key=lambda s: s.ordinal_in_aisle))
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
        source={},
    )


def test_resource_expands_into_two_forward_directional_states():
    resource, states = _build_service_resources_and_states(_plan(_segment()))

    assert len(resource) == 1
    assert resource[0].segment_id == "aisle_001.segment_001"
    assert resource[0].coverage_length_m == pytest.approx(4.0)
    assert len(states) == 2

    low_to_high = next(s for s in states if s.service_type == SERVICE_LOW_TO_HIGH)
    high_to_low = next(s for s in states if s.service_type == SERVICE_HIGH_TO_LOW)

    assert low_to_high.service_state_id == "aisle_001.segment_001.service_low_to_high"
    assert low_to_high.entry_endpoint_type == LOW_U_HEADLAND
    assert low_to_high.exit_endpoint_type == HIGH_U_HEADLAND
    assert low_to_high.entry_pose == (0.0, 0.0, 0.0, 0.0)
    assert low_to_high.exit_pose == (4.0, 0.0, 0.0, 0.0)
    assert low_to_high.service_motion_direction == FORWARD
    assert low_to_high.forward_service_distance_m == pytest.approx(4.0)
    assert low_to_high.reverse_service_distance_m == pytest.approx(0.0)
    assert low_to_high.coverage_segment_id == resource[0].segment_id
    assert low_to_high.coverage_reward_length_m == pytest.approx(4.0)
    assert low_to_high.validation_status == TOPOLOGY_SERVICE_CANDIDATE
    assert low_to_high.external_reachability_status == HEADLAND_TOPOLOGY_CANDIDATE

    assert high_to_low.service_state_id == "aisle_001.segment_001.service_high_to_low"
    assert high_to_low.entry_endpoint_type == HIGH_U_HEADLAND
    assert high_to_low.exit_endpoint_type == LOW_U_HEADLAND
    assert high_to_low.entry_pose[:3] == (4.0, 0.0, 0.0)
    assert high_to_low.exit_pose[:3] == (0.0, 0.0, 0.0)
    assert high_to_low.entry_pose[3] == pytest.approx(-math.pi)
    assert high_to_low.exit_pose[3] == pytest.approx(-math.pi)
    assert high_to_low.service_motion_direction == FORWARD
```

- [ ] **Step 2: Run the focused test and verify RED**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_resource_expands_into_two_forward_directional_states
```

Expected: FAIL during import because `vehicle_feasible_service_graph.py` and the A2 API do not exist.

- [ ] **Step 3: Commit the verified RED test only**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 directional service resources"
```

- [ ] **Step 4: Implement only resource copy-through, angle reversal, and two ordinary states**

Create `vehicle_feasible_service_graph.py` with:

```python
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from .vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    VehicleFeasibleSegmentPlan,
)

SERVICE_LOW_TO_HIGH = "SERVICE_LOW_TO_HIGH"
SERVICE_HIGH_TO_LOW = "SERVICE_HIGH_TO_LOW"
DEAD_END_FORWARD_IN_REVERSE_OUT = "DEAD_END_FORWARD_IN_REVERSE_OUT"
FORWARD = "FORWARD"
TOPOLOGY_SERVICE_CANDIDATE = "TOPOLOGY_SERVICE_CANDIDATE"
REQUIRES_A3_REVERSE_SERVICE_VALIDATION = "REQUIRES_A3_REVERSE_SERVICE_VALIDATION"
HEADLAND_TOPOLOGY_CANDIDATE = "HEADLAND_TOPOLOGY_CANDIDATE"
EXTERNAL_REACHABILITY_UNPROVEN = "EXTERNAL_REACHABILITY_UNPROVEN"

_HEADLAND_TYPES = {LOW_U_HEADLAND, HIGH_U_HEADLAND}


def _normalize_angle(yaw: float) -> float:
    value = (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi
    return -math.pi if value == math.pi else value


def _reverse_heading(pose):
    x, y, z, yaw = pose
    return (float(x), float(y), float(z), _normalize_angle(float(yaw) + math.pi))
```

Add the exact `ServiceResource` and `ServiceState` dataclasses from the Frozen A2 Public Contract.

Implement `_build_service_resources_and_states()` so it:

```text
1. flattens all A1 active segments;
2. sorts by (aisle_id, ordinal_in_aisle, segment_id);
3. creates exactly one ServiceResource per active segment;
4. creates SERVICE_LOW_TO_HIGH from A1 low pose to high pose;
5. creates SERVICE_HIGH_TO_LOW from reverse_heading(high pose) to reverse_heading(low pose);
6. sets external_reachability_status from the actual entry endpoint only;
7. does not create dead-end states yet.
```

At this task boundary, state order is deterministic:

```text
segment_id lexical, then SERVICE_LOW_TO_HIGH, SERVICE_HIGH_TO_LOW
```

- [ ] **Step 5: Run the focused test and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_resource_expands_into_two_forward_directional_states
```

Expected: `1 passed`.

- [ ] **Step 6: Run the existing A1 focused regression**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_segment.py
```

Expected: existing A1 tests remain green because A2 does not modify A1 code.

- [ ] **Step 7: Commit Task 1 GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): expand A1 segments into directional service states"
```

---

### Task 2: Add one-headland dead-end candidates and fail-closed A2 input validation

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`

**Interfaces:**
- Consumes: `VehicleFeasibleSegmentPlan`, `TurnZoneSet`.
- Produces internally: `_validate_a2_inputs(segment_plan, turn_zones) -> None`.
- Extends: `_build_service_resources_and_states()` to add `DEAD_END_FORWARD_IN_REVERSE_OUT` only for exactly-one-headland resources.

- [ ] **Step 1: Add failing tests for LOW headland, HIGH headland, and double-interior behavior**

Append:

```python
from agt_offline_assets.turn_zones import TurnZone, TurnZoneSet
from agt_offline_assets.vehicle_feasible_service_graph import (
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    _validate_a2_inputs,
)


def _zones(*zones, frame_id="map", row_direction=(1.0, 0.0)):
    return TurnZoneSet(
        frame_id=frame_id,
        row_direction_xy=row_direction,
        zones=tuple(zones),
        source={},
    )


def _zone(zone_id="turn_low_u", side="LOW_U", aisle_ids=("aisle_001",), allow_turn=True):
    return TurnZone(
        zone_id=zone_id,
        side=side,
        polygon_xy=((-1.0, -2.0), (1.0, -2.0), (1.0, 2.0), (-1.0, 2.0)),
        supported_aisle_ids=tuple(aisle_ids),
        endpoint_count=len(tuple(aisle_ids)),
        free_fraction=1.0,
        allow_turn=allow_turn,
    )


def test_exactly_one_low_headland_adds_forward_in_reverse_out_candidate():
    segment = _segment(high_type=INTERIOR_BLOCKED_END)
    _resources, states = _build_service_resources_and_states(_plan(segment))
    dead_end = [s for s in states if s.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT]
    assert len(dead_end) == 1
    state = dead_end[0]
    assert state.entry_endpoint_type == LOW_U_HEADLAND
    assert state.exit_endpoint_type == LOW_U_HEADLAND
    assert state.entry_pose == segment.low_endpoint_pose
    assert state.exit_pose == segment.low_endpoint_pose
    assert state.forward_service_distance_m == pytest.approx(segment.length_m)
    assert state.reverse_service_distance_m == pytest.approx(segment.length_m)
    assert state.coverage_segment_id == segment.segment_id
    assert state.validation_status == REQUIRES_A3_REVERSE_SERVICE_VALIDATION
    assert state.external_reachability_status == HEADLAND_TOPOLOGY_CANDIDATE


def test_exactly_one_high_headland_reverses_dead_end_entry_heading():
    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=HIGH_U_HEADLAND,
        yaw=0.25,
    )
    _resources, states = _build_service_resources_and_states(_plan(segment))
    state = next(s for s in states if s.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT)
    assert state.entry_endpoint_type == HIGH_U_HEADLAND
    assert state.exit_endpoint_type == HIGH_U_HEADLAND
    assert state.entry_pose[:3] == segment.high_endpoint_pose[:3]
    assert state.entry_pose[3] == pytest.approx(_normalize_angle(0.25 + math.pi))
    assert state.exit_pose == state.entry_pose


def test_double_interior_is_preserved_without_dead_end_candidate():
    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    resources, states = _build_service_resources_and_states(_plan(segment))
    assert len(resources) == 1
    assert len(states) == 2
    assert all(s.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT for s in states)
    assert all(s.external_reachability_status == EXTERNAL_REACHABILITY_UNPROVEN for s in states)
```

Import `_normalize_angle` in the test module for the explicit high-headland assertion.

- [ ] **Step 2: Run the three tests and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_exactly_one_low_headland_adds_forward_in_reverse_out_candidate \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_exactly_one_high_headland_reverses_dead_end_entry_heading \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_double_interior_is_preserved_without_dead_end_candidate
```

Expected: the exactly-one-headland tests FAIL because dead-end state generation does not exist yet.

- [ ] **Step 3: Commit the verified dead-end RED tests**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 dead-end service candidates"
```

- [ ] **Step 4: Implement the minimal dead-end expansion**

For each resource compute:

```python
low_is_headland = resource.low_endpoint_type in _HEADLAND_TYPES
high_is_headland = resource.high_endpoint_type in _HEADLAND_TYPES
```

Generate one dead-end state only when `low_is_headland != high_is_headland`.

LOW headland semantics:

```python
entry_endpoint_type = LOW_U_HEADLAND
exit_endpoint_type = LOW_U_HEADLAND
entry_pose = resource.low_endpoint_pose
exit_pose = resource.low_endpoint_pose
```

HIGH headland semantics:

```python
entry_endpoint_type = HIGH_U_HEADLAND
exit_endpoint_type = HIGH_U_HEADLAND
entry_pose = _reverse_heading(resource.high_endpoint_pose)
exit_pose = _reverse_heading(resource.high_endpoint_pose)
```

The state ID is exactly:

```python
f"{resource.segment_id}.dead_end_forward_in_reverse_out"
```

After expansion, deterministic per-resource service-type order is exactly:

```text
SERVICE_LOW_TO_HIGH
SERVICE_HIGH_TO_LOW
DEAD_END_FORWARD_IN_REVERSE_OUT
```

when the dead-end state exists.

- [ ] **Step 5: Run the dead-end tests and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'dead_end or double_interior'
```

Expected: all selected tests pass.

- [ ] **Step 6: Add failing fail-closed validation tests**

Append:

```python
@pytest.mark.parametrize(
    ("plan_frame", "zone_frame"),
    (("map", "odom"), ("odom", "map")),
)
def test_validate_a2_inputs_rejects_frame_mismatch(plan_frame, zone_frame):
    with pytest.raises(ValueError, match="frame"):
        _validate_a2_inputs(
            _plan(_segment(), frame_id=plan_frame),
            _zones(_zone(), frame_id=zone_frame),
        )


def test_validate_a2_inputs_rejects_opposite_row_direction():
    with pytest.raises(ValueError, match="row_direction"):
        _validate_a2_inputs(
            _plan(_segment(), row_direction=(1.0, 0.0)),
            _zones(_zone(), row_direction=(-1.0, 0.0)),
        )


def test_validate_a2_inputs_rejects_duplicate_zone_ids():
    duplicate = _zone()
    with pytest.raises(ValueError, match="duplicate.*zone"):
        _validate_a2_inputs(
            _plan(_segment()),
            _zones(duplicate, duplicate),
        )
```

Also add one invalid endpoint test by constructing `_segment(low_type="UNKNOWN_ENDPOINT")` and requiring `ValueError` containing `endpoint`.

- [ ] **Step 7: Run the validation tests and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'validate_a2_inputs'
```

Expected: FAIL because `_validate_a2_inputs()` does not exist.

- [ ] **Step 8: Commit the verified validation RED tests**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 fail-closed topology inputs"
```

- [ ] **Step 9: Implement `_validate_a2_inputs()` exactly**

Validation must reject before any graph construction:

```text
segment_plan.schema != agt_vehicle_feasible_segment_plan/v1
empty frame/platform/profile hash
segment_plan.frame_id != turn_zones.frame_id
non-finite or zero row direction
normalized row directions with dot <= 0
normalized component difference > 1e-9
empty / duplicate segment_id
length_m <= 0 or non-finite
unknown endpoint type
non-finite endpoint pose
empty centerline
empty / duplicate service-state identity after expansion
empty / duplicate zone_id
zone.side not in {LOW_U, HIGH_U}
duplicate aisle ID inside one zone.supported_aisle_ids
```

Use the explicit row-direction rule:

```python
a = _normalize_xy(segment_plan.row_direction_xy)
b = _normalize_xy(turn_zones.row_direction_xy)
if a[0] * b[0] + a[1] * b[1] <= 0.0:
    raise ValueError("A2 row_direction_xy orientation mismatch")
if max(abs(a[0] - b[0]), abs(a[1] - b[1])) > 1.0e-9:
    raise ValueError("A2 row_direction_xy mismatch")
```

Do not treat opposite directions as equivalent axes because LOW_U/HIGH_U semantics depend on orientation.

- [ ] **Step 10: Run all Task 1-2 tests and verify GREEN**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
```

- [ ] **Step 11: Commit Task 2 GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): add A2 dead-end states and input gates"
```

---

### Task 3: Generate directed same-side Turn-Zone connector candidates

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`

**Interfaces:**
- Produces: `ConnectorCandidate`.
- Produces internally: `_build_connector_candidates(states: tuple[ServiceState, ...], turn_zones: TurnZoneSet) -> tuple[ConnectorCandidate, ...]`.
- Candidate identity key: `(from_service_state_id, to_service_state_id, turn_zone_id)`.

- [ ] **Step 1: Add the failing directed-pair test**

Use two full-through segments on different aisles and one LOW_U zone supporting both:

```python
def test_same_side_shared_turn_zone_emits_independent_directed_candidates():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001", y=0.0)
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    _resources, states = _build_service_resources_and_states(_plan(s1, s2))
    zone = _zone(aisle_ids=("aisle_001", "aisle_002"))

    candidates = _build_connector_candidates(states, _zones(zone))

    low_candidates = [c for c in candidates if c.side == "LOW_U"]
    assert len(low_candidates) == 2

    ids = {(c.from_segment_id, c.to_segment_id) for c in low_candidates}
    assert ids == {
        ("aisle_001.segment_001", "aisle_002.segment_001"),
        ("aisle_002.segment_001", "aisle_001.segment_001"),
    }
    for candidate in low_candidates:
        from_state = next(s for s in states if s.service_state_id == candidate.from_service_state_id)
        to_state = next(s for s in states if s.service_state_id == candidate.to_service_state_id)
        assert candidate.start_pose == from_state.exit_pose
        assert candidate.goal_pose == to_state.entry_pose
        assert candidate.validation_status == REQUIRES_A3_CONNECTOR_VALIDATION
```

Import `_build_connector_candidates` and `REQUIRES_A3_CONNECTOR_VALIDATION`.

- [ ] **Step 2: Run the directed-pair test and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_side_shared_turn_zone_emits_independent_directed_candidates
```

Expected: FAIL because connector candidate construction does not exist.

- [ ] **Step 3: Commit the verified connector RED test**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 directed headland candidates"
```

- [ ] **Step 4: Implement internal headland ports and directed candidate construction**

Add `ConnectorCandidate` and constants from the Frozen A2 Public Contract.

Build internal outgoing ports from states whose `exit_endpoint_type` is a headland and incoming ports from states whose `entry_endpoint_type` is a headland. Each internal record carries:

```text
service_state_id
segment_id
aisle_id
side
pose
```

For each `TurnZone`, emit a directed candidate if and only if:

```text
from.exit is headland
AND to.entry is headland
AND from.side == to.side == zone.side
AND from.aisle_id in zone.supported_aisle_ids
AND to.aisle_id in zone.supported_aisle_ids
AND zone.allow_turn is true
AND from.segment_id != to.segment_id
```

The serialized candidate ID is exactly:

```python
f"headland.{zone.zone_id}.{from_state.service_state_id}.to.{to_state.service_state_id}"
```

Sort final candidates by:

```python
(zone_id, from_service_state_id, to_service_state_id)
```

Track the exact uniqueness triple and raise `ValueError` instead of overwriting a duplicate.

- [ ] **Step 5: Run the directed-pair test and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_side_shared_turn_zone_emits_independent_directed_candidates
```

- [ ] **Step 6: Add failing negative topology tests**

Append explicit tests that require:

```python
def test_connector_candidates_never_use_interior_ports():
    # one resource has only interior entry/exit; assert no candidate references it
    ...


def test_connector_candidates_do_not_cross_low_and_high_domains():
    # provide both LOW_U and HIGH_U zones; every candidate.side must match
    # both source exit and destination entry endpoint side
    ...


def test_connector_candidates_respect_allow_turn_and_supported_aisles():
    # allow_turn=False -> zero candidates for that zone
    # unsupported destination aisle -> zero candidates for that pair
    ...


def test_connector_candidates_forbid_same_physical_segment():
    # one full-through segment with a zone supporting its aisle -> no self candidate
    ...


def test_same_aisle_different_segment_is_not_hard_forbidden():
    # construct two distinct same-aisle resources that both expose the same
    # headland side; assert the normal Turn-Zone rules may connect them
    ...
```

For the same-aisle fixture, use two distinct segment IDs and ordinals. The test is about absence of an artificial `aisle_id` inequality gate, not about current A1 endpoint-classification frequency.

- [ ] **Step 7: Run the new negative tests and verify at least one RED before broadening implementation**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'connector_candidates'
```

Expected before completing all filters: at least one newly added negative case FAILS. Do not continue unless the operator confirms the RED output.

- [ ] **Step 8: Commit the verified connector-filter RED tests**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): lock A2 connector topology filters"
```

- [ ] **Step 9: Implement only the missing filters, then run GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'connector'
```

Expected: all connector tests pass.

- [ ] **Step 10: Commit Task 3 GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): derive directed A2 headland connector candidates"
```

---

### Task 4: Derive diagnostic candidate topology components and graph diagnostics

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`

**Interfaces:**
- Produces: `CandidateTopologyComponent`, `ServiceGraphDiagnostics`, `VehicleFeasibleServiceGraph`.
- Produces internally: `_build_candidate_components(resources, states, candidates) -> tuple[CandidateTopologyComponent, ...]` and `_build_diagnostics(...) -> ServiceGraphDiagnostics`.
- Produces public: `derive_vehicle_feasible_service_graph(segment_plan, turn_zones, *, source=None)`.

- [ ] **Step 1: Add the failing same-resource component test**

```python
def test_same_resource_states_share_component_without_fake_motion_edge():
    segment = _segment(
        low_type=INTERIOR_BLOCKED_END,
        high_type=INTERIOR_BLOCKED_END,
    )
    resources, states = _build_service_resources_and_states(_plan(segment))
    components = _build_candidate_components(resources, states, ())

    assert len(components) == 1
    component = components[0]
    assert component.classification == CANDIDATE_TOPOLOGY_COMPONENT
    assert component.service_state_ids == tuple(sorted(s.service_state_id for s in states))
    assert component.segment_ids == (segment.segment_id,)
    assert component.connector_candidate_ids == ()
    assert component.total_unique_coverage_length_m == pytest.approx(segment.length_m)
```

Import `_build_candidate_components` and `CANDIDATE_TOPOLOGY_COMPONENT`.

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_resource_states_share_component_without_fake_motion_edge
```

Expected: FAIL because component derivation does not exist.

- [ ] **Step 3: Commit the verified component RED test**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 candidate topology components"
```

- [ ] **Step 4: Implement component grouping as a diagnostic undirected relation only**

Use service states as nodes. Build undirected diagnostic adjacency from exactly two relation classes:

```text
1. all states sharing one segment_id;
2. each directed ConnectorCandidate joins its from/to states for grouping only.
```

Do not append synthetic same-resource `ConnectorCandidate` records.

For each connected component:

```text
service_state_ids = lexical unique tuple
segment_ids = lexical unique tuple
connector_candidate_ids = lexical tuple of candidates whose endpoints are both in component
coverage = sum(ServiceResource.coverage_length_m for unique component segment_ids)
distinct_aisle_count = unique resource aisle IDs
```

Sort components by minimum member `service_state_id`, then assign:

```text
candidate_component_001
candidate_component_002
...
```

- [ ] **Step 5: Run component test and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_same_resource_states_share_component_without_fake_motion_edge
```

- [ ] **Step 6: Add a failing inter-resource component / coverage-dedup test**

```python
def test_connector_candidate_joins_resources_and_component_coverage_deduplicates_segment():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001", length=4.0)
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", length=6.0, y=2.0)
    plan = _plan(s1, s2)
    resources, states = _build_service_resources_and_states(plan)
    candidates = _build_connector_candidates(
        states,
        _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))),
    )
    components = _build_candidate_components(resources, states, candidates)
    assert len(components) == 1
    assert components[0].segment_ids == (
        "aisle_001.segment_001",
        "aisle_002.segment_001",
    )
    assert components[0].total_unique_coverage_length_m == pytest.approx(10.0)
```

- [ ] **Step 7: Run and verify RED if component connector membership is not yet implemented**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_connector_candidate_joins_resources_and_component_coverage_deduplicates_segment
```

Do not accept a false GREEN caused by over-implementation without inspection; if it already passes because Step 4 necessarily implemented the complete rule, record that evidence and proceed without inventing a failing test.

- [ ] **Step 8: Add the failing top-level derive / diagnostics test**

```python
def test_derive_service_graph_reports_static_candidate_diagnostics():
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(
        segment_id="aisle_002.segment_001",
        aisle_id="aisle_002",
        y=2.0,
        high_type=INTERIOR_BLOCKED_END,
    )
    plan = _plan(s1, s2)
    zones = _zones(_zone(aisle_ids=("aisle_001", "aisle_002")))

    graph = derive_vehicle_feasible_service_graph(
        plan,
        zones,
        source={
            "vehicle_feasible_segments_asset": "vehicle_feasible_segments.yaml",
            "turn_zones_asset": "turn_zones.yaml",
        },
    )

    assert graph.schema == VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA
    assert graph.status == TOPOLOGY_CANDIDATE_ONLY
    assert graph.frame_id == "map"
    assert graph.platform_id == "mk_mini"
    assert graph.diagnostics.resource_count == 2
    assert graph.diagnostics.ordinary_service_state_count == 4
    assert graph.diagnostics.dead_end_candidate_count == 1
    assert graph.diagnostics.total_service_state_count == 5
    assert graph.diagnostics.unique_coverage_length_m == pytest.approx(8.0)
    assert graph.source["derivation_kind"] == "V25_12G_A2_STATIC_SERVICE_TOPOLOGY"
    assert graph.source["vehicle_feasible_segment_schema"] == plan.schema
    assert graph.source["turn_zone_schema"] == zones.schema
```

Import the top-level graph API/constants.

- [ ] **Step 9: Run the top-level derive test and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py::test_derive_service_graph_reports_static_candidate_diagnostics
```

Expected: FAIL because the top-level graph/diagnostics API does not exist.

- [ ] **Step 10: Commit the verified graph/diagnostics RED test**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 graph diagnostics contract"
```

- [ ] **Step 11: Implement diagnostics and top-level derive**

`derive_vehicle_feasible_service_graph()` must:

```text
1. call _validate_a2_inputs before constructing any graph data;
2. build resources/states;
3. build directed connector candidates;
4. build candidate topology components;
5. compute diagnostics;
6. verify resource count equals A1 active-segment count;
7. verify abs(unique A2 coverage - A1 active-segment length total) <= 1e-9 m;
8. merge caller source with fixed provenance;
9. return immutable graph.
```

Fixed provenance keys are:

```python
{
    "vehicle_feasible_segment_schema": segment_plan.schema,
    "turn_zone_schema": turn_zones.schema,
    "derivation_kind": "V25_12G_A2_STATIC_SERVICE_TOPOLOGY",
}
```

These fixed keys override conflicting caller values.

Diagnostics definitions are exact:

```text
ordinary_service_state_count = count of LOW_TO_HIGH + HIGH_TO_LOW states
dead_end_candidate_count = count of DEAD_END_FORWARD_IN_REVERSE_OUT states
total_service_state_count = len(service_states)
low_u_connector_candidate_count = candidates whose side == LOW_U
high_u_connector_candidate_count = candidates whose side == HIGH_U
isolated_component_count = components with zero connector_candidate_ids
externally_unproven_state_count = states with EXTERNAL_REACHABILITY_UNPROVEN
unique_coverage_length_m = sum all ServiceResource coverage lengths once
distinct_aisle_count = unique ServiceResource aisle IDs
```

- [ ] **Step 12: Run all focused A2 derivation tests and verify GREEN**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
```

- [ ] **Step 13: Commit Task 4 GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "feat(v25-12g): assemble A2 static candidate topology graph"
```

---

### Task 5: Add deterministic strict YAML IO and register the package API

**Files:**
- Modify: `src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py`
- Modify: `src/agt_offline_assets/agt_offline_assets/__init__.py`
- Modify: `src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py`
- Modify: `src/agt_offline_assets/CMakeLists.txt`

**Interfaces:**
- Produces public: `vehicle_feasible_service_graph_to_dict`, `write_vehicle_feasible_service_graph`, `load_vehicle_feasible_service_graph`.
- Writer default: `overwrite=False`.
- Loader accepts only `agt_vehicle_feasible_service_graph/v1` and reconstructs the frozen A2 dataclasses.

- [ ] **Step 1: Add the failing deterministic round-trip tests**

Append:

```python
def test_service_graph_yaml_round_trip_is_semantically_equal_and_byte_stable(tmp_path):
    s1 = _segment(segment_id="aisle_001.segment_001", aisle_id="aisle_001")
    s2 = _segment(segment_id="aisle_002.segment_001", aisle_id="aisle_002", y=2.0)
    graph = derive_vehicle_feasible_service_graph(
        _plan(s1, s2),
        _zones(_zone(aisle_ids=("aisle_001", "aisle_002"))),
    )

    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    write_vehicle_feasible_service_graph(graph, first)
    loaded = load_vehicle_feasible_service_graph(first)
    write_vehicle_feasible_service_graph(loaded, second)

    assert loaded == graph
    assert first.read_bytes() == second.read_bytes()


def test_service_graph_writer_refuses_overwrite_by_default(tmp_path):
    graph = derive_vehicle_feasible_service_graph(
        _plan(_segment()),
        _zones(_zone()),
    )
    output = tmp_path / "vehicle_feasible_service_graph.yaml"
    write_vehicle_feasible_service_graph(graph, output)
    with pytest.raises(FileExistsError, match="vehicle_feasible_service_graph.yaml"):
        write_vehicle_feasible_service_graph(graph, output)
```

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'yaml_round_trip or writer_refuses'
```

Expected: FAIL because A2 serializer/writer/loader do not exist.

- [ ] **Step 3: Commit the verified IO RED tests**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): specify A2 deterministic graph YAML"
```

- [ ] **Step 4: Implement deterministic serialization and writer**

Top-level YAML field order is exactly:

```text
schema
status
frame_id
platform_id
platform_profile_sha256
row_direction_xy
source
diagnostics
service_resources
service_states
connector_candidates
candidate_components
```

The serializer must sort defensively even if a caller constructs a graph manually:

```text
service_resources: segment_id lexical
service_states: segment_id lexical, then service-type rank LOW_TO_HIGH, HIGH_TO_LOW, DEAD_END
connector_candidates: turn_zone_id, from_service_state_id, to_service_state_id
candidate_components: numeric component order already frozen by component_id
all ID lists inside components: lexical
```

Use:

```python
yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)
```

Do not round domain values before semantic validation. YAML float rendering from identical Python values is sufficient for byte stability.

Writer behavior:

```python
output = Path(path).expanduser().resolve()
if output.exists() and not overwrite:
    raise FileExistsError(...)
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(..., encoding="utf-8")
```

- [ ] **Step 5: Implement strict loader in the same module**

Loader must reject:

```text
missing required top-level keys
unknown schema
unknown top-level status
non-finite / zero row direction
empty platform_id or platform_profile_sha256
missing/duplicate segment_id
missing/duplicate service_state_id
unknown service_type
unknown endpoint type
unknown service_motion_direction
unknown validation status
coverage_segment_id != segment_id
coverage reward != referenced resource length beyond 1e-9
entry/exit pose non-finite or wrong length
missing/duplicate connector_candidate_id
duplicate (from_state, to_state, turn_zone_id) triple
connector references unknown states/resources
connector start_pose != from_state.exit_pose beyond 1e-9
connector goal_pose != to_state.entry_pose beyond 1e-9
connector side inconsistent with actual headland endpoint types
unknown candidate kind / validation status
missing/duplicate component_id
component references unknown states/resources/candidates
component coverage total inconsistent beyond 1e-9
diagnostics inconsistent with recomputed graph beyond 1e-9 for floats
```

After parsing, recompute candidate components and diagnostics from resources/states/candidates. The serialized component and diagnostic payload must match the recomputed result exactly apart from `1e-9` float tolerance. This makes the loader fail closed on hand-edited topology claims.

The loader must not rerun A1 extraction or A3 motion validation.

- [ ] **Step 6: Run the round-trip tests and verify GREEN**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'yaml_round_trip or writer_refuses'
```

- [ ] **Step 7: Add failing tamper tests**

Add explicit payload tamper cases using `yaml.safe_load()` / `yaml.safe_dump()`:

```python
def test_loader_rejects_unknown_schema(tmp_path): ...
def test_loader_rejects_duplicate_service_state_id(tmp_path): ...
def test_loader_rejects_connector_pose_not_matching_state_endpoint(tmp_path): ...
def test_loader_rejects_tampered_component_coverage(tmp_path): ...
def test_loader_rejects_tampered_diagnostics(tmp_path): ...
```

Each test first writes one valid graph with the writer, mutates exactly one field in the loaded dict, writes the tampered YAML, and asserts `ValueError` with a field-specific message.

- [ ] **Step 8: Run tamper tests and verify RED before adding missing strict checks**

```bash
python3 -m pytest -q \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py -k 'loader_rejects'
```

Expected: at least one tamper case fails to be rejected before the strict checks are complete.

- [ ] **Step 9: Commit the verified loader RED tests**

```bash
git add src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
git commit -m "test(v25-12g): lock A2 strict graph loader"
```

- [ ] **Step 10: Complete only the strict checks required by the tests/spec, then verify GREEN**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
```

- [ ] **Step 11: Export the frozen A2 public API from `__init__.py`**

Add one import block for all constants, dataclasses, and four public functions listed in the Frozen A2 Public Contract. Add the same names to `__all__`.

Do not export internal helpers such as `_build_connector_candidates` or `_build_candidate_components`.

- [ ] **Step 12: Register the package test with ament**

Add exactly one line in the existing `if(BUILD_TESTING)` block:

```cmake
ament_add_pytest_test(test_vehicle_feasible_service_graph test/test_vehicle_feasible_service_graph.py)
```

Place it next to the existing `test_vehicle_feasible_segment` registration.

- [ ] **Step 13: Rebuild the package before trusting installed-overlay imports**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_offline_assets --symlink-install
source install/setup.bash
```

Expected: package build succeeds. If a test still imports stale installed code, do not change production semantics; first confirm the overlay was sourced from this build.

- [ ] **Step 14: Run focused and package-level verification**

```bash
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py

colcon test --packages-select agt_offline_assets --event-handlers console_direct+
colcon test-result --test-result-base build/agt_offline_assets --verbose
```

Expected: focused A2 test file passes and the `agt_offline_assets` package test set has no new A2 failure. Do not use repository-global `colcon test-result` as the A2 gate because unrelated historical third-party results are outside this scope.

- [ ] **Step 15: Commit Task 5 GREEN**

```bash
git add \
  src/agt_offline_assets/agt_offline_assets/vehicle_feasible_service_graph.py \
  src/agt_offline_assets/agt_offline_assets/__init__.py \
  src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py \
  src/agt_offline_assets/CMakeLists.txt
git commit -m "feat(v25-12g): serialize A2 static service graph"
```

---

### Task 6: Add the diagnostic-only A2 acceptance harness and narrow-write contract

**Files:**
- Create: `tools/v25_12g_a2_acceptance.py`
- Create: `tests/test_v25_12g_a2_contract.py`

**Interfaces:**
- Consumes sibling assets: `vehicle_feasible_segments.yaml`, `turn_zones.yaml`.
- Produces stdout JSON report only by default.
- Optional output: `vehicle_feasible_service_graph.yaml` only when `--write-graph` is passed.
- Report constants:

```python
REPORT_SCHEMA = "agt_v25_12g_a2_acceptance_report/v1"
VALIDATION_SCOPE = "A2_STATIC_SERVICE_TOPOLOGY_DIAGNOSTIC_NOT_ROUTE_READY"
```

- [ ] **Step 1: Write the failing harness CLI/preflight contract test**

Create `tests/test_v25_12g_a2_contract.py` beginning with:

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


def test_a2_acceptance_harness_freezes_contract_and_parser_defaults():
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

- [ ] **Step 2: Run and verify RED**

```bash
python3 -m pytest -q \
  tests/test_v25_12g_a2_contract.py::test_a2_acceptance_harness_freezes_contract_and_parser_defaults
```

Expected: FAIL because `tools/v25_12g_a2_acceptance.py` does not exist.

- [ ] **Step 3: Commit the verified harness RED test**

```bash
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): specify A2 acceptance harness contract"
```

- [ ] **Step 4: Implement the minimal parser and preflight**

The parser is exactly:

```python
def build_parser():
    parser = argparse.ArgumentParser(
        description="Inspect V25-12G-A2 static service topology from frozen A1 assets",
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--segments", default="vehicle_feasible_segments.yaml")
    parser.add_argument("--turn-zones", default="turn_zones.yaml")
    parser.add_argument("--write-graph", action="store_true")
    parser.add_argument("--overwrite-graph", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    return parser
```

Preflight requires only:

```text
run_dir / segments
run_dir / turn_zones
```

The output is exactly:

```text
run_dir / vehicle_feasible_service_graph.yaml
```

If `--write-graph` is set and output exists without `--overwrite-graph`, raise `FileExistsError` before loading/deriving graph content.

- [ ] **Step 5: Add failing required-input and overwrite tests**

Add parameterized tests requiring both input filenames and one test preserving a sentinel existing A2 output unless `--overwrite-graph` is explicitly passed.

- [ ] **Step 6: Run and verify RED for the new preflight cases**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py -k 'requires or refuses'
```

- [ ] **Step 7: Commit the verified preflight RED tests**

```bash
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): lock A2 acceptance narrow-write preflight"
```

- [ ] **Step 8: Implement frozen input loading and report generation**

Use existing loaders:

```python
from agt_offline_assets.agricultural_route_io import load_turn_zones
from agt_offline_assets.vehicle_feasible_segment import load_vehicle_feasible_segment_plan
from agt_offline_assets.vehicle_feasible_service_graph import (
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    derive_vehicle_feasible_service_graph,
    write_vehicle_feasible_service_graph,
)
```

Call:

```python
graph = derive_vehicle_feasible_service_graph(
    segment_plan,
    turn_zones,
    source={
        "acceptance_stage": "v25_12g_a2",
        "vehicle_feasible_segments_asset": str(args.segments),
        "turn_zones_asset": str(args.turn_zones),
    },
)
```

The deterministic report contains:

```text
schema
validation_scope
run_dir
frame_id
platform_id
segments_asset
turn_zones_asset
resources[]
components[]
summary
```

Each `resources[]` item contains:

```text
segment_id
aisle_id
coverage_length_m
low_endpoint_type
high_endpoint_type
service_state_ids
dead_end_candidate_count
candidate_component_id
```

Each `components[]` item contains:

```text
component_id
classification
service_state_count
unique_segment_count
connector_candidate_count
unique_coverage_length_m
distinct_aisle_count
```

`summary` mirrors the graph diagnostics plus these acceptance comparisons:

```text
a1_active_segment_count
a1_active_segment_length_m
coverage_preserved
all_a1_segment_ids_preserved
interior_cross_aisle_connector_count
cross_side_connector_count
```

`coverage_preserved` is true only when absolute A1-vs-A2 unique coverage difference is `<= 1e-9 m`.

`all_a1_segment_ids_preserved` compares exact segment-ID sets.

The two illegal-connector counters must be computed independently from serialized candidates and must be zero for a successful diagnostic report. The harness must raise `ValueError` rather than merely print a report if either illegal counter is nonzero.

The report must not contain any key named:

```text
route_ready
executable
reachable_from_start
optimal
```

- [ ] **Step 9: Add a synthetic valid-report test before implementing the full report body**

Construct one tiny A1 plan and TurnZoneSet in the test using the package dataclasses/writers, write them into a temporary run directory, call `tool.main([...])`, parse stdout JSON, and assert:

```python
assert report["schema"] == tool.REPORT_SCHEMA
assert report["validation_scope"] == tool.VALIDATION_SCOPE
assert report["summary"]["coverage_preserved"] is True
assert report["summary"]["all_a1_segment_ids_preserved"] is True
assert report["summary"]["interior_cross_aisle_connector_count"] == 0
assert report["summary"]["cross_side_connector_count"] == 0
```

Also recursively assert the forbidden route-readiness keys are absent.

- [ ] **Step 10: Run the synthetic report test and verify RED**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py -k 'reports_static_topology'
```

Expected: FAIL until the full report path exists.

- [ ] **Step 11: Commit the verified report RED test**

```bash
git add tests/test_v25_12g_a2_contract.py
git commit -m "test(v25-12g): specify A2 acceptance report evidence"
```

- [ ] **Step 12: Complete report generation, narrow write, and JSON output**

Default invocation is read-only. Only when `args.write_graph` is true call:

```python
write_vehicle_feasible_service_graph(
    graph,
    graph_output,
    overwrite=bool(args.overwrite_graph),
)
```

JSON output behavior follows A1:

```python
if pretty:
    json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False)
else:
    json.dumps(report, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
```

- [ ] **Step 13: Run the complete harness contract test file and focused package test**

```bash
python3 -m pytest -q tests/test_v25_12g_a2_contract.py
python3 -m pytest -q src/agt_offline_assets/test/test_vehicle_feasible_service_graph.py
```

Expected: both focused suites pass.

- [ ] **Step 14: Commit Task 6 GREEN**

```bash
git add \
  tools/v25_12g_a2_acceptance.py \
  tests/test_v25_12g_a2_contract.py
git commit -m "feat(v25-12g): add A2 static topology acceptance harness"
```

---

### Task 7: Run the frozen real-greenhouse A2 checkpoint and record evidence

**Files:**
- Create after evidence exists: `docs/v2.5/V25_12G_A2_REAL_DATA_2026-08-16.md`
- Create or update after evidence exists: `docs/v2.5/V25_12G_A2_CURRENT_STATE.md`
- Runtime output only: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_service_graph.yaml`

**Interfaces:**
- Consumes accepted A1 artifact: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_segments.yaml`.
- Consumes frozen Turn Zones: `/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/turn_zones.yaml`.
- Produces A2 diagnostic evidence and optionally one new sibling A2 YAML.

- [ ] **Step 1: Rebuild and run all focused automated gates before touching runtime assets**

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

Stop if any A2-focused or `agt_offline_assets` package test fails. Do not reinterpret unrelated repository-wide third-party results as an A2 failure.

- [ ] **Step 2: Record pre-write hashes of every runtime asset A2 is forbidden to mutate**

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

Save the exact output for the acceptance document.

- [ ] **Step 3: Run A2 acceptance read-only first**

```bash
python3 tools/v25_12g_a2_acceptance.py \
  --run-dir "$RUN" \
  --pretty
```

The already accepted A1 evidence makes these checks non-negotiable:

```text
resource_count == 14
ordinary_service_state_count == 28
unique_coverage_length_m == 97.35667393520538 within 1e-9
coverage_preserved == true
all_a1_segment_ids_preserved == true
interior_cross_aisle_connector_count == 0
cross_side_connector_count == 0
```

Do not hard-code an expected total connector candidate count into production code or the harness. Connector count, LOW/HIGH split, component count, and isolated-component count are observations to record from the real Turn Zone input.

The current frozen A1 endpoint classifications imply one dead-end candidate per exactly-one-headland resource; verify that property from the report rather than embedding a greenhouse-specific constant in the implementation.

- [ ] **Step 4: Inspect real topology evidence before writing the sibling asset**

For every resource, verify:

```text
all A1 segment IDs remain present
through segments have only the two ordinary states
exactly-one-headland segments have one additional forward-in/reverse-out state
double-interior segments remain represented
no interior endpoint becomes a connector port
LOW_U and HIGH_U connector domains remain separate
all connector candidates remain REQUIRES_A3_CONNECTOR_VALIDATION
all dead-end candidates remain REQUIRES_A3_REVERSE_SERVICE_VALIDATION
```

Record candidate-component membership, especially isolated double-interior resources. An isolated component is valid A2 evidence, not a reason to invent a connector.

- [ ] **Step 5: Perform the first narrow write without overwrite**

Only after Steps 3-4 are accepted:

```bash
python3 tools/v25_12g_a2_acceptance.py \
  --run-dir "$RUN" \
  --write-graph \
  --pretty
```

Expected: only this new file is created:

```text
$RUN/vehicle_feasible_service_graph.yaml
```

If it already exists, stop and inspect it. Do not use `--overwrite-graph` until the existing A2 asset has been explicitly reviewed.

- [ ] **Step 6: Verify forbidden runtime inputs were not mutated**

Repeat exactly the Step 2 `sha256sum` command and compare all seven hashes byte-for-byte with the pre-write values.

Any change to a forbidden input is a Task 7 failure even if the A2 graph appears reasonable.

- [ ] **Step 7: Strict-load the written artifact and verify byte-stable rewrite in a temporary location**

```bash
python3 - <<'PY'
from pathlib import Path
from tempfile import TemporaryDirectory
from agt_offline_assets.vehicle_feasible_service_graph import (
    load_vehicle_feasible_service_graph,
    write_vehicle_feasible_service_graph,
)

src = Path("/home/yangxuan/agt_navigation_v2/runtime/maps/agt_workbench_run/vehicle_feasible_service_graph.yaml")
graph = load_vehicle_feasible_service_graph(src)
with TemporaryDirectory() as tmp:
    dst = Path(tmp) / src.name
    write_vehicle_feasible_service_graph(graph, dst)
    assert src.read_bytes() == dst.read_bytes()
print("A2 strict-load and byte-stable rewrite: PASS")
PY
```

- [ ] **Step 8: Write the real-data evidence document using only observed values**

Create `docs/v2.5/V25_12G_A2_REAL_DATA_2026-08-16.md` containing:

```text
frozen run path
input A1 / Turn Zone asset names
pre/post forbidden-asset SHA256 evidence
resource/state/dead-end counts
unique coverage length and preservation result
LOW_U / HIGH_U connector candidate counts
candidate component count and isolated component count
per-resource service-state summary
per-component unique coverage summary
confirmation of zero interior cross-aisle edges
confirmation of zero cross-side edges
strict-load / byte-stable rewrite result
```

Do not describe a candidate connector as executable. Do not claim route readiness.

The strongest permitted classification is:

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED
```

or, if observations justify it:

```text
A2 STATIC SERVICE TOPOLOGY SUPPORTED WITH CONNECTIVITY / MAP CAVEATS
```

Always append:

```text
DIAGNOSTIC ONLY
NOT ROUTE-READY
```

- [ ] **Step 9: Freeze current state and the A2 -> A3 gate**

Create/update `docs/v2.5/V25_12G_A2_CURRENT_STATE.md` with automated verification evidence and real-data evidence. The next stage is only:

```text
V25-12G-A3 Connector + Reverse Motion Feasibility Design
```

A3 must independently prove connector and reverse-service executability; A2 candidate connectivity is not inherited as executable truth.

- [ ] **Step 10: Commit Task 7 documentation only after operator evidence exists**

```bash
git add \
  docs/v2.5/V25_12G_A2_REAL_DATA_2026-08-16.md \
  docs/v2.5/V25_12G_A2_CURRENT_STATE.md
git commit -m "docs(v25-12g): record A2 real-data topology checkpoint"
```

Do not add runtime map artifacts to git unless the repository's existing runtime-data policy explicitly requires it.

---

## Final Verification Gate

Before calling A2 complete, the operator must provide evidence for all of the following:

```text
1. Focused A2 unit/IO tests PASS
2. A2 acceptance harness contract tests PASS
3. agt_offline_assets package-level colcon tests PASS with no new A2 failure
4. Real greenhouse A1 -> A2 derivation preserves all 14 physical resources
5. Unique A2 coverage remains 97.35667393520538 m within 1e-9
6. No interior cross-aisle candidate exists
7. No LOW_U <-> HIGH_U direct candidate exists
8. Written A2 YAML strict-loads and rewrites byte-identically
9. Forbidden runtime inputs retain identical SHA256 values
10. Real-data docs classify A2 as diagnostic static topology only, NOT ROUTE-READY
```

Do not merge A3 behavior into this gate to make A2 appear more complete. A2 is complete when its static topology contract is correct, deterministic, observable, and honestly bounded.
