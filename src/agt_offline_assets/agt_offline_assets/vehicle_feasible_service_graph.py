"""Static V25-12G-A2 service-state topology primitives."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping

from .turn_zones import TurnZoneSet
from .vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    VEHICLE_FEASIBLE_SEGMENT_SCHEMA,
    VehicleFeasibleSegmentPlan,
)

VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA = "agt_vehicle_feasible_service_graph/v1"
TOPOLOGY_CANDIDATE_ONLY = "TOPOLOGY_CANDIDATE_ONLY"

SERVICE_LOW_TO_HIGH = "SERVICE_LOW_TO_HIGH"
SERVICE_HIGH_TO_LOW = "SERVICE_HIGH_TO_LOW"
DEAD_END_FORWARD_IN_REVERSE_OUT = "DEAD_END_FORWARD_IN_REVERSE_OUT"
FORWARD = "FORWARD"
TOPOLOGY_SERVICE_CANDIDATE = "TOPOLOGY_SERVICE_CANDIDATE"
REQUIRES_A3_REVERSE_SERVICE_VALIDATION = (
    "REQUIRES_A3_REVERSE_SERVICE_VALIDATION"
)
HEADLAND_TOPOLOGY_CANDIDATE = "HEADLAND_TOPOLOGY_CANDIDATE"
EXTERNAL_REACHABILITY_UNPROVEN = "EXTERNAL_REACHABILITY_UNPROVEN"

HEADLAND_CONNECTOR_CANDIDATE = "HEADLAND_CONNECTOR_CANDIDATE"
REQUIRES_A3_CONNECTOR_VALIDATION = "REQUIRES_A3_CONNECTOR_VALIDATION"
CANDIDATE_TOPOLOGY_COMPONENT = "CANDIDATE_TOPOLOGY_COMPONENT"

_HEADLAND_TYPES = {LOW_U_HEADLAND, HIGH_U_HEADLAND}
_VALID_ENDPOINT_TYPES = {
    LOW_U_HEADLAND,
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
}
_VALID_ZONE_SIDES = {"LOW_U", "HIGH_U"}


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


def _normalize_angle(yaw: float) -> float:
    return (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi


def _reverse_heading(
    pose: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, yaw = pose
    return (float(x), float(y), float(z), _normalize_angle(float(yaw) + math.pi))


def _entry_reachability(endpoint_type: str) -> str:
    if endpoint_type in _HEADLAND_TYPES:
        return HEADLAND_TOPOLOGY_CANDIDATE
    return EXTERNAL_REACHABILITY_UNPROVEN


def _endpoint_side(endpoint_type: str) -> str | None:
    if endpoint_type == LOW_U_HEADLAND:
        return "LOW_U"
    if endpoint_type == HIGH_U_HEADLAND:
        return "HIGH_U"
    return None


def _normalized_xy(
    value: tuple[float, float],
    *,
    field_name: str,
) -> tuple[float, float]:
    x = float(value[0])
    y = float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"A2 {field_name} must be finite")
    norm = math.hypot(x, y)
    if norm <= 1.0e-12:
        raise ValueError(f"A2 {field_name} must be non-zero")
    return (x / norm, y / norm)


def _pose_is_finite(pose: tuple[float, float, float, float]) -> bool:
    return len(pose) == 4 and all(math.isfinite(float(value)) for value in pose)


def _validate_a2_inputs(
    segment_plan: VehicleFeasibleSegmentPlan,
    turn_zones: TurnZoneSet,
) -> None:
    """Fail closed on malformed or semantically inconsistent A2 inputs."""
    if segment_plan.schema != VEHICLE_FEASIBLE_SEGMENT_SCHEMA:
        raise ValueError(
            "A2 vehicle-feasible segment schema mismatch: "
            f"expected {VEHICLE_FEASIBLE_SEGMENT_SCHEMA}, got {segment_plan.schema}"
        )
    if not str(segment_plan.frame_id):
        raise ValueError("A2 segment frame_id must not be empty")
    if not str(segment_plan.platform_id):
        raise ValueError("A2 platform_id must not be empty")
    if not str(segment_plan.platform_profile_sha256):
        raise ValueError("A2 platform_profile_sha256 must not be empty")
    if segment_plan.frame_id != turn_zones.frame_id:
        raise ValueError(
            "A2 frame mismatch: "
            f"segments={segment_plan.frame_id} turn_zones={turn_zones.frame_id}"
        )

    segment_direction = _normalized_xy(
        segment_plan.row_direction_xy,
        field_name="row_direction_xy",
    )
    zone_direction = _normalized_xy(
        turn_zones.row_direction_xy,
        field_name="row_direction_xy",
    )
    dot = (
        segment_direction[0] * zone_direction[0]
        + segment_direction[1] * zone_direction[1]
    )
    if dot <= 0.0:
        raise ValueError("A2 row_direction_xy orientation mismatch")
    if max(
        abs(segment_direction[0] - zone_direction[0]),
        abs(segment_direction[1] - zone_direction[1]),
    ) > 1.0e-9:
        raise ValueError("A2 row_direction_xy mismatch")

    seen_segment_ids: set[str] = set()
    for aisle in segment_plan.aisles:
        for segment in aisle.active_segments:
            segment_id = str(segment.segment_id)
            if not segment_id or segment_id in seen_segment_ids:
                raise ValueError(f"A2 duplicate or empty segment_id: {segment_id}")
            seen_segment_ids.add(segment_id)

            length_m = float(segment.length_m)
            if not math.isfinite(length_m) or length_m <= 0.0:
                raise ValueError(f"A2 segment {segment_id} length_m must be finite and > 0")

            if segment.low_endpoint_type not in _VALID_ENDPOINT_TYPES:
                raise ValueError(
                    f"A2 segment {segment_id} has invalid low endpoint type: "
                    f"{segment.low_endpoint_type}"
                )
            if segment.high_endpoint_type not in _VALID_ENDPOINT_TYPES:
                raise ValueError(
                    f"A2 segment {segment_id} has invalid high endpoint type: "
                    f"{segment.high_endpoint_type}"
                )
            if not _pose_is_finite(segment.low_endpoint_pose):
                raise ValueError(f"A2 segment {segment_id} low endpoint pose is invalid")
            if not _pose_is_finite(segment.high_endpoint_pose):
                raise ValueError(f"A2 segment {segment_id} high endpoint pose is invalid")
            if not segment.centerline_xyz:
                raise ValueError(f"A2 segment {segment_id} centerline must not be empty")
            for point in segment.centerline_xyz:
                if len(point) != 3 or not all(
                    math.isfinite(float(value)) for value in point
                ):
                    raise ValueError(
                        f"A2 segment {segment_id} centerline contains invalid point"
                    )

    seen_zone_ids: set[str] = set()
    for zone in turn_zones.zones:
        zone_id = str(zone.zone_id)
        if not zone_id or zone_id in seen_zone_ids:
            raise ValueError(f"A2 duplicate or empty zone_id: {zone_id}")
        seen_zone_ids.add(zone_id)
        if zone.side not in _VALID_ZONE_SIDES:
            raise ValueError(f"A2 zone {zone_id} has invalid side: {zone.side}")
        aisle_ids = tuple(str(value) for value in zone.supported_aisle_ids)
        if len(aisle_ids) != len(set(aisle_ids)):
            raise ValueError(f"A2 zone {zone_id} has duplicate supported aisle IDs")


def _build_service_resources_and_states(
    segment_plan: VehicleFeasibleSegmentPlan,
) -> tuple[tuple[ServiceResource, ...], tuple[ServiceState, ...]]:
    """Copy A1 segment truth and expand ordinary plus one-headland states."""
    segments = sorted(
        (
            segment
            for aisle in segment_plan.aisles
            for segment in aisle.active_segments
        ),
        key=lambda segment: (
            str(segment.aisle_id),
            int(segment.ordinal_in_aisle),
            str(segment.segment_id),
        ),
    )

    resources: list[ServiceResource] = []
    states: list[ServiceState] = []

    for segment in segments:
        resource = ServiceResource(
            segment_id=str(segment.segment_id),
            aisle_id=str(segment.aisle_id),
            ordinal_in_aisle=int(segment.ordinal_in_aisle),
            coverage_length_m=float(segment.length_m),
            coverage_fraction_of_aisle=float(segment.coverage_fraction_of_aisle),
            low_endpoint_type=str(segment.low_endpoint_type),
            high_endpoint_type=str(segment.high_endpoint_type),
            low_endpoint_pose=tuple(float(value) for value in segment.low_endpoint_pose),
            high_endpoint_pose=tuple(float(value) for value in segment.high_endpoint_pose),
            centerline_xyz=tuple(
                tuple(float(value) for value in point)
                for point in segment.centerline_xyz
            ),
        )
        resources.append(resource)

        states.append(
            ServiceState(
                service_state_id=f"{resource.segment_id}.service_low_to_high",
                segment_id=resource.segment_id,
                aisle_id=resource.aisle_id,
                service_type=SERVICE_LOW_TO_HIGH,
                entry_endpoint_type=resource.low_endpoint_type,
                exit_endpoint_type=resource.high_endpoint_type,
                entry_pose=resource.low_endpoint_pose,
                exit_pose=resource.high_endpoint_pose,
                service_motion_direction=FORWARD,
                forward_service_distance_m=resource.coverage_length_m,
                reverse_service_distance_m=0.0,
                coverage_segment_id=resource.segment_id,
                coverage_reward_length_m=resource.coverage_length_m,
                external_reachability_status=_entry_reachability(
                    resource.low_endpoint_type
                ),
                validation_status=TOPOLOGY_SERVICE_CANDIDATE,
            )
        )

        states.append(
            ServiceState(
                service_state_id=f"{resource.segment_id}.service_high_to_low",
                segment_id=resource.segment_id,
                aisle_id=resource.aisle_id,
                service_type=SERVICE_HIGH_TO_LOW,
                entry_endpoint_type=resource.high_endpoint_type,
                exit_endpoint_type=resource.low_endpoint_type,
                entry_pose=_reverse_heading(resource.high_endpoint_pose),
                exit_pose=_reverse_heading(resource.low_endpoint_pose),
                service_motion_direction=FORWARD,
                forward_service_distance_m=resource.coverage_length_m,
                reverse_service_distance_m=0.0,
                coverage_segment_id=resource.segment_id,
                coverage_reward_length_m=resource.coverage_length_m,
                external_reachability_status=_entry_reachability(
                    resource.high_endpoint_type
                ),
                validation_status=TOPOLOGY_SERVICE_CANDIDATE,
            )
        )

        low_is_headland = resource.low_endpoint_type in _HEADLAND_TYPES
        high_is_headland = resource.high_endpoint_type in _HEADLAND_TYPES
        if low_is_headland != high_is_headland:
            if low_is_headland:
                endpoint_type = resource.low_endpoint_type
                endpoint_pose = resource.low_endpoint_pose
            else:
                endpoint_type = resource.high_endpoint_type
                endpoint_pose = _reverse_heading(resource.high_endpoint_pose)

            states.append(
                ServiceState(
                    service_state_id=(
                        f"{resource.segment_id}.dead_end_forward_in_reverse_out"
                    ),
                    segment_id=resource.segment_id,
                    aisle_id=resource.aisle_id,
                    service_type=DEAD_END_FORWARD_IN_REVERSE_OUT,
                    entry_endpoint_type=endpoint_type,
                    exit_endpoint_type=endpoint_type,
                    entry_pose=endpoint_pose,
                    exit_pose=endpoint_pose,
                    service_motion_direction=FORWARD,
                    forward_service_distance_m=resource.coverage_length_m,
                    reverse_service_distance_m=resource.coverage_length_m,
                    coverage_segment_id=resource.segment_id,
                    coverage_reward_length_m=resource.coverage_length_m,
                    external_reachability_status=HEADLAND_TOPOLOGY_CANDIDATE,
                    validation_status=REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
                )
            )

    return tuple(resources), tuple(states)


def _build_connector_candidates(
    states: tuple[ServiceState, ...],
    turn_zones: TurnZoneSet,
) -> tuple[ConnectorCandidate, ...]:
    """Build directed headland exit-to-entry topology candidates."""
    candidates: list[ConnectorCandidate] = []

    for zone in turn_zones.zones:
        if not zone.allow_turn:
            continue
        supported = set(str(value) for value in zone.supported_aisle_ids)

        for from_state in states:
            from_side = _endpoint_side(from_state.exit_endpoint_type)
            if from_side != zone.side or from_state.aisle_id not in supported:
                continue

            for to_state in states:
                to_side = _endpoint_side(to_state.entry_endpoint_type)
                if to_side != zone.side or to_state.aisle_id not in supported:
                    continue
                if from_state.segment_id == to_state.segment_id:
                    continue

                candidates.append(
                    ConnectorCandidate(
                        connector_candidate_id=(
                            f"headland.{zone.zone_id}."
                            f"{from_state.service_state_id}.to."
                            f"{to_state.service_state_id}"
                        ),
                        from_service_state_id=from_state.service_state_id,
                        to_service_state_id=to_state.service_state_id,
                        from_segment_id=from_state.segment_id,
                        to_segment_id=to_state.segment_id,
                        side=str(zone.side),
                        turn_zone_id=str(zone.zone_id),
                        start_pose=from_state.exit_pose,
                        goal_pose=to_state.entry_pose,
                    )
                )

    return tuple(
        sorted(
            candidates,
            key=lambda candidate: (
                candidate.turn_zone_id,
                candidate.from_service_state_id,
                candidate.to_service_state_id,
            ),
        )
    )


def _build_candidate_components(
    resources: tuple[ServiceResource, ...],
    states: tuple[ServiceState, ...],
    candidates: tuple[ConnectorCandidate, ...],
) -> tuple[CandidateTopologyComponent, ...]:
    """Group A2 states into weak diagnostic candidate-topology components."""
    if not states:
        return ()

    state_by_id = {state.service_state_id: state for state in states}
    resource_by_id = {resource.segment_id: resource for resource in resources}
    adjacency = {state_id: set() for state_id in state_by_id}

    state_ids_by_segment: dict[str, list[str]] = {}
    for state in states:
        state_ids_by_segment.setdefault(state.segment_id, []).append(
            state.service_state_id
        )

    for state_ids in state_ids_by_segment.values():
        ordered = sorted(state_ids)
        if not ordered:
            continue
        anchor = ordered[0]
        for state_id in ordered[1:]:
            adjacency[anchor].add(state_id)
            adjacency[state_id].add(anchor)

    for candidate in candidates:
        from_id = candidate.from_service_state_id
        to_id = candidate.to_service_state_id
        if from_id not in adjacency or to_id not in adjacency:
            raise ValueError(
                "A2 connector candidate references missing service state: "
                f"{candidate.connector_candidate_id}"
            )
        adjacency[from_id].add(to_id)
        adjacency[to_id].add(from_id)

    raw_components: list[tuple[str, ...]] = []
    remaining = set(state_by_id)
    while remaining:
        start = min(remaining)
        stack = [start]
        visited: set[str] = set()
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            stack.extend(sorted(adjacency[current] - visited, reverse=True))
        remaining.difference_update(visited)
        raw_components.append(tuple(sorted(visited)))

    raw_components.sort(key=lambda ids: ids[0])
    components: list[CandidateTopologyComponent] = []
    for index, service_state_ids in enumerate(raw_components, start=1):
        member_set = set(service_state_ids)
        segment_ids = tuple(
            sorted({state_by_id[state_id].segment_id for state_id in service_state_ids})
        )
        missing_resources = [
            segment_id for segment_id in segment_ids if segment_id not in resource_by_id
        ]
        if missing_resources:
            raise ValueError(
                "A2 component references missing service resource: "
                + ", ".join(missing_resources)
            )

        connector_candidate_ids = tuple(
            sorted(
                candidate.connector_candidate_id
                for candidate in candidates
                if candidate.from_service_state_id in member_set
                and candidate.to_service_state_id in member_set
            )
        )
        total_unique_coverage_length_m = sum(
            resource_by_id[segment_id].coverage_length_m
            for segment_id in segment_ids
        )
        distinct_aisle_count = len(
            {
                resource_by_id[segment_id].aisle_id
                for segment_id in segment_ids
            }
        )
        components.append(
            CandidateTopologyComponent(
                component_id=f"candidate_component_{index:03d}",
                classification=CANDIDATE_TOPOLOGY_COMPONENT,
                service_state_ids=service_state_ids,
                segment_ids=segment_ids,
                connector_candidate_ids=connector_candidate_ids,
                total_unique_coverage_length_m=float(
                    total_unique_coverage_length_m
                ),
                distinct_aisle_count=distinct_aisle_count,
            )
        )

    return tuple(components)


def _build_diagnostics(
    resources: tuple[ServiceResource, ...],
    states: tuple[ServiceState, ...],
    candidates: tuple[ConnectorCandidate, ...],
    components: tuple[CandidateTopologyComponent, ...],
) -> ServiceGraphDiagnostics:
    """Summarize static A2 candidate topology without feasibility claims."""
    ordinary_types = {SERVICE_LOW_TO_HIGH, SERVICE_HIGH_TO_LOW}
    return ServiceGraphDiagnostics(
        resource_count=len(resources),
        ordinary_service_state_count=sum(
            state.service_type in ordinary_types for state in states
        ),
        dead_end_candidate_count=sum(
            state.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT
            for state in states
        ),
        total_service_state_count=len(states),
        connector_candidate_count=len(candidates),
        low_u_connector_candidate_count=sum(
            candidate.side == "LOW_U" for candidate in candidates
        ),
        high_u_connector_candidate_count=sum(
            candidate.side == "HIGH_U" for candidate in candidates
        ),
        candidate_component_count=len(components),
        isolated_component_count=sum(
            not component.connector_candidate_ids for component in components
        ),
        externally_unproven_state_count=sum(
            state.external_reachability_status == EXTERNAL_REACHABILITY_UNPROVEN
            for state in states
        ),
        unique_coverage_length_m=float(
            sum(resource.coverage_length_m for resource in resources)
        ),
        distinct_aisle_count=len({resource.aisle_id for resource in resources}),
    )


def derive_vehicle_feasible_service_graph(
    segment_plan: VehicleFeasibleSegmentPlan,
    turn_zones: TurnZoneSet,
    *,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleServiceGraph:
    """Derive static A2 service topology from frozen A1 and Turn Zone evidence."""
    _validate_a2_inputs(segment_plan, turn_zones)

    resources, states = _build_service_resources_and_states(segment_plan)
    candidates = _build_connector_candidates(states, turn_zones)
    components = _build_candidate_components(resources, states, candidates)
    diagnostics = _build_diagnostics(resources, states, candidates, components)

    a1_active_segments = tuple(
        segment
        for aisle in segment_plan.aisles
        for segment in aisle.active_segments
    )
    if len(resources) != len(a1_active_segments):
        raise ValueError(
            "A2 resource count does not preserve A1 active-segment count"
        )

    a1_coverage_m = sum(float(segment.length_m) for segment in a1_active_segments)
    if abs(diagnostics.unique_coverage_length_m - a1_coverage_m) > 1.0e-9:
        raise ValueError("A2 unique coverage does not preserve A1 active coverage")

    merged_source = dict(source or {})
    merged_source.update(
        {
            "vehicle_feasible_segment_schema": segment_plan.schema,
            "turn_zone_schema": turn_zones.schema,
            "derivation_kind": "V25_12G_A2_STATIC_SERVICE_TOPOLOGY",
        }
    )

    return VehicleFeasibleServiceGraph(
        frame_id=str(segment_plan.frame_id),
        platform_id=str(segment_plan.platform_id),
        platform_profile_sha256=str(segment_plan.platform_profile_sha256),
        row_direction_xy=tuple(
            float(value) for value in segment_plan.row_direction_xy
        ),
        service_resources=resources,
        service_states=states,
        connector_candidates=candidates,
        candidate_components=components,
        diagnostics=diagnostics,
        source=merged_source,
    )
