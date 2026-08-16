"""Deterministic strict YAML IO for the V25-12G-A2 service topology graph."""

from __future__ import annotations

from pathlib import Path
import math
from typing import Any, Mapping

import yaml

from .turn_zones import TURN_ZONE_SCHEMA
from .vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    VEHICLE_FEASIBLE_SEGMENT_SCHEMA,
)
from .vehicle_feasible_service_graph import (
    CANDIDATE_TOPOLOGY_COMPONENT,
    DEAD_END_FORWARD_IN_REVERSE_OUT,
    EXTERNAL_REACHABILITY_UNPROVEN,
    FORWARD,
    HEADLAND_CONNECTOR_CANDIDATE,
    HEADLAND_TOPOLOGY_CANDIDATE,
    REQUIRES_A3_CONNECTOR_VALIDATION,
    REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
    SERVICE_HIGH_TO_LOW,
    SERVICE_LOW_TO_HIGH,
    TOPOLOGY_CANDIDATE_ONLY,
    TOPOLOGY_SERVICE_CANDIDATE,
    VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA,
    CandidateTopologyComponent,
    ConnectorCandidate,
    ServiceGraphDiagnostics,
    ServiceResource,
    ServiceState,
    VehicleFeasibleServiceGraph,
    _build_candidate_components,
    _build_diagnostics,
    _endpoint_side,
    _reverse_heading,
)

_SERVICE_TYPE_RANK = {
    SERVICE_LOW_TO_HIGH: 0,
    SERVICE_HIGH_TO_LOW: 1,
    DEAD_END_FORWARD_IN_REVERSE_OUT: 2,
}
_VALID_ENDPOINT_TYPES = {
    LOW_U_HEADLAND,
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
}
_VALID_SERVICE_TYPES = set(_SERVICE_TYPE_RANK)
_VALID_REACHABILITY_STATUSES = {
    HEADLAND_TOPOLOGY_CANDIDATE,
    EXTERNAL_REACHABILITY_UNPROVEN,
}
_VALID_STATE_VALIDATION_STATUSES = {
    TOPOLOGY_SERVICE_CANDIDATE,
    REQUIRES_A3_REVERSE_SERVICE_VALIDATION,
}
_VALID_CONNECTOR_SIDES = {"LOW_U", "HIGH_U"}
_FORBIDDEN_FIELD_NAMES = {
    "executable",
    "reachable",
    "reachable_from_start",
    "connector_feasible",
    "route_ready",
    "optimal",
    "connector_length_m",
    "reverse_connector_length_m",
    "cusp_count",
    "search_cost",
}
_TOP_LEVEL_KEYS = (
    "schema",
    "status",
    "frame_id",
    "platform_id",
    "platform_profile_sha256",
    "row_direction_xy",
    "source",
    "diagnostics",
    "service_resources",
    "service_states",
    "connector_candidates",
    "candidate_components",
)
_DIAGNOSTIC_FIELDS = (
    "resource_count",
    "ordinary_service_state_count",
    "dead_end_candidate_count",
    "total_service_state_count",
    "connector_candidate_count",
    "low_u_connector_candidate_count",
    "high_u_connector_candidate_count",
    "candidate_component_count",
    "isolated_component_count",
    "externally_unproven_state_count",
    "unique_coverage_length_m",
    "distinct_aisle_count",
)
_RESOURCE_FIELDS = (
    "segment_id",
    "aisle_id",
    "ordinal_in_aisle",
    "coverage_length_m",
    "coverage_fraction_of_aisle",
    "low_endpoint_type",
    "high_endpoint_type",
    "low_endpoint_pose",
    "high_endpoint_pose",
    "centerline_xyz",
)
_STATE_FIELDS = (
    "service_state_id",
    "segment_id",
    "aisle_id",
    "service_type",
    "entry_endpoint_type",
    "exit_endpoint_type",
    "entry_pose",
    "exit_pose",
    "service_motion_direction",
    "forward_service_distance_m",
    "reverse_service_distance_m",
    "coverage_segment_id",
    "coverage_reward_length_m",
    "external_reachability_status",
    "validation_status",
)
_CONNECTOR_FIELDS = (
    "connector_candidate_id",
    "from_service_state_id",
    "to_service_state_id",
    "from_segment_id",
    "to_segment_id",
    "side",
    "turn_zone_id",
    "start_pose",
    "goal_pose",
    "candidate_kind",
    "validation_status",
)
_COMPONENT_FIELDS = (
    "component_id",
    "classification",
    "service_state_ids",
    "segment_ids",
    "connector_candidate_ids",
    "total_unique_coverage_length_m",
    "distinct_aisle_count",
)


def _resource_to_dict(resource: ServiceResource) -> dict[str, Any]:
    return {
        "segment_id": resource.segment_id,
        "aisle_id": resource.aisle_id,
        "ordinal_in_aisle": int(resource.ordinal_in_aisle),
        "coverage_length_m": float(resource.coverage_length_m),
        "coverage_fraction_of_aisle": float(resource.coverage_fraction_of_aisle),
        "low_endpoint_type": resource.low_endpoint_type,
        "high_endpoint_type": resource.high_endpoint_type,
        "low_endpoint_pose": [float(value) for value in resource.low_endpoint_pose],
        "high_endpoint_pose": [float(value) for value in resource.high_endpoint_pose],
        "centerline_xyz": [
            [float(value) for value in point] for point in resource.centerline_xyz
        ],
    }


def _state_to_dict(state: ServiceState) -> dict[str, Any]:
    return {
        "service_state_id": state.service_state_id,
        "segment_id": state.segment_id,
        "aisle_id": state.aisle_id,
        "service_type": state.service_type,
        "entry_endpoint_type": state.entry_endpoint_type,
        "exit_endpoint_type": state.exit_endpoint_type,
        "entry_pose": [float(value) for value in state.entry_pose],
        "exit_pose": [float(value) for value in state.exit_pose],
        "service_motion_direction": state.service_motion_direction,
        "forward_service_distance_m": float(state.forward_service_distance_m),
        "reverse_service_distance_m": float(state.reverse_service_distance_m),
        "coverage_segment_id": state.coverage_segment_id,
        "coverage_reward_length_m": float(state.coverage_reward_length_m),
        "external_reachability_status": state.external_reachability_status,
        "validation_status": state.validation_status,
    }


def _connector_to_dict(candidate: ConnectorCandidate) -> dict[str, Any]:
    return {
        "connector_candidate_id": candidate.connector_candidate_id,
        "from_service_state_id": candidate.from_service_state_id,
        "to_service_state_id": candidate.to_service_state_id,
        "from_segment_id": candidate.from_segment_id,
        "to_segment_id": candidate.to_segment_id,
        "side": candidate.side,
        "turn_zone_id": candidate.turn_zone_id,
        "start_pose": [float(value) for value in candidate.start_pose],
        "goal_pose": [float(value) for value in candidate.goal_pose],
        "candidate_kind": candidate.candidate_kind,
        "validation_status": candidate.validation_status,
    }


def _component_to_dict(component: CandidateTopologyComponent) -> dict[str, Any]:
    return {
        "component_id": component.component_id,
        "classification": component.classification,
        "service_state_ids": sorted(component.service_state_ids),
        "segment_ids": sorted(component.segment_ids),
        "connector_candidate_ids": sorted(component.connector_candidate_ids),
        "total_unique_coverage_length_m": float(
            component.total_unique_coverage_length_m
        ),
        "distinct_aisle_count": int(component.distinct_aisle_count),
    }


def _diagnostics_to_dict(diagnostics: ServiceGraphDiagnostics) -> dict[str, Any]:
    return {
        "resource_count": int(diagnostics.resource_count),
        "ordinary_service_state_count": int(
            diagnostics.ordinary_service_state_count
        ),
        "dead_end_candidate_count": int(diagnostics.dead_end_candidate_count),
        "total_service_state_count": int(diagnostics.total_service_state_count),
        "connector_candidate_count": int(diagnostics.connector_candidate_count),
        "low_u_connector_candidate_count": int(
            diagnostics.low_u_connector_candidate_count
        ),
        "high_u_connector_candidate_count": int(
            diagnostics.high_u_connector_candidate_count
        ),
        "candidate_component_count": int(diagnostics.candidate_component_count),
        "isolated_component_count": int(diagnostics.isolated_component_count),
        "externally_unproven_state_count": int(
            diagnostics.externally_unproven_state_count
        ),
        "unique_coverage_length_m": float(diagnostics.unique_coverage_length_m),
        "distinct_aisle_count": int(diagnostics.distinct_aisle_count),
    }


def vehicle_feasible_service_graph_to_dict(
    graph: VehicleFeasibleServiceGraph,
) -> dict[str, Any]:
    """Serialize A2 static topology with frozen field and item ordering."""
    resources = sorted(graph.service_resources, key=lambda item: item.segment_id)
    states = sorted(
        graph.service_states,
        key=lambda item: (
            item.segment_id,
            _SERVICE_TYPE_RANK.get(item.service_type, 99),
            item.service_state_id,
        ),
    )
    candidates = sorted(
        graph.connector_candidates,
        key=lambda item: (
            item.turn_zone_id,
            item.from_service_state_id,
            item.to_service_state_id,
        ),
    )
    components = sorted(graph.candidate_components, key=lambda item: item.component_id)
    return {
        "schema": graph.schema,
        "status": graph.status,
        "frame_id": graph.frame_id,
        "platform_id": graph.platform_id,
        "platform_profile_sha256": graph.platform_profile_sha256,
        "row_direction_xy": [float(value) for value in graph.row_direction_xy],
        "source": dict(graph.source),
        "diagnostics": _diagnostics_to_dict(graph.diagnostics),
        "service_resources": [_resource_to_dict(item) for item in resources],
        "service_states": [_state_to_dict(item) for item in states],
        "connector_candidates": [_connector_to_dict(item) for item in candidates],
        "candidate_components": [_component_to_dict(item) for item in components],
    }


def write_vehicle_feasible_service_graph(
    graph: VehicleFeasibleServiceGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    """Write one deterministic A2 service-topology YAML asset."""
    output = Path(path).expanduser().resolve()
    if output.exists() and not overwrite:
        raise FileExistsError(f"vehicle-feasible service graph already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            vehicle_feasible_service_graph_to_dict(graph),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output


def _require_mapping(value: Any, field_name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field_name} must be a mapping")
    return value


def _require_sequence(value: Any, field_name: str) -> list[Any] | tuple[Any, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{field_name} must be a sequence")
    return value


def _require_exact_keys(
    data: Mapping[str, Any],
    expected: tuple[str, ...],
    field_name: str,
) -> None:
    missing = set(expected).difference(data)
    extra = set(data).difference(expected)
    if missing:
        raise ValueError(f"{field_name} missing required keys: {sorted(missing)}")
    if extra:
        raise ValueError(f"{field_name} has unsupported keys: {sorted(extra)}")


def _finite_float(value: Any, field_name: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{field_name} must be finite")
    return numeric


def _fixed_floats(value: Any, length: int, field_name: str) -> tuple[float, ...]:
    values = _require_sequence(value, field_name)
    if len(values) != length:
        raise ValueError(f"{field_name} must contain exactly {length} values")
    return tuple(_finite_float(item, field_name) for item in values)


def _poses_match(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> bool:
    return all(abs(a - b) <= 1.0e-9 for a, b in zip(first, second))


def _reject_forbidden_fields(value: Any, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            key_text = str(key)
            if key_text in _FORBIDDEN_FIELD_NAMES:
                raise ValueError(f"forbidden A2 field {path}.{key_text}")
            _reject_forbidden_fields(child, f"{path}.{key_text}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _reject_forbidden_fields(child, f"{path}[{index}]")


def _load_resource(raw: Any, seen_ids: set[str]) -> ServiceResource:
    data = _require_mapping(raw, "service resource")
    _require_exact_keys(data, _RESOURCE_FIELDS, "service resource")
    segment_id = str(data["segment_id"])
    if not segment_id or segment_id in seen_ids:
        raise ValueError(f"duplicate or empty service resource segment_id: {segment_id}")
    seen_ids.add(segment_id)
    aisle_id = str(data["aisle_id"])
    if not aisle_id:
        raise ValueError(f"service resource {segment_id} aisle_id must not be empty")
    ordinal = int(data["ordinal_in_aisle"])
    if ordinal <= 0:
        raise ValueError(f"service resource {segment_id} ordinal_in_aisle must be > 0")
    length_m = _finite_float(data["coverage_length_m"], "coverage_length_m")
    if length_m <= 0.0:
        raise ValueError(f"service resource {segment_id} coverage_length_m must be > 0")
    coverage_fraction = _finite_float(
        data["coverage_fraction_of_aisle"], "coverage_fraction_of_aisle"
    )
    if coverage_fraction < 0.0:
        raise ValueError(
            f"service resource {segment_id} coverage_fraction_of_aisle must be >= 0"
        )
    low_type = str(data["low_endpoint_type"])
    high_type = str(data["high_endpoint_type"])
    if low_type not in _VALID_ENDPOINT_TYPES or high_type not in _VALID_ENDPOINT_TYPES:
        raise ValueError(f"service resource {segment_id} has invalid endpoint type")
    low_pose = _fixed_floats(data["low_endpoint_pose"], 4, "low_endpoint_pose")
    high_pose = _fixed_floats(data["high_endpoint_pose"], 4, "high_endpoint_pose")
    raw_points = _require_sequence(data["centerline_xyz"], "centerline_xyz")
    if not raw_points:
        raise ValueError(f"service resource {segment_id} centerline_xyz must not be empty")
    points = tuple(
        _fixed_floats(point, 3, f"service resource {segment_id} centerline_xyz")
        for point in raw_points
    )
    return ServiceResource(
        segment_id=segment_id,
        aisle_id=aisle_id,
        ordinal_in_aisle=ordinal,
        coverage_length_m=length_m,
        coverage_fraction_of_aisle=coverage_fraction,
        low_endpoint_type=low_type,
        high_endpoint_type=high_type,
        low_endpoint_pose=low_pose,
        high_endpoint_pose=high_pose,
        centerline_xyz=points,
    )


def _expected_state_id(segment_id: str, service_type: str) -> str:
    suffix = {
        SERVICE_LOW_TO_HIGH: "service_low_to_high",
        SERVICE_HIGH_TO_LOW: "service_high_to_low",
        DEAD_END_FORWARD_IN_REVERSE_OUT: "dead_end_forward_in_reverse_out",
    }[service_type]
    return f"{segment_id}.{suffix}"


def _validate_state_semantics(state: ServiceState, resource: ServiceResource) -> None:
    if state.service_state_id != _expected_state_id(resource.segment_id, state.service_type):
        raise ValueError(f"service_state {state.service_state_id} has invalid deterministic id")
    if state.aisle_id != resource.aisle_id:
        raise ValueError(f"service_state {state.service_state_id} aisle_id mismatch")
    if state.coverage_segment_id != resource.segment_id:
        raise ValueError(f"service_state {state.service_state_id} coverage_segment_id mismatch")
    if abs(state.coverage_reward_length_m - resource.coverage_length_m) > 1.0e-9:
        raise ValueError(f"service_state {state.service_state_id} coverage reward mismatch")
    if state.service_motion_direction != FORWARD:
        raise ValueError(f"service_state {state.service_state_id} motion direction mismatch")

    if state.service_type == SERVICE_LOW_TO_HIGH:
        expected_entry_type = resource.low_endpoint_type
        expected_exit_type = resource.high_endpoint_type
        expected_entry_pose = resource.low_endpoint_pose
        expected_exit_pose = resource.high_endpoint_pose
        expected_reverse = 0.0
        expected_validation = TOPOLOGY_SERVICE_CANDIDATE
    elif state.service_type == SERVICE_HIGH_TO_LOW:
        expected_entry_type = resource.high_endpoint_type
        expected_exit_type = resource.low_endpoint_type
        expected_entry_pose = _reverse_heading(resource.high_endpoint_pose)
        expected_exit_pose = _reverse_heading(resource.low_endpoint_pose)
        expected_reverse = 0.0
        expected_validation = TOPOLOGY_SERVICE_CANDIDATE
    else:
        low_is_headland = resource.low_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
        high_is_headland = resource.high_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
        if low_is_headland == high_is_headland:
            raise ValueError(
                f"service_state {state.service_state_id} dead-end state is not allowed for this resource"
            )
        if low_is_headland:
            expected_entry_type = resource.low_endpoint_type
            expected_entry_pose = resource.low_endpoint_pose
        else:
            expected_entry_type = resource.high_endpoint_type
            expected_entry_pose = _reverse_heading(resource.high_endpoint_pose)
        expected_exit_type = expected_entry_type
        expected_exit_pose = expected_entry_pose
        expected_reverse = resource.coverage_length_m
        expected_validation = REQUIRES_A3_REVERSE_SERVICE_VALIDATION

    if state.entry_endpoint_type != expected_entry_type or state.exit_endpoint_type != expected_exit_type:
        raise ValueError(f"service_state {state.service_state_id} endpoint type mismatch")
    if not _poses_match(state.entry_pose, expected_entry_pose):
        raise ValueError(f"service_state {state.service_state_id} entry_pose mismatch")
    if not _poses_match(state.exit_pose, expected_exit_pose):
        raise ValueError(f"service_state {state.service_state_id} exit_pose mismatch")
    if abs(state.forward_service_distance_m - resource.coverage_length_m) > 1.0e-9:
        raise ValueError(f"service_state {state.service_state_id} forward distance mismatch")
    if abs(state.reverse_service_distance_m - expected_reverse) > 1.0e-9:
        raise ValueError(f"service_state {state.service_state_id} reverse distance mismatch")
    if state.validation_status != expected_validation:
        raise ValueError(f"service_state {state.service_state_id} validation_status mismatch")
    expected_reachability = (
        HEADLAND_TOPOLOGY_CANDIDATE
        if state.entry_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
        else EXTERNAL_REACHABILITY_UNPROVEN
    )
    if state.external_reachability_status != expected_reachability:
        raise ValueError(
            f"service_state {state.service_state_id} external reachability mismatch"
        )


def _load_state(
    raw: Any,
    resource_by_id: Mapping[str, ServiceResource],
    seen_ids: set[str],
) -> ServiceState:
    data = _require_mapping(raw, "service_state")
    _require_exact_keys(data, _STATE_FIELDS, "service_state")
    state_id = str(data["service_state_id"])
    if not state_id or state_id in seen_ids:
        raise ValueError(f"duplicate or empty service_state_id: {state_id}")
    seen_ids.add(state_id)
    segment_id = str(data["segment_id"])
    resource = resource_by_id.get(segment_id)
    if resource is None:
        raise ValueError(f"service_state {state_id} references unknown segment_id")
    service_type = str(data["service_type"])
    if service_type not in _VALID_SERVICE_TYPES:
        raise ValueError(f"service_state {state_id} has invalid service_type")
    entry_type = str(data["entry_endpoint_type"])
    exit_type = str(data["exit_endpoint_type"])
    if entry_type not in _VALID_ENDPOINT_TYPES or exit_type not in _VALID_ENDPOINT_TYPES:
        raise ValueError(f"service_state {state_id} has invalid endpoint type")
    reachability = str(data["external_reachability_status"])
    if reachability not in _VALID_REACHABILITY_STATUSES:
        raise ValueError(f"service_state {state_id} has invalid reachability status")
    validation_status = str(data["validation_status"])
    if validation_status not in _VALID_STATE_VALIDATION_STATUSES:
        raise ValueError(f"service_state {state_id} has invalid validation_status")
    state = ServiceState(
        service_state_id=state_id,
        segment_id=segment_id,
        aisle_id=str(data["aisle_id"]),
        service_type=service_type,
        entry_endpoint_type=entry_type,
        exit_endpoint_type=exit_type,
        entry_pose=_fixed_floats(data["entry_pose"], 4, "entry_pose"),
        exit_pose=_fixed_floats(data["exit_pose"], 4, "exit_pose"),
        service_motion_direction=str(data["service_motion_direction"]),
        forward_service_distance_m=_finite_float(
            data["forward_service_distance_m"], "forward_service_distance_m"
        ),
        reverse_service_distance_m=_finite_float(
            data["reverse_service_distance_m"], "reverse_service_distance_m"
        ),
        coverage_segment_id=str(data["coverage_segment_id"]),
        coverage_reward_length_m=_finite_float(
            data["coverage_reward_length_m"], "coverage_reward_length_m"
        ),
        external_reachability_status=reachability,
        validation_status=validation_status,
    )
    if state.forward_service_distance_m < 0.0 or state.reverse_service_distance_m < 0.0:
        raise ValueError(f"service_state {state_id} distances must be >= 0")
    _validate_state_semantics(state, resource)
    return state


def _validate_state_inventory(
    resources: tuple[ServiceResource, ...],
    states: tuple[ServiceState, ...],
) -> None:
    states_by_segment: dict[str, list[ServiceState]] = {}
    for state in states:
        states_by_segment.setdefault(state.segment_id, []).append(state)
    for resource in resources:
        resource_states = states_by_segment.get(resource.segment_id, [])
        actual_types = [state.service_type for state in resource_states]
        expected_types = [SERVICE_LOW_TO_HIGH, SERVICE_HIGH_TO_LOW]
        low_is_headland = resource.low_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
        high_is_headland = resource.high_endpoint_type in {LOW_U_HEADLAND, HIGH_U_HEADLAND}
        if low_is_headland != high_is_headland:
            expected_types.append(DEAD_END_FORWARD_IN_REVERSE_OUT)
        if sorted(actual_types) != sorted(expected_types):
            raise ValueError(
                f"service_state inventory mismatch for segment {resource.segment_id}"
            )


def _load_connector(
    raw: Any,
    state_by_id: Mapping[str, ServiceState],
    seen_ids: set[str],
) -> ConnectorCandidate:
    data = _require_mapping(raw, "connector candidate")
    _require_exact_keys(data, _CONNECTOR_FIELDS, "connector candidate")
    candidate_id = str(data["connector_candidate_id"])
    if not candidate_id or candidate_id in seen_ids:
        raise ValueError(f"duplicate or empty connector_candidate_id: {candidate_id}")
    seen_ids.add(candidate_id)
    from_state_id = str(data["from_service_state_id"])
    to_state_id = str(data["to_service_state_id"])
    from_state = state_by_id.get(from_state_id)
    to_state = state_by_id.get(to_state_id)
    if from_state is None or to_state is None:
        raise ValueError(f"connector candidate {candidate_id} references unknown service_state")
    from_segment_id = str(data["from_segment_id"])
    to_segment_id = str(data["to_segment_id"])
    if from_segment_id != from_state.segment_id or to_segment_id != to_state.segment_id:
        raise ValueError(f"connector candidate {candidate_id} segment reference mismatch")
    if from_segment_id == to_segment_id:
        raise ValueError(f"connector candidate {candidate_id} cannot self-connect a segment")
    side = str(data["side"])
    if side not in _VALID_CONNECTOR_SIDES:
        raise ValueError(f"connector candidate {candidate_id} has invalid side")
    from_side = _endpoint_side(from_state.exit_endpoint_type)
    to_side = _endpoint_side(to_state.entry_endpoint_type)
    if from_side != side or to_side != side:
        raise ValueError(f"connector candidate {candidate_id} headland side mismatch")
    turn_zone_id = str(data["turn_zone_id"])
    if not turn_zone_id:
        raise ValueError(f"connector candidate {candidate_id} turn_zone_id must not be empty")
    expected_id = f"headland.{turn_zone_id}.{from_state_id}.to.{to_state_id}"
    if candidate_id != expected_id:
        raise ValueError(f"connector candidate {candidate_id} deterministic id mismatch")
    start_pose = _fixed_floats(data["start_pose"], 4, "start_pose")
    goal_pose = _fixed_floats(data["goal_pose"], 4, "goal_pose")
    if not _poses_match(start_pose, from_state.exit_pose):
        raise ValueError(f"connector candidate {candidate_id} start_pose mismatch")
    if not _poses_match(goal_pose, to_state.entry_pose):
        raise ValueError(f"connector candidate {candidate_id} goal_pose mismatch")
    candidate_kind = str(data["candidate_kind"])
    validation_status = str(data["validation_status"])
    if candidate_kind != HEADLAND_CONNECTOR_CANDIDATE:
        raise ValueError(f"connector candidate {candidate_id} candidate_kind mismatch")
    if validation_status != REQUIRES_A3_CONNECTOR_VALIDATION:
        raise ValueError(f"connector candidate {candidate_id} validation_status mismatch")
    return ConnectorCandidate(
        connector_candidate_id=candidate_id,
        from_service_state_id=from_state_id,
        to_service_state_id=to_state_id,
        from_segment_id=from_segment_id,
        to_segment_id=to_segment_id,
        side=side,
        turn_zone_id=turn_zone_id,
        start_pose=start_pose,
        goal_pose=goal_pose,
        candidate_kind=candidate_kind,
        validation_status=validation_status,
    )


def _load_component(
    raw: Any,
    resource_by_id: Mapping[str, ServiceResource],
    state_by_id: Mapping[str, ServiceState],
    candidate_by_id: Mapping[str, ConnectorCandidate],
    seen_ids: set[str],
) -> CandidateTopologyComponent:
    data = _require_mapping(raw, "candidate component")
    _require_exact_keys(data, _COMPONENT_FIELDS, "candidate component")
    component_id = str(data["component_id"])
    if not component_id or component_id in seen_ids:
        raise ValueError(f"duplicate or empty component_id: {component_id}")
    seen_ids.add(component_id)
    classification = str(data["classification"])
    if classification != CANDIDATE_TOPOLOGY_COMPONENT:
        raise ValueError(f"candidate component {component_id} classification mismatch")
    service_state_ids = tuple(
        str(value) for value in _require_sequence(data["service_state_ids"], "service_state_ids")
    )
    segment_ids = tuple(
        str(value) for value in _require_sequence(data["segment_ids"], "segment_ids")
    )
    connector_ids = tuple(
        str(value)
        for value in _require_sequence(
            data["connector_candidate_ids"], "connector_candidate_ids"
        )
    )
    if len(service_state_ids) != len(set(service_state_ids)):
        raise ValueError(f"candidate component {component_id} has duplicate service_state ids")
    if len(segment_ids) != len(set(segment_ids)):
        raise ValueError(f"candidate component {component_id} has duplicate segment ids")
    if len(connector_ids) != len(set(connector_ids)):
        raise ValueError(f"candidate component {component_id} has duplicate connector ids")
    if any(value not in state_by_id for value in service_state_ids):
        raise ValueError(f"candidate component {component_id} references unknown service_state")
    if any(value not in resource_by_id for value in segment_ids):
        raise ValueError(f"candidate component {component_id} references unknown segment")
    if any(value not in candidate_by_id for value in connector_ids):
        raise ValueError(f"candidate component {component_id} references unknown connector")
    total_coverage = _finite_float(
        data["total_unique_coverage_length_m"], "total_unique_coverage_length_m"
    )
    distinct_aisle_count = int(data["distinct_aisle_count"])
    if distinct_aisle_count < 0:
        raise ValueError(f"candidate component {component_id} distinct_aisle_count must be >= 0")
    return CandidateTopologyComponent(
        component_id=component_id,
        classification=classification,
        service_state_ids=service_state_ids,
        segment_ids=segment_ids,
        connector_candidate_ids=connector_ids,
        total_unique_coverage_length_m=total_coverage,
        distinct_aisle_count=distinct_aisle_count,
    )


def _load_diagnostics(raw: Any) -> ServiceGraphDiagnostics:
    data = _require_mapping(raw, "diagnostics")
    _require_exact_keys(data, _DIAGNOSTIC_FIELDS, "diagnostics")
    integer_fields = set(_DIAGNOSTIC_FIELDS) - {"unique_coverage_length_m"}
    values: dict[str, Any] = {}
    for name in _DIAGNOSTIC_FIELDS:
        if name in integer_fields:
            value = int(data[name])
            if value < 0:
                raise ValueError(f"diagnostics.{name} must be >= 0")
            values[name] = value
        else:
            value = _finite_float(data[name], f"diagnostics.{name}")
            if value < 0.0:
                raise ValueError(f"diagnostics.{name} must be >= 0")
            values[name] = value
    return ServiceGraphDiagnostics(**values)


def _validate_components(
    claimed: tuple[CandidateTopologyComponent, ...],
    expected: tuple[CandidateTopologyComponent, ...],
) -> None:
    if len(claimed) != len(expected):
        raise ValueError("component count mismatch")
    for actual, wanted in zip(claimed, expected):
        if actual.component_id != wanted.component_id:
            raise ValueError("component id/order mismatch")
        if abs(
            actual.total_unique_coverage_length_m
            - wanted.total_unique_coverage_length_m
        ) > 1.0e-9:
            raise ValueError(f"component {actual.component_id} coverage mismatch")
        if actual != wanted:
            raise ValueError(f"component {actual.component_id} topology mismatch")


def _validate_diagnostics(
    claimed: ServiceGraphDiagnostics,
    expected: ServiceGraphDiagnostics,
) -> None:
    for name in _DIAGNOSTIC_FIELDS:
        actual = getattr(claimed, name)
        wanted = getattr(expected, name)
        if name == "unique_coverage_length_m":
            if abs(float(actual) - float(wanted)) > 1.0e-9:
                raise ValueError("diagnostics.unique_coverage_length_m mismatch")
        elif actual != wanted:
            raise ValueError(f"diagnostics.{name} mismatch")


def load_vehicle_feasible_service_graph(
    path: str | Path,
) -> VehicleFeasibleServiceGraph:
    """Load and fail-closed validate one v1 A2 static service-topology asset."""
    input_path = Path(path).expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"vehicle-feasible service graph YAML not found: {input_path}")
    payload = yaml.safe_load(input_path.read_text(encoding="utf-8")) or {}
    data = _require_mapping(payload, "vehicle-feasible service graph YAML")
    _reject_forbidden_fields(data)
    _require_exact_keys(data, _TOP_LEVEL_KEYS, "vehicle-feasible service graph YAML")
    if str(data["schema"]) != VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA:
        raise ValueError(
            "vehicle-feasible service graph schema mismatch: "
            f"expected {VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA}, got {data['schema']}"
        )
    if str(data["status"]) != TOPOLOGY_CANDIDATE_ONLY:
        raise ValueError("vehicle-feasible service graph status must be TOPOLOGY_CANDIDATE_ONLY")

    frame_id = str(data["frame_id"])
    platform_id = str(data["platform_id"])
    profile_sha = str(data["platform_profile_sha256"])
    if not frame_id or not platform_id or not profile_sha:
        raise ValueError("frame_id, platform_id, and platform_profile_sha256 must be non-empty")
    row_direction = _fixed_floats(data["row_direction_xy"], 2, "row_direction_xy")
    if math.hypot(row_direction[0], row_direction[1]) <= 1.0e-12:
        raise ValueError("row_direction_xy must be non-zero")

    source = dict(_require_mapping(data["source"], "source"))
    if source.get("vehicle_feasible_segment_schema") != VEHICLE_FEASIBLE_SEGMENT_SCHEMA:
        raise ValueError("source.vehicle_feasible_segment_schema mismatch")
    if source.get("turn_zone_schema") != TURN_ZONE_SCHEMA:
        raise ValueError("source.turn_zone_schema mismatch")
    if source.get("derivation_kind") != "V25_12G_A2_STATIC_SERVICE_TOPOLOGY":
        raise ValueError("source.derivation_kind mismatch")

    resource_ids: set[str] = set()
    resources = tuple(
        _load_resource(item, resource_ids)
        for item in _require_sequence(data["service_resources"], "service_resources")
    )
    if tuple(resource.segment_id for resource in resources) != tuple(
        sorted(resource.segment_id for resource in resources)
    ):
        raise ValueError("service_resources must be deterministically sorted")
    resource_by_id = {resource.segment_id: resource for resource in resources}

    state_ids: set[str] = set()
    states = tuple(
        _load_state(item, resource_by_id, state_ids)
        for item in _require_sequence(data["service_states"], "service_states")
    )
    expected_state_order = tuple(
        state.service_state_id
        for state in sorted(
            states,
            key=lambda item: (
                item.segment_id,
                _SERVICE_TYPE_RANK[item.service_type],
                item.service_state_id,
            ),
        )
    )
    if tuple(state.service_state_id for state in states) != expected_state_order:
        raise ValueError("service_states must be deterministically sorted")
    _validate_state_inventory(resources, states)
    state_by_id = {state.service_state_id: state for state in states}

    candidate_ids: set[str] = set()
    candidates = tuple(
        _load_connector(item, state_by_id, candidate_ids)
        for item in _require_sequence(
            data["connector_candidates"], "connector_candidates"
        )
    )
    expected_candidate_order = tuple(
        item.connector_candidate_id
        for item in sorted(
            candidates,
            key=lambda item: (
                item.turn_zone_id,
                item.from_service_state_id,
                item.to_service_state_id,
            ),
        )
    )
    if tuple(item.connector_candidate_id for item in candidates) != expected_candidate_order:
        raise ValueError("connector_candidates must be deterministically sorted")
    candidate_by_id = {item.connector_candidate_id: item for item in candidates}

    component_ids: set[str] = set()
    components = tuple(
        _load_component(
            item,
            resource_by_id,
            state_by_id,
            candidate_by_id,
            component_ids,
        )
        for item in _require_sequence(data["candidate_components"], "candidate_components")
    )
    if tuple(item.component_id for item in components) != tuple(
        sorted(item.component_id for item in components)
    ):
        raise ValueError("candidate_components must be deterministically sorted")
    for component in components:
        if component.service_state_ids != tuple(sorted(component.service_state_ids)):
            raise ValueError(f"component {component.component_id} service_state_ids not sorted")
        if component.segment_ids != tuple(sorted(component.segment_ids)):
            raise ValueError(f"component {component.component_id} segment_ids not sorted")
        if component.connector_candidate_ids != tuple(
            sorted(component.connector_candidate_ids)
        ):
            raise ValueError(
                f"component {component.component_id} connector_candidate_ids not sorted"
            )

    expected_components = _build_candidate_components(resources, states, candidates)
    _validate_components(components, expected_components)
    diagnostics = _load_diagnostics(data["diagnostics"])
    expected_diagnostics = _build_diagnostics(
        resources,
        states,
        candidates,
        expected_components,
    )
    _validate_diagnostics(diagnostics, expected_diagnostics)

    graph = VehicleFeasibleServiceGraph(
        frame_id=frame_id,
        platform_id=platform_id,
        platform_profile_sha256=profile_sha,
        row_direction_xy=(float(row_direction[0]), float(row_direction[1])),
        service_resources=resources,
        service_states=states,
        connector_candidates=candidates,
        candidate_components=components,
        diagnostics=diagnostics,
        source=source,
        schema=str(data["schema"]),
        status=str(data["status"]),
    )

    if vehicle_feasible_service_graph_to_dict(graph) != dict(data):
        raise ValueError("vehicle-feasible service graph YAML is not canonical")
    return graph
