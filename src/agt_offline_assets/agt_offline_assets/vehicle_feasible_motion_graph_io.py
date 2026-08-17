"""Deterministic strict YAML IO for V25-12G-A3 motion evidence."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from .forward_connector_navigation_gate import GridPathEvidence
from .vehicle_feasible_motion_graph import (
    A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
    A1_CENTERLINE_EXACT_REVERSE_RETRACE,
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
    BOUNDED_SEARCH_NO_SOLUTION,
    EXECUTABLE,
    FORWARD_DUBINS_NAVIGATION_GATE,
    LOCAL_MOTION_EXECUTABLE,
    MAP_EVIDENCE_INSUFFICIENT,
    MIXED_EVIDENCE_REQUIRES_REVIEW,
    MOTION_EVIDENCE_ONLY,
    MotionGraphDiagnostics,
    MotionSample,
    POLICY_REVIEW_REQUIRED,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    REJECTED,
    ServiceActionValidation,
    TransitionValidation,
    UNRESOLVED,
    VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA,
    VehicleFeasibleMotionGraph,
)


_FORBIDDEN_KEYS = {"route_ready", "reachable_from_start", "optimal"}
_VALID_STATUSES = {EXECUTABLE, REJECTED, UNRESOLVED}
_VALID_PROOF_SCOPES = {
    LOCAL_MOTION_EXECUTABLE,
    PROVEN_HARD_CONSTRAINT_REJECTION,
    BOUNDED_SEARCH_NO_SOLUTION,
    MAP_EVIDENCE_INSUFFICIENT,
    MIXED_EVIDENCE_REQUIRES_REVIEW,
    POLICY_REVIEW_REQUIRED,
}
_VALID_BACKENDS = {
    A1_CENTERLINE_DIRECTIONAL_REVALIDATION,
    A1_CENTERLINE_EXACT_REVERSE_RETRACE,
    FORWARD_DUBINS_NAVIGATION_GATE,
    BOUNDED_REVERSE_PRIMITIVE_SEARCH,
}
_DIAGNOSTIC_FIELDS = (
    "a2_service_state_count",
    "service_validation_count",
    "executable_service_action_count",
    "rejected_service_action_count",
    "unresolved_service_action_count",
    "ordinary_service_action_count",
    "dead_end_service_action_count",
    "executable_dead_end_service_action_count",
    "a2_connector_candidate_count",
    "transition_validation_count",
    "executable_transition_count",
    "forward_executable_transition_count",
    "reverse_executable_transition_count",
    "rejected_transition_count",
    "unresolved_transition_count",
    "locally_validated_segment_count",
    "locally_validated_unique_coverage_length_m",
    "distinct_locally_validated_aisle_count",
)


def _plain(value: Any):
    if isinstance(value, Mapping):
        return {str(key): _plain(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_plain(child) for child in value]
    if isinstance(value, list):
        return [_plain(child) for child in value]
    return value


def _evidence_to_dict(evidence: GridPathEvidence) -> dict[str, Any]:
    return {
        "cell_count": int(evidence.cell_count),
        "free_count": int(evidence.free_count),
        "occupied_count": int(evidence.occupied_count),
        "unknown_count": int(evidence.unknown_count),
        "free_fraction": float(evidence.free_fraction),
        "occupied_fraction": float(evidence.occupied_fraction),
        "unknown_fraction": float(evidence.unknown_fraction),
        "grid_coverage_fraction": float(evidence.grid_coverage_fraction),
    }


def _sample_to_dict(sample: MotionSample) -> dict[str, Any]:
    return {
        "x": float(sample.x),
        "y": float(sample.y),
        "z": float(sample.z),
        "yaw": float(sample.yaw),
        "motion_direction": sample.motion_direction,
        "segment_index": int(sample.segment_index),
        "is_cusp": bool(sample.is_cusp),
    }


def _service_to_dict(item: ServiceActionValidation) -> dict[str, Any]:
    return {
        "service_state_id": item.service_state_id,
        "segment_id": item.segment_id,
        "aisle_id": item.aisle_id,
        "service_type": item.service_type,
        "entry_pose": [float(value) for value in item.entry_pose],
        "exit_pose": [float(value) for value in item.exit_pose],
        "status": item.status,
        "proof_scope": item.proof_scope,
        "backend": item.backend,
        "backend_status": item.backend_status,
        "reason": item.reason,
        "coverage_segment_id": item.coverage_segment_id,
        "coverage_reward_length_m": float(item.coverage_reward_length_m),
        "path_length_m": float(item.path_length_m),
        "forward_distance_m": float(item.forward_distance_m),
        "reverse_distance_m": float(item.reverse_distance_m),
        "cusp_count": int(item.cusp_count),
        "samples": [_sample_to_dict(sample) for sample in item.samples],
        "footprint_evidence": _evidence_to_dict(item.footprint_evidence),
    }


def _transition_to_dict(item: TransitionValidation) -> dict[str, Any]:
    return {
        "connector_candidate_id": item.connector_candidate_id,
        "from_service_state_id": item.from_service_state_id,
        "to_service_state_id": item.to_service_state_id,
        "from_segment_id": item.from_segment_id,
        "to_segment_id": item.to_segment_id,
        "side": item.side,
        "turn_zone_id": item.turn_zone_id,
        "start_pose": [float(value) for value in item.start_pose],
        "goal_pose": [float(value) for value in item.goal_pose],
        "status": item.status,
        "proof_scope": item.proof_scope,
        "backend": item.backend,
        "backend_status": item.backend_status,
        "reason": item.reason,
        "path_length_m": float(item.path_length_m),
        "forward_distance_m": float(item.forward_distance_m),
        "reverse_distance_m": float(item.reverse_distance_m),
        "cusp_count": int(item.cusp_count),
        "search_expansions": int(item.search_expansions),
        "samples": [_sample_to_dict(sample) for sample in item.samples],
        "forward_evidence": _plain(item.forward_evidence),
        "reverse_admission_evidence": _plain(item.reverse_admission_evidence),
    }


def _diagnostics_to_dict(item: MotionGraphDiagnostics) -> dict[str, Any]:
    return {
        "a2_service_state_count": int(item.a2_service_state_count),
        "service_validation_count": int(item.service_validation_count),
        "executable_service_action_count": int(item.executable_service_action_count),
        "rejected_service_action_count": int(item.rejected_service_action_count),
        "unresolved_service_action_count": int(item.unresolved_service_action_count),
        "ordinary_service_action_count": int(item.ordinary_service_action_count),
        "dead_end_service_action_count": int(item.dead_end_service_action_count),
        "executable_dead_end_service_action_count": int(item.executable_dead_end_service_action_count),
        "a2_connector_candidate_count": int(item.a2_connector_candidate_count),
        "transition_validation_count": int(item.transition_validation_count),
        "executable_transition_count": int(item.executable_transition_count),
        "forward_executable_transition_count": int(item.forward_executable_transition_count),
        "reverse_executable_transition_count": int(item.reverse_executable_transition_count),
        "rejected_transition_count": int(item.rejected_transition_count),
        "unresolved_transition_count": int(item.unresolved_transition_count),
        "locally_validated_segment_count": int(item.locally_validated_segment_count),
        "locally_validated_unique_coverage_length_m": float(item.locally_validated_unique_coverage_length_m),
        "distinct_locally_validated_aisle_count": int(item.distinct_locally_validated_aisle_count),
    }


def vehicle_feasible_motion_graph_to_dict(graph: VehicleFeasibleMotionGraph) -> dict[str, Any]:
    """Serialize A3 with stable item and field ordering."""
    return {
        "schema": graph.schema,
        "status": graph.status,
        "frame_id": graph.frame_id,
        "platform_id": graph.platform_id,
        "platform_profile_sha256": graph.platform_profile_sha256,
        "row_direction_xy": [float(value) for value in graph.row_direction_xy],
        "source": _plain(graph.source),
        "diagnostics": _diagnostics_to_dict(graph.diagnostics),
        "service_actions": [
            _service_to_dict(item)
            for item in sorted(graph.service_actions, key=lambda value: value.service_state_id)
        ],
        "transition_validations": [
            _transition_to_dict(item)
            for item in sorted(graph.transition_validations, key=lambda value: value.connector_candidate_id)
        ],
        "executable_service_action_ids": sorted(graph.executable_service_action_ids),
        "executable_transition_ids": sorted(graph.executable_transition_ids),
    }


def write_vehicle_feasible_motion_graph(
    graph: VehicleFeasibleMotionGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    output = Path(path).expanduser().resolve()
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


def _reject_forbidden(value: Any, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            key = str(key)
            if key in _FORBIDDEN_KEYS:
                raise ValueError(f"forbidden A3 semantic key {path}.{key}")
            _reject_forbidden(child, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _reject_forbidden(child, f"{path}[{index}]")


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _sequence(value: Any, label: str):
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{label} must be a sequence")
    return value


def _finite(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _nonnegative(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result < 0.0:
        raise ValueError(f"{label} must be >= 0")
    return result


def _pose(value: Any, label: str):
    values = _sequence(value, label)
    if len(values) != 4:
        raise ValueError(f"{label} must contain four values")
    return tuple(_finite(item, label) for item in values)


def _sample(raw: Any) -> MotionSample:
    data = _mapping(raw, "motion sample")
    direction = str(data.get("motion_direction", ""))
    if direction not in {"FORWARD", "REVERSE"}:
        raise ValueError("motion sample motion_direction must be FORWARD or REVERSE")
    segment_index = int(data.get("segment_index", -1))
    if segment_index < 0:
        raise ValueError("motion sample segment_index must be >= 0")
    return MotionSample(
        x=_finite(data.get("x"), "sample.x"),
        y=_finite(data.get("y"), "sample.y"),
        z=_finite(data.get("z"), "sample.z"),
        yaw=_finite(data.get("yaw"), "sample.yaw"),
        motion_direction=direction,
        segment_index=segment_index,
        is_cusp=bool(data.get("is_cusp", False)),
    )


def _evidence(raw: Any) -> GridPathEvidence:
    data = _mapping(raw, "footprint_evidence")
    return GridPathEvidence(
        cell_count=int(data.get("cell_count", 0)),
        free_count=int(data.get("free_count", 0)),
        occupied_count=int(data.get("occupied_count", 0)),
        unknown_count=int(data.get("unknown_count", 0)),
        free_fraction=_nonnegative(data.get("free_fraction", 0.0), "free_fraction"),
        occupied_fraction=_nonnegative(data.get("occupied_fraction", 0.0), "occupied_fraction"),
        unknown_fraction=_nonnegative(data.get("unknown_fraction", 0.0), "unknown_fraction"),
        grid_coverage_fraction=_nonnegative(data.get("grid_coverage_fraction", 0.0), "grid_coverage_fraction"),
    )


def _status_fields(data: Mapping[str, Any]):
    status = str(data.get("status", ""))
    proof_scope = str(data.get("proof_scope", ""))
    backend = str(data.get("backend", ""))
    if status not in _VALID_STATUSES:
        raise ValueError(f"invalid A3 status: {status}")
    if proof_scope not in _VALID_PROOF_SCOPES:
        raise ValueError(f"invalid A3 proof_scope: {proof_scope}")
    if backend not in _VALID_BACKENDS:
        raise ValueError(f"invalid A3 backend: {backend}")
    return status, proof_scope, backend


def _load_service(raw: Any, seen: set[str]) -> ServiceActionValidation:
    data = _mapping(raw, "service action")
    state_id = str(data.get("service_state_id", ""))
    if not state_id or state_id in seen:
        raise ValueError(f"duplicate or empty service_state_id: {state_id}")
    seen.add(state_id)
    status, proof_scope, backend = _status_fields(data)
    samples = tuple(_sample(item) for item in _sequence(data.get("samples", ()), "service samples"))
    cusp_count = int(data.get("cusp_count", 0))
    if cusp_count < 0 or cusp_count != sum(sample.is_cusp for sample in samples):
        raise ValueError(f"service action {state_id} cusp_count mismatch")
    return ServiceActionValidation(
        service_state_id=state_id,
        segment_id=str(data.get("segment_id", "")),
        aisle_id=str(data.get("aisle_id", "")),
        service_type=str(data.get("service_type", "")),
        entry_pose=_pose(data.get("entry_pose"), "entry_pose"),
        exit_pose=_pose(data.get("exit_pose"), "exit_pose"),
        status=status,
        proof_scope=proof_scope,
        backend=backend,
        backend_status=str(data.get("backend_status", "")),
        reason=str(data.get("reason", "")),
        coverage_segment_id=str(data.get("coverage_segment_id", "")),
        coverage_reward_length_m=_nonnegative(data.get("coverage_reward_length_m", 0.0), "coverage_reward_length_m"),
        path_length_m=_nonnegative(data.get("path_length_m", 0.0), "path_length_m"),
        forward_distance_m=_nonnegative(data.get("forward_distance_m", 0.0), "forward_distance_m"),
        reverse_distance_m=_nonnegative(data.get("reverse_distance_m", 0.0), "reverse_distance_m"),
        cusp_count=cusp_count,
        samples=samples,
        footprint_evidence=_evidence(data.get("footprint_evidence", {})),
    )


def _load_transition(raw: Any, seen: set[str]) -> TransitionValidation:
    data = _mapping(raw, "transition validation")
    connector_id = str(data.get("connector_candidate_id", ""))
    if not connector_id or connector_id in seen:
        raise ValueError(f"duplicate or empty connector_candidate_id: {connector_id}")
    seen.add(connector_id)
    status, proof_scope, backend = _status_fields(data)
    samples = tuple(_sample(item) for item in _sequence(data.get("samples", ()), "transition samples"))
    cusp_count = int(data.get("cusp_count", 0))
    if cusp_count < 0 or cusp_count != sum(sample.is_cusp for sample in samples):
        raise ValueError(f"transition {connector_id} cusp_count mismatch")
    search_expansions = int(data.get("search_expansions", 0))
    if search_expansions < 0:
        raise ValueError("search_expansions must be >= 0")
    return TransitionValidation(
        connector_candidate_id=connector_id,
        from_service_state_id=str(data.get("from_service_state_id", "")),
        to_service_state_id=str(data.get("to_service_state_id", "")),
        from_segment_id=str(data.get("from_segment_id", "")),
        to_segment_id=str(data.get("to_segment_id", "")),
        side=str(data.get("side", "")),
        turn_zone_id=str(data.get("turn_zone_id", "")),
        start_pose=_pose(data.get("start_pose"), "start_pose"),
        goal_pose=_pose(data.get("goal_pose"), "goal_pose"),
        status=status,
        proof_scope=proof_scope,
        backend=backend,
        backend_status=str(data.get("backend_status", "")),
        reason=str(data.get("reason", "")),
        path_length_m=_nonnegative(data.get("path_length_m", 0.0), "path_length_m"),
        forward_distance_m=_nonnegative(data.get("forward_distance_m", 0.0), "forward_distance_m"),
        reverse_distance_m=_nonnegative(data.get("reverse_distance_m", 0.0), "reverse_distance_m"),
        cusp_count=cusp_count,
        search_expansions=search_expansions,
        samples=samples,
        forward_evidence=dict(_mapping(data.get("forward_evidence", {}), "forward_evidence")),
        reverse_admission_evidence=dict(_mapping(data.get("reverse_admission_evidence", {}), "reverse_admission_evidence")),
    )


def _load_diagnostics(raw: Any) -> MotionGraphDiagnostics:
    data = _mapping(raw, "diagnostics")
    missing = [field for field in _DIAGNOSTIC_FIELDS if field not in data]
    if missing:
        raise ValueError("diagnostics missing fields: " + ", ".join(missing))
    kwargs = {}
    for field in _DIAGNOSTIC_FIELDS:
        if field == "locally_validated_unique_coverage_length_m":
            kwargs[field] = _nonnegative(data[field], field)
        else:
            value = int(data[field])
            if value < 0:
                raise ValueError(f"diagnostic {field} must be >= 0")
            kwargs[field] = value
    return MotionGraphDiagnostics(**kwargs)


def _validate_loaded(graph: VehicleFeasibleMotionGraph) -> None:
    if graph.schema != VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA:
        raise ValueError(f"expected {VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA}, got {graph.schema}")
    if graph.status != MOTION_EVIDENCE_ONLY:
        raise ValueError("A3 motion graph status must be MOTION_EVIDENCE_ONLY")
    if not graph.frame_id or not graph.platform_id or not graph.platform_profile_sha256:
        raise ValueError("A3 motion graph identity fields must not be empty")
    if len(graph.row_direction_xy) != 2 or not all(math.isfinite(value) for value in graph.row_direction_xy):
        raise ValueError("row_direction_xy must be finite")
    service_ids = {item.service_state_id for item in graph.service_actions}
    transition_ids = {item.connector_candidate_id for item in graph.transition_validations}
    expected_services = tuple(sorted(item.service_state_id for item in graph.service_actions if item.status == EXECUTABLE))
    expected_transitions = tuple(sorted(item.connector_candidate_id for item in graph.transition_validations if item.status == EXECUTABLE))
    if tuple(graph.executable_service_action_ids) != expected_services:
        raise ValueError("executable_service_action_ids do not match executable service records")
    if tuple(graph.executable_transition_ids) != expected_transitions:
        raise ValueError("executable_transition_ids do not match executable transition records")
    for item in graph.transition_validations:
        if item.from_service_state_id not in service_ids or item.to_service_state_id not in service_ids:
            raise ValueError(f"transition {item.connector_candidate_id} references missing service state")
    d = graph.diagnostics
    if d.a2_service_state_count != len(graph.service_actions) or d.service_validation_count != len(graph.service_actions):
        raise ValueError("service diagnostics/count consistency failure")
    if d.a2_connector_candidate_count != len(graph.transition_validations) or d.transition_validation_count != len(graph.transition_validations):
        raise ValueError("transition diagnostics/count consistency failure")
    if d.executable_service_action_count != len(expected_services):
        raise ValueError("executable service diagnostic mismatch")
    if d.executable_transition_count != len(expected_transitions):
        raise ValueError("executable transition diagnostic mismatch")


def load_vehicle_feasible_motion_graph(path: str | Path) -> VehicleFeasibleMotionGraph:
    input_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(input_path.read_text(encoding="utf-8")) or {}
    data = _mapping(raw, "A3 motion graph")
    _reject_forbidden(data)
    if str(data.get("schema", "")) != VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA:
        raise ValueError(f"expected {VEHICLE_FEASIBLE_MOTION_GRAPH_SCHEMA}, got {data.get('schema')}")
    if str(data.get("status", "")) != MOTION_EVIDENCE_ONLY:
        raise ValueError("A3 motion graph status must be MOTION_EVIDENCE_ONLY")
    row = _sequence(data.get("row_direction_xy"), "row_direction_xy")
    if len(row) != 2:
        raise ValueError("row_direction_xy must contain two values")
    seen_services: set[str] = set()
    seen_transitions: set[str] = set()
    services = tuple(
        sorted(
            (_load_service(item, seen_services) for item in _sequence(data.get("service_actions", ()), "service_actions")),
            key=lambda item: item.service_state_id,
        )
    )
    transitions = tuple(
        sorted(
            (_load_transition(item, seen_transitions) for item in _sequence(data.get("transition_validations", ()), "transition_validations")),
            key=lambda item: item.connector_candidate_id,
        )
    )
    executable_services = tuple(str(value) for value in _sequence(data.get("executable_service_action_ids", ()), "executable_service_action_ids"))
    executable_transitions = tuple(str(value) for value in _sequence(data.get("executable_transition_ids", ()), "executable_transition_ids"))
    graph = VehicleFeasibleMotionGraph(
        frame_id=str(data.get("frame_id", "")),
        platform_id=str(data.get("platform_id", "")),
        platform_profile_sha256=str(data.get("platform_profile_sha256", "")),
        row_direction_xy=(_finite(row[0], "row_direction_xy"), _finite(row[1], "row_direction_xy")),
        service_actions=services,
        transition_validations=transitions,
        executable_service_action_ids=executable_services,
        executable_transition_ids=executable_transitions,
        diagnostics=_load_diagnostics(data.get("diagnostics", {})),
        source=dict(_mapping(data.get("source", {}), "source")),
        schema=str(data.get("schema")),
        status=str(data.get("status")),
    )
    _validate_loaded(graph)
    return graph
