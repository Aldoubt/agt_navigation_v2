"""V25-12G-A3 vehicle-feasible motion evidence contract and assembly."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

from .forward_connector_navigation_gate import GridPathEvidence


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


@dataclass(frozen=True)
class VehicleFeasibleMotionGraphConfig:
    preview_footprint_padding_m: float = 0.05
    pose_position_tolerance_m: float = 1.0e-6
    pose_yaw_tolerance_rad: float = 1.0e-6

    def validate(self) -> None:
        if (
            not math.isfinite(self.preview_footprint_padding_m)
            or self.preview_footprint_padding_m < 0.0
        ):
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")
        if (
            not math.isfinite(self.pose_position_tolerance_m)
            or self.pose_position_tolerance_m <= 0.0
        ):
            raise ValueError("pose_position_tolerance_m must be finite and > 0")
        if (
            not math.isfinite(self.pose_yaw_tolerance_rad)
            or self.pose_yaw_tolerance_rad <= 0.0
        ):
            raise ValueError("pose_yaw_tolerance_rad must be finite and > 0")


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


def _normalized_direction(value, *, label: str) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{label} must contain two values")
    x, y = float(value[0]), float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"{label} must be finite")
    norm = math.hypot(x, y)
    if norm <= 1.0e-12:
        raise ValueError(f"{label} must be non-zero")
    return x / norm, y / norm


def _validate_inputs(service_graph, turn_zones, navigation, vehicle, site_boundary):
    from .vehicle_feasible_service_graph import (
        TOPOLOGY_CANDIDATE_ONLY,
        VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA,
    )

    if service_graph.schema != VEHICLE_FEASIBLE_SERVICE_GRAPH_SCHEMA:
        raise ValueError("A3 service graph schema mismatch")
    if service_graph.status != TOPOLOGY_CANDIDATE_ONLY:
        raise ValueError("A3 service graph status must remain TOPOLOGY_CANDIDATE_ONLY")
    if not service_graph.frame_id == turn_zones.frame_id == navigation.frame_id:
        raise ValueError("A3 input frame mismatch")
    if service_graph.platform_id != vehicle.profile_id:
        raise ValueError("A3 platform does not match canonical vehicle profile")
    if service_graph.platform_profile_sha256 != vehicle.profile_sha256:
        raise ValueError("A3 canonical vehicle profile hash mismatch")
    graph_direction = _normalized_direction(service_graph.row_direction_xy, label="service graph row_direction_xy")
    zone_direction = _normalized_direction(turn_zones.row_direction_xy, label="Turn Zone row_direction_xy")
    if max(abs(graph_direction[0] - zone_direction[0]), abs(graph_direction[1] - zone_direction[1])) > 1.0e-9:
        raise ValueError("A3 row_direction_xy mismatch")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=service_graph.frame_id)


def _build_diagnostics(service_graph, service_actions, transitions):
    from .vehicle_feasible_service_graph import DEAD_END_FORWARD_IN_REVERSE_OUT

    executable_actions = [item for item in service_actions if item.status == EXECUTABLE]
    executable_transitions = [item for item in transitions if item.status == EXECUTABLE]
    resources = {str(item.segment_id): item for item in service_graph.service_resources}
    executable_segment_ids = sorted({item.coverage_segment_id for item in executable_actions})
    missing = [segment_id for segment_id in executable_segment_ids if segment_id not in resources]
    if missing:
        raise ValueError("A3 executable service references missing coverage resources: " + ", ".join(missing))
    return MotionGraphDiagnostics(
        a2_service_state_count=len(service_graph.service_states),
        service_validation_count=len(service_actions),
        executable_service_action_count=len(executable_actions),
        rejected_service_action_count=sum(item.status == REJECTED for item in service_actions),
        unresolved_service_action_count=sum(item.status == UNRESOLVED for item in service_actions),
        ordinary_service_action_count=sum(
            item.service_type != DEAD_END_FORWARD_IN_REVERSE_OUT for item in service_actions
        ),
        dead_end_service_action_count=sum(
            item.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT for item in service_actions
        ),
        executable_dead_end_service_action_count=sum(
            item.service_type == DEAD_END_FORWARD_IN_REVERSE_OUT and item.status == EXECUTABLE
            for item in service_actions
        ),
        a2_connector_candidate_count=len(service_graph.connector_candidates),
        transition_validation_count=len(transitions),
        executable_transition_count=len(executable_transitions),
        forward_executable_transition_count=sum(
            item.status == EXECUTABLE and item.backend == FORWARD_DUBINS_NAVIGATION_GATE
            for item in transitions
        ),
        reverse_executable_transition_count=sum(
            item.status == EXECUTABLE and item.backend == BOUNDED_REVERSE_PRIMITIVE_SEARCH
            for item in transitions
        ),
        rejected_transition_count=sum(item.status == REJECTED for item in transitions),
        unresolved_transition_count=sum(item.status == UNRESOLVED for item in transitions),
        locally_validated_segment_count=len(executable_segment_ids),
        locally_validated_unique_coverage_length_m=float(
            sum(resources[segment_id].coverage_length_m for segment_id in executable_segment_ids)
        ),
        distinct_locally_validated_aisle_count=len(
            {resources[segment_id].aisle_id for segment_id in executable_segment_ids}
        ),
    )


def derive_vehicle_feasible_motion_graph(
    service_graph,
    turn_zones,
    navigation,
    vehicle,
    config: VehicleFeasibleMotionGraphConfig | None = None,
    *,
    site_boundary=None,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleMotionGraph:
    """Derive local A3 service/transition evidence without START reachability claims."""
    from .vehicle_feasible_service_motion import validate_service_actions
    from .vehicle_feasible_transition_motion import validate_transition_candidates

    cfg = config or VehicleFeasibleMotionGraphConfig()
    cfg.validate()
    _validate_inputs(service_graph, turn_zones, navigation, vehicle, site_boundary)
    service_actions = tuple(
        sorted(
            validate_service_actions(
                service_graph,
                navigation,
                vehicle,
                cfg,
                site_boundary=site_boundary,
            ),
            key=lambda item: item.service_state_id,
        )
    )
    transitions = tuple(
        sorted(
            validate_transition_candidates(
                service_graph,
                turn_zones,
                navigation,
                vehicle,
                cfg,
                site_boundary=site_boundary,
            ),
            key=lambda item: item.connector_candidate_id,
        )
    )
    if len(service_actions) != len(service_graph.service_states):
        raise ValueError("A3 service validation cardinality does not match A2 service states")
    if len(transitions) != len(service_graph.connector_candidates):
        raise ValueError("A3 transition validation cardinality does not match A2 connector candidates")

    diagnostics = _build_diagnostics(service_graph, service_actions, transitions)
    merged_source = dict(service_graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "a2_schema": service_graph.schema,
            "validation_scope": "LOCAL_MOTION_EVIDENCE_NOT_START_REACHABILITY",
            "site_boundary_enforced": site_boundary is not None,
            "preview_footprint_padding_m": float(cfg.preview_footprint_padding_m),
            "operator_approved_mixed_connector_ids": [],
        }
    )
    return VehicleFeasibleMotionGraph(
        frame_id=str(service_graph.frame_id),
        platform_id=str(service_graph.platform_id),
        platform_profile_sha256=str(service_graph.platform_profile_sha256),
        row_direction_xy=tuple(float(value) for value in service_graph.row_direction_xy),
        service_actions=service_actions,
        transition_validations=transitions,
        executable_service_action_ids=tuple(
            sorted(item.service_state_id for item in service_actions if item.status == EXECUTABLE)
        ),
        executable_transition_ids=tuple(
            sorted(item.connector_candidate_id for item in transitions if item.status == EXECUTABLE)
        ),
        diagnostics=diagnostics,
        source=merged_source,
    )


def vehicle_feasible_motion_graph_to_dict(graph: VehicleFeasibleMotionGraph) -> dict[str, Any]:
    from .vehicle_feasible_motion_graph_io import vehicle_feasible_motion_graph_to_dict as _impl
    return _impl(graph)


def write_vehicle_feasible_motion_graph(
    graph: VehicleFeasibleMotionGraph,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    from .vehicle_feasible_motion_graph_io import write_vehicle_feasible_motion_graph as _impl
    return _impl(graph, path, overwrite=overwrite)


def load_vehicle_feasible_motion_graph(path: str | Path) -> VehicleFeasibleMotionGraph:
    from .vehicle_feasible_motion_graph_io import load_vehicle_feasible_motion_graph as _impl
    return _impl(path)
