"""V25-12G-A3 vehicle-feasible motion evidence contract.

This module defines the frozen public data model for A3. Behavioral derivation
is added only after the corresponding RED tests are verified.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
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
