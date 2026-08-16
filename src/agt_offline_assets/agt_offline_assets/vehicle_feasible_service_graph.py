"""Static V25-12G-A2 service-state topology primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    LOW_U_HEADLAND,
    VehicleFeasibleSegmentPlan,
)

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

_HEADLAND_TYPES = {LOW_U_HEADLAND, HIGH_U_HEADLAND}


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
