"""Static V25-12G-A2 service-state topology primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .turn_zones import TurnZoneSet
from .vehicle_feasible_segment import (
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
    LOW_U_HEADLAND,
    VEHICLE_FEASIBLE_SEGMENT_SCHEMA,
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
