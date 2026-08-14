"""Forward-only connector generation for V25-12E agricultural routes.

R5 consumes R4 ``ConnectorRequest`` objects, R2 Turn Zones, and the canonical
Vehicle Profile.  It enumerates the six classical Dubins families and selects
the shortest candidate whose sampled centerline remains inside the requested
Turn Zone.

This is intentionally a *centerline kinematic preview gate*.  It does not
perform full-footprint swept collision checking and therefore cannot promote a
Route Asset to READY.  Full vehicle-envelope / occupancy validation remains an
R8 responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable, Mapping

import numpy as np

from .agricultural_coverage_ordering import ConnectorRequest
from .turn_zones import TurnZone, TurnZoneSet
from .vehicle_profile import CanonicalVehicleProfile


FORWARD_CONNECTOR_SCHEMA = "agt_forward_connector_plan/v1"
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class ForwardConnectorConfig:
    sample_step_m: float = 0.10
    endpoint_tolerance_m: float = 1.0e-5
    polygon_tolerance_m: float = 1.0e-7

    def validate(self) -> None:
        if not math.isfinite(self.sample_step_m) or self.sample_step_m <= 0.0:
            raise ValueError("sample_step_m must be finite and > 0")
        if self.endpoint_tolerance_m < 0.0:
            raise ValueError("endpoint_tolerance_m must be >= 0")
        if self.polygon_tolerance_m < 0.0:
            raise ValueError("polygon_tolerance_m must be >= 0")


@dataclass(frozen=True)
class ForwardConnectorSample:
    x: float
    y: float
    z: float
    yaw: float
    motion_direction: str = "FORWARD"


@dataclass(frozen=True)
class ForwardConnectorResult:
    connector_id: str
    from_aisle_id: str
    to_aisle_id: str
    turn_zone_id: str
    status: str
    backend: str
    path_type: str | None
    length_m: float | None
    minimum_turning_radius_m: float
    sample_step_m: float
    samples: tuple[ForwardConnectorSample, ...]
    inside_turn_zone_fraction: float
    candidate_count: int
    rejected_candidate_count: int
    reason: str = ""


@dataclass(frozen=True)
class ForwardConnectorPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    connectors: tuple[ForwardConnectorResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = FORWARD_CONNECTOR_SCHEMA
    status: str = "DRAFT"

    @property
    def accepted_count(self) -> int:
        return sum(item.status == "ACCEPTED_CENTERLINE" for item in self.connectors)

    @property
    def rejected_count(self) -> int:
        return len(self.connectors) - self.accepted_count


def _mod2pi(angle: float) -> float:
    return float(angle % _TWO_PI)


def _wrap_pi(angle: float) -> float:
    return float((angle + math.pi) % _TWO_PI - math.pi)


def _dubins_candidates(
    start_pose: tuple[float, float, float, float],
    goal_pose: tuple[float, float, float, float],
    radius_m: float,
) -> list[tuple[str, tuple[float, float, float]]]:
    """Return feasible normalized Dubins candidates sorted by path length."""
    x0, y0, _, yaw0 = start_pose
    x1, y1, _, yaw1 = goal_pose
    dx = float(x1 - x0)
    dy = float(y1 - y0)
    distance = math.hypot(dx, dy)
    d = distance / radius_m
    theta = math.atan2(dy, dx) if distance > 1.0e-15 else 0.0
    alpha = _mod2pi(yaw0 - theta)
    beta = _mod2pi(yaw1 - theta)

    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    candidates: list[tuple[str, tuple[float, float, float]]] = []

    # LSL
    tmp0 = d + sa - sb
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp1 = math.atan2(cb - ca, tmp0)
        candidates.append(("LSL", (_mod2pi(-alpha + tmp1), p, _mod2pi(beta - tmp1))))

    # RSR
    tmp0 = d - sa + sb
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (-sa + sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp1 = math.atan2(ca - cb, tmp0)
        candidates.append(("RSR", (_mod2pi(alpha - tmp1), p, _mod2pi(-beta + tmp1))))

    # LSR
    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        candidates.append(("LSR", (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp))))

    # RSL
    p2 = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        candidates.append(("RSL", (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))))

    # RLR
    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0 + 1.0e-12:
        tmp = min(1.0, max(-1.0, tmp))
        p = _mod2pi(_TWO_PI - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + 0.5 * p)
        q = _mod2pi(alpha - beta - t + p)
        candidates.append(("RLR", (t, p, q)))

    # LRL
    tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0 + 1.0e-12:
        tmp = min(1.0, max(-1.0, tmp))
        p = _mod2pi(_TWO_PI - math.acos(tmp))
        t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + 0.5 * p)
        q = _mod2pi(beta - alpha - t + p)
        candidates.append(("LRL", (t, p, q)))

    candidates.sort(key=lambda item: (sum(item[1]), item[0]))
    return candidates


def _advance(
    x: float,
    y: float,
    yaw: float,
    segment: str,
    distance_m: float,
    radius_m: float,
) -> tuple[float, float, float]:
    if segment == "S":
        return (
            float(x + distance_m * math.cos(yaw)),
            float(y + distance_m * math.sin(yaw)),
            float(yaw),
        )
    curvature = (1.0 if segment == "L" else -1.0) / radius_m
    next_yaw = yaw + curvature * distance_m
    next_x = x + (math.sin(next_yaw) - math.sin(yaw)) / curvature
    next_y = y + (-math.cos(next_yaw) + math.cos(yaw)) / curvature
    return float(next_x), float(next_y), float(next_yaw)


def _sample_candidate(
    start_pose: tuple[float, float, float, float],
    goal_pose: tuple[float, float, float, float],
    path_type: str,
    normalized_lengths: tuple[float, float, float],
    radius_m: float,
    sample_step_m: float,
) -> tuple[tuple[ForwardConnectorSample, ...], float]:
    x, y, z0, yaw = (float(v) for v in start_pose)
    z1 = float(goal_pose[2])
    total_length = float(sum(normalized_lengths) * radius_m)
    samples_raw: list[tuple[float, float, float, float]] = [(x, y, yaw, 0.0)]
    cumulative = 0.0

    for segment, normalized in zip(path_type, normalized_lengths):
        segment_length = float(normalized * radius_m)
        remaining = segment_length
        while remaining > 1.0e-12:
            ds = min(sample_step_m, remaining)
            x, y, yaw = _advance(x, y, yaw, segment, ds, radius_m)
            cumulative += ds
            samples_raw.append((x, y, yaw, cumulative))
            remaining -= ds

    # Preserve exact endpoint identity even after accumulated floating-point integration.
    gx, gy, _, gyaw = (float(v) for v in goal_pose)
    if samples_raw:
        samples_raw[-1] = (gx, gy, gyaw, total_length)

    samples: list[ForwardConnectorSample] = []
    for sx, sy, syaw, distance_along in samples_raw:
        ratio = 0.0 if total_length <= 1.0e-12 else min(1.0, max(0.0, distance_along / total_length))
        z = z0 + ratio * (z1 - z0)
        samples.append(
            ForwardConnectorSample(
                x=float(sx),
                y=float(sy),
                z=float(z),
                yaw=_wrap_pi(float(syaw)),
            )
        )
    return tuple(samples), total_length


def _point_on_segment(px: float, py: float, a, b, tolerance: float) -> bool:
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    length2 = abx * abx + aby * aby
    if length2 <= tolerance * tolerance:
        return math.hypot(apx, apy) <= tolerance
    cross = abs(abx * apy - aby * apx)
    if cross > tolerance * max(1.0, math.sqrt(length2)):
        return False
    dot = apx * abx + apy * aby
    return -tolerance <= dot <= length2 + tolerance


def _point_inside_polygon(px: float, py: float, polygon_xy, tolerance: float) -> bool:
    polygon = tuple((float(x), float(y)) for x, y in polygon_xy)
    if len(polygon) < 3:
        return False
    for index, a in enumerate(polygon):
        b = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(px, py, a, b, tolerance):
            return True
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def _inside_fraction(samples: Iterable[ForwardConnectorSample], zone: TurnZone, tolerance: float) -> float:
    sample_list = tuple(samples)
    if not sample_list:
        return 0.0
    inside = sum(
        _point_inside_polygon(sample.x, sample.y, zone.polygon_xy, tolerance)
        for sample in sample_list
    )
    return float(inside / len(sample_list))


def _zone_by_id(zones: TurnZoneSet) -> dict[str, TurnZone]:
    mapping: dict[str, TurnZone] = {}
    for zone in zones.zones:
        if zone.zone_id in mapping:
            raise ValueError(f"duplicate turn zone id: {zone.zone_id}")
        mapping[zone.zone_id] = zone
    return mapping


def derive_forward_connector_plan(
    connector_requests: Iterable[ConnectorRequest],
    zones: TurnZoneSet,
    vehicle: CanonicalVehicleProfile,
    config: ForwardConnectorConfig | None = None,
    *,
    source: Mapping[str, Any] | None = None,
) -> ForwardConnectorPlan:
    """Generate forward-only Dubins centerline candidates for connector requests."""
    cfg = config or ForwardConnectorConfig()
    cfg.validate()
    requests = tuple(connector_requests)
    if vehicle.kinematics != "ackermann":
        raise ValueError("R5 forward Dubins backend currently requires Ackermann kinematics")
    if not vehicle.planning_preview_ready:
        raise ValueError(f"vehicle profile {vehicle.profile_id} is not ready for planning preview")
    radius = float(vehicle.minimum_turning_radius_m)
    if not vehicle.minimum_turning_radius_verified or radius <= 0.0:
        raise ValueError("forward Dubins planning requires verified minimum turning radius")

    zone_map = _zone_by_id(zones)
    results: list[ForwardConnectorResult] = []
    for request in requests:
        zone = zone_map.get(request.turn_zone_id)
        if zone is None:
            results.append(
                ForwardConnectorResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_NOT_FOUND",
                    backend="ANALYTIC_DUBINS_FORWARD_ONLY",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    sample_step_m=cfg.sample_step_m,
                    samples=(),
                    inside_turn_zone_fraction=0.0,
                    candidate_count=0,
                    rejected_candidate_count=0,
                    reason="requested turn zone does not exist",
                )
            )
            continue
        if not zone.allow_turn:
            results.append(
                ForwardConnectorResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_DISABLED_BY_ZONE_POLICY",
                    backend="ANALYTIC_DUBINS_FORWARD_ONLY",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    sample_step_m=cfg.sample_step_m,
                    samples=(),
                    inside_turn_zone_fraction=0.0,
                    candidate_count=0,
                    rejected_candidate_count=0,
                    reason="turn permission is disabled for requested zone",
                )
            )
            continue
        if zone.side != request.side:
            results.append(
                ForwardConnectorResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="TURN_ZONE_SIDE_MISMATCH",
                    backend="ANALYTIC_DUBINS_FORWARD_ONLY",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    sample_step_m=cfg.sample_step_m,
                    samples=(),
                    inside_turn_zone_fraction=0.0,
                    candidate_count=0,
                    rejected_candidate_count=0,
                    reason=f"request side={request.side}, zone side={zone.side}",
                )
            )
            continue

        candidates = _dubins_candidates(request.start_pose, request.goal_pose, radius)
        best_rejected_fraction = 0.0
        rejected_count = 0
        accepted = None
        for path_type, normalized_lengths in candidates:
            samples, length_m = _sample_candidate(
                request.start_pose,
                request.goal_pose,
                path_type,
                normalized_lengths,
                radius,
                cfg.sample_step_m,
            )
            fraction = _inside_fraction(samples, zone, cfg.polygon_tolerance_m)
            if fraction >= 1.0 - 1.0e-12:
                accepted = (path_type, samples, length_m, fraction)
                break
            rejected_count += 1
            best_rejected_fraction = max(best_rejected_fraction, fraction)

        if accepted is None:
            results.append(
                ForwardConnectorResult(
                    connector_id=request.connector_id,
                    from_aisle_id=request.from_aisle_id,
                    to_aisle_id=request.to_aisle_id,
                    turn_zone_id=request.turn_zone_id,
                    status="NO_FORWARD_DUBINS_IN_TURN_ZONE",
                    backend="ANALYTIC_DUBINS_FORWARD_ONLY",
                    path_type=None,
                    length_m=None,
                    minimum_turning_radius_m=radius,
                    sample_step_m=cfg.sample_step_m,
                    samples=(),
                    inside_turn_zone_fraction=best_rejected_fraction,
                    candidate_count=len(candidates),
                    rejected_candidate_count=rejected_count,
                    reason="all forward-only Dubins candidates leave the requested Turn Zone",
                )
            )
            continue

        path_type, samples, length_m, fraction = accepted
        results.append(
            ForwardConnectorResult(
                connector_id=request.connector_id,
                from_aisle_id=request.from_aisle_id,
                to_aisle_id=request.to_aisle_id,
                turn_zone_id=request.turn_zone_id,
                status="ACCEPTED_CENTERLINE",
                backend="ANALYTIC_DUBINS_FORWARD_ONLY",
                path_type=path_type,
                length_m=float(length_m),
                minimum_turning_radius_m=radius,
                sample_step_m=cfg.sample_step_m,
                samples=samples,
                inside_turn_zone_fraction=float(fraction),
                candidate_count=len(candidates),
                rejected_candidate_count=rejected_count,
                reason="centerline satisfies forward Dubins curvature and Turn Zone envelope",
            )
        )

    merged_source = dict(zones.source)
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "connector_backend": "ANALYTIC_DUBINS_FORWARD_ONLY",
            "minimum_turning_radius_m": radius,
            "validation_scope": "CENTERLINE_KINEMATICS_AND_TURN_ZONE_ONLY",
        }
    )
    return ForwardConnectorPlan(
        frame_id=zones.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        connectors=tuple(results),
        source=merged_source,
    )


def forward_connector_plan_to_dict(plan: ForwardConnectorPlan) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "source": dict(plan.source),
        "accepted_count": plan.accepted_count,
        "rejected_count": plan.rejected_count,
        "connector_count": len(plan.connectors),
        "connectors": [
            {
                "connector_id": item.connector_id,
                "from_aisle_id": item.from_aisle_id,
                "to_aisle_id": item.to_aisle_id,
                "turn_zone_id": item.turn_zone_id,
                "status": item.status,
                "backend": item.backend,
                "path_type": item.path_type,
                "length_m": item.length_m,
                "minimum_turning_radius_m": item.minimum_turning_radius_m,
                "sample_step_m": item.sample_step_m,
                "inside_turn_zone_fraction": item.inside_turn_zone_fraction,
                "candidate_count": item.candidate_count,
                "rejected_candidate_count": item.rejected_candidate_count,
                "reason": item.reason,
                "motion_direction": "FORWARD",
                "validation_scope": "CENTERLINE_KINEMATICS_AND_TURN_ZONE_ONLY",
                "samples": [
                    {
                        "x": sample.x,
                        "y": sample.y,
                        "z": sample.z,
                        "yaw": sample.yaw,
                        "motion_direction": sample.motion_direction,
                    }
                    for sample in item.samples
                ],
            }
            for item in plan.connectors
        ],
    }
