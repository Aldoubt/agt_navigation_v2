"""Vehicle-pose-free aisle lanes for V25-12E route production.

The Agricultural Aisle Graph is structural corridor evidence. Its centerline is
not required to be a configuration-space FREE trajectory for a particular
vehicle. This layer keeps that graph immutable and derives a vehicle-specific
lane near each structural centerline using the frozen Navigation Grid and the
canonical preview footprint.

V25-12F optionally adds one independent hard invariant: when a Site Boundary is
provided, every continuous preview footprint must stay strictly inside the
vehicle-permitted inner perimeter. This does not weaken the existing Navigation
Grid FREE requirement.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .forward_connector import ForwardConnectorSample
from .forward_connector_navigation_gate import (
    _preview_local_footprint,
    _transform_polygon,
)
from .navigation_grid import NavigationGridEvidence
from .reverse_primitive_connector import _preview_pose_free
from .site_boundary import SiteBoundary, polygon_strictly_inside_site_boundary
from .vehicle_profile import CanonicalVehicleProfile


VEHICLE_SAFE_LANE_SCHEMA = "agt_vehicle_safe_aisle_lane/v1"


@dataclass(frozen=True)
class VehicleSafeLaneConfig:
    sample_spacing_m: float = 0.10
    lateral_search_step_m: float = 0.05
    maximum_lateral_shift_m: float = 0.50
    maximum_lateral_step_m: float = 0.15
    preview_footprint_padding_m: float = 0.05
    minimum_lane_coverage_fraction: float = 0.70
    maximum_endpoint_retreat_m: float = 2.00
    minimum_contiguous_span_m: float = 1.00

    def validate(self) -> None:
        for name in (
            "sample_spacing_m",
            "lateral_search_step_m",
            "maximum_lateral_shift_m",
            "maximum_lateral_step_m",
            "maximum_endpoint_retreat_m",
            "minimum_contiguous_span_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if (
            not math.isfinite(self.preview_footprint_padding_m)
            or self.preview_footprint_padding_m < 0.0
        ):
            raise ValueError("preview_footprint_padding_m must be finite and >= 0")
        if not 0.0 <= self.minimum_lane_coverage_fraction <= 1.0:
            raise ValueError("minimum_lane_coverage_fraction must be in [0, 1]")


@dataclass(frozen=True)
class VehicleSafeLane:
    aisle_id: str
    status: str
    structural_length_m: float
    selected_span_m: float
    coverage_fraction: float
    low_u_retreat_m: float | None
    high_u_retreat_m: float | None
    allowed_lateral_shift_m: float
    maximum_used_lateral_shift_m: float
    safe_sample_count: int
    total_sample_count: int
    centerline_xyz: tuple[tuple[float, float, float], ...]
    lateral_offsets_m: tuple[float, ...]
    reason: str
    site_boundary_rejected_pose_count: int = 0
    site_boundary_limited_sample_count: int = 0
    grid_rejected_pose_count: int = 0

    @property
    def low_u_pose(self) -> tuple[float, float, float] | None:
        return None if not self.centerline_xyz else self.centerline_xyz[0]

    @property
    def high_u_pose(self) -> tuple[float, float, float] | None:
        return None if not self.centerline_xyz else self.centerline_xyz[-1]


@dataclass(frozen=True)
class VehicleSafeLanePlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    row_direction_xy: tuple[float, float]
    lanes: tuple[VehicleSafeLane, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_SAFE_LANE_SCHEMA
    status: str = "DRAFT"

    @property
    def ready_count(self) -> int:
        return sum(lane.status == "VEHICLE_SAFE_LANE_READY" for lane in self.lanes)

    @property
    def partial_count(self) -> int:
        return sum(lane.status == "VEHICLE_SAFE_LANE_PARTIAL" for lane in self.lanes)

    @property
    def unavailable_count(self) -> int:
        return sum(lane.status == "NO_VEHICLE_SAFE_LANE" for lane in self.lanes)


def _normalize(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("row direction must be finite and non-zero")
    return direction / norm


def _resample_polyline(
    aisle: AislePrimitive,
    spacing_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(aisle.centerline_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 3:
        raise ValueError(f"aisle {aisle.aisle_id} centerline is invalid")
    segment_lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 1.0e-9:
        return points[:1].copy(), np.array([0.0], dtype=np.float64)
    count = max(2, int(math.ceil(total / spacing_m)) + 1)
    distances = np.linspace(0.0, total, count)
    output = np.empty((count, 3), dtype=np.float64)
    for out_index, distance in enumerate(distances):
        index = int(np.searchsorted(cumulative, distance, side="right") - 1)
        index = min(max(index, 0), len(cumulative) - 2)
        span = float(cumulative[index + 1] - cumulative[index])
        ratio = (
            0.0
            if span <= 1.0e-12
            else (float(distance) - float(cumulative[index])) / span
        )
        output[out_index] = points[index] + ratio * (
            points[index + 1] - points[index]
        )
    return output, distances


def _offset_candidates(maximum_shift_m: float, step_m: float) -> tuple[float, ...]:
    if maximum_shift_m <= 1.0e-12:
        return (0.0,)
    count = int(math.floor(maximum_shift_m / step_m + 1.0e-9))
    values = [0.0]
    for index in range(1, count + 1):
        value = index * step_m
        values.extend((-value, value))
    if count * step_m < maximum_shift_m - 1.0e-9:
        values.extend((-maximum_shift_m, maximum_shift_m))
    return tuple(
        sorted(set(float(v) for v in values), key=lambda v: (abs(v), v))
    )


def _empty_lane(
    aisle: AislePrimitive,
    *,
    structural_length_m: float,
    allowed_lateral_shift_m: float,
    total_sample_count: int,
    reason: str,
    site_boundary_rejected_pose_count: int = 0,
    site_boundary_limited_sample_count: int = 0,
    grid_rejected_pose_count: int = 0,
) -> VehicleSafeLane:
    return VehicleSafeLane(
        aisle_id=aisle.aisle_id,
        status="NO_VEHICLE_SAFE_LANE",
        structural_length_m=structural_length_m,
        selected_span_m=0.0,
        coverage_fraction=0.0,
        low_u_retreat_m=None,
        high_u_retreat_m=None,
        allowed_lateral_shift_m=allowed_lateral_shift_m,
        maximum_used_lateral_shift_m=0.0,
        safe_sample_count=0,
        total_sample_count=total_sample_count,
        centerline_xyz=(),
        lateral_offsets_m=(),
        reason=reason,
        site_boundary_rejected_pose_count=int(site_boundary_rejected_pose_count),
        site_boundary_limited_sample_count=int(site_boundary_limited_sample_count),
        grid_rejected_pose_count=int(grid_rejected_pose_count),
    )


def _pose_inside_site_boundary(
    x: float,
    y: float,
    yaw: float,
    local_footprint,
    site_boundary: SiteBoundary | None,
) -> bool:
    if site_boundary is None:
        return True
    sample = ForwardConnectorSample(x=float(x), y=float(y), z=0.0, yaw=float(yaw))
    polygon = _transform_polygon(local_footprint, sample)
    return polygon_strictly_inside_site_boundary(site_boundary, polygon)


def _derive_one_lane(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    direction: np.ndarray,
    perpendicular: np.ndarray,
    local_footprint,
    cfg: VehicleSafeLaneConfig,
    site_boundary: SiteBoundary | None,
) -> VehicleSafeLane:
    samples, distances = _resample_polyline(aisle, cfg.sample_spacing_m)
    total = float(distances[-1]) if distances.size else 0.0
    required_lateral_width = float(
        vehicle.navigation_width_m + 2.0 * cfg.preview_footprint_padding_m
    )

    if float(aisle.geometric_width_m) + 1.0e-9 < required_lateral_width:
        return _empty_lane(
            aisle,
            structural_length_m=total,
            allowed_lateral_shift_m=0.0,
            total_sample_count=int(samples.shape[0]),
            reason=(
                "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH: "
                f"aisle={aisle.geometric_width_m:.3f} m "
                f"required={required_lateral_width:.3f} m"
            ),
        )

    width_surplus = float(aisle.geometric_width_m) - required_lateral_width
    allowed_shift = min(float(cfg.maximum_lateral_shift_m), 0.5 * width_surplus)
    offsets = _offset_candidates(allowed_shift, cfg.lateral_search_step_m)
    yaw = math.atan2(float(direction[1]), float(direction[0]))

    selected: list[tuple[float, float, float] | None] = []
    selected_offsets: list[float | None] = []
    previous_offset: float | None = None
    boundary_rejections = 0
    boundary_limited_samples = 0
    grid_rejections = 0

    for sample in samples:
        feasible: list[tuple[float, float, tuple[float, float, float]]] = []
        grid_free_ignoring_boundary = False
        for offset in offsets:
            if (
                previous_offset is not None
                and abs(float(offset) - previous_offset)
                > cfg.maximum_lateral_step_m + 1.0e-9
            ):
                continue
            x = float(sample[0] + offset * perpendicular[0])
            y = float(sample[1] + offset * perpendicular[1])
            grid_free = _preview_pose_free(
                x,
                y,
                yaw,
                navigation,
                local_footprint,
            )
            if grid_free:
                grid_free_ignoring_boundary = True
            inside_boundary = _pose_inside_site_boundary(
                x,
                y,
                yaw,
                local_footprint,
                site_boundary,
            )
            if not inside_boundary:
                boundary_rejections += 1
                continue
            if not grid_free:
                grid_rejections += 1
                continue
            continuity = (
                0.0
                if previous_offset is None
                else abs(float(offset) - previous_offset)
            )
            score = abs(float(offset)) + 0.5 * continuity
            feasible.append((score, float(offset), (x, y, float(sample[2]))))

        if not feasible:
            if site_boundary is not None and grid_free_ignoring_boundary:
                boundary_limited_samples += 1
            selected.append(None)
            selected_offsets.append(None)
            previous_offset = None
            continue

        feasible.sort(key=lambda item: (item[0], abs(item[1]), item[1]))
        _, chosen_offset, point = feasible[0]
        selected.append(point)
        selected_offsets.append(chosen_offset)
        previous_offset = chosen_offset

    segments: list[tuple[int, int]] = []
    start: int | None = None
    for index, point in enumerate(selected):
        if point is not None and start is None:
            start = index
        if point is None and start is not None:
            segments.append((start, index - 1))
            start = None
    if start is not None:
        segments.append((start, len(selected) - 1))

    if not segments:
        reasons = []
        if boundary_limited_samples > 0:
            reasons.append(
                "SITE_BOUNDARY_CONFLICT: boundary removes all grid-free candidates at "
                f"{boundary_limited_samples}/{len(selected)} sampled aisle stations"
            )
        if grid_rejections > 0:
            reasons.append(
                "no preview-footprint-free pose was found in the Navigation Grid near the structural aisle centerline"
            )
        return _empty_lane(
            aisle,
            structural_length_m=total,
            allowed_lateral_shift_m=allowed_shift,
            total_sample_count=len(selected),
            reason="; ".join(reasons)
            or "no preview-footprint-free pose was found near the structural aisle centerline",
            site_boundary_rejected_pose_count=boundary_rejections,
            site_boundary_limited_sample_count=boundary_limited_samples,
            grid_rejected_pose_count=grid_rejections,
        )

    def segment_span(segment: tuple[int, int]) -> float:
        a, b = segment
        return float(distances[b] - distances[a])

    best_start, best_end = max(
        segments,
        key=lambda segment: (
            segment_span(segment),
            segment[1] - segment[0],
            -segment[0],
        ),
    )
    span = segment_span((best_start, best_end))
    low_retreat = float(distances[best_start])
    high_retreat = float(total - distances[best_end])
    coverage = 0.0 if total <= 1.0e-12 else float(span / total)
    lane_points = tuple(
        selected[index]
        for index in range(best_start, best_end + 1)
        if selected[index] is not None
    )
    lane_offsets = tuple(
        float(selected_offsets[index])
        for index in range(best_start, best_end + 1)
        if selected_offsets[index] is not None
    )
    maximum_used_shift = max((abs(v) for v in lane_offsets), default=0.0)
    safe_count = sum(point is not None for point in selected)

    endpoint_ok = (
        low_retreat <= cfg.maximum_endpoint_retreat_m + 1.0e-9
        and high_retreat <= cfg.maximum_endpoint_retreat_m + 1.0e-9
    )
    coverage_ok = coverage + 1.0e-9 >= cfg.minimum_lane_coverage_fraction
    span_ok = span + 1.0e-9 >= cfg.minimum_contiguous_span_m
    if endpoint_ok and coverage_ok and span_ok:
        status = "VEHICLE_SAFE_LANE_READY"
        reason = (
            "a continuous preview-footprint-free lane spans the structural aisle "
            "with bounded endpoint retreat"
        )
    else:
        status = "VEHICLE_SAFE_LANE_PARTIAL"
        reasons = []
        if not endpoint_ok:
            reasons.append("endpoint retreat exceeds configured bound")
        if not coverage_ok:
            reasons.append("continuous lane coverage fraction is below threshold")
        if not span_ok:
            reasons.append("continuous lane span is below threshold")
        reason = "; ".join(reasons) or "vehicle-safe lane is incomplete"

    return VehicleSafeLane(
        aisle_id=aisle.aisle_id,
        status=status,
        structural_length_m=total,
        selected_span_m=span,
        coverage_fraction=coverage,
        low_u_retreat_m=low_retreat,
        high_u_retreat_m=high_retreat,
        allowed_lateral_shift_m=allowed_shift,
        maximum_used_lateral_shift_m=maximum_used_shift,
        safe_sample_count=safe_count,
        total_sample_count=len(selected),
        centerline_xyz=lane_points,
        lateral_offsets_m=lane_offsets,
        reason=reason,
        site_boundary_rejected_pose_count=boundary_rejections,
        site_boundary_limited_sample_count=boundary_limited_samples,
        grid_rejected_pose_count=grid_rejections,
    )


def derive_vehicle_safe_lane_plan(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> VehicleSafeLanePlan:
    """Derive vehicle-pose-free lanes without mutating structural map evidence."""
    cfg = config or VehicleSafeLaneConfig()
    cfg.validate()
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=graph.frame_id)
    if not vehicle.planning_preview_ready:
        raise ValueError(
            f"vehicle profile {vehicle.profile_id} is not ready for planning preview"
        )

    direction = _normalize(graph.row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    local_footprint = _preview_local_footprint(
        vehicle,
        cfg.preview_footprint_padding_m,
    )
    lanes = tuple(
        _derive_one_lane(
            aisle,
            navigation,
            vehicle,
            direction,
            perpendicular,
            local_footprint,
            cfg,
            site_boundary,
        )
        for aisle in graph.aisles
    )
    merged_source = dict(graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "lane_policy": "BOUNDED_LATERAL_SEARCH_NEAR_IMMUTABLE_STRUCTURAL_AISLE",
            "vehicle_navigation_width_m": float(vehicle.navigation_width_m),
            "preview_footprint_padding_m": float(cfg.preview_footprint_padding_m),
            "validation_scope": "PREVIEW_ONLY_NOT_R8_VEHICLE_READY",
            "structural_width_gate_preserved": True,
            "structural_aisle_graph_mutated": False,
            "navigation_occupancy_mutated": False,
            "site_boundary_enforced": site_boundary is not None,
        }
    )
    return VehicleSafeLanePlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        row_direction_xy=(float(direction[0]), float(direction[1])),
        lanes=lanes,
        source=merged_source,
    )


def vehicle_safe_lane_plan_to_dict(plan: VehicleSafeLanePlan) -> dict[str, Any]:
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "row_direction_xy": [plan.row_direction_xy[0], plan.row_direction_xy[1]],
        "source": dict(plan.source),
        "aisle_count": len(plan.lanes),
        "summary": {
            "ready": plan.ready_count,
            "partial": plan.partial_count,
            "unavailable": plan.unavailable_count,
        },
        "aisles": [
            {
                "aisle_id": lane.aisle_id,
                "status": lane.status,
                "structural_length_m": lane.structural_length_m,
                "selected_span_m": lane.selected_span_m,
                "coverage_fraction": lane.coverage_fraction,
                "low_u_retreat_m": lane.low_u_retreat_m,
                "high_u_retreat_m": lane.high_u_retreat_m,
                "allowed_lateral_shift_m": lane.allowed_lateral_shift_m,
                "maximum_used_lateral_shift_m": lane.maximum_used_lateral_shift_m,
                "safe_sample_count": lane.safe_sample_count,
                "total_sample_count": lane.total_sample_count,
                "site_boundary_rejected_pose_count": lane.site_boundary_rejected_pose_count,
                "site_boundary_limited_sample_count": lane.site_boundary_limited_sample_count,
                "grid_rejected_pose_count": lane.grid_rejected_pose_count,
                "reason": lane.reason,
                "centerline_xyz": [list(point) for point in lane.centerline_xyz],
                "lateral_offsets_m": list(lane.lateral_offsets_m),
            }
            for lane in plan.lanes
        ],
    }


def write_vehicle_safe_lane_plan(plan: VehicleSafeLanePlan, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            vehicle_safe_lane_plan_to_dict(plan),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return output
