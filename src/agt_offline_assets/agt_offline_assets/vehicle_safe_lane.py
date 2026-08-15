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

V25-12G-A1 moves the sample-level feasibility evaluation into
``vehicle_lane_feasibility``.  This module remains the stable longest-run
reducer so existing VehicleSafeLanePlan behavior and serialization do not
change.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from .agricultural_aisle_graph import AislePrimitive, AgriculturalAisleGraph
from .navigation_grid import NavigationGridEvidence
from .site_boundary import SiteBoundary
from .vehicle_lane_feasibility import (
    VehicleSafeLaneConfig,
    derive_vehicle_lane_feasibility_trace,
    lateral_offset_candidates as _offset_candidates,
    normalize_row_direction as _normalize,
    resample_aisle_polyline as _resample_polyline,
)
from .vehicle_profile import CanonicalVehicleProfile


VEHICLE_SAFE_LANE_SCHEMA = "agt_vehicle_safe_aisle_lane/v1"


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


def _contiguous_feasible_runs(
    selected_points: tuple[tuple[float, float, float] | None, ...],
) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, point in enumerate(selected_points):
        if point is not None and start is None:
            start = index
        if point is None and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(selected_points) - 1))
    return runs


def _derive_one_lane(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    direction_xy: tuple[float, float],
    cfg: VehicleSafeLaneConfig,
    site_boundary: SiteBoundary | None,
) -> VehicleSafeLane:
    trace = derive_vehicle_lane_feasibility_trace(
        aisle,
        navigation,
        vehicle,
        cfg,
        row_direction_xy=direction_xy,
        site_boundary=site_boundary,
    )
    total = float(trace.structural_length_m)
    selected = trace.selected_points
    selected_offsets = trace.selected_offsets_m
    distances = trace.distances_m

    if trace.structural_width_blocked_reason is not None:
        return _empty_lane(
            aisle,
            structural_length_m=total,
            allowed_lateral_shift_m=trace.allowed_lateral_shift_m,
            total_sample_count=len(selected),
            reason=trace.structural_width_blocked_reason,
            site_boundary_rejected_pose_count=trace.site_boundary_rejected_pose_count,
            site_boundary_limited_sample_count=trace.site_boundary_limited_sample_count,
            grid_rejected_pose_count=trace.grid_rejected_pose_count,
        )

    segments = _contiguous_feasible_runs(selected)
    if not segments:
        reasons = []
        if trace.site_boundary_limited_sample_count > 0:
            reasons.append(
                "SITE_BOUNDARY_CONFLICT: boundary removes all grid-free candidates at "
                f"{trace.site_boundary_limited_sample_count}/{len(selected)} sampled aisle stations"
            )
        if trace.grid_rejected_pose_count > 0:
            reasons.append(
                "no preview-footprint-free pose was found in the Navigation Grid near the structural aisle centerline"
            )
        return _empty_lane(
            aisle,
            structural_length_m=total,
            allowed_lateral_shift_m=trace.allowed_lateral_shift_m,
            total_sample_count=len(selected),
            reason="; ".join(reasons)
            or "no preview-footprint-free pose was found near the structural aisle centerline",
            site_boundary_rejected_pose_count=trace.site_boundary_rejected_pose_count,
            site_boundary_limited_sample_count=trace.site_boundary_limited_sample_count,
            grid_rejected_pose_count=trace.grid_rejected_pose_count,
        )

    def segment_span(segment: tuple[int, int]) -> float:
        start_index, end_index = segment
        return float(distances[end_index] - distances[start_index])

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
    maximum_used_shift = max((abs(value) for value in lane_offsets), default=0.0)
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
        allowed_lateral_shift_m=trace.allowed_lateral_shift_m,
        maximum_used_lateral_shift_m=maximum_used_shift,
        safe_sample_count=safe_count,
        total_sample_count=len(selected),
        centerline_xyz=lane_points,
        lateral_offsets_m=lane_offsets,
        reason=reason,
        site_boundary_rejected_pose_count=trace.site_boundary_rejected_pose_count,
        site_boundary_limited_sample_count=trace.site_boundary_limited_sample_count,
        grid_rejected_pose_count=trace.grid_rejected_pose_count,
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
    direction_xy = (float(direction[0]), float(direction[1]))
    lanes = tuple(
        _derive_one_lane(
            aisle,
            navigation,
            vehicle,
            direction_xy,
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
        row_direction_xy=direction_xy,
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
