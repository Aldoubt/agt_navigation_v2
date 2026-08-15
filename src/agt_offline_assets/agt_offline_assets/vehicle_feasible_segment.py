"""Vehicle-feasible aisle segment extraction for V25-12G-A1.

This layer consumes the shared sample-level vehicle lane feasibility trace and
preserves every maximal contiguous feasible run that already satisfies the
minimum useful span. Rejected short fragments remain diagnostic-only, while
endpoint classification marks only outer active-segment ends as headland
candidates when their retreat stays inside the frozen configured bound.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping

from .agricultural_aisle_graph import AgriculturalAisleGraph
from .navigation_grid import NavigationGridEvidence
from .site_boundary import SiteBoundary
from .vehicle_lane_feasibility import (
    VehicleSafeLaneConfig,
    derive_vehicle_lane_feasibility_trace,
    normalize_row_direction,
)
from .vehicle_profile import CanonicalVehicleProfile


VEHICLE_FEASIBLE_SEGMENT_SCHEMA = "agt_vehicle_feasible_segment_plan/v1"

LOW_U_HEADLAND = "LOW_U_HEADLAND"
HIGH_U_HEADLAND = "HIGH_U_HEADLAND"
INTERIOR_BLOCKED_END = "INTERIOR_BLOCKED_END"

ACTIVE_COVERAGE_SEGMENT = "ACTIVE_COVERAGE_SEGMENT"
BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH = "BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH"


@dataclass(frozen=True)
class VehicleFeasibleSegment:
    segment_id: str
    aisle_id: str
    ordinal_in_aisle: int
    start_distance_m: float
    end_distance_m: float
    length_m: float
    coverage_fraction_of_aisle: float
    low_endpoint_type: str
    high_endpoint_type: str
    centerline_xyz: tuple[tuple[float, float, float], ...]
    lateral_offsets_m: tuple[float, ...]
    maximum_used_lateral_shift_m: float
    low_endpoint_pose: tuple[float, float, float, float]
    high_endpoint_pose: tuple[float, float, float, float]
    status: str = ACTIVE_COVERAGE_SEGMENT
    reason: str = "contiguous preview-footprint-free vehicle segment"


@dataclass(frozen=True)
class RejectedFeasibleFragment:
    fragment_id: str
    aisle_id: str
    start_distance_m: float
    end_distance_m: float
    length_m: float
    reason: str = BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH


@dataclass(frozen=True)
class AisleFeasibleSegmentResult:
    aisle_id: str
    structural_length_m: float
    active_segments: tuple[VehicleFeasibleSegment, ...]
    rejected_fragments: tuple[RejectedFeasibleFragment, ...]
    raw_feasible_fragment_count: int
    allowed_lateral_shift_m: float
    site_boundary_rejected_pose_count: int
    site_boundary_limited_sample_count: int
    grid_rejected_pose_count: int
    reason: str


@dataclass(frozen=True)
class VehicleFeasibleSegmentPlan:
    frame_id: str
    platform_id: str
    platform_profile_sha256: str
    row_direction_xy: tuple[float, float]
    aisles: tuple[AisleFeasibleSegmentResult, ...]
    source: Mapping[str, Any] = field(default_factory=dict)
    schema: str = VEHICLE_FEASIBLE_SEGMENT_SCHEMA
    status: str = "DRAFT"


@dataclass(frozen=True)
class _RawRun:
    start_index: int
    end_index: int
    start_distance_m: float
    end_distance_m: float
    length_m: float


def _maximal_feasible_runs(trace) -> tuple[_RawRun, ...]:
    runs: list[_RawRun] = []
    start_index: int | None = None
    for index, point in enumerate(trace.selected_points):
        if point is not None and start_index is None:
            start_index = index
        if point is None and start_index is not None:
            end_index = index - 1
            start_distance = float(trace.distances_m[start_index])
            end_distance = float(trace.distances_m[end_index])
            runs.append(
                _RawRun(
                    start_index=start_index,
                    end_index=end_index,
                    start_distance_m=start_distance,
                    end_distance_m=end_distance,
                    length_m=float(end_distance - start_distance),
                )
            )
            start_index = None
    if start_index is not None:
        end_index = len(trace.selected_points) - 1
        start_distance = float(trace.distances_m[start_index])
        end_distance = float(trace.distances_m[end_index])
        runs.append(
            _RawRun(
                start_index=start_index,
                end_index=end_index,
                start_distance_m=start_distance,
                end_distance_m=end_distance,
                length_m=float(end_distance - start_distance),
            )
        )
    return tuple(runs)


def derive_vehicle_feasible_segment_plan(
    graph: AgriculturalAisleGraph,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneConfig | None = None,
    *,
    site_boundary: SiteBoundary | None = None,
    source: Mapping[str, Any] | None = None,
) -> VehicleFeasibleSegmentPlan:
    """Preserve every useful maximal contiguous vehicle-feasible aisle run."""
    cfg = config or VehicleSafeLaneConfig()
    direction = normalize_row_direction(graph.row_direction_xy)
    row_direction_xy = (float(direction[0]), float(direction[1]))
    yaw = math.atan2(row_direction_xy[1], row_direction_xy[0])

    aisle_results: list[AisleFeasibleSegmentResult] = []
    for aisle in graph.aisles:
        trace = derive_vehicle_lane_feasibility_trace(
            aisle,
            navigation,
            vehicle,
            cfg,
            row_direction_xy=row_direction_xy,
            site_boundary=site_boundary,
        )
        raw_runs = _maximal_feasible_runs(trace)
        active_raw = tuple(
            run
            for run in raw_runs
            if run.length_m + 1.0e-9 >= cfg.minimum_contiguous_span_m
        )
        active_segments: list[VehicleFeasibleSegment] = []
        rejected_fragments: list[RejectedFeasibleFragment] = []
        segment_ordinal = 0
        fragment_ordinal = 0

        for run in raw_runs:
            if run.length_m + 1.0e-9 < cfg.minimum_contiguous_span_m:
                fragment_ordinal += 1
                rejected_fragments.append(
                    RejectedFeasibleFragment(
                        fragment_id=f"{aisle.aisle_id}.fragment_{fragment_ordinal:03d}",
                        aisle_id=aisle.aisle_id,
                        start_distance_m=run.start_distance_m,
                        end_distance_m=run.end_distance_m,
                        length_m=run.length_m,
                    )
                )
                continue

            segment_ordinal += 1
            selected_points = tuple(
                trace.selected_points[index]
                for index in range(run.start_index, run.end_index + 1)
                if trace.selected_points[index] is not None
            )
            selected_offsets = tuple(
                float(trace.selected_offsets_m[index])
                for index in range(run.start_index, run.end_index + 1)
                if trace.selected_offsets_m[index] is not None
            )
            low_point = selected_points[0]
            high_point = selected_points[-1]
            structural_length = float(trace.structural_length_m)
            coverage_fraction = (
                0.0
                if structural_length <= 1.0e-12
                else float(run.length_m / structural_length)
            )
            is_first_active = bool(active_raw) and run == active_raw[0]
            is_last_active = bool(active_raw) and run == active_raw[-1]
            low_endpoint_type = (
                LOW_U_HEADLAND
                if is_first_active
                and run.start_distance_m <= cfg.maximum_endpoint_retreat_m + 1.0e-9
                else INTERIOR_BLOCKED_END
            )
            high_endpoint_type = (
                HIGH_U_HEADLAND
                if is_last_active
                and structural_length - run.end_distance_m
                <= cfg.maximum_endpoint_retreat_m + 1.0e-9
                else INTERIOR_BLOCKED_END
            )

            active_segments.append(
                VehicleFeasibleSegment(
                    segment_id=f"{aisle.aisle_id}.segment_{segment_ordinal:03d}",
                    aisle_id=aisle.aisle_id,
                    ordinal_in_aisle=segment_ordinal,
                    start_distance_m=run.start_distance_m,
                    end_distance_m=run.end_distance_m,
                    length_m=run.length_m,
                    coverage_fraction_of_aisle=coverage_fraction,
                    low_endpoint_type=low_endpoint_type,
                    high_endpoint_type=high_endpoint_type,
                    centerline_xyz=selected_points,
                    lateral_offsets_m=selected_offsets,
                    maximum_used_lateral_shift_m=max(
                        (abs(value) for value in selected_offsets),
                        default=0.0,
                    ),
                    low_endpoint_pose=(
                        float(low_point[0]),
                        float(low_point[1]),
                        float(low_point[2]),
                        float(yaw),
                    ),
                    high_endpoint_pose=(
                        float(high_point[0]),
                        float(high_point[1]),
                        float(high_point[2]),
                        float(yaw),
                    ),
                )
            )

        if trace.structural_width_blocked_reason is not None:
            reason = trace.structural_width_blocked_reason
        elif active_segments:
            reason = "vehicle-feasible segments extracted from maximal contiguous runs"
        elif raw_runs:
            reason = "all vehicle-feasible fragments are below minimum useful segment length"
        else:
            reason = "no useful preview-footprint-free vehicle segment found"

        aisle_results.append(
            AisleFeasibleSegmentResult(
                aisle_id=aisle.aisle_id,
                structural_length_m=float(trace.structural_length_m),
                active_segments=tuple(active_segments),
                rejected_fragments=tuple(rejected_fragments),
                raw_feasible_fragment_count=len(raw_runs),
                allowed_lateral_shift_m=float(trace.allowed_lateral_shift_m),
                site_boundary_rejected_pose_count=int(
                    trace.site_boundary_rejected_pose_count
                ),
                site_boundary_limited_sample_count=int(
                    trace.site_boundary_limited_sample_count
                ),
                grid_rejected_pose_count=int(trace.grid_rejected_pose_count),
                reason=reason,
            )
        )

    merged_source = dict(graph.source)
    merged_source.update(dict(navigation.source))
    merged_source.update(dict(source or {}))
    merged_source.update(
        {
            "segment_policy": "ALL_MAXIMAL_CONTIGUOUS_PREVIEW_FOOTPRINT_FREE_RUNS",
            "minimum_contiguous_span_m": float(cfg.minimum_contiguous_span_m),
            "site_boundary_enforced": site_boundary is not None,
            "structural_aisle_graph_mutated": False,
            "navigation_occupancy_mutated": False,
        }
    )

    return VehicleFeasibleSegmentPlan(
        frame_id=graph.frame_id,
        platform_id=vehicle.profile_id,
        platform_profile_sha256=vehicle.profile_sha256,
        row_direction_xy=row_direction_xy,
        aisles=tuple(aisle_results),
        source=merged_source,
    )
