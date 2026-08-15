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
from pathlib import Path
from typing import Any, Mapping

import yaml

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

_CONFIG_FIELDS = (
    "sample_spacing_m",
    "lateral_search_step_m",
    "maximum_lateral_shift_m",
    "maximum_lateral_step_m",
    "preview_footprint_padding_m",
    "minimum_lane_coverage_fraction",
    "maximum_endpoint_retreat_m",
    "minimum_contiguous_span_m",
)
_CONFIG_SOURCE_KEY = "vehicle_safe_lane_configuration"
_VALID_ENDPOINT_TYPES = {
    LOW_U_HEADLAND,
    HIGH_U_HEADLAND,
    INTERIOR_BLOCKED_END,
}


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


def _config_to_dict(config: VehicleSafeLaneConfig) -> dict[str, float]:
    return {name: float(getattr(config, name)) for name in _CONFIG_FIELDS}


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
    cfg.validate()
    if graph.frame_id != navigation.frame_id:
        raise ValueError("Aisle Graph and Navigation Grid frame_id must match")
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=graph.frame_id)
    if not vehicle.planning_preview_ready:
        raise ValueError(
            f"vehicle profile {vehicle.profile_id} is not ready for planning preview"
        )

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
            _CONFIG_SOURCE_KEY: _config_to_dict(cfg),
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


def _serialized_configuration(plan: VehicleFeasibleSegmentPlan) -> dict[str, float]:
    raw = plan.source.get(_CONFIG_SOURCE_KEY)
    if not isinstance(raw, Mapping):
        raise ValueError(
            "vehicle-feasible segment plan is missing serialized VehicleSafeLaneConfig"
        )
    missing = [name for name in _CONFIG_FIELDS if name not in raw]
    if missing:
        raise ValueError(
            "vehicle-feasible segment plan configuration missing fields: "
            + ", ".join(missing)
        )
    config = VehicleSafeLaneConfig(**{name: float(raw[name]) for name in _CONFIG_FIELDS})
    config.validate()
    return _config_to_dict(config)


def _segment_to_dict(segment: VehicleFeasibleSegment) -> dict[str, Any]:
    return {
        "segment_id": segment.segment_id,
        "aisle_id": segment.aisle_id,
        "ordinal_in_aisle": int(segment.ordinal_in_aisle),
        "start_distance_m": float(segment.start_distance_m),
        "end_distance_m": float(segment.end_distance_m),
        "length_m": float(segment.length_m),
        "coverage_fraction_of_aisle": float(segment.coverage_fraction_of_aisle),
        "low_endpoint_type": segment.low_endpoint_type,
        "high_endpoint_type": segment.high_endpoint_type,
        "centerline_xyz": [list(map(float, point)) for point in segment.centerline_xyz],
        "lateral_offsets_m": [float(value) for value in segment.lateral_offsets_m],
        "maximum_used_lateral_shift_m": float(segment.maximum_used_lateral_shift_m),
        "low_endpoint_pose": list(map(float, segment.low_endpoint_pose)),
        "high_endpoint_pose": list(map(float, segment.high_endpoint_pose)),
        "status": segment.status,
        "reason": segment.reason,
    }


def _fragment_to_dict(fragment: RejectedFeasibleFragment) -> dict[str, Any]:
    return {
        "fragment_id": fragment.fragment_id,
        "aisle_id": fragment.aisle_id,
        "start_distance_m": float(fragment.start_distance_m),
        "end_distance_m": float(fragment.end_distance_m),
        "length_m": float(fragment.length_m),
        "reason": fragment.reason,
    }


def _aisle_result_to_dict(aisle: AisleFeasibleSegmentResult) -> dict[str, Any]:
    return {
        "aisle_id": aisle.aisle_id,
        "structural_length_m": float(aisle.structural_length_m),
        "active_segments": [_segment_to_dict(item) for item in aisle.active_segments],
        "rejected_fragments": [
            _fragment_to_dict(item) for item in aisle.rejected_fragments
        ],
        "raw_feasible_fragment_count": int(aisle.raw_feasible_fragment_count),
        "allowed_lateral_shift_m": float(aisle.allowed_lateral_shift_m),
        "site_boundary_rejected_pose_count": int(
            aisle.site_boundary_rejected_pose_count
        ),
        "site_boundary_limited_sample_count": int(
            aisle.site_boundary_limited_sample_count
        ),
        "grid_rejected_pose_count": int(aisle.grid_rejected_pose_count),
        "reason": aisle.reason,
    }


def vehicle_feasible_segment_plan_to_dict(
    plan: VehicleFeasibleSegmentPlan,
) -> dict[str, Any]:
    """Serialize an A1 segment plan with frozen field ordering and summary."""
    configuration = _serialized_configuration(plan)
    active_segment_count = sum(len(aisle.active_segments) for aisle in plan.aisles)
    rejected_fragment_count = sum(
        len(aisle.rejected_fragments) for aisle in plan.aisles
    )
    active_segment_length = float(
        sum(
            segment.length_m
            for aisle in plan.aisles
            for segment in aisle.active_segments
        )
    )
    structural_length = float(sum(aisle.structural_length_m for aisle in plan.aisles))
    recovery_fraction = (
        0.0
        if structural_length <= 1.0e-12
        else float(active_segment_length / structural_length)
    )
    return {
        "schema": plan.schema,
        "status": plan.status,
        "frame_id": plan.frame_id,
        "platform_id": plan.platform_id,
        "platform_profile_sha256": plan.platform_profile_sha256,
        "row_direction_xy": [float(v) for v in plan.row_direction_xy],
        "source": dict(plan.source),
        "configuration": configuration,
        "summary": {
            "aisle_count": len(plan.aisles),
            "active_segment_count": active_segment_count,
            "rejected_fragment_count": rejected_fragment_count,
            "active_segment_length_m": active_segment_length,
            "structural_length_m": structural_length,
            "segment_recovery_fraction": recovery_fraction,
        },
        "aisles": [_aisle_result_to_dict(aisle) for aisle in plan.aisles],
    }


def write_vehicle_feasible_segment_plan(
    plan: VehicleFeasibleSegmentPlan,
    path: str | Path,
    *,
    overwrite: bool = False,
) -> Path:
    """Write one deterministic vehicle-feasible segment YAML asset."""
    output = Path(path).expanduser().resolve()
    if output.exists() and not overwrite:
        raise FileExistsError(
            f"vehicle-feasible segment asset already exists: {output}"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = vehicle_feasible_segment_plan_to_dict(plan)
    output.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
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


def _load_segment(
    raw: Any,
    *,
    aisle_id: str,
    minimum_contiguous_span_m: float,
    seen_ids: set[str],
) -> VehicleFeasibleSegment:
    data = _require_mapping(raw, "active segment")
    required = {
        "segment_id",
        "aisle_id",
        "ordinal_in_aisle",
        "start_distance_m",
        "end_distance_m",
        "length_m",
        "coverage_fraction_of_aisle",
        "low_endpoint_type",
        "high_endpoint_type",
        "centerline_xyz",
        "lateral_offsets_m",
        "maximum_used_lateral_shift_m",
        "low_endpoint_pose",
        "high_endpoint_pose",
        "status",
        "reason",
    }
    missing = required.difference(data)
    if missing:
        raise ValueError(f"active segment missing required keys: {sorted(missing)}")

    segment_id = str(data["segment_id"])
    if not segment_id or segment_id in seen_ids:
        raise ValueError(f"duplicate or empty vehicle-feasible segment id: {segment_id}")
    seen_ids.add(segment_id)
    serialized_aisle_id = str(data["aisle_id"])
    if serialized_aisle_id != aisle_id:
        raise ValueError(f"segment {segment_id} aisle_id mismatch")

    low_endpoint_type = str(data["low_endpoint_type"])
    high_endpoint_type = str(data["high_endpoint_type"])
    if low_endpoint_type not in _VALID_ENDPOINT_TYPES:
        raise ValueError(f"segment {segment_id} has invalid low_endpoint_type")
    if high_endpoint_type not in _VALID_ENDPOINT_TYPES:
        raise ValueError(f"segment {segment_id} has invalid high_endpoint_type")

    start_distance = _finite_float(data["start_distance_m"], "start_distance_m")
    end_distance = _finite_float(data["end_distance_m"], "end_distance_m")
    length_m = _finite_float(data["length_m"], "length_m")
    if end_distance + 1.0e-9 < start_distance:
        raise ValueError(f"segment {segment_id} has decreasing distance interval")
    if abs(length_m - (end_distance - start_distance)) > 1.0e-6:
        raise ValueError(f"segment {segment_id} length does not match its interval")
    if length_m + 1.0e-9 < minimum_contiguous_span_m:
        raise ValueError(
            f"segment {segment_id} is below serialized minimum useful segment length"
        )

    raw_points = _require_sequence(data["centerline_xyz"], "centerline_xyz")
    if not raw_points:
        raise ValueError(f"segment {segment_id} centerline_xyz must not be empty")
    points = tuple(
        _fixed_floats(point, 3, f"segment {segment_id} centerline_xyz")
        for point in raw_points
    )
    raw_offsets = _require_sequence(data["lateral_offsets_m"], "lateral_offsets_m")
    offsets = tuple(
        _finite_float(value, f"segment {segment_id} lateral_offsets_m")
        for value in raw_offsets
    )
    if len(offsets) != len(points):
        raise ValueError(
            f"segment {segment_id} lateral_offsets_m must match centerline_xyz length"
        )

    ordinal = int(data["ordinal_in_aisle"])
    if ordinal <= 0:
        raise ValueError(f"segment {segment_id} ordinal_in_aisle must be > 0")

    return VehicleFeasibleSegment(
        segment_id=segment_id,
        aisle_id=serialized_aisle_id,
        ordinal_in_aisle=ordinal,
        start_distance_m=start_distance,
        end_distance_m=end_distance,
        length_m=length_m,
        coverage_fraction_of_aisle=_finite_float(
            data["coverage_fraction_of_aisle"], "coverage_fraction_of_aisle"
        ),
        low_endpoint_type=low_endpoint_type,
        high_endpoint_type=high_endpoint_type,
        centerline_xyz=points,
        lateral_offsets_m=offsets,
        maximum_used_lateral_shift_m=_finite_float(
            data["maximum_used_lateral_shift_m"], "maximum_used_lateral_shift_m"
        ),
        low_endpoint_pose=_fixed_floats(
            data["low_endpoint_pose"], 4, f"segment {segment_id} low_endpoint_pose"
        ),
        high_endpoint_pose=_fixed_floats(
            data["high_endpoint_pose"], 4, f"segment {segment_id} high_endpoint_pose"
        ),
        status=str(data["status"]),
        reason=str(data["reason"]),
    )


def _load_fragment(
    raw: Any,
    *,
    aisle_id: str,
    minimum_contiguous_span_m: float,
    seen_ids: set[str],
) -> RejectedFeasibleFragment:
    data = _require_mapping(raw, "rejected fragment")
    required = {
        "fragment_id",
        "aisle_id",
        "start_distance_m",
        "end_distance_m",
        "length_m",
        "reason",
    }
    missing = required.difference(data)
    if missing:
        raise ValueError(f"rejected fragment missing required keys: {sorted(missing)}")
    fragment_id = str(data["fragment_id"])
    if not fragment_id or fragment_id in seen_ids:
        raise ValueError(f"duplicate or empty vehicle-feasible fragment id: {fragment_id}")
    seen_ids.add(fragment_id)
    serialized_aisle_id = str(data["aisle_id"])
    if serialized_aisle_id != aisle_id:
        raise ValueError(f"fragment {fragment_id} aisle_id mismatch")
    start_distance = _finite_float(data["start_distance_m"], "start_distance_m")
    end_distance = _finite_float(data["end_distance_m"], "end_distance_m")
    length_m = _finite_float(data["length_m"], "length_m")
    if end_distance + 1.0e-9 < start_distance:
        raise ValueError(f"fragment {fragment_id} has decreasing distance interval")
    if abs(length_m - (end_distance - start_distance)) > 1.0e-6:
        raise ValueError(f"fragment {fragment_id} length does not match its interval")
    if length_m + 1.0e-9 >= minimum_contiguous_span_m:
        raise ValueError(
            f"fragment {fragment_id} is not below serialized minimum useful segment length"
        )
    reason = str(data["reason"])
    if reason != BELOW_MINIMUM_USEFUL_SEGMENT_LENGTH:
        raise ValueError(f"fragment {fragment_id} has invalid rejection reason")
    return RejectedFeasibleFragment(
        fragment_id=fragment_id,
        aisle_id=serialized_aisle_id,
        start_distance_m=start_distance,
        end_distance_m=end_distance,
        length_m=length_m,
        reason=reason,
    )


def load_vehicle_feasible_segment_plan(
    path: str | Path,
) -> VehicleFeasibleSegmentPlan:
    """Load and strictly validate one v1 A1 vehicle-feasible segment asset."""
    input_path = Path(path).expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"vehicle-feasible segment YAML not found: {input_path}")
    payload = yaml.safe_load(input_path.read_text(encoding="utf-8")) or {}
    data = _require_mapping(payload, "vehicle-feasible segment YAML")
    required_top = {
        "schema",
        "status",
        "frame_id",
        "platform_id",
        "platform_profile_sha256",
        "row_direction_xy",
        "source",
        "configuration",
        "summary",
        "aisles",
    }
    missing_top = required_top.difference(data)
    if missing_top:
        raise ValueError(
            f"vehicle-feasible segment YAML missing required keys: {sorted(missing_top)}"
        )
    if str(data["schema"]) != VEHICLE_FEASIBLE_SEGMENT_SCHEMA:
        raise ValueError(
            f"expected {VEHICLE_FEASIBLE_SEGMENT_SCHEMA}, got {data['schema']}"
        )

    config_data = _require_mapping(data["configuration"], "configuration")
    if set(config_data) != set(_CONFIG_FIELDS):
        raise ValueError(
            "configuration must contain exactly the frozen VehicleSafeLaneConfig fields"
        )
    config = VehicleSafeLaneConfig(
        **{name: _finite_float(config_data[name], name) for name in _CONFIG_FIELDS}
    )
    config.validate()

    source = dict(_require_mapping(data["source"], "source"))
    source_config = source.get(_CONFIG_SOURCE_KEY)
    if not isinstance(source_config, Mapping):
        raise ValueError(
            f"source.{_CONFIG_SOURCE_KEY} must preserve the serialized configuration"
        )
    normalized_source_config = {
        name: _finite_float(source_config.get(name), f"source.{_CONFIG_SOURCE_KEY}.{name}")
        for name in _CONFIG_FIELDS
    }
    if normalized_source_config != _config_to_dict(config):
        raise ValueError("source configuration snapshot does not match configuration")
    source[_CONFIG_SOURCE_KEY] = normalized_source_config

    row_direction = _fixed_floats(data["row_direction_xy"], 2, "row_direction_xy")
    norm = math.hypot(row_direction[0], row_direction[1])
    if norm <= 1.0e-12:
        raise ValueError("row_direction_xy must be non-zero")

    seen_ids: set[str] = set()
    raw_aisles = _require_sequence(data["aisles"], "aisles")
    aisles: list[AisleFeasibleSegmentResult] = []
    seen_aisle_ids: set[str] = set()
    for raw_aisle in raw_aisles:
        aisle_data = _require_mapping(raw_aisle, "aisle result")
        required_aisle = {
            "aisle_id",
            "structural_length_m",
            "active_segments",
            "rejected_fragments",
            "raw_feasible_fragment_count",
            "allowed_lateral_shift_m",
            "site_boundary_rejected_pose_count",
            "site_boundary_limited_sample_count",
            "grid_rejected_pose_count",
            "reason",
        }
        missing_aisle = required_aisle.difference(aisle_data)
        if missing_aisle:
            raise ValueError(f"aisle result missing required keys: {sorted(missing_aisle)}")
        aisle_id = str(aisle_data["aisle_id"])
        if not aisle_id or aisle_id in seen_aisle_ids:
            raise ValueError(f"duplicate or empty aisle_id: {aisle_id}")
        seen_aisle_ids.add(aisle_id)

        segments = tuple(
            _load_segment(
                item,
                aisle_id=aisle_id,
                minimum_contiguous_span_m=config.minimum_contiguous_span_m,
                seen_ids=seen_ids,
            )
            for item in _require_sequence(
                aisle_data["active_segments"], "active_segments"
            )
        )
        for index, segment in enumerate(segments):
            if segment.ordinal_in_aisle != index + 1:
                raise ValueError(
                    f"aisle {aisle_id} active segment ordinals must be contiguous"
                )
            if index > 0 and (
                segments[index - 1].end_distance_m
                >= segment.start_distance_m - 1.0e-9
            ):
                raise ValueError(
                    f"aisle {aisle_id} active segments must be strictly ordered and non-overlapping"
                )

        fragments = tuple(
            _load_fragment(
                item,
                aisle_id=aisle_id,
                minimum_contiguous_span_m=config.minimum_contiguous_span_m,
                seen_ids=seen_ids,
            )
            for item in _require_sequence(
                aisle_data["rejected_fragments"], "rejected_fragments"
            )
        )
        structural_length = _finite_float(
            aisle_data["structural_length_m"], "structural_length_m"
        )
        if structural_length < 0.0:
            raise ValueError("structural_length_m must be >= 0")
        raw_count = int(aisle_data["raw_feasible_fragment_count"])
        if raw_count != len(segments) + len(fragments):
            raise ValueError(
                f"aisle {aisle_id} raw_feasible_fragment_count is inconsistent"
            )
        aisles.append(
            AisleFeasibleSegmentResult(
                aisle_id=aisle_id,
                structural_length_m=structural_length,
                active_segments=segments,
                rejected_fragments=fragments,
                raw_feasible_fragment_count=raw_count,
                allowed_lateral_shift_m=_finite_float(
                    aisle_data["allowed_lateral_shift_m"], "allowed_lateral_shift_m"
                ),
                site_boundary_rejected_pose_count=int(
                    aisle_data["site_boundary_rejected_pose_count"]
                ),
                site_boundary_limited_sample_count=int(
                    aisle_data["site_boundary_limited_sample_count"]
                ),
                grid_rejected_pose_count=int(aisle_data["grid_rejected_pose_count"]),
                reason=str(aisle_data["reason"]),
            )
        )

    plan = VehicleFeasibleSegmentPlan(
        frame_id=str(data["frame_id"]),
        platform_id=str(data["platform_id"]),
        platform_profile_sha256=str(data["platform_profile_sha256"]),
        row_direction_xy=(float(row_direction[0]), float(row_direction[1])),
        aisles=tuple(aisles),
        source=source,
        schema=str(data["schema"]),
        status=str(data["status"]),
    )

    summary = _require_mapping(data["summary"], "summary")
    expected_summary = vehicle_feasible_segment_plan_to_dict(plan)["summary"]
    for key, expected in expected_summary.items():
        if key not in summary:
            raise ValueError(f"summary missing required key: {key}")
        actual = summary[key]
        if isinstance(expected, float):
            if abs(_finite_float(actual, f"summary.{key}") - expected) > 1.0e-9:
                raise ValueError(f"summary.{key} is inconsistent")
        elif int(actual) != expected:
            raise ValueError(f"summary.{key} is inconsistent")
    return plan
