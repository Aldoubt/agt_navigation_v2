"""Shared deterministic vehicle-pose feasibility sampling for aisle planning.

This module owns the sample-level preview-footprint evaluation that was
historically embedded inside ``vehicle_safe_lane``.  It intentionally stops at
an auditable longitudinal trace: reducers decide later whether to keep one run
or every useful run.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .agricultural_aisle_graph import AislePrimitive
from .forward_connector import ForwardConnectorSample
from .forward_connector_navigation_gate import (
    _preview_local_footprint,
    _transform_polygon,
)
from .navigation_grid import NavigationGridEvidence
from .reverse_primitive_connector import _preview_pose_free
from .site_boundary import SiteBoundary, polygon_strictly_inside_site_boundary
from .vehicle_profile import CanonicalVehicleProfile


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
class VehicleLaneFeasibilityTrace:
    aisle_id: str
    structural_length_m: float
    distances_m: tuple[float, ...]
    selected_points: tuple[tuple[float, float, float] | None, ...]
    selected_offsets_m: tuple[float | None, ...]
    allowed_lateral_shift_m: float
    site_boundary_rejected_pose_count: int
    site_boundary_limited_sample_count: int
    grid_rejected_pose_count: int
    structural_width_blocked_reason: str | None = None


def normalize_row_direction(direction_xy) -> np.ndarray:
    direction = np.asarray(direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("row direction must be finite and non-zero")
    return direction / norm


def resample_aisle_polyline(
    aisle: AislePrimitive,
    spacing_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(aisle.centerline_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 3:
        raise ValueError(f"aisle {aisle.aisle_id} centerline is invalid")
    if not np.all(np.isfinite(points[:, :3])):
        raise ValueError(f"aisle {aisle.aisle_id} centerline contains non-finite values")
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


def lateral_offset_candidates(
    maximum_shift_m: float,
    step_m: float,
) -> tuple[float, ...]:
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


def derive_vehicle_lane_feasibility_trace(
    aisle: AislePrimitive,
    navigation: NavigationGridEvidence,
    vehicle: CanonicalVehicleProfile,
    config: VehicleSafeLaneConfig,
    *,
    row_direction_xy: tuple[float, float],
    site_boundary: SiteBoundary | None = None,
) -> VehicleLaneFeasibilityTrace:
    """Evaluate one deterministic preview-footprint pose per sampled station."""
    config.validate()
    if site_boundary is not None:
        site_boundary.validate(expected_frame_id=navigation.frame_id)
    if not vehicle.planning_preview_ready:
        raise ValueError(
            f"vehicle profile {vehicle.profile_id} is not ready for planning preview"
        )

    direction = normalize_row_direction(row_direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    local_footprint = _preview_local_footprint(
        vehicle,
        config.preview_footprint_padding_m,
    )
    samples, distances = resample_aisle_polyline(aisle, config.sample_spacing_m)
    total = float(distances[-1]) if distances.size else 0.0
    required_lateral_width = float(
        vehicle.navigation_width_m + 2.0 * config.preview_footprint_padding_m
    )

    if float(aisle.geometric_width_m) + 1.0e-9 < required_lateral_width:
        reason = (
            "STRUCTURAL_WIDTH_BELOW_PREVIEW_VEHICLE_WIDTH: "
            f"aisle={aisle.geometric_width_m:.3f} m "
            f"required={required_lateral_width:.3f} m"
        )
        unavailable_count = int(samples.shape[0])
        return VehicleLaneFeasibilityTrace(
            aisle_id=aisle.aisle_id,
            structural_length_m=total,
            distances_m=tuple(float(value) for value in distances),
            selected_points=tuple(None for _ in range(unavailable_count)),
            selected_offsets_m=tuple(None for _ in range(unavailable_count)),
            allowed_lateral_shift_m=0.0,
            site_boundary_rejected_pose_count=0,
            site_boundary_limited_sample_count=0,
            grid_rejected_pose_count=0,
            structural_width_blocked_reason=reason,
        )

    width_surplus = float(aisle.geometric_width_m) - required_lateral_width
    allowed_shift = min(float(config.maximum_lateral_shift_m), 0.5 * width_surplus)
    offsets = lateral_offset_candidates(
        allowed_shift,
        config.lateral_search_step_m,
    )
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
                > config.maximum_lateral_step_m + 1.0e-9
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

    return VehicleLaneFeasibilityTrace(
        aisle_id=aisle.aisle_id,
        structural_length_m=total,
        distances_m=tuple(float(value) for value in distances),
        selected_points=tuple(selected),
        selected_offsets_m=tuple(selected_offsets),
        allowed_lateral_shift_m=float(allowed_shift),
        site_boundary_rejected_pose_count=int(boundary_rejections),
        site_boundary_limited_sample_count=int(boundary_limited_samples),
        grid_rejected_pose_count=int(grid_rejections),
        structural_width_blocked_reason=None,
    )
