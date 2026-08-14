"""Refine agricultural row evidence into explicit crop-row and aisle geometry.

This layer keeps three concepts separate:

* row centerline / structural band: nominal crop-row geometry
* vegetation envelope: observed raw obstacle evidence around plants
* aisle candidate: corridor that exists only between adjacent valid rows

It deliberately does not mutate the final OccupancyGrid.  The output is review
 evidence for the later explicit navigation fusion policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .navigation_map_derivation import NavigationMapResult
from .navigation_structure import NavigationStructureResult


@dataclass(frozen=True)
class CorridorRefinementConfig:
    boundary_exclusion_m: float = 0.45
    row_structural_half_width_m: float = 0.20
    aisle_side_clearance_m: float = 0.12
    raw_obstacle_clearance_m: float = 0.12
    aisle_minimum_width_m: float = 0.45
    aisle_centerline_half_width_m: float = 0.06
    spacing_tolerance_ratio: float = 0.38
    minimum_row_longitudinal_span_m: float = 1.50
    minimum_ground_confidence: float = 0.30

    def validate(self) -> None:
        if self.boundary_exclusion_m < 0.0:
            raise ValueError("boundary_exclusion_m must be >= 0")
        if self.row_structural_half_width_m <= 0.0:
            raise ValueError("row_structural_half_width_m must be > 0")
        if self.aisle_side_clearance_m < 0.0:
            raise ValueError("aisle_side_clearance_m must be >= 0")
        if self.raw_obstacle_clearance_m < 0.0:
            raise ValueError("raw_obstacle_clearance_m must be >= 0")
        if self.aisle_minimum_width_m <= 0.0:
            raise ValueError("aisle_minimum_width_m must be > 0")
        if self.aisle_centerline_half_width_m <= 0.0:
            raise ValueError("aisle_centerline_half_width_m must be > 0")
        if not 0.0 <= self.spacing_tolerance_ratio < 1.0:
            raise ValueError("spacing_tolerance_ratio must be in [0, 1)")
        if self.minimum_row_longitudinal_span_m <= 0.0:
            raise ValueError("minimum_row_longitudinal_span_m must be > 0")
        if not 0.0 <= self.minimum_ground_confidence <= 1.0:
            raise ValueError("minimum_ground_confidence must be in [0, 1]")


@dataclass(frozen=True)
class CorridorRefinementResult:
    row_centerline: np.ndarray
    row_structural_band: np.ndarray
    vegetation_envelope: np.ndarray
    boundary_exclusion: np.ndarray
    aisle_candidate: np.ndarray
    aisle_centerline: np.ndarray
    accepted_row_centers_v_m: tuple[float, ...]
    rejected_row_centers_v_m: tuple[float, ...]
    nominal_row_spacing_m: float | None
    config: CorridorRefinementConfig


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("navigation corridor refinement requires scipy") from exc
    return ndimage


def _grid_xy(result: NavigationMapResult) -> tuple[np.ndarray, np.ndarray]:
    columns = np.arange(result.width, dtype=np.float64)
    rows = np.arange(result.height, dtype=np.float64)
    return np.meshgrid(
        result.origin_x_m + (columns + 0.5) * result.resolution_m,
        result.origin_y_m + (rows + 0.5) * result.resolution_m,
    )


def _normalize_direction(direction_xy: Iterable[float]) -> np.ndarray:
    direction = np.asarray(tuple(direction_xy), dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("row direction must be finite and non-zero")
    return direction / norm


def _filter_row_centers(
    centers: np.ndarray,
    *,
    v_min: float,
    v_max: float,
    boundary_exclusion_m: float,
    spacing_tolerance_ratio: float,
) -> tuple[np.ndarray, np.ndarray, float | None]:
    if centers.size == 0:
        return centers, centers, None
    centers = np.sort(np.asarray(centers, dtype=np.float64))
    interior = (
        (centers >= v_min + boundary_exclusion_m)
        & (centers <= v_max - boundary_exclusion_m)
    )
    interior_centers = centers[interior]
    rejected = list(centers[~interior])
    if interior_centers.size < 3:
        return interior_centers, np.asarray(rejected, dtype=np.float64), None

    gaps = np.diff(interior_centers)
    positive = gaps[gaps > 1e-6]
    if positive.size == 0:
        return interior_centers, np.asarray(rejected, dtype=np.float64), None
    nominal = float(np.median(positive))
    low = nominal * (1.0 - spacing_tolerance_ratio)
    high = nominal * (1.0 + spacing_tolerance_ratio)

    keep = np.zeros(interior_centers.size, dtype=bool)
    for index in range(interior_centers.size):
        adjacent = []
        if index > 0:
            adjacent.append(interior_centers[index] - interior_centers[index - 1])
        if index + 1 < interior_centers.size:
            adjacent.append(interior_centers[index + 1] - interior_centers[index])
        # A missing crop row can create a 2x spacing gap.  Accept either one
        # nominal spacing or a clear integer multiple rather than deleting both
        # neighbors merely because one row is absent.
        for gap in adjacent:
            if low <= gap <= high or 2.0 * low <= gap <= 2.0 * high:
                keep[index] = True
                break
    rejected.extend(interior_centers[~keep].tolist())
    accepted = interior_centers[keep]
    return accepted, np.sort(np.asarray(rejected, dtype=np.float64)), nominal


def derive_corridor_refinement(
    navigation: NavigationMapResult,
    structure: NavigationStructureResult,
    config: CorridorRefinementConfig | None = None,
) -> CorridorRefinementResult:
    cfg = config or CorridorRefinementConfig()
    cfg.validate()
    ndimage = _require_scipy()

    xx, yy = _grid_xy(navigation)
    direction = _normalize_direction(structure.row_model.direction_xy)
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    uu = xx * direction[0] + yy * direction[1]
    vv = xx * perpendicular[0] + yy * perpendicular[1]

    observed = np.asarray(navigation.point_count) > 0
    if np.any(observed):
        v_min = float(np.min(vv[observed]))
        v_max = float(np.max(vv[observed]))
    else:
        v_min = float(np.min(vv))
        v_max = float(np.max(vv))

    centers = np.asarray(structure.row_model.centers_v_m, dtype=np.float64)
    accepted, rejected, nominal_spacing = _filter_row_centers(
        centers,
        v_min=v_min,
        v_max=v_max,
        boundary_exclusion_m=float(cfg.boundary_exclusion_m),
        spacing_tolerance_ratio=float(cfg.spacing_tolerance_ratio),
    )

    raw_obstacle = (
        np.asarray(navigation.obstacle_count)
        >= int(navigation.config.minimum_obstacle_points)
    )
    vegetation_envelope = raw_obstacle.copy()

    # Cells close to the outer observed support are not treated as aisles.  This
    # prevents wall-to-row gaps and map exterior from becoming free corridors.
    observed_distance = ndimage.distance_transform_edt(observed) * navigation.resolution_m
    boundary_exclusion = observed & (
        observed_distance <= float(cfg.boundary_exclusion_m)
    )

    row_centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    row_structural_band = np.zeros(navigation.occupancy.shape, dtype=bool)
    row_u_ranges: list[tuple[float, float] | None] = []

    source_rows = np.asarray(structure.row_regularized_obstacle, dtype=bool)
    source_half_width = max(
        float(structure.row_model.half_width_m),
        float(cfg.row_structural_half_width_m),
    )
    for center in accepted:
        source_band = np.abs(vv - center) <= source_half_width
        support = source_band & source_rows
        if np.any(support):
            u_min = float(np.min(uu[support]))
            u_max = float(np.max(uu[support]))
        else:
            # Keep the evidence explicit: a row peak with no longitudinal
            # support is not extended across the whole map.
            row_u_ranges.append(None)
            continue
        if u_max - u_min < cfg.minimum_row_longitudinal_span_m:
            row_u_ranges.append(None)
            continue
        active_u = (uu >= u_min) & (uu <= u_max)
        row_centerline |= active_u & (
            np.abs(vv - center) <= max(navigation.resolution_m * 0.55, 0.03)
        )
        row_structural_band |= active_u & (
            np.abs(vv - center) <= float(cfg.row_structural_half_width_m)
        )
        row_u_ranges.append((u_min, u_max))

    # Aisles exist only between adjacent accepted crop rows.  They are not the
    # generic complement of row obstacles.  This is the key distinction that
    # prevents map boundary/wall gaps from being labelled traversable.
    aisle_geometry = np.zeros(navigation.occupancy.shape, dtype=bool)
    aisle_centerline = np.zeros(navigation.occupancy.shape, dtype=bool)
    for index in range(max(0, accepted.size - 1)):
        left = float(accepted[index])
        right = float(accepted[index + 1])
        left_range = row_u_ranges[index] if index < len(row_u_ranges) else None
        right_range = row_u_ranges[index + 1] if index + 1 < len(row_u_ranges) else None
        if left_range is None or right_range is None:
            continue
        corridor_min = left + cfg.row_structural_half_width_m + cfg.aisle_side_clearance_m
        corridor_max = right - cfg.row_structural_half_width_m - cfg.aisle_side_clearance_m
        if corridor_max - corridor_min < cfg.aisle_minimum_width_m:
            continue
        overlap_min = max(left_range[0], right_range[0])
        overlap_max = min(left_range[1], right_range[1])
        if overlap_max - overlap_min < cfg.minimum_row_longitudinal_span_m:
            continue
        longitudinal = (uu >= overlap_min) & (uu <= overlap_max)
        aisle_geometry |= longitudinal & (vv >= corridor_min) & (vv <= corridor_max)
        midpoint = 0.5 * (left + right)
        aisle_centerline |= longitudinal & (
            np.abs(vv - midpoint) <= float(cfg.aisle_centerline_half_width_m)
        )

    obstacle_distance = (
        ndimage.distance_transform_edt(~raw_obstacle) * navigation.resolution_m
    )
    clear_of_raw_obstacle = obstacle_distance >= float(cfg.raw_obstacle_clearance_m)
    confident_ground = (
        np.asarray(structure.ground_confidence)
        >= float(cfg.minimum_ground_confidence)
    )
    slope_ok = np.isfinite(structure.robust_slope_deg) & (
        structure.robust_slope_deg <= navigation.config.maximum_slope_deg
    )

    aisle_candidate = (
        aisle_geometry
        & confident_ground
        & slope_ok
        & clear_of_raw_obstacle
        & ~boundary_exclusion
        & ~row_structural_band
    )
    aisle_centerline &= aisle_candidate

    return CorridorRefinementResult(
        row_centerline=row_centerline,
        row_structural_band=row_structural_band,
        vegetation_envelope=vegetation_envelope,
        boundary_exclusion=boundary_exclusion,
        aisle_candidate=aisle_candidate,
        aisle_centerline=aisle_centerline,
        accepted_row_centers_v_m=tuple(float(v) for v in accepted),
        rejected_row_centers_v_m=tuple(float(v) for v in rejected),
        nominal_row_spacing_m=nominal_spacing,
        config=cfg,
    )
