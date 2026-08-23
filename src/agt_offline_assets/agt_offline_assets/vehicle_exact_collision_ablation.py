"""E3 full-grid exact vehicle collision evidence over the frozen A3 ground surface.

E3 is experimental vehicle-review evidence only. Unlike D3's conservative
whole-layer overlap, E3 replays the raw PCD and counts only points whose
*continuous* ground-relative height intersects the recorded vehicle collision
envelope. It does not mutate the environmental Navigation Map, Formal/Accepted
PGM, D2 sidecar, or map authority.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np

from .height_layer_ablation import _iter_xyz_chunks
from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN, NavigationMapResult
from .vehicle_collision_envelope import VehicleCollisionEnvelope


E3_EXACT_CONTRACT = "RAW_PCD_CONTINUOUS_GROUND_RELATIVE_HEIGHT"


def exact_vehicle_height_interval_m(
    navigation: NavigationMapResult,
    envelope: VehicleCollisionEnvelope,
) -> tuple[float, float]:
    """Return the obstacle-evidence interval that physically overlaps the vehicle."""

    lower = max(
        float(navigation.config.obstacle_min_height_m),
        float(envelope.collision_z_min_m),
    )
    upper = min(
        float(navigation.config.obstacle_max_height_m),
        float(envelope.collision_z_max_m),
    )
    return lower, upper


def derive_exact_vehicle_obstacle_count(
    cloud: Any,
    navigation: NavigationMapResult,
    envelope: VehicleCollisionEnvelope,
    *,
    chunk_size: int = 1_000_000,
) -> np.ndarray:
    """Aggregate exact ground-relative vehicle-collision point counts per XY cell.

    The lower boundary is inclusive. The vehicle upper boundary is exclusive,
    matching the E2 ``MID_COLLIDING=[low, vehicle_z_max)`` contract. Only when
    the envelope reaches the environmental obstacle maximum is that final
    environmental endpoint included so an exact full-height replay can match
    the original obstacle-evidence contract.
    """

    if int(chunk_size) < 1:
        raise ValueError("E3 chunk_size must be >= 1")
    ground = np.asarray(navigation.ground_height_m, dtype=np.float64)
    occupancy = np.asarray(navigation.occupancy)
    if ground.ndim != 2 or ground.shape != occupancy.shape:
        raise ValueError("E3 navigation ground/grid shape mismatch")

    height, width = ground.shape
    resolution = float(navigation.resolution_m)
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    cell_count = int(height * width)
    lower, upper = exact_vehicle_height_interval_m(navigation, envelope)
    if upper <= lower + 1.0e-12:
        return np.zeros((height, width), dtype=np.int32)

    environmental_max = float(navigation.config.obstacle_max_height_m)
    include_upper_endpoint = (
        upper >= environmental_max - 1.0e-12
        and float(envelope.collision_z_max_m) >= environmental_max - 1.0e-12
    )
    counts = np.zeros(cell_count, dtype=np.int64)

    for xyz in _iter_xyz_chunks(cloud, int(chunk_size)):
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        if not np.any(finite):
            continue
        xyz = xyz[finite]
        cols = np.floor((xyz[:, 0] - origin_x) / resolution).astype(np.int64)
        rows = np.floor((xyz[:, 1] - origin_y) / resolution).astype(np.int64)
        in_grid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        if not np.any(in_grid):
            continue

        rows = rows[in_grid]
        cols = cols[in_grid]
        z = xyz[in_grid, 2]
        cell_ground = ground[rows, cols]
        classifiable = np.isfinite(cell_ground)
        if not np.any(classifiable):
            continue
        rows = rows[classifiable]
        cols = cols[classifiable]
        z = z[classifiable]
        cell_ground = cell_ground[classifiable]
        relative_height = z - cell_ground
        selected = relative_height >= lower
        if include_upper_endpoint:
            selected &= relative_height <= upper
        else:
            selected &= relative_height < upper
        if not np.any(selected):
            continue
        cell_ids = rows[selected] * width + cols[selected]
        counts += np.bincount(cell_ids, minlength=cell_count)

    if counts.size and int(np.max(counts)) > np.iinfo(np.int32).max:
        raise OverflowError("E3 exact vehicle obstacle count exceeds int32 range")
    return counts.reshape(height, width).astype(np.int32)


def _navigation_from_exact_obstacle_count(
    a3_navigation: NavigationMapResult,
    obstacle_count: np.ndarray,
) -> NavigationMapResult:
    obstacle_count = np.asarray(obstacle_count, dtype=np.int32)
    if obstacle_count.shape != np.asarray(a3_navigation.occupancy).shape:
        raise ValueError("E3 exact obstacle-count/navigation shape mismatch")

    cfg = a3_navigation.config
    slope = np.asarray(a3_navigation.slope_deg, dtype=np.float64)
    step = np.asarray(a3_navigation.step_m, dtype=np.float64)
    direct_obstacle = obstacle_count >= int(cfg.minimum_obstacle_points)
    geometry_bad = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            (np.isfinite(slope) & (slope > float(cfg.maximum_slope_deg)))
            | (np.isfinite(step) & (step > float(cfg.maximum_step_m)))
        )
    )
    occupied = direct_obstacle | geometry_bad
    free = (
        np.asarray(a3_navigation.ground_valid, dtype=bool)
        & (
            np.asarray(a3_navigation.ground_support_count, dtype=np.int32)
            >= int(cfg.minimum_ground_support_points)
        )
        & ~occupied
    )
    occupancy = np.full(a3_navigation.occupancy.shape, UNKNOWN, dtype=np.uint8)
    occupancy[free] = FREE
    occupancy[occupied] = OCCUPIED
    return replace(
        a3_navigation,
        obstacle_count=obstacle_count,
        occupancy=occupancy,
    )


def derive_exact_vehicle_envelope_navigation(
    a3_navigation: NavigationMapResult,
    cloud: Any,
    envelope: VehicleCollisionEnvelope,
    *,
    chunk_size: int = 1_000_000,
) -> NavigationMapResult:
    """Return a review-only A3 terrain map with exact vehicle-height sensor evidence."""

    counts = derive_exact_vehicle_obstacle_count(
        cloud,
        a3_navigation,
        envelope,
        chunk_size=int(chunk_size),
    )
    return _navigation_from_exact_obstacle_count(a3_navigation, counts)
