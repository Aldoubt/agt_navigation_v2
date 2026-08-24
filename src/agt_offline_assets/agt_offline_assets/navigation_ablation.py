"""Controlled A0/A1/A2/A3 ablations for Ground-relative Navigation Maps.

The production derivation remains unchanged.  This module takes one already
computed Ground-relative evidence grid and deterministically re-materializes
its FREE/OCCUPIED/UNKNOWN classification so one mechanism changes at a time:

A0  current baseline, unchanged;
A1  A0 without raster obstacle padding;
A2  A1 plus terrain slope/step HARD only where real ground support exists;
A3  A2 plus a local-linear discontinuity step metric instead of a 5x5
    max-minus-min terrain range.

This separation is intentional: ablation results are review evidence until one
policy is explicitly promoted into the canonical production contract.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    NavigationMapResult,
)


ABLATION_PROFILE_KEYS = ("A0", "A1", "A2", "A3")


@dataclass(frozen=True)
class NavigationAblationSpec:
    key: str
    label: str
    disable_raster_padding: bool
    require_ground_support_for_geometry: bool
    step_mode: str

    def to_dict(self) -> dict[str, object]:
        return {
            "key": self.key,
            "label": self.label,
            "disable_raster_padding": self.disable_raster_padding,
            "require_ground_support_for_geometry": (
                self.require_ground_support_for_geometry
            ),
            "step_mode": self.step_mode,
        }


_SPECS = {
    "A0": NavigationAblationSpec(
        key="A0",
        label="A0 当前基线",
        disable_raster_padding=False,
        require_ground_support_for_geometry=False,
        step_mode="window_range",
    ),
    "A1": NavigationAblationSpec(
        key="A1",
        label="A1 去除 Formal PGM 栅格 Padding",
        disable_raster_padding=True,
        require_ground_support_for_geometry=False,
        step_mode="window_range",
    ),
    "A2": NavigationAblationSpec(
        key="A2",
        label="A2 A1 + 仅可信 Ground 参与坡度/台阶 HARD",
        disable_raster_padding=True,
        require_ground_support_for_geometry=True,
        step_mode="window_range",
    ),
    "A3": NavigationAblationSpec(
        key="A3",
        label="A3 A2 + 局部线性不连续 Step",
        disable_raster_padding=True,
        require_ground_support_for_geometry=True,
        step_mode="local_linear_discontinuity",
    ),
}


def navigation_ablation_spec(profile: str) -> NavigationAblationSpec:
    key = str(profile).strip().upper()
    try:
        return _SPECS[key]
    except KeyError as exc:
        raise ValueError(
            f"unknown navigation ablation profile {profile!r}; "
            f"expected one of {', '.join(ABLATION_PROFILE_KEYS)}"
        ) from exc


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("navigation ablation A3 requires scipy") from exc
    return ndimage


def derive_local_linear_step_map(ground_height_m: np.ndarray) -> np.ndarray:
    """Return a slope-invariant local discontinuity estimate in metres.

    A planar surface has zero second difference regardless of its constant
    slope.  A genuine height discontinuity produces a non-zero second
    difference near the edge.  Four opposite-neighbour directions are used so
    the metric is not tied to the grid axes.

    Invalid ground cells remain NaN.  They are nearest-filled only to provide
    finite neighbours for supported cells; A2/A3 terrain HARD gating still
    requires real ground support at the classified centre cell.
    """

    surface = np.asarray(ground_height_m, dtype=np.float64)
    valid = np.isfinite(surface)
    result = np.full(surface.shape, np.nan, dtype=np.float64)
    if not np.any(valid):
        return result

    ndimage = _require_scipy()
    _, nearest = ndimage.distance_transform_edt(~valid, return_indices=True)
    filled = surface[tuple(nearest)]

    def shift(dr: int, dc: int) -> np.ndarray:
        return ndimage.shift(
            filled,
            shift=(dr, dc),
            order=0,
            mode="nearest",
            prefilter=False,
        )

    centre = filled
    residuals = (
        np.abs(shift(0, -1) - 2.0 * centre + shift(0, 1)),
        np.abs(shift(-1, 0) - 2.0 * centre + shift(1, 0)),
        np.abs(shift(-1, -1) - 2.0 * centre + shift(1, 1)),
        np.abs(shift(-1, 1) - 2.0 * centre + shift(1, -1)),
    )
    discontinuity = np.maximum.reduce(residuals)
    result[valid] = discontinuity[valid]
    return result


def apply_navigation_ablation_profile(
    navigation: NavigationMapResult,
    profile: str,
) -> NavigationMapResult:
    """Reclassify one navigation evidence grid under one controlled profile."""

    spec = navigation_ablation_spec(profile)
    if spec.key == "A0":
        return navigation

    config = navigation.config
    ground_valid = np.asarray(navigation.ground_valid, dtype=bool)
    point_count = np.asarray(navigation.point_count, dtype=np.int32)
    ground_support = np.asarray(navigation.ground_support_count, dtype=np.int32)
    obstacle_count = np.asarray(navigation.obstacle_count, dtype=np.int32)
    slope = np.asarray(navigation.slope_deg, dtype=np.float64).copy()
    step = np.asarray(navigation.step_m, dtype=np.float64).copy()

    if spec.step_mode == "local_linear_discontinuity":
        step = derive_local_linear_step_map(navigation.ground_height_m)

    direct_obstacle = obstacle_count >= int(config.minimum_obstacle_points)
    terrain_support = ground_valid & (point_count > 0)
    if spec.require_ground_support_for_geometry:
        terrain_support &= ground_support >= int(config.minimum_ground_support_points)
        # Formal hard-occupancy provenance consumes these evidence arrays.  NaN
        # outside trusted Ground ensures the later diagnostic/materialization
        # layer cannot accidentally re-promote an ignored interpolation-only
        # slope/step into HARD OCCUPIED.
        slope = np.where(terrain_support, slope, np.nan)
        step = np.where(terrain_support, step, np.nan)

    geometry_bad = terrain_support & (
        (slope > float(config.maximum_slope_deg))
        | (step > float(config.maximum_step_m))
    )
    occupied = direct_obstacle | geometry_bad

    # A1/A2/A3 deliberately keep the environmental raster uninflated.  Vehicle
    # footprint and clearance belong to downstream vehicle-feasibility/costmap
    # logic, not to the immutable environment occupancy map.
    free = (
        ground_valid
        & (ground_support >= int(config.minimum_ground_support_points))
        & ~occupied
    )
    occupancy = np.full(navigation.occupancy.shape, UNKNOWN, dtype=np.uint8)
    occupancy[free] = FREE
    occupancy[occupied] = OCCUPIED

    effective_config = replace(config, obstacle_padding_m=0.0)
    return replace(
        navigation,
        config=effective_config,
        slope_deg=slope,
        step_m=step,
        occupancy=occupancy,
    )
