"""Vehicle-width review corridor derived from refined agricultural aisles.

This module is intentionally an evidence/review layer.  It does not mutate the
final OccupancyGrid and does not declare a route feasible by itself.  The same
vehicle-width semantics can later be reused by route-feasibility validation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .navigation_corridor import CorridorRefinementResult
from .navigation_map_derivation import NavigationMapResult


@dataclass(frozen=True)
class VehicleCorridorConfig:
    """Physical envelope used to review an aisle centerline.

    ``vehicle_width_m`` is the nominal body/footprint width.  The lateral
    safety margin is applied independently on both sides.
    """

    vehicle_width_m: float = 0.60
    lateral_safety_margin_m: float = 0.10

    @property
    def required_width_m(self) -> float:
        return float(self.vehicle_width_m + 2.0 * self.lateral_safety_margin_m)

    @property
    def required_half_width_m(self) -> float:
        return 0.5 * self.required_width_m

    def validate(self) -> None:
        if self.vehicle_width_m <= 0.0:
            raise ValueError("vehicle_width_m must be > 0")
        if self.lateral_safety_margin_m < 0.0:
            raise ValueError("lateral_safety_margin_m must be >= 0")


@dataclass(frozen=True)
class VehicleCorridorResult:
    """Review ribbon around the currently accepted aisle centerlines."""

    corridor_mask: np.ndarray
    centerline_mask: np.ndarray
    required_width_m: float
    required_half_width_m: float
    covered_centerline_cells: int
    corridor_cells: int
    config: VehicleCorridorConfig


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("vehicle corridor derivation requires scipy") from exc
    return ndimage


def derive_vehicle_corridor(
    navigation: NavigationMapResult,
    corridor: CorridorRefinementResult,
    config: VehicleCorridorConfig | None = None,
) -> VehicleCorridorResult:
    """Buffer accepted centerlines by vehicle width, clipped to refined aisles.

    This is a visualization/inspection envelope rather than the final collision
    model.  The mask is deliberately clipped to ``aisle_candidate`` so the 3D
    reviewer never paints a vehicle ribbon through cells already rejected by
    Ground Confidence, robust slope, raw-obstacle clearance, row geometry, or
    map-boundary evidence.
    """

    cfg = config or VehicleCorridorConfig()
    cfg.validate()
    centerline = np.asarray(corridor.aisle_centerline, dtype=bool)
    aisle = np.asarray(corridor.aisle_candidate, dtype=bool)
    if centerline.shape != navigation.occupancy.shape or aisle.shape != navigation.occupancy.shape:
        raise ValueError("corridor masks must match the navigation grid shape")

    if not np.any(centerline):
        mask = np.zeros(centerline.shape, dtype=bool)
    else:
        ndimage = _require_scipy()
        distance_m = (
            ndimage.distance_transform_edt(~centerline) * float(navigation.resolution_m)
        )
        mask = aisle & (distance_m <= float(cfg.required_half_width_m))

    return VehicleCorridorResult(
        corridor_mask=mask,
        centerline_mask=centerline.copy(),
        required_width_m=float(cfg.required_width_m),
        required_half_width_m=float(cfg.required_half_width_m),
        covered_centerline_cells=int(np.count_nonzero(centerline & mask)),
        corridor_cells=int(np.count_nonzero(mask)),
        config=cfg,
    )
