"""Vehicle-clearance-aware aisle connectivity diagnostics.

The formal Navigation Map remains an environment occupancy product. This module
adds a separate review layer that asks whether a vehicle center can traverse an
aisle while maintaining a requested clearance radius from non-FREE cells and
from the geometric aisle boundary.

It never mutates or inflates the formal PGM.
"""

from __future__ import annotations

import heapq
from typing import Any

import numpy as np

from .navigation_map_derivation import FREE


_ACCEPTED_AISLE_STATES = {"ACCEPTED", "ACCEPTED_NO_CENTERLINE"}
_NEIGHBOURS = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("vehicle feasibility audit requires scipy") from exc
    return ndimage


def _grid_uv(navigation: Any, structure: Any) -> tuple[np.ndarray, np.ndarray]:
    occupancy = np.asarray(navigation.occupancy)
    rows, cols = np.indices(occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (cols + 0.5) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (rows + 0.5) * float(navigation.resolution_m)
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("vehicle feasibility row direction must be finite and non-zero")
    direction = direction / norm
    perpendicular = np.array([-direction[1], direction[0]], dtype=np.float64)
    u = xx * direction[0] + yy * direction[1]
    v = xx * perpendicular[0] + yy * perpendicular[1]
    return u, v


def _owned_mask(geometric: np.ndarray, v: np.ndarray, diagnostic: Any) -> np.ndarray:
    low = min(float(diagnostic.left_row_center_v_m), float(diagnostic.right_row_center_v_m))
    high = max(float(diagnostic.left_row_center_v_m), float(diagnostic.right_row_center_v_m))
    return geometric & (v >= low - 1.0e-9) & (v <= high + 1.0e-9)


def _end_masks(owned: np.ndarray, u: np.ndarray, resolution_m: float) -> tuple[np.ndarray, np.ndarray]:
    if not np.any(owned):
        empty = np.zeros_like(owned, dtype=bool)
        return empty, empty
    u_min = float(np.min(u[owned]))
    u_max = float(np.max(u[owned]))
    tolerance = 0.51 * float(resolution_m)
    return owned & (u <= u_min + tolerance), owned & (u >= u_max - tolerance)


def _clearance_field_m(traversable: np.ndarray, resolution_m: float) -> np.ndarray:
    """Cell-center clearance to nearest blocked/aisle-exterior cell boundary."""

    ndimage = _require_scipy()
    padded = np.pad(np.asarray(traversable, dtype=bool), 1, constant_values=False)
    distance_centers = ndimage.distance_transform_edt(padded)[1:-1, 1:-1]
    # EDT measures center-to-center distance to the nearest blocked cell. Subtract
    # half a cell so a one-cell-wide corridor has 0.5*resolution geometric
    # clearance rather than 1.0*resolution.
    clearance = distance_centers * float(resolution_m) - 0.5 * float(resolution_m)
    return np.maximum(clearance, 0.0)


def _widest_end_to_end_clearance(
    traversable: np.ndarray,
    clearance_m: np.ndarray,
    start_zone: np.ndarray,
    end_zone: np.ndarray,
) -> float:
    """Return the maximum bottleneck clearance available on any 8-neighbour path."""

    start = np.asarray(start_zone, dtype=bool) & traversable
    end = np.asarray(end_zone, dtype=bool) & traversable
    if not np.any(start) or not np.any(end):
        return 0.0

    height, width = traversable.shape
    best = np.full((height, width), -1.0, dtype=np.float64)
    heap: list[tuple[float, int, int]] = []
    for row, col in np.argwhere(start):
        rr = int(row)
        cc = int(col)
        capacity = float(clearance_m[rr, cc])
        if capacity > best[rr, cc]:
            best[rr, cc] = capacity
            heapq.heappush(heap, (-capacity, rr, cc))

    while heap:
        negative_capacity, row, col = heapq.heappop(heap)
        capacity = -float(negative_capacity)
        if capacity + 1.0e-12 < float(best[row, col]):
            continue
        if end[row, col]:
            return capacity
        for dr, dc in _NEIGHBOURS:
            rr = row + dr
            cc = col + dc
            if not (0 <= rr < height and 0 <= cc < width):
                continue
            if not traversable[rr, cc]:
                continue
            candidate = min(capacity, float(clearance_m[rr, cc]))
            if candidate > float(best[rr, cc]) + 1.0e-12:
                best[rr, cc] = candidate
                heapq.heappush(heap, (-candidate, rr, cc))
    return 0.0


def build_vehicle_feasible_aisle_audit(
    navigation: Any,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    *,
    clearance_radius_m: float,
) -> list[dict[str, object]]:
    """Return raster and vehicle-clearance connectivity for accepted ROW_ROW aisles."""

    if clearance_radius_m < 0.0:
        raise ValueError("clearance_radius_m must be >= 0")
    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if occupancy.shape != geometric.shape:
        raise ValueError("vehicle feasibility audit grid shape mismatch")
    if np.asarray(navigation.occupancy).shape != occupancy.shape:
        raise ValueError("vehicle feasibility navigation shape mismatch")

    resolution = float(navigation.resolution_m)
    if resolution <= 0.0:
        raise ValueError("vehicle feasibility resolution must be > 0")
    u, v = _grid_uv(navigation, structure)

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        if str(getattr(diagnostic, "status", "")) not in _ACCEPTED_AISLE_STATES:
            continue

        owned = _owned_mask(geometric, v, diagnostic)
        traversable = owned & (occupancy == FREE)
        start_zone, end_zone = _end_masks(owned, u, resolution)
        start_has_free = bool(np.any(start_zone & traversable))
        end_has_free = bool(np.any(end_zone & traversable))

        clearance = _clearance_field_m(traversable, resolution)
        widest = _widest_end_to_end_clearance(
            traversable,
            clearance,
            start_zone,
            end_zone,
        )
        raster_connected = bool(start_has_free and end_has_free and widest > 0.0)

        feasible = traversable & (clearance + 1.0e-12 >= float(clearance_radius_m))
        start_has_feasible = bool(np.any(start_zone & feasible))
        end_has_feasible = bool(np.any(end_zone & feasible))
        vehicle_connected = bool(
            raster_connected
            and start_has_feasible
            and end_has_feasible
            and widest + 1.0e-12 >= float(clearance_radius_m)
        )

        maximum_clearance = (
            float(np.max(clearance[traversable])) if np.any(traversable) else 0.0
        )
        reports.append(
            {
                "aisle_id": f"aisle_{int(diagnostic.pair_index):03d}",
                "pair_index": int(diagnostic.pair_index),
                "diagnostic_status": str(diagnostic.status),
                "raster_grid_connectivity": raster_connected,
                "required_clearance_radius_m": float(clearance_radius_m),
                "vehicle_feasible_connectivity": vehicle_connected,
                "start_has_free": start_has_free,
                "end_has_free": end_has_free,
                "start_has_feasible": start_has_feasible,
                "end_has_feasible": end_has_feasible,
                "traversable_cell_count": int(np.count_nonzero(traversable)),
                "feasible_cell_count": int(np.count_nonzero(feasible)),
                "maximum_clearance_m": maximum_clearance,
                "maximum_end_to_end_clearance_radius_m": float(widest),
                "minimum_end_to_end_free_width_m": float(2.0 * widest),
            }
        )
    return reports
