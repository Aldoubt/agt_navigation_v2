"""Vehicle-clearance-aware aisle connectivity diagnostics.

The formal Navigation Map remains an environment occupancy product. This module
adds a separate review layer that asks whether a vehicle center can traverse an
aisle while maintaining a requested clearance radius from non-FREE cells and
from the *lateral* geometric aisle boundary.

D1 deliberately does not treat the longitudinal start/end caps of an aisle as
virtual obstacles. Start and end are interior terminal bands inset from the
geometric extrema, while clearance is constrained by real environment evidence
and by lateral aisle width. The audit never mutates or inflates the formal PGM.

D1.1 makes the connectivity scope explicit. ``interior_terminal_raster_connectivity``
means connectivity between the inset terminal bands, not full geometric-extent
QA connectivity. ``raster_grid_connectivity`` is retained as a compatibility
alias for existing review JSON consumers.
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
_CLEARANCE_CONTRACT = "ENVIRONMENT_PLUS_LATERAL_AISLE_BOUNDARY"
_CONNECTIVITY_SCOPE = "INTERIOR_TERMINAL_BANDS"


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


def _nearest_u_band(owned: np.ndarray, u: np.ndarray, target_u: float, resolution_m: float) -> np.ndarray:
    tolerance = 0.51 * float(resolution_m)
    band = owned & (np.abs(u - float(target_u)) <= tolerance)
    if np.any(band):
        return band
    distances = np.where(owned, np.abs(u - float(target_u)), np.inf)
    nearest = float(np.min(distances))
    if not np.isfinite(nearest):
        return np.zeros_like(owned, dtype=bool)
    return owned & (np.abs(distances - nearest) <= 1.0e-9)


def _terminal_masks(
    owned: np.ndarray,
    u: np.ndarray,
    resolution_m: float,
    terminal_inset_m: float,
) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    """Return interior start/end terminal bands without virtual end-cap clearance."""

    if not np.any(owned):
        empty = np.zeros_like(owned, dtype=bool)
        return empty, empty, 0.0, 0.0, 0.0
    u_min = float(np.min(u[owned]))
    u_max = float(np.max(u[owned]))
    span = max(0.0, u_max - u_min)
    maximum_inset = max(0.0, 0.5 * (span - float(resolution_m)))
    effective_inset = min(float(terminal_inset_m), maximum_inset)
    start_u = u_min + effective_inset
    end_u = u_max - effective_inset
    return (
        _nearest_u_band(owned, u, start_u, resolution_m),
        _nearest_u_band(owned, u, end_u, resolution_m),
        effective_inset,
        start_u,
        end_u,
    )


def _environment_clearance_field_m(
    accepted_occupancy: np.ndarray,
    resolution_m: float,
) -> np.ndarray:
    """Distance from FREE cell centers to real non-FREE/map-exterior evidence."""

    ndimage = _require_scipy()
    free = np.asarray(accepted_occupancy, dtype=np.uint8) == FREE
    padded = np.pad(free, 1, constant_values=False)
    distance_centers = ndimage.distance_transform_edt(padded)[1:-1, 1:-1]
    clearance = distance_centers * float(resolution_m) - 0.5 * float(resolution_m)
    return np.maximum(clearance, 0.0)


def _lateral_clearance_field_m(
    owned: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    resolution_m: float,
) -> np.ndarray:
    """Approximate local clearance to aisle side boundaries, excluding end caps."""

    clearance = np.zeros_like(u, dtype=np.float64)
    if not np.any(owned):
        return clearance
    resolution = float(resolution_m)
    u_min = float(np.min(u[owned]))
    bin_index = np.floor((u - u_min) / resolution + 1.0e-9).astype(np.int64)
    owned_bins = bin_index[owned]
    bin_count = int(np.max(owned_bins)) + 1
    minimum_v = np.full(bin_count, np.inf, dtype=np.float64)
    maximum_v = np.full(bin_count, -np.inf, dtype=np.float64)
    np.minimum.at(minimum_v, owned_bins, v[owned])
    np.maximum.at(maximum_v, owned_bins, v[owned])

    local_minimum = minimum_v.copy()
    local_maximum = maximum_v.copy()
    for shift in (-1, 1):
        source = np.arange(bin_count, dtype=np.int64) + shift
        valid = (source >= 0) & (source < bin_count)
        local_minimum[valid] = np.minimum(local_minimum[valid], minimum_v[source[valid]])
        local_maximum[valid] = np.maximum(local_maximum[valid], maximum_v[source[valid]])

    bins = np.clip(bin_index[owned], 0, bin_count - 1)
    low_edge = local_minimum[bins] - 0.5 * resolution
    high_edge = local_maximum[bins] + 0.5 * resolution
    values = np.minimum(v[owned] - low_edge, high_edge - v[owned])
    clearance[owned] = np.maximum(values, 0.0)
    return clearance


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
    terminal_inset_m: float = 0.50,
) -> list[dict[str, object]]:
    """Return D1.1 interior-terminal and vehicle-clearance connectivity for ROW_ROW aisles."""

    if clearance_radius_m < 0.0:
        raise ValueError("clearance_radius_m must be >= 0")
    if terminal_inset_m < 0.0:
        raise ValueError("terminal_inset_m must be >= 0")
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
    environment_clearance = _environment_clearance_field_m(occupancy, resolution)

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        if str(getattr(diagnostic, "status", "")) not in _ACCEPTED_AISLE_STATES:
            continue

        owned = _owned_mask(geometric, v, diagnostic)
        traversable = owned & (occupancy == FREE)
        start_zone, end_zone, effective_inset, start_u, end_u = _terminal_masks(
            owned, u, resolution, terminal_inset_m
        )
        start_has_free = bool(np.any(start_zone & traversable))
        end_has_free = bool(np.any(end_zone & traversable))

        lateral_clearance = _lateral_clearance_field_m(owned, u, v, resolution)
        clearance = np.minimum(environment_clearance, lateral_clearance)
        clearance = np.where(traversable, clearance, 0.0)
        widest = _widest_end_to_end_clearance(traversable, clearance, start_zone, end_zone)
        interior_terminal_connected = bool(start_has_free and end_has_free and widest > 0.0)

        feasible = traversable & (clearance + 1.0e-12 >= float(clearance_radius_m))
        start_has_feasible = bool(np.any(start_zone & feasible))
        end_has_feasible = bool(np.any(end_zone & feasible))
        vehicle_connected = bool(
            interior_terminal_connected
            and start_has_feasible
            and end_has_feasible
            and widest + 1.0e-12 >= float(clearance_radius_m)
        )

        maximum_clearance = float(np.max(clearance[traversable])) if np.any(traversable) else 0.0
        reports.append(
            {
                "aisle_id": f"aisle_{int(diagnostic.pair_index):03d}",
                "pair_index": int(diagnostic.pair_index),
                "diagnostic_status": str(diagnostic.status),
                "clearance_contract": _CLEARANCE_CONTRACT,
                "connectivity_scope": _CONNECTIVITY_SCOPE,
                "interior_terminal_raster_connectivity": interior_terminal_connected,
                "raster_grid_connectivity": interior_terminal_connected,
                "required_clearance_radius_m": float(clearance_radius_m),
                "terminal_inset_m": float(terminal_inset_m),
                "effective_terminal_inset_m": float(effective_inset),
                "start_terminal_u_m": float(start_u),
                "end_terminal_u_m": float(end_u),
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
