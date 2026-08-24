"""Planner-independent per-aisle blocker root-cause audit.

This module is diagnostic only. It never repairs or mutates the formal
Navigation Map. For each accepted interior aisle it asks one question:

    what is the minimum set of non-FREE cells that prevents an end-to-end
    traversal through the validated aisle geometry, and what evidence owns
    those blocker cells?

The minimum-blocker path uses an 8-neighbour Dijkstra search where FREE cells
cost 0 and non-FREE cells cost 1. This is intentionally different from a route
planner: the result is not a driveable path, only an explanation of the
smallest raster barrier that still separates the longitudinal ends.
"""

from __future__ import annotations

from collections import Counter, deque
import heapq
from typing import Any

import numpy as np

from .navigation_map_derivation import FREE, OCCUPIED, UNKNOWN


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


def _normalized_direction(structure: Any) -> np.ndarray:
    direction = np.asarray(structure.row_model.direction_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("aisle blocker audit row direction must be finite and non-zero")
    return direction / norm


def _grid_uv(navigation: Any, structure: Any) -> tuple[np.ndarray, np.ndarray]:
    occupancy = np.asarray(navigation.occupancy)
    rows, cols = np.indices(occupancy.shape, dtype=np.float64)
    xx = float(navigation.origin_x_m) + (cols + 0.5) * float(navigation.resolution_m)
    yy = float(navigation.origin_y_m) + (rows + 0.5) * float(navigation.resolution_m)
    direction = _normalized_direction(structure)
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


def _minimum_blocker_path(
    owned: np.ndarray,
    occupancy: np.ndarray,
    u: np.ndarray,
    resolution_m: float,
) -> tuple[np.ndarray, int, bool, bool]:
    """Return minimum-blocker path mask and end-availability flags."""

    path_mask = np.zeros_like(owned, dtype=bool)
    if not np.any(owned):
        return path_mask, 0, False, False

    free = occupancy == FREE
    start_zone, end_zone = _end_masks(owned, u, resolution_m)
    start_free = bool(np.any(start_zone & free))
    end_free = bool(np.any(end_zone & free))

    height, width = owned.shape
    inf = np.iinfo(np.int32).max
    dist = np.full((height, width), inf, dtype=np.int32)
    parent_row = np.full((height, width), -1, dtype=np.int32)
    parent_col = np.full((height, width), -1, dtype=np.int32)
    heap: list[tuple[int, int, int]] = []

    for row, col in np.argwhere(start_zone):
        rr = int(row)
        cc = int(col)
        cost = 0 if free[rr, cc] else 1
        if cost < dist[rr, cc]:
            dist[rr, cc] = cost
            heapq.heappush(heap, (cost, rr, cc))

    target: tuple[int, int] | None = None
    while heap:
        cost, row, col = heapq.heappop(heap)
        if cost != int(dist[row, col]):
            continue
        if end_zone[row, col]:
            target = (row, col)
            break
        for dr, dc in _NEIGHBOURS:
            rr = row + dr
            cc = col + dc
            if not (0 <= rr < height and 0 <= cc < width):
                continue
            if not owned[rr, cc]:
                continue
            next_cost = cost + (0 if free[rr, cc] else 1)
            if next_cost < int(dist[rr, cc]):
                dist[rr, cc] = next_cost
                parent_row[rr, cc] = row
                parent_col[rr, cc] = col
                heapq.heappush(heap, (next_cost, rr, cc))

    if target is None:
        return path_mask, 0, start_free, end_free

    row, col = target
    while True:
        path_mask[row, col] = True
        prev_row = int(parent_row[row, col])
        prev_col = int(parent_col[row, col])
        if prev_row < 0 or prev_col < 0:
            break
        row, col = prev_row, prev_col

    blocker_count = int(np.count_nonzero(path_mask & ~free))
    return path_mask, blocker_count, start_free, end_free


def _exclusive_blocker_cause(
    row: int,
    col: int,
    occupancy: np.ndarray,
    provenance: Any,
    materialized: Any | None,
) -> str:
    if materialized is not None:
        if bool(np.asarray(materialized.site_boundary_blocked_mask, dtype=bool)[row, col]):
            return "SITE_BOUNDARY_BLOCK"
        if bool(np.asarray(materialized.row_structural_blocked_mask, dtype=bool)[row, col]):
            return "ROW_STRUCTURAL_BLOCK"

    strong = bool(np.asarray(provenance.strong_sensor_obstacle_mask, dtype=bool)[row, col])
    slope = bool(np.asarray(provenance.slope_hard_mask, dtype=bool)[row, col])
    step = bool(np.asarray(provenance.step_hard_mask, dtype=bool)[row, col])
    soft = bool(np.asarray(provenance.soft_occupied_mask, dtype=bool)[row, col])

    if strong and (slope or step):
        return "SENSOR_AND_TERRAIN_HARD"
    if strong:
        return "STRONG_SENSOR_OBSTACLE"
    if slope and step:
        return "SLOPE_AND_STEP_HARD"
    if slope:
        return "SLOPE_HARD"
    if step:
        return "STEP_HARD"
    if soft:
        return "SOFT_OCCUPIED"
    if int(occupancy[row, col]) == int(UNKNOWN):
        return "UNKNOWN"
    if int(occupancy[row, col]) == int(OCCUPIED):
        return "OTHER_OCCUPIED"
    return "OTHER_NONFREE"


def _failure_mode(*, connected: bool, start_free: bool, end_free: bool) -> str:
    if connected:
        return "CONNECTED"
    if not start_free:
        return "NO_START_FREE"
    if not end_free:
        return "NO_END_FREE"
    return "NO_END_TO_END_COMPONENT"


def _neighbor_count(mask: np.ndarray, owned: np.ndarray, row: int, col: int, radius: int) -> int:
    height, width = mask.shape
    r0 = max(0, row - radius)
    r1 = min(height, row + radius + 1)
    c0 = max(0, col - radius)
    c1 = min(width, col + radius + 1)
    local = np.asarray(mask[r0:r1, c0:c1], dtype=bool) & np.asarray(
        owned[r0:r1, c0:c1], dtype=bool
    )
    count = int(np.count_nonzero(local))
    if bool(mask[row, col]) and bool(owned[row, col]):
        count -= 1
    return max(0, count)


def _component_cells(
    mask: np.ndarray,
    owned: np.ndarray,
    row: int,
    col: int,
) -> list[tuple[int, int]]:
    active = np.asarray(mask, dtype=bool) & np.asarray(owned, dtype=bool)
    if not bool(active[row, col]):
        return []
    height, width = active.shape
    seen = {(int(row), int(col))}
    queue = deque([(int(row), int(col))])
    while queue:
        rr, cc = queue.popleft()
        for dr, dc in _NEIGHBOURS:
            nr = rr + dr
            nc = cc + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            key = (nr, nc)
            if key in seen or not bool(active[nr, nc]):
                continue
            seen.add(key)
            queue.append(key)
    return sorted(seen)


def _component_size(mask: np.ndarray, owned: np.ndarray, row: int, col: int) -> int:
    return len(_component_cells(mask, owned, row, col))


def _component_shape(
    mask: np.ndarray,
    owned: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    row: int,
    col: int,
    resolution_m: float,
) -> dict[str, float | int]:
    cells = _component_cells(mask, owned, row, col)
    if not cells:
        return {
            "size": 0,
            "longitudinal_span_m": 0.0,
            "transverse_span_m": 0.0,
            "transverse_fraction_of_aisle": 0.0,
        }

    rr = np.fromiter((item[0] for item in cells), dtype=np.int64)
    cc = np.fromiter((item[1] for item in cells), dtype=np.int64)
    u_values = np.asarray(u[rr, cc], dtype=np.float64)
    v_values = np.asarray(v[rr, cc], dtype=np.float64)
    longitudinal_span = float(np.max(u_values) - np.min(u_values) + resolution_m)
    transverse_span = float(np.max(v_values) - np.min(v_values) + resolution_m)

    owned_v = np.asarray(v[owned], dtype=np.float64)
    aisle_width = (
        float(np.max(owned_v) - np.min(owned_v) + resolution_m)
        if owned_v.size
        else 0.0
    )
    fraction = (
        min(1.0, max(0.0, transverse_span / aisle_width))
        if aisle_width > 0.0
        else 0.0
    )
    return {
        "size": len(cells),
        "longitudinal_span_m": longitudinal_span,
        "transverse_span_m": transverse_span,
        "transverse_fraction_of_aisle": float(fraction),
    }


def _critical_blocker_cells(
    navigation: Any,
    critical: np.ndarray,
    owned: np.ndarray,
    occupancy: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    provenance: Any,
    materialized: Any | None,
) -> list[dict[str, object]]:
    point_count = np.asarray(getattr(navigation, "point_count", np.zeros_like(occupancy)), dtype=np.int32)
    obstacle_count = np.asarray(getattr(navigation, "obstacle_count", np.zeros_like(occupancy)), dtype=np.int32)
    ground_support = np.asarray(
        getattr(navigation, "ground_support_count", np.zeros_like(occupancy)),
        dtype=np.int32,
    )
    slope = np.asarray(getattr(navigation, "slope_deg", np.zeros_like(occupancy, dtype=float)), dtype=np.float64)
    step = np.asarray(getattr(navigation, "step_m", np.zeros_like(occupancy, dtype=float)), dtype=np.float64)
    strong = np.asarray(
        getattr(provenance, "strong_sensor_obstacle_mask", np.zeros_like(occupancy)),
        dtype=bool,
    )
    soft = np.asarray(
        getattr(provenance, "soft_occupied_mask", np.zeros_like(occupancy)),
        dtype=bool,
    )
    direct = np.asarray(
        getattr(provenance, "direct_obstacle_mask", strong | soft),
        dtype=bool,
    )
    resolution = float(navigation.resolution_m)
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    owned_v = np.asarray(v[owned], dtype=np.float64)
    if owned_v.size:
        geometric_low_v = float(np.min(owned_v) - 0.5 * resolution)
        geometric_high_v = float(np.max(owned_v) + 0.5 * resolution)
    else:
        geometric_low_v = geometric_high_v = 0.0
    geometric_width_v = max(0.0, geometric_high_v - geometric_low_v)

    records: list[dict[str, object]] = []
    for row, col in np.argwhere(critical):
        rr = int(row)
        cc = int(col)
        points = int(point_count[rr, cc])
        obstacles = int(obstacle_count[rr, cc])
        strong_shape = _component_shape(strong, owned, u, v, rr, cc, resolution)
        direct_shape = _component_shape(direct, owned, u, v, rr, cc, resolution)

        section = owned & (np.abs(u - float(u[rr, cc])) <= 0.51 * resolution)
        section_count = int(np.count_nonzero(section))
        section_free = int(np.count_nonzero(section & (occupancy == FREE)))
        section_occupied = int(np.count_nonzero(section & (occupancy == OCCUPIED)))
        section_unknown = int(np.count_nonzero(section & (occupancy == UNKNOWN)))
        section_direct = int(np.count_nonzero(section & direct))
        section_strong = int(np.count_nonzero(section & strong))

        v_here = float(v[rr, cc])
        if geometric_width_v > 0.0:
            normalized_transverse = min(
                1.0,
                max(0.0, (v_here - geometric_low_v) / geometric_width_v),
            )
            distance_to_edge = max(
                0.0,
                min(v_here - geometric_low_v, geometric_high_v - v_here),
            )
            offset_from_mid = v_here - 0.5 * (geometric_low_v + geometric_high_v)
        else:
            normalized_transverse = 0.5
            distance_to_edge = 0.0
            offset_from_mid = 0.0

        records.append(
            {
                "row": rr,
                "col": cc,
                "x_m": float(origin_x + (cc + 0.5) * resolution),
                "y_m": float(origin_y + (rr + 0.5) * resolution),
                "u_m": float(u[rr, cc]),
                "v_m": v_here,
                "cause": _exclusive_blocker_cause(
                    rr,
                    cc,
                    occupancy,
                    provenance,
                    materialized,
                ),
                "occupancy": int(occupancy[rr, cc]),
                "obstacle_count": obstacles,
                "point_count": points,
                "obstacle_ratio": float(obstacles / max(points, 1)),
                "ground_support_count": int(ground_support[rr, cc]),
                "slope_deg": float(slope[rr, cc]),
                "step_m": float(step[rr, cc]),
                "normalized_transverse_position": float(normalized_transverse),
                "offset_from_geometric_mid_m": float(offset_from_mid),
                "distance_to_geometric_edge_m": float(distance_to_edge),
                "cross_section_cell_count": section_count,
                "cross_section_free_cell_count": section_free,
                "cross_section_occupied_cell_count": section_occupied,
                "cross_section_unknown_cell_count": section_unknown,
                "cross_section_direct_obstacle_count": section_direct,
                "cross_section_direct_obstacle_fraction": (
                    float(section_direct / section_count) if section_count else 0.0
                ),
                "cross_section_strong_sensor_count": section_strong,
                "cross_section_strong_sensor_fraction": (
                    float(section_strong / section_count) if section_count else 0.0
                ),
                "direct_obstacle_neighbors_r1": _neighbor_count(direct, owned, rr, cc, 1),
                "direct_obstacle_neighbors_r2": _neighbor_count(direct, owned, rr, cc, 2),
                "direct_obstacle_component_size_in_aisle": int(direct_shape["size"]),
                "direct_obstacle_component_longitudinal_span_m": float(
                    direct_shape["longitudinal_span_m"]
                ),
                "direct_obstacle_component_transverse_span_m": float(
                    direct_shape["transverse_span_m"]
                ),
                "direct_obstacle_component_transverse_fraction_of_aisle": float(
                    direct_shape["transverse_fraction_of_aisle"]
                ),
                "strong_sensor_neighbors_r1": _neighbor_count(strong, owned, rr, cc, 1),
                "strong_sensor_neighbors_r2": _neighbor_count(strong, owned, rr, cc, 2),
                "strong_sensor_component_size_in_aisle": int(strong_shape["size"]),
                "strong_sensor_component_longitudinal_span_m": float(
                    strong_shape["longitudinal_span_m"]
                ),
                "strong_sensor_component_transverse_span_m": float(
                    strong_shape["transverse_span_m"]
                ),
                "strong_sensor_component_transverse_fraction_of_aisle": float(
                    strong_shape["transverse_fraction_of_aisle"]
                ),
            }
        )
    return records


def build_aisle_blocker_audit(
    navigation: Any,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    provenance: Any,
    *,
    materialized: Any | None = None,
) -> list[dict[str, object]]:
    """Return one minimum-blocker root-cause report per accepted interior aisle."""

    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if occupancy.shape != geometric.shape:
        raise ValueError("aisle blocker audit grid shape mismatch")
    if np.asarray(navigation.occupancy).shape != occupancy.shape:
        raise ValueError("aisle blocker audit navigation shape mismatch")

    u, v = _grid_uv(navigation, structure)
    resolution = float(navigation.resolution_m)
    if resolution <= 0.0:
        raise ValueError("aisle blocker audit resolution must be > 0")

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        if str(getattr(diagnostic, "status", "")) not in _ACCEPTED_AISLE_STATES:
            continue

        owned = _owned_mask(geometric, v, diagnostic)
        total = int(np.count_nonzero(owned))
        path_mask, blocker_count, start_free, end_free = _minimum_blocker_path(
            owned,
            occupancy,
            u,
            resolution,
        )
        critical = path_mask & (occupancy != FREE)
        blocker_cells = _critical_blocker_cells(
            navigation,
            critical,
            owned,
            occupancy,
            u,
            v,
            provenance,
            materialized,
        )
        cause_counts: Counter[str] = Counter(
            str(item["cause"]) for item in blocker_cells
        )

        connected = bool(total > 0 and blocker_count == 0 and start_free and end_free)
        if connected:
            dominant = "NONE"
        elif cause_counts:
            dominant = sorted(cause_counts.items(), key=lambda item: (-item[1], item[0]))[0][0]
        else:
            dominant = "NO_PATH_WITHIN_GEOMETRY"

        reports.append(
            {
                "aisle_id": f"aisle_{int(diagnostic.pair_index):03d}",
                "pair_index": int(diagnostic.pair_index),
                "diagnostic_status": str(diagnostic.status),
                "geometric_cell_count": total,
                "free_cell_count": int(np.count_nonzero(owned & (occupancy == FREE))),
                "occupied_cell_count": int(np.count_nonzero(owned & (occupancy == OCCUPIED))),
                "unknown_cell_count": int(np.count_nonzero(owned & (occupancy == UNKNOWN))),
                "start_has_free": start_free,
                "end_has_free": end_free,
                "grid_connectivity": connected,
                "failure_mode": _failure_mode(
                    connected=connected,
                    start_free=start_free,
                    end_free=end_free,
                ),
                "minimum_blocker_cell_count": int(blocker_count),
                "critical_path_cell_count": int(np.count_nonzero(path_mask)),
                "critical_blocker_cause_counts": dict(sorted(cause_counts.items())),
                "critical_blocker_cells": blocker_cells,
                "dominant_blocker_cause": dominant,
            }
        )
    return reports


def aggregate_aisle_blocker_causes(
    reports: list[dict[str, object]],
) -> dict[str, int]:
    """Count dominant failure causes across disconnected interior aisles."""

    counts: Counter[str] = Counter()
    for report in reports:
        if bool(report.get("grid_connectivity")):
            continue
        counts[str(report.get("dominant_blocker_cause", "UNKNOWN_CAUSE"))] += 1
    return dict(sorted(counts.items()))
