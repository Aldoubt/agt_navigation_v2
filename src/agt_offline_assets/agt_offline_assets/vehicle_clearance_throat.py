"""D3.2 explainable clearance-throat diagnostics for aisle-aligned vehicle review.

The formal Navigation Map and D2 vertical evidence remain unchanged. This
module consumes the same D1.1 clearance contract used by vehicle feasibility
and explains *where* an interior-terminal path becomes too narrow for the
requested vehicle radius.

D3.2 separates two kinds of evidence that D3.1 previously conflated:

* ``nearest_environment_constraint`` is the exact non-FREE cell selected by the
  Euclidean distance transform (or ``MAP_EXTERIOR`` when padding is nearest);
* ``LOWER_V`` / ``UPPER_V`` constraints remain same-u cross-section context for
  interpreting the local aisle shape.

For every accepted ROW_ROW aisle it distinguishes three states:

* ``VEHICLE_FEASIBLE`` -- an end-to-end path satisfies the requested radius;
* ``CLEARANCE_THROAT`` -- raster connectivity exists, but every path contains a
  narrower bottleneck; and
* ``NO_INTERIOR_TERMINAL_PATH`` -- even the review raster is disconnected.

The audit is derived review evidence only and never mutates the formal PGM.
"""

from __future__ import annotations

from collections import Counter, deque
from pathlib import Path
from typing import Any

import numpy as np

from .navigation_map_derivation import FREE
from .vehicle_feasibility import (
    _ACCEPTED_AISLE_STATES,
    _CONNECTIVITY_SCOPE,
    _grid_uv,
    _lateral_clearance_field_m,
    _owned_mask,
    _require_scipy,
    _terminal_masks,
    _widest_end_to_end_clearance,
)


_SCHEMA = "agt_vehicle_clearance_throat_audit/v2"
_AUTHORITY = "DERIVED_VEHICLE_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY"
_ENVIRONMENT_ATTRIBUTION_CONTRACT = "EDT_NEAREST_NON_FREE_CELL"
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


def _mask(owner: Any | None, name: str, shape: tuple[int, int]) -> np.ndarray:
    if owner is None or not hasattr(owner, name):
        return np.zeros(shape, dtype=bool)
    values = np.asarray(getattr(owner, name), dtype=bool)
    if values.shape != shape:
        raise ValueError(f"clearance throat source mask {name} shape mismatch")
    return values


def _source_masks(
    shape: tuple[int, int],
    *,
    materialized: Any | None,
    provenance: Any | None,
) -> list[tuple[str, np.ndarray]]:
    slope = _mask(provenance, "slope_hard_mask", shape)
    step = _mask(provenance, "step_hard_mask", shape)
    return [
        ("SITE_BOUNDARY_BLOCK", _mask(materialized, "site_boundary_blocked_mask", shape)),
        ("ROW_STRUCTURAL_BLOCK", _mask(materialized, "row_structural_blocked_mask", shape)),
        ("SLOPE_AND_STEP_HARD", slope & step),
        ("SLOPE_HARD", slope & ~step),
        ("STEP_HARD", step & ~slope),
        (
            "STRONG_SENSOR_OBSTACLE",
            _mask(provenance, "strong_sensor_obstacle_mask", shape),
        ),
        ("SOFT_OCCUPIED", _mask(provenance, "soft_occupied_mask", shape)),
        ("UNRESOLVED_UNKNOWN", _mask(materialized, "unresolved_unknown_mask", shape)),
        ("BASE_HARD_OCCUPIED", _mask(materialized, "base_hard_occupied_mask", shape)),
        ("BASE_SOFT_OCCUPIED", _mask(materialized, "base_soft_occupied_mask", shape)),
    ]


def _source_at(
    row: int,
    col: int,
    occupancy: np.ndarray,
    source_masks: list[tuple[str, np.ndarray]],
) -> str:
    for name, mask in source_masks:
        if bool(mask[row, col]):
            return name
    if np.asarray(occupancy, dtype=np.uint8)[row, col] != FREE:
        return "OTHER_NON_FREE"
    return "FREE"


def _environment_clearance_with_nearest(
    accepted_occupancy: np.ndarray,
    resolution_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return D1.1 environment clearance plus exact EDT nearest-cell indices.

    The one-cell non-FREE padding is identical to the D1.1 map-exterior
    clearance contract. Returned nearest row/col indices are in the original
    unpadded grid frame; values outside ``[0, height) x [0, width)`` therefore
    identify map-exterior padding rather than a real grid cell.
    """

    resolution = float(resolution_m)
    if resolution <= 0.0:
        raise ValueError("environment clearance resolution must be > 0")
    ndimage = _require_scipy()
    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    free = occupancy == FREE
    padded = np.pad(free, 1, constant_values=False)
    distance_centers, indices = ndimage.distance_transform_edt(
        padded,
        return_indices=True,
    )
    center_distance = np.asarray(distance_centers[1:-1, 1:-1], dtype=np.float64)
    nearest_row = np.asarray(indices[0, 1:-1, 1:-1], dtype=np.int64) - 1
    nearest_col = np.asarray(indices[1, 1:-1, 1:-1], dtype=np.int64) - 1
    clearance = center_distance * resolution - 0.5 * resolution
    return np.maximum(clearance, 0.0), nearest_row, nearest_col


def _threshold_path(
    traversable: np.ndarray,
    clearance_m: np.ndarray,
    start_zone: np.ndarray,
    end_zone: np.ndarray,
    widest_m: float,
) -> list[tuple[int, int]]:
    """Recover one deterministic path that realizes the widest bottleneck."""

    if widest_m <= 0.0:
        return []
    allowed = (
        np.asarray(traversable, dtype=bool)
        & (np.asarray(clearance_m, dtype=np.float64) + 1.0e-12 >= float(widest_m))
    )
    start = np.asarray(start_zone, dtype=bool) & allowed
    end = np.asarray(end_zone, dtype=bool) & allowed
    if not np.any(start) or not np.any(end):
        return []

    height, width = allowed.shape
    visited = np.zeros_like(allowed, dtype=bool)
    parent_row = np.full((height, width), -1, dtype=np.int32)
    parent_col = np.full((height, width), -1, dtype=np.int32)
    queue: deque[tuple[int, int]] = deque()
    for row, col in np.argwhere(start):
        rr = int(row)
        cc = int(col)
        visited[rr, cc] = True
        queue.append((rr, cc))

    goal: tuple[int, int] | None = None
    while queue:
        row, col = queue.popleft()
        if bool(end[row, col]):
            goal = (row, col)
            break
        for dr, dc in _NEIGHBOURS:
            rr = row + dr
            cc = col + dc
            if not (0 <= rr < height and 0 <= cc < width):
                continue
            if not allowed[rr, cc] or visited[rr, cc]:
                continue
            visited[rr, cc] = True
            parent_row[rr, cc] = row
            parent_col[rr, cc] = col
            queue.append((rr, cc))

    if goal is None:
        return []
    path: list[tuple[int, int]] = []
    row, col = goal
    while True:
        path.append((row, col))
        pr = int(parent_row[row, col])
        pc = int(parent_col[row, col])
        if pr < 0 or pc < 0:
            break
        row, col = pr, pc
    path.reverse()
    return path


def _limiting_constraint(environment_m: float, lateral_m: float) -> str:
    tolerance = 1.0e-9
    if environment_m + tolerance < lateral_m:
        return "ENVIRONMENT_NON_FREE"
    if lateral_m + tolerance < environment_m:
        return "LATERAL_AISLE_BOUNDARY"
    return "MIXED_ENVIRONMENT_AND_LATERAL"


def _constraint_on_side(
    *,
    side: str,
    throat_row: int,
    throat_col: int,
    owned: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    occupancy: np.ndarray,
    resolution_m: float,
    source_masks: list[tuple[str, np.ndarray]],
    navigation: Any,
) -> dict[str, object]:
    """Return same-u transverse context; this does not define EDT provenance."""

    throat_u = float(u[throat_row, throat_col])
    throat_v = float(v[throat_row, throat_col])
    tolerance_u = 0.51 * float(resolution_m)
    cross_section = np.abs(u - throat_u) <= tolerance_u

    local_owned = owned & cross_section
    if not np.any(local_owned):
        return {
            "side": side,
            "source": "NO_LOCAL_AISLE_GEOMETRY",
            "distance_m": None,
        }
    local_v = v[local_owned]
    low_edge = float(np.min(local_v)) - 0.5 * float(resolution_m)
    high_edge = float(np.max(local_v)) + 0.5 * float(resolution_m)
    boundary_distance = (
        throat_v - low_edge if side == "LOWER_V" else high_edge - throat_v
    )
    boundary_distance = max(0.0, float(boundary_distance))

    non_free = cross_section & (np.asarray(occupancy, dtype=np.uint8) != FREE)
    if side == "LOWER_V":
        candidates = np.argwhere(non_free & (v < throat_v - 1.0e-12))
    else:
        candidates = np.argwhere(non_free & (v > throat_v + 1.0e-12))

    environment_distance = float("inf")
    environment_cell: tuple[int, int] | None = None
    if candidates.size:
        distances = np.abs(v[candidates[:, 0], candidates[:, 1]] - throat_v)
        index = int(np.argmin(distances))
        rr = int(candidates[index, 0])
        cc = int(candidates[index, 1])
        environment_distance = max(
            0.0,
            float(distances[index]) - 0.5 * float(resolution_m),
        )
        environment_cell = (rr, cc)

    if environment_cell is not None and environment_distance <= boundary_distance + 1.0e-12:
        rr, cc = environment_cell
        return {
            "side": side,
            "source": _source_at(rr, cc, occupancy, source_masks),
            "distance_m": float(environment_distance),
            "row": rr,
            "col": cc,
            "world_x_m": float(navigation.origin_x_m)
            + (cc + 0.5) * float(resolution_m),
            "world_y_m": float(navigation.origin_y_m)
            + (rr + 0.5) * float(resolution_m),
            "u_m": float(u[rr, cc]),
            "v_m": float(v[rr, cc]),
        }
    return {
        "side": side,
        "source": "LATERAL_AISLE_BOUNDARY",
        "distance_m": float(boundary_distance),
        "boundary_v_m": float(low_edge if side == "LOWER_V" else high_edge),
    }


def _nearest_environment_constraint(
    *,
    throat_row: int,
    throat_col: int,
    environment_clearance_m: np.ndarray,
    nearest_row: np.ndarray,
    nearest_col: np.ndarray,
    occupancy: np.ndarray,
    source_masks: list[tuple[str, np.ndarray]],
    navigation: Any,
    u: np.ndarray,
    v: np.ndarray,
) -> dict[str, object]:
    """Describe the exact non-FREE source selected by the EDT at a throat."""

    rr = int(nearest_row[throat_row, throat_col])
    cc = int(nearest_col[throat_row, throat_col])
    resolution = float(navigation.resolution_m)
    clearance = float(environment_clearance_m[throat_row, throat_col])
    center_distance = clearance + 0.5 * resolution
    height, width = np.asarray(occupancy).shape
    if not (0 <= rr < height and 0 <= cc < width):
        return {
            "source": "MAP_EXTERIOR",
            "distance_m": clearance,
            "center_distance_m": center_distance,
            "row": None,
            "col": None,
        }
    return {
        "source": _source_at(rr, cc, occupancy, source_masks),
        "distance_m": clearance,
        "center_distance_m": center_distance,
        "row": rr,
        "col": cc,
        "world_x_m": float(navigation.origin_x_m) + (cc + 0.5) * resolution,
        "world_y_m": float(navigation.origin_y_m) + (rr + 0.5) * resolution,
        "u_m": float(u[rr, cc]),
        "v_m": float(v[rr, cc]),
    }


def _primary_throat(
    path: list[tuple[int, int]],
    clearance: np.ndarray,
    widest: float,
) -> tuple[int, int] | None:
    if not path:
        return None
    candidates = [
        (index, row, col)
        for index, (row, col) in enumerate(path)
        if float(clearance[row, col]) <= float(widest) + 1.0e-9
    ]
    if not candidates:
        return None
    midpoint = 0.5 * (len(path) - 1)
    _, row, col = min(candidates, key=lambda item: (abs(item[0] - midpoint), item[0]))
    return int(row), int(col)


def build_vehicle_clearance_throat_audit(
    navigation: Any,
    structure: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    *,
    clearance_radius_m: float,
    terminal_inset_m: float = 0.50,
    materialized: Any | None = None,
    provenance: Any | None = None,
) -> dict[str, object]:
    """Explain aisle bottlenecks under the same D1.1 clearance contract."""

    if clearance_radius_m < 0.0:
        raise ValueError("clearance_radius_m must be >= 0")
    if terminal_inset_m < 0.0:
        raise ValueError("terminal_inset_m must be >= 0")
    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    if occupancy.shape != geometric.shape:
        raise ValueError("clearance throat grid shape mismatch")
    if np.asarray(navigation.occupancy).shape != occupancy.shape:
        raise ValueError("clearance throat navigation shape mismatch")

    resolution = float(navigation.resolution_m)
    if resolution <= 0.0:
        raise ValueError("clearance throat resolution must be > 0")
    u, v = _grid_uv(navigation, structure)
    environment_clearance, nearest_row, nearest_col = _environment_clearance_with_nearest(
        occupancy,
        resolution,
    )
    source_masks = _source_masks(
        occupancy.shape,
        materialized=materialized,
        provenance=provenance,
    )

    reports: list[dict[str, object]] = []
    for diagnostic in corridor.aisle_pair_diagnostics:
        if str(getattr(diagnostic, "pair_kind", "ROW_ROW")) != "ROW_ROW":
            continue
        if str(getattr(diagnostic, "status", "")) not in _ACCEPTED_AISLE_STATES:
            continue

        owned = _owned_mask(geometric, v, diagnostic)
        traversable = owned & (occupancy == FREE)
        start_zone, end_zone, effective_inset, start_u, end_u = _terminal_masks(
            owned,
            u,
            resolution,
            terminal_inset_m,
        )
        lateral_clearance = _lateral_clearance_field_m(owned, u, v, resolution)
        clearance = np.minimum(environment_clearance, lateral_clearance)
        clearance = np.where(traversable, clearance, 0.0)
        widest = _widest_end_to_end_clearance(
            traversable,
            clearance,
            start_zone,
            end_zone,
        )
        path = _threshold_path(
            traversable,
            clearance,
            start_zone,
            end_zone,
            widest,
        )
        connected = bool(widest > 0.0 and path)
        vehicle_feasible = bool(
            connected and widest + 1.0e-12 >= float(clearance_radius_m)
        )
        if not connected:
            status = "NO_INTERIOR_TERMINAL_PATH"
        elif vehicle_feasible:
            status = "VEHICLE_FEASIBLE"
        else:
            status = "CLEARANCE_THROAT"

        report: dict[str, object] = {
            "aisle_id": f"aisle_{int(diagnostic.pair_index):03d}",
            "pair_index": int(diagnostic.pair_index),
            "status": status,
            "connectivity_scope": _CONNECTIVITY_SCOPE,
            "required_clearance_radius_m": float(clearance_radius_m),
            "terminal_inset_m": float(terminal_inset_m),
            "effective_terminal_inset_m": float(effective_inset),
            "start_terminal_u_m": float(start_u),
            "end_terminal_u_m": float(end_u),
            "interior_terminal_raster_connectivity": connected,
            "vehicle_feasible_connectivity": vehicle_feasible,
            "bottleneck_clearance_m": float(widest),
            "clearance_deficit_m": float(max(0.0, float(clearance_radius_m) - widest)),
            "widest_path_cell_count": len(path),
            "primary_throat": None,
            "limiting_constraint": None,
        }
        throat = _primary_throat(path, clearance, widest)
        if throat is not None:
            row, col = throat
            env_value = float(environment_clearance[row, col])
            lateral_value = float(lateral_clearance[row, col])
            limiting = _limiting_constraint(env_value, lateral_value)
            report["limiting_constraint"] = limiting
            nearest_environment = None
            if limiting in {"ENVIRONMENT_NON_FREE", "MIXED_ENVIRONMENT_AND_LATERAL"}:
                nearest_environment = _nearest_environment_constraint(
                    throat_row=row,
                    throat_col=col,
                    environment_clearance_m=environment_clearance,
                    nearest_row=nearest_row,
                    nearest_col=nearest_col,
                    occupancy=occupancy,
                    source_masks=source_masks,
                    navigation=navigation,
                    u=u,
                    v=v,
                )
            report["primary_throat"] = {
                "row": row,
                "col": col,
                "world_x_m": float(navigation.origin_x_m) + (col + 0.5) * resolution,
                "world_y_m": float(navigation.origin_y_m) + (row + 0.5) * resolution,
                "u_m": float(u[row, col]),
                "v_m": float(v[row, col]),
                "clearance_m": float(clearance[row, col]),
                "environment_clearance_m": env_value,
                "lateral_clearance_m": lateral_value,
                "nearest_environment_constraint": nearest_environment,
                "lower_v_constraint": _constraint_on_side(
                    side="LOWER_V",
                    throat_row=row,
                    throat_col=col,
                    owned=owned,
                    u=u,
                    v=v,
                    occupancy=occupancy,
                    resolution_m=resolution,
                    source_masks=source_masks,
                    navigation=navigation,
                ),
                "upper_v_constraint": _constraint_on_side(
                    side="UPPER_V",
                    throat_row=row,
                    throat_col=col,
                    owned=owned,
                    u=u,
                    v=v,
                    occupancy=occupancy,
                    resolution_m=resolution,
                    source_masks=source_masks,
                    navigation=navigation,
                ),
            }
        reports.append(report)

    nearest_causes: Counter[str] = Counter()
    for report in reports:
        if report.get("status") != "CLEARANCE_THROAT":
            continue
        throat = report.get("primary_throat")
        if not isinstance(throat, dict):
            continue
        nearest = throat.get("nearest_environment_constraint")
        if not isinstance(nearest, dict):
            continue
        source = str(nearest.get("source", "UNKNOWN"))
        nearest_causes[source] += 1

    return {
        "schema": _SCHEMA,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": _AUTHORITY,
        "connectivity_scope": _CONNECTIVITY_SCOPE,
        "environment_attribution_contract": _ENVIRONMENT_ATTRIBUTION_CONTRACT,
        "clearance_radius_m": float(clearance_radius_m),
        "terminal_inset_m": float(terminal_inset_m),
        "aisle_count": len(reports),
        "clearance_throat_aisles": sum(item["status"] == "CLEARANCE_THROAT" for item in reports),
        "no_interior_terminal_path_aisles": sum(
            item["status"] == "NO_INTERIOR_TERMINAL_PATH" for item in reports
        ),
        "vehicle_feasible_aisles": sum(item["status"] == "VEHICLE_FEASIBLE" for item in reports),
        "nearest_environment_constraint_causes": dict(sorted(nearest_causes.items())),
        "aisles": reports,
    }


def write_vehicle_clearance_throat_overlays(
    audit: dict[str, object],
    navigation: Any,
    corridor: Any,
    accepted_occupancy: np.ndarray,
    output_dir: str | Path,
    *,
    half_window_m: float = 1.50,
) -> list[Path]:
    """Write compact local PNG crops for clearance-throat cases only."""

    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("clearance throat overlays require Pillow") from exc
    if half_window_m <= 0.0:
        raise ValueError("half_window_m must be > 0")

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    occupancy = np.asarray(accepted_occupancy, dtype=np.uint8)
    geometric = np.asarray(corridor.aisle_geometric_envelope, dtype=bool)
    resolution = float(navigation.resolution_m)
    radius = max(1, int(np.ceil(float(half_window_m) / resolution)))
    written: list[Path] = []

    for report in audit.get("aisles", []):
        if report.get("status") != "CLEARANCE_THROAT":
            continue
        throat = report.get("primary_throat")
        if not isinstance(throat, dict):
            continue
        row = int(throat["row"])
        col = int(throat["col"])
        r0 = max(0, row - radius)
        r1 = min(occupancy.shape[0], row + radius + 1)
        c0 = max(0, col - radius)
        c1 = min(occupancy.shape[1], col + radius + 1)
        local_occ = occupancy[r0:r1, c0:c1]
        local_geo = geometric[r0:r1, c0:c1]

        rgb = np.full((*local_occ.shape, 3), 235, dtype=np.uint8)
        rgb[~local_geo] = (170, 170, 170)
        rgb[local_occ != FREE] = (45, 45, 45)
        image = Image.fromarray(np.flipud(rgb), mode="RGB").resize(
            (rgb.shape[1] * 8, rgb.shape[0] * 8),
            resample=Image.Resampling.NEAREST,
        )
        draw = ImageDraw.Draw(image)
        x = (col - c0 + 0.5) * 8
        y = (r1 - 1 - row + 0.5) * 8
        draw.line((x - 7, y, x + 7, y), fill=(220, 40, 40), width=2)
        draw.line((x, y - 7, x, y + 7), fill=(220, 40, 40), width=2)
        path = output / f"{report['aisle_id']}_throat.png"
        image.save(path)
        written.append(path)
    return written
