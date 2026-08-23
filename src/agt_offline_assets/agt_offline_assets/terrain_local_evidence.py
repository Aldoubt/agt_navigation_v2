"""Raw-PCD local terrain corroboration for E3-EXACT terrain blockers.

This module is diagnostic only. It never changes Navigation Map occupancy,
terrain thresholds, formal map materialization, or Accepted PGM assets.

For each selected terrain blocker it builds an independent local lower-envelope
surface directly from raw PCD returns, fits a local plane for slope evidence,
and evaluates the same three-point second-difference semantics used by the A3
local-linear Step metric. The result answers whether a raster SLOPE/STEP HARD
classification is corroborated by local raw geometry rather than merely by the
smoothed Ground surface used during map derivation.
"""

from __future__ import annotations

from collections import Counter
from math import atan, ceil, degrees, floor, hypot
from typing import Any, Iterator

import numpy as np


_SCHEMA = "agt_terrain_local_evidence_audit/v1"
_STATUS = "EXPERIMENTAL_REVIEW_EVIDENCE"
_AUTHORITY = "DERIVED_TERRAIN_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY"
_TERRAIN_CAUSES = {
    "SLOPE_HARD",
    "STEP_HARD",
    "SLOPE_AND_STEP_HARD",
    "SENSOR_AND_TERRAIN_HARD",
}
_STEP_DIRECTIONS = (
    (0, -1, 0, 1, "X"),
    (-1, 0, 1, 0, "Y"),
    (-1, -1, 1, 1, "DIAG_POS"),
    (-1, 1, 1, -1, "DIAG_NEG"),
)


def _iter_xyz_chunks(cloud: Any, chunk_size: int) -> Iterator[np.ndarray]:
    if chunk_size < 1:
        raise ValueError("terrain local evidence chunk_size must be >= 1")
    points = getattr(cloud, "points", None)
    names = getattr(getattr(points, "dtype", None), "names", None)
    if points is not None and names and all(name in names for name in ("x", "y", "z")):
        total = len(points)
        for start in range(0, total, chunk_size):
            block = points[start : start + chunk_size]
            yield np.column_stack(
                [
                    np.asarray(block["x"], dtype=np.float64),
                    np.asarray(block["y"], dtype=np.float64),
                    np.asarray(block["z"], dtype=np.float64),
                ]
            )
        return

    xyz = np.asarray(cloud.xyz(), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("terrain local evidence requires Nx3 XYZ points")
    for start in range(0, xyz.shape[0], chunk_size):
        yield xyz[start : start + chunk_size, :3]


def _is_terrain_cause(value: object) -> bool:
    return str(value or "").strip().upper() in _TERRAIN_CAUSES


def select_terrain_review_targets(
    throat_audit: dict[str, object],
    blocker_audit: dict[str, object],
) -> list[dict[str, object]]:
    """Select final E3 terrain cases without hard-coding aisle IDs.

    Clearance-throat cases contribute their exact nearest environment cell when
    that source is terrain HARD. For raster-disconnected cases, only aisles that
    the *interior terminal* throat audit marks ``NO_INTERIOR_TERMINAL_PATH`` are
    eligible for B0 minimum-blocker terrain cells. This intentionally excludes
    full-extent-only endpoint discrepancies such as the earlier aisle_017 case.
    """

    throat_reports = throat_audit.get("aisles") or []
    blocker_reports = blocker_audit.get("aisles") or []
    if not isinstance(throat_reports, list) or not isinstance(blocker_reports, list):
        raise ValueError("terrain target selection requires aisle report lists")

    targets: list[dict[str, object]] = []
    interior_disconnected: set[str] = set()
    for report in throat_reports:
        if not isinstance(report, dict):
            continue
        aisle_id = str(report.get("aisle_id", ""))
        status = str(report.get("status", ""))
        if status == "NO_INTERIOR_TERMINAL_PATH":
            interior_disconnected.add(aisle_id)
            continue
        if status != "CLEARANCE_THROAT":
            continue
        primary = report.get("primary_throat")
        if not isinstance(primary, dict):
            continue
        nearest = primary.get("nearest_environment_constraint")
        if not isinstance(nearest, dict) or not _is_terrain_cause(nearest.get("source")):
            continue
        targets.append(
            {
                "aisle_id": aisle_id,
                "row": int(nearest["row"]),
                "col": int(nearest["col"]),
                "raster_cause": str(nearest["source"]),
                "target_source": "CLEARANCE_THROAT_NEAREST_ENVIRONMENT",
            }
        )

    for report in blocker_reports:
        if not isinstance(report, dict):
            continue
        aisle_id = str(report.get("aisle_id", ""))
        if aisle_id not in interior_disconnected:
            continue
        blockers = report.get("critical_blocker_cells") or []
        if not isinstance(blockers, list):
            continue
        for blocker in blockers:
            if not isinstance(blocker, dict) or not _is_terrain_cause(blocker.get("cause")):
                continue
            targets.append(
                {
                    "aisle_id": aisle_id,
                    "row": int(blocker["row"]),
                    "col": int(blocker["col"]),
                    "raster_cause": str(blocker["cause"]),
                    "target_source": "INTERIOR_DISCONNECTED_B0_MINIMUM_BLOCKER",
                }
            )

    unique: dict[tuple[str, int, int, str], dict[str, object]] = {}
    for target in targets:
        key = (
            str(target["aisle_id"]),
            int(target["row"]),
            int(target["col"]),
            str(target["target_source"]),
        )
        unique[key] = target
    return sorted(
        unique.values(),
        key=lambda item: (
            str(item["aisle_id"]),
            int(item["row"]),
            int(item["col"]),
            str(item["target_source"]),
        ),
    )


def _discrete_lower_quantile(values: list[np.ndarray], quantile: float) -> float:
    if not values:
        raise ValueError("cannot compute lower quantile from no values")
    merged = np.concatenate(values).astype(np.float64, copy=False)
    merged = merged[np.isfinite(merged)]
    if merged.size == 0:
        raise ValueError("cannot compute lower quantile from no finite values")
    merged.sort()
    rank = int(floor(float(quantile) * (merged.size - 1)))
    return float(merged[rank])


def _fit_plane(
    surface: dict[tuple[int, int], float],
    *,
    origin_x_m: float,
    origin_y_m: float,
    resolution_m: float,
) -> tuple[float | None, float | None, dict[str, float] | None]:
    if len(surface) < 3:
        return None, None, None
    rows = np.asarray([key[0] for key in surface], dtype=np.float64)
    cols = np.asarray([key[1] for key in surface], dtype=np.float64)
    z = np.asarray(list(surface.values()), dtype=np.float64)
    x = origin_x_m + (cols + 0.5) * resolution_m
    y = origin_y_m + (rows + 0.5) * resolution_m
    design = np.column_stack([x, y, np.ones_like(x)])
    if np.linalg.matrix_rank(design) < 3:
        return None, None, None
    coeff, _, _, _ = np.linalg.lstsq(design, z, rcond=None)
    a, b, c = (float(value) for value in coeff)
    predicted = design @ coeff
    residual = z - predicted
    slope_deg = degrees(atan(hypot(a, b)))
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    return float(slope_deg), rmse, {"dz_dx": a, "dz_dy": b, "intercept": c}


def _raw_second_difference(
    surface: dict[tuple[int, int], float],
    row: int,
    col: int,
) -> tuple[float | None, list[dict[str, object]]]:
    centre = surface.get((row, col))
    if centre is None:
        return None, []
    values: list[dict[str, object]] = []
    for dr1, dc1, dr2, dc2, name in _STEP_DIRECTIONS:
        first = surface.get((row + dr1, col + dc1))
        second = surface.get((row + dr2, col + dc2))
        if first is None or second is None:
            continue
        residual = abs(float(first) - 2.0 * float(centre) + float(second))
        values.append({"direction": name, "second_difference_m": float(residual)})
    if not values:
        return None, []
    return max(float(item["second_difference_m"]) for item in values), values


def _corroboration_status(
    *,
    raster_slope_hard: bool,
    raster_step_hard: bool,
    raw_slope_hard: bool | None,
    raw_step_hard: bool | None,
) -> str:
    expected: list[bool | None] = []
    if raster_slope_hard:
        expected.append(raw_slope_hard)
    if raster_step_hard:
        expected.append(raw_step_hard)
    if not expected or any(value is None for value in expected):
        return "INSUFFICIENT_RAW_SURFACE"
    matched = sum(bool(value) for value in expected)
    if matched == len(expected):
        return "FULLY_CORROBORATED"
    if matched > 0:
        return "PARTIALLY_CORROBORATED"
    return "NOT_CORROBORATED"


def build_terrain_local_evidence_audit(
    cloud: Any,
    navigation: Any,
    targets: list[dict[str, object]],
    *,
    local_half_window_m: float = 0.35,
    lower_quantile: float = 0.10,
    minimum_points_per_cell: int = 3,
    minimum_surface_cells: int = 9,
    chunk_size: int = 1_000_000,
) -> dict[str, object]:
    """Corroborate raster terrain HARD cells with a raw lower-envelope surface."""

    if local_half_window_m <= 0.0:
        raise ValueError("terrain local half window must be > 0")
    if not 0.0 <= lower_quantile <= 1.0:
        raise ValueError("terrain lower quantile must be in [0, 1]")
    if minimum_points_per_cell < 1:
        raise ValueError("minimum_points_per_cell must be >= 1")
    if minimum_surface_cells < 3:
        raise ValueError("minimum_surface_cells must be >= 3")
    if chunk_size < 1:
        raise ValueError("chunk_size must be >= 1")

    occupancy = np.asarray(navigation.occupancy)
    slope_grid = np.asarray(navigation.slope_deg, dtype=np.float64)
    step_grid = np.asarray(navigation.step_m, dtype=np.float64)
    if slope_grid.shape != occupancy.shape or step_grid.shape != occupancy.shape:
        raise ValueError("terrain local evidence grid shapes differ")

    resolution = float(navigation.resolution_m)
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    height, width = occupancy.shape
    radius_cells = max(1, int(ceil(local_half_window_m / resolution)))
    max_slope = float(navigation.config.maximum_slope_deg)
    max_step = float(navigation.config.maximum_step_m)

    normalized_targets: list[dict[str, object]] = []
    for target in targets:
        row = int(target["row"])
        col = int(target["col"])
        if not (0 <= row < height and 0 <= col < width):
            raise ValueError(f"terrain target outside navigation grid: row={row} col={col}")
        normalized_targets.append(
            {
                **target,
                "row": row,
                "col": col,
                "cell_values": {},
                "raw_point_count": 0,
            }
        )

    for xyz in _iter_xyz_chunks(cloud, chunk_size):
        xyz = np.asarray(xyz, dtype=np.float64)
        finite = np.all(np.isfinite(xyz[:, :3]), axis=1)
        if not np.any(finite):
            continue
        xyz = xyz[finite, :3]
        rows = np.floor((xyz[:, 1] - origin_y) / resolution).astype(np.int64)
        cols = np.floor((xyz[:, 0] - origin_x) / resolution).astype(np.int64)
        in_grid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        if not np.any(in_grid):
            continue
        rows = rows[in_grid]
        cols = cols[in_grid]
        z = xyz[in_grid, 2]

        for target in normalized_targets:
            row0 = int(target["row"])
            col0 = int(target["col"])
            selected = (
                (np.abs(rows - row0) <= radius_cells)
                & (np.abs(cols - col0) <= radius_cells)
            )
            if not np.any(selected):
                continue
            local_rows = rows[selected]
            local_cols = cols[selected]
            local_z = z[selected]
            target["raw_point_count"] = int(target["raw_point_count"]) + int(local_z.size)
            cell_values = target["cell_values"]
            assert isinstance(cell_values, dict)
            cell_ids = local_rows * width + local_cols
            for cell_id in np.unique(cell_ids):
                cell_selected = cell_ids == cell_id
                rr = int(cell_id // width)
                cc = int(cell_id % width)
                cell_values.setdefault((rr, cc), []).append(
                    np.asarray(local_z[cell_selected], dtype=np.float64)
                )

    records: list[dict[str, object]] = []
    for target in normalized_targets:
        row = int(target["row"])
        col = int(target["col"])
        cell_values = target["cell_values"]
        assert isinstance(cell_values, dict)
        surface: dict[tuple[int, int], float] = {}
        point_counts: dict[tuple[int, int], int] = {}
        for key, chunks in cell_values.items():
            assert isinstance(chunks, list)
            count = int(sum(np.asarray(chunk).size for chunk in chunks))
            point_counts[key] = count
            if count < minimum_points_per_cell:
                continue
            surface[key] = _discrete_lower_quantile(chunks, lower_quantile)

        plane_slope, plane_rmse, plane = (
            _fit_plane(
                surface,
                origin_x_m=origin_x,
                origin_y_m=origin_y,
                resolution_m=resolution,
            )
            if len(surface) >= minimum_surface_cells
            else (None, None, None)
        )
        raw_step, step_directions = _raw_second_difference(surface, row, col)

        raster_slope = float(slope_grid[row, col])
        raster_step = float(step_grid[row, col])
        raster_slope_hard = bool(np.isfinite(raster_slope) and raster_slope > max_slope)
        raster_step_hard = bool(np.isfinite(raster_step) and raster_step > max_step)
        raw_slope_hard = None if plane_slope is None else bool(plane_slope > max_slope)
        raw_step_hard = None if raw_step is None else bool(raw_step > max_step)
        status = _corroboration_status(
            raster_slope_hard=raster_slope_hard,
            raster_step_hard=raster_step_hard,
            raw_slope_hard=raw_slope_hard,
            raw_step_hard=raw_step_hard,
        )
        records.append(
            {
                "aisle_id": str(target.get("aisle_id", "")),
                "row": row,
                "col": col,
                "world_x_m": origin_x + (col + 0.5) * resolution,
                "world_y_m": origin_y + (row + 0.5) * resolution,
                "target_source": str(target.get("target_source", "UNKNOWN")),
                "raster_cause": str(target.get("raster_cause", "UNKNOWN")),
                "raster_slope_deg": raster_slope,
                "raster_step_m": raster_step,
                "maximum_slope_deg": max_slope,
                "maximum_step_m": max_step,
                "raster_slope_hard": raster_slope_hard,
                "raster_step_hard": raster_step_hard,
                "raw_point_count": int(target["raw_point_count"]),
                "raw_lower_envelope_surface_cell_count": len(surface),
                "raw_center_cell_point_count": int(point_counts.get((row, col), 0)),
                "raw_center_lower_quantile_z_m": surface.get((row, col)),
                "raw_plane_slope_deg": plane_slope,
                "raw_plane_fit_rmse_m": plane_rmse,
                "raw_plane": plane,
                "raw_step_m": raw_step,
                "raw_step_directions": step_directions,
                "raw_slope_hard": raw_slope_hard,
                "raw_step_hard": raw_step_hard,
                "corroboration_status": status,
            }
        )

    status_counts = Counter(str(record["corroboration_status"]) for record in records)
    return {
        "schema": _SCHEMA,
        "status": _STATUS,
        "authority": _AUTHORITY,
        "terrain_thresholds": {
            "maximum_slope_deg": max_slope,
            "maximum_step_m": max_step,
        },
        "raw_surface_contract": {
            "cell_resolution_m": resolution,
            "local_half_window_m": float(local_half_window_m),
            "radius_cells": radius_cells,
            "lower_quantile": float(lower_quantile),
            "minimum_points_per_cell": int(minimum_points_per_cell),
            "minimum_surface_cells": int(minimum_surface_cells),
            "step_metric": "A3_THREE_POINT_SECOND_DIFFERENCE_MAX_4_DIRECTIONS",
            "slope_metric": "LEAST_SQUARES_PLANE_ON_RAW_CELL_LOWER_ENVELOPE",
        },
        "target_count": len(records),
        "corroboration_status_counts": dict(sorted(status_counts.items())),
        "records": records,
    }
