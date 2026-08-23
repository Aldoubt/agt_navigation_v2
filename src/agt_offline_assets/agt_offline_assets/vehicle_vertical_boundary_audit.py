"""E2 exact vertical-boundary diagnostics for MID-dominant sensor throats.

E2 is review evidence only. It revisits the exact E1 blocker cells against the
raw PCD and the frozen A3 ground surface, splitting the coarse D2 MID layer at
the vehicle's real collision_z_max. It never mutates D2/D3 evidence, obstacle
policy, Formal/Accepted PGM, or map authority.
"""

from __future__ import annotations

from collections import Counter
from typing import Any

import numpy as np

from .height_layer_ablation import _iter_xyz_chunks


_SCHEMA = "agt_vehicle_vertical_boundary_audit/v1"
_AUTHORITY = "DERIVED_VERTICAL_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY"
_E1_SCHEMA = "agt_vehicle_sensor_evidence_root_cause/v1"


def _histogram(
    values: np.ndarray,
    *,
    lower_m: float,
    upper_m: float,
    bin_size_m: float,
) -> list[dict[str, object]]:
    if bin_size_m <= 0.0:
        raise ValueError("E2 mid_bin_size_m must be > 0")
    span = float(upper_m) - float(lower_m)
    if span <= 0.0:
        raise ValueError("E2 MID histogram requires upper > lower")
    bin_count = max(1, int(np.ceil(span / float(bin_size_m) - 1.0e-12)))
    edges = float(lower_m) + np.arange(bin_count + 1, dtype=np.float64) * float(bin_size_m)
    edges[-1] = float(upper_m)
    counts, _ = np.histogram(np.asarray(values, dtype=np.float64), bins=edges)
    return [
        {
            "lower_m": float(edges[index]),
            "upper_m": float(edges[index + 1]),
            "interval": "[lower, upper)" if index < len(counts) - 1 else "[lower, upper]",
            "count": int(counts[index]),
        }
        for index in range(len(counts))
    ]


def _exact_sensor_reason(
    obstacle_count: int,
    obstacle_ratio: float,
    *,
    minimum_obstacle_points: int,
    soft_obstacle_max_count: int,
    soft_obstacle_max_ratio: float,
) -> tuple[str, str]:
    if int(obstacle_count) < int(minimum_obstacle_points):
        return "NOT_DIRECT_OBSTACLE", "COARSE_MID_FALSE_POSITIVE"
    count_hard = int(obstacle_count) > int(soft_obstacle_max_count)
    ratio_hard = float(obstacle_ratio) > float(soft_obstacle_max_ratio) + 1.0e-12
    if count_hard and ratio_hard:
        return "COUNT_AND_RATIO", "EXACT_COLLISION_STILL_STRONG"
    if count_hard:
        return "COUNT_ONLY", "EXACT_COLLISION_STILL_STRONG"
    if ratio_hard:
        return "RATIO_ONLY", "EXACT_COLLISION_STILL_STRONG"
    return "SOFT_OR_WEAK", "EXACT_COLLISION_SOFT_OR_WEAK"


def build_vehicle_vertical_boundary_audit(
    cloud: Any,
    navigation: Any,
    e1_audit: dict[str, object],
    *,
    obstacle_min_height_m: float,
    low_max_height_m: float,
    mid_max_height_m: float,
    obstacle_max_height_m: float,
    mid_bin_size_m: float = 0.01,
    chunk_size: int = 1_000_000,
) -> dict[str, object]:
    """Resolve coarse MID evidence at exact E1 blocker cells against vehicle z_max."""

    if e1_audit.get("schema") != _E1_SCHEMA:
        raise ValueError(f"E2 requires {_E1_SCHEMA}")
    vehicle = e1_audit.get("vehicle_envelope")
    if not isinstance(vehicle, dict):
        raise ValueError("E2 requires E1 vehicle_envelope")
    collision_z_max = float(vehicle.get("collision_z_max_m"))
    obstacle_min = float(obstacle_min_height_m)
    low_max = float(low_max_height_m)
    mid_max = float(mid_max_height_m)
    obstacle_max = float(obstacle_max_height_m)
    if not obstacle_min < low_max < collision_z_max < mid_max < obstacle_max:
        raise ValueError(
            "E2 requires obstacle_min < LOW max < vehicle z_max < MID max < obstacle_max"
        )
    if float(mid_bin_size_m) <= 0.0:
        raise ValueError("E2 mid_bin_size_m must be > 0")

    ground = np.asarray(navigation.ground_height_m, dtype=np.float64)
    point_count_grid = np.asarray(navigation.point_count, dtype=np.int64)
    if ground.shape != point_count_grid.shape or ground.ndim != 2:
        raise ValueError("E2 navigation ground/point-count grids must share one 2D shape")
    height, width = ground.shape
    resolution = float(navigation.resolution_m)
    origin_x = float(navigation.origin_x_m)
    origin_y = float(navigation.origin_y_m)
    minimum_obstacle_points = int(navigation.config.minimum_obstacle_points)
    soft_max_count = int(e1_audit.get("soft_obstacle_max_count", 4))
    soft_max_ratio = float(e1_audit.get("soft_obstacle_max_ratio", 0.05))

    source_records = [
        record
        for record in e1_audit.get("records", [])
        if isinstance(record, dict) and record.get("height_layer_dominance") == "MID"
    ]
    target_by_cell: dict[int, dict[str, object]] = {}
    heights_by_cell: dict[int, list[np.ndarray]] = {}
    for record in source_records:
        row = int(record["blocker_row"])
        col = int(record["blocker_col"])
        if not (0 <= row < height and 0 <= col < width):
            raise ValueError("E2 blocker index outside navigation grid")
        cell_id = row * width + col
        if cell_id in target_by_cell:
            raise ValueError("E2 requires unique MID-dominant blocker cells")
        if int(record.get("point_count", -1)) != int(point_count_grid[row, col]):
            raise ValueError("E2 E1 point_count disagrees with replay navigation")
        if not np.isfinite(ground[row, col]):
            raise ValueError("E2 exact blocker requires finite A3 ground height")
        target_by_cell[cell_id] = record
        heights_by_cell[cell_id] = []

    target_ids = np.asarray(sorted(target_by_cell), dtype=np.int64)
    if target_ids.size:
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
            cell_ids = rows * width + cols
            selected = np.isin(cell_ids, target_ids)
            if not np.any(selected):
                continue
            rows = rows[selected]
            cols = cols[selected]
            z = z[selected]
            cell_ids = cell_ids[selected]
            relative = z - ground[rows, cols]
            for cell_id in np.unique(cell_ids):
                mask = cell_ids == cell_id
                heights_by_cell[int(cell_id)].append(np.asarray(relative[mask], dtype=np.float64))

    records: list[dict[str, object]] = []
    for source in source_records:
        row = int(source["blocker_row"])
        col = int(source["blocker_col"])
        cell_id = row * width + col
        pieces = heights_by_cell[cell_id]
        relative = np.concatenate(pieces) if pieces else np.empty(0, dtype=np.float64)

        low_mask = (relative >= obstacle_min) & (relative < low_max)
        mid_colliding_mask = (relative >= low_max) & (relative < collision_z_max)
        mid_above_mask = (relative >= collision_z_max) & (relative < mid_max)
        high_mask = (relative >= mid_max) & (relative <= obstacle_max)
        mid_mask = mid_colliding_mask | mid_above_mask

        low_count = int(np.count_nonzero(low_mask))
        mid_colliding = int(np.count_nonzero(mid_colliding_mask))
        mid_above = int(np.count_nonzero(mid_above_mask))
        high_count = int(np.count_nonzero(high_mask))
        mid_total = mid_colliding + mid_above
        expected_low = int(source.get("low_count", -1))
        expected_mid = int(source.get("mid_count", -1))
        expected_high = int(source.get("high_count", -1))
        if (low_count, mid_total, high_count) != (expected_low, expected_mid, expected_high):
            raise ValueError(
                "E2 exact PCD replay disagrees with E1/D2 layer counts: "
                f"aisle={source.get('aisle_id')} exact={(low_count, mid_total, high_count)} "
                f"recorded={(expected_low, expected_mid, expected_high)}"
            )

        coarse_selected = low_count + mid_total
        exact_selected = low_count + mid_colliding
        points = int(point_count_grid[row, col])
        exact_ratio = float(exact_selected) / float(max(points, 1))
        exact_reason, outcome = _exact_sensor_reason(
            exact_selected,
            exact_ratio,
            minimum_obstacle_points=minimum_obstacle_points,
            soft_obstacle_max_count=soft_max_count,
            soft_obstacle_max_ratio=soft_max_ratio,
        )
        histogram = _histogram(
            relative[mid_mask],
            lower_m=low_max,
            upper_m=mid_max,
            bin_size_m=float(mid_bin_size_m),
        )
        if sum(int(item["count"]) for item in histogram) != mid_total:
            raise RuntimeError("E2 MID histogram count mismatch")

        records.append(
            {
                "aisle_id": str(source.get("aisle_id")),
                "blocker_row": row,
                "blocker_col": col,
                "world_x_m": source.get("world_x_m"),
                "world_y_m": source.get("world_y_m"),
                "clearance_distance_m": source.get("clearance_distance_m"),
                "point_count": points,
                "coarse_selected_count": coarse_selected,
                "coarse_strong_sensor_reason": source.get("strong_sensor_reason"),
                "low_exact_count": low_count,
                "mid_colliding_count": mid_colliding,
                "mid_above_vehicle_count": mid_above,
                "high_exact_count": high_count,
                "exact_vehicle_selected_count": exact_selected,
                "exact_vehicle_selected_ratio": exact_ratio,
                "mid_colliding_fraction_of_mid": (
                    float(mid_colliding) / float(mid_total) if mid_total else 0.0
                ),
                "mid_above_vehicle_fraction_of_mid": (
                    float(mid_above) / float(mid_total) if mid_total else 0.0
                ),
                "exact_sensor_reason": exact_reason,
                "outcome": outcome,
                "mid_histogram_1cm": histogram,
            }
        )

    outcomes = Counter(str(record["outcome"]) for record in records)
    return {
        "schema": _SCHEMA,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": _AUTHORITY,
        "source_e1_schema": str(e1_audit.get("schema")),
        "scope": "MID_DOMINANT_EXACT_STRONG_SENSOR_THROATS_ONLY",
        "interval_contract": {
            "LOW_EXACT": f"[{obstacle_min}, {low_max})",
            "MID_COLLIDING": f"[{low_max}, {collision_z_max})",
            "MID_ABOVE_VEHICLE": f"[{collision_z_max}, {mid_max})",
            "HIGH": f"[{mid_max}, {obstacle_max}]",
        },
        "collision_z_max_m": collision_z_max,
        "mid_bin_size_m": float(mid_bin_size_m),
        "minimum_obstacle_points": minimum_obstacle_points,
        "soft_obstacle_max_count": soft_max_count,
        "soft_obstacle_max_ratio": soft_max_ratio,
        "mid_dominant_blocker_count": len(records),
        "outcome_counts": dict(sorted(outcomes.items())),
        "records": records,
    }
