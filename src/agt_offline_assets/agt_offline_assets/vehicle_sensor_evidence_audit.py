"""E1 root-cause diagnostics for exact strong-sensor clearance-throat blockers.

This module is review evidence only. It consumes D3.2 exact nearest-obstacle
provenance and explains why a blocker cell was classified as strong sensor
evidence under the vehicle-selected vertical layers. It never mutates the
formal Navigation Map, D2 evidence, or obstacle policy.
"""

from __future__ import annotations

from collections import Counter, deque
from typing import Any, Iterable

import numpy as np


_SCHEMA = "agt_vehicle_sensor_evidence_root_cause/v1"
_AUTHORITY = "DERIVED_SENSOR_REVIEW_NOT_NAVIGATION_MAP_AUTHORITY"
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
_VALID_LAYERS = {"LOW", "MID", "HIGH"}


def _finite_or_none(value: float) -> float | None:
    result = float(value)
    return result if np.isfinite(result) else None


def _neighbor_count(mask: np.ndarray, row: int, col: int, radius: int) -> int:
    r0 = max(0, row - radius)
    r1 = min(mask.shape[0], row + radius + 1)
    c0 = max(0, col - radius)
    c1 = min(mask.shape[1], col + radius + 1)
    count = int(np.count_nonzero(mask[r0:r1, c0:c1]))
    return count - int(bool(mask[row, col]))


def _component_size(mask: np.ndarray, row: int, col: int) -> int:
    if not bool(mask[row, col]):
        return 0
    visited = np.zeros_like(mask, dtype=bool)
    queue: deque[tuple[int, int]] = deque([(row, col)])
    visited[row, col] = True
    count = 0
    while queue:
        rr, cc = queue.popleft()
        count += 1
        for dr, dc in _NEIGHBOURS:
            nr = rr + dr
            nc = cc + dc
            if not (0 <= nr < mask.shape[0] and 0 <= nc < mask.shape[1]):
                continue
            if visited[nr, nc] or not bool(mask[nr, nc]):
                continue
            visited[nr, nc] = True
            queue.append((nr, nc))
    return count


def _strong_reason(
    obstacle_count: int,
    obstacle_ratio: float,
    *,
    soft_obstacle_max_count: int,
    soft_obstacle_max_ratio: float,
) -> str:
    count_hard = int(obstacle_count) > int(soft_obstacle_max_count)
    ratio_hard = float(obstacle_ratio) > float(soft_obstacle_max_ratio) + 1.0e-12
    if count_hard and ratio_hard:
        return "COUNT_AND_RATIO"
    if count_hard:
        return "COUNT_ONLY"
    if ratio_hard:
        return "RATIO_ONLY"
    return "NOT_STRONG_BY_SOFT_THRESHOLDS"


def _height_layer_dominance(counts: dict[str, int], selected_layers: tuple[str, ...]) -> str:
    selected = [(layer, int(counts[layer])) for layer in selected_layers]
    maximum = max((count for _, count in selected), default=0)
    if maximum <= 0:
        return "NONE"
    winners = [layer for layer, count in selected if count == maximum]
    return winners[0] if len(winners) == 1 else "TIED"


def _normalize_selected_layers(selected_layers: Iterable[str]) -> tuple[str, ...]:
    result = tuple(str(layer).strip().upper() for layer in selected_layers)
    if not result:
        raise ValueError("E1 selected_layers must not be empty")
    if len(set(result)) != len(result) or any(layer not in _VALID_LAYERS for layer in result):
        raise ValueError("E1 selected_layers must be unique LOW/MID/HIGH values")
    return result


def build_vehicle_sensor_evidence_root_cause_audit(
    navigation: Any,
    vertical_evidence: Any,
    clearance_throat_audit: dict[str, object],
    provenance: Any,
    *,
    selected_layers: Iterable[str],
    soft_obstacle_max_count: int,
    soft_obstacle_max_ratio: float,
) -> dict[str, object]:
    """Explain exact D3.2 throat cells whose nearest source is strong sensor evidence."""

    if clearance_throat_audit.get("schema") != "agt_vehicle_clearance_throat_audit/v2":
        raise ValueError("E1 requires D3.2 clearance-throat schema v2")
    if int(soft_obstacle_max_count) < 0:
        raise ValueError("soft_obstacle_max_count must be >= 0")
    if float(soft_obstacle_max_ratio) < 0.0:
        raise ValueError("soft_obstacle_max_ratio must be >= 0")

    layers = _normalize_selected_layers(selected_layers)
    point_count = np.asarray(navigation.point_count, dtype=np.int64)
    ground_support = np.asarray(navigation.ground_support_count, dtype=np.int64)
    obstacle_count = np.asarray(navigation.obstacle_count, dtype=np.int64)
    slope = np.asarray(navigation.slope_deg, dtype=np.float64)
    step = np.asarray(navigation.step_m, dtype=np.float64)
    shape = tuple(point_count.shape)
    arrays = {
        "ground_support_count": ground_support,
        "obstacle_count": obstacle_count,
        "slope_deg": slope,
        "step_m": step,
        "low_count": np.asarray(vertical_evidence.low_count, dtype=np.int64),
        "mid_count": np.asarray(vertical_evidence.mid_count, dtype=np.int64),
        "high_count": np.asarray(vertical_evidence.high_count, dtype=np.int64),
    }
    for name, array in arrays.items():
        if tuple(array.shape) != shape:
            raise ValueError(f"E1 {name} shape mismatch")

    strong = np.asarray(provenance.strong_sensor_obstacle_mask, dtype=bool)
    if tuple(strong.shape) != shape:
        raise ValueError("E1 strong sensor provenance shape mismatch")

    minimum_ground_support = int(getattr(navigation.config, "minimum_ground_support_points", 1))
    layer_arrays = {
        "LOW": arrays["low_count"],
        "MID": arrays["mid_count"],
        "HIGH": arrays["high_count"],
    }

    records: list[dict[str, object]] = []
    for aisle in clearance_throat_audit.get("aisles", []):
        if not isinstance(aisle, dict) or aisle.get("status") != "CLEARANCE_THROAT":
            continue
        throat = aisle.get("primary_throat")
        if not isinstance(throat, dict):
            continue
        nearest = throat.get("nearest_environment_constraint")
        if not isinstance(nearest, dict) or nearest.get("source") != "STRONG_SENSOR_OBSTACLE":
            continue
        row = int(nearest["row"])
        col = int(nearest["col"])
        if not (0 <= row < shape[0] and 0 <= col < shape[1]):
            raise ValueError("E1 exact blocker index outside navigation grid")
        if not bool(strong[row, col]):
            raise ValueError("E1 D3.2 strong-sensor source disagrees with provenance mask")

        counts = {layer: int(array[row, col]) for layer, array in layer_arrays.items()}
        selected_sum = sum(counts[layer] for layer in layers)
        selected_obstacle_count = int(obstacle_count[row, col])
        if selected_sum != selected_obstacle_count:
            raise ValueError(
                "E1 selected vertical-layer counts must reproduce vehicle obstacle_count "
                f"at exact blocker cell: aisle={aisle.get('aisle_id')} "
                f"selected_sum={selected_sum} navigation={selected_obstacle_count}"
            )
        points = int(point_count[row, col])
        ratio = float(selected_obstacle_count) / float(max(points, 1))
        reason = _strong_reason(
            selected_obstacle_count,
            ratio,
            soft_obstacle_max_count=int(soft_obstacle_max_count),
            soft_obstacle_max_ratio=float(soft_obstacle_max_ratio),
        )
        dominance = _height_layer_dominance(counts, layers)
        selected_total = max(selected_sum, 1)
        records.append(
            {
                "aisle_id": str(aisle.get("aisle_id")),
                "blocker_row": row,
                "blocker_col": col,
                "clearance_distance_m": float(nearest.get("distance_m", 0.0)),
                "selected_layers": list(layers),
                "low_count": counts["LOW"],
                "mid_count": counts["MID"],
                "high_count": counts["HIGH"],
                "selected_obstacle_count": selected_obstacle_count,
                "selected_obstacle_ratio": ratio,
                "selected_layer_fractions": {
                    layer: float(counts[layer]) / float(selected_total) for layer in layers
                },
                "point_count": points,
                "ground_support_count": int(ground_support[row, col]),
                "ground_supported": int(ground_support[row, col]) >= minimum_ground_support,
                "slope_deg": _finite_or_none(slope[row, col]),
                "step_m": _finite_or_none(step[row, col]),
                "strong_sensor_reason": reason,
                "height_layer_dominance": dominance,
                "strong_sensor_neighbors_r1": _neighbor_count(strong, row, col, 1),
                "strong_sensor_neighbors_r2": _neighbor_count(strong, row, col, 2),
                "strong_sensor_component_size": _component_size(strong, row, col),
                "world_x_m": nearest.get("world_x_m"),
                "world_y_m": nearest.get("world_y_m"),
                "u_m": nearest.get("u_m"),
                "v_m": nearest.get("v_m"),
            }
        )

    reason_counts = Counter(str(record["strong_sensor_reason"]) for record in records)
    dominance_counts = Counter(str(record["height_layer_dominance"]) for record in records)
    return {
        "schema": _SCHEMA,
        "status": "EXPERIMENTAL_REVIEW_EVIDENCE",
        "authority": _AUTHORITY,
        "source_clearance_throat_schema": str(clearance_throat_audit.get("schema")),
        "selected_layers": list(layers),
        "soft_obstacle_max_count": int(soft_obstacle_max_count),
        "soft_obstacle_max_ratio": float(soft_obstacle_max_ratio),
        "strong_sensor_throat_count": len(records),
        "strong_reason_counts": dict(sorted(reason_counts.items())),
        "height_layer_dominance_counts": dict(sorted(dominance_counts.items())),
        "ground_supported_count": sum(bool(record["ground_supported"]) for record in records),
        "records": records,
    }
