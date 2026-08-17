from __future__ import annotations

import math
from typing import Sequence
from .contracts import PathPoint


def _distance(a: PathPoint, b: PathPoint) -> float:
    return math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def compute_path_metrics(
    points: Sequence[PathPoint],
    *,
    min_clearance_m: float | None = None,
    footprint_collision_count: int | None = None,
    semantic_violation_count: int | None = None,
) -> dict:
    length = 0.0
    reverse_distance = 0.0
    reverse_segments = 0
    max_curvature = 0.0
    zero_length_segments = 0
    for a, b in zip(points, points[1:]):
        ds = _distance(a, b)
        length += ds
        if b.direction == "R":
            reverse_segments += 1
            reverse_distance += ds
        if ds <= 1e-12:
            zero_length_segments += 1
            continue
        max_curvature = max(max_curvature, abs(_angle_delta(a.yaw_rad, b.yaw_rad)) / ds)
    return {
        "path_length_m": length,
        "reverse_distance_m": reverse_distance,
        "reverse_segment_count": reverse_segments,
        "max_abs_curvature_1pm": max_curvature,
        "zero_length_segment_count": zero_length_segments,
        "min_clearance_m": min_clearance_m,
        "footprint_collision_count": footprint_collision_count,
        "semantic_violation_count": semantic_violation_count,
    }


def compute_mission_metrics(
    *,
    required_semantic_ids: Sequence[str],
    reachable_semantic_ids: Sequence[str],
    visited_semantic_ids: Sequence[str],
    path_points: Sequence[PathPoint],
) -> dict:
    required = list(dict.fromkeys(required_semantic_ids))
    reachable = list(dict.fromkeys(reachable_semantic_ids))
    visited = set(visited_semantic_ids)
    reachable_set = set(reachable)
    required_set = set(required)
    visited_required = visited & required_set
    visited_reachable = visited & reachable_set
    required_ratio = len(visited_required) / len(required) if required else 1.0
    reachable_ratio = len(visited_reachable) / len(reachable) if reachable else 1.0
    deadhead = 0.0
    for a, b in zip(path_points, path_points[1:]):
        if b.segment_type != "SWATH":
            deadhead += _distance(a, b)
    return {
        "required_task_coverage_ratio": required_ratio,
        "reachable_task_coverage_ratio": reachable_ratio,
        "required_but_unreachable": sorted(required_set - reachable_set),
        "missed_reachable": sorted(reachable_set - visited),
        "deadhead_distance_m": deadhead,
    }
