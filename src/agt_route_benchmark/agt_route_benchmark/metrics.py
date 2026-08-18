from __future__ import annotations

import math
from typing import Sequence
from .contracts import PathPoint


_POSITION_CONTINUITY_EPS = 1e-9
_HEADING_CONTINUITY_EPS = 1e-6


def _distance(a: PathPoint, b: PathPoint) -> float:
    return math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _chord_curvature(delta_yaw: float, chord_m: float) -> float:
    """Estimate constant-curvature motion from a pose-pair chord.

    The chord relation remains invariant to finite angular sampling:
    kappa = 2 sin(|delta_yaw| / 2) / chord.
    """
    if chord_m <= 1e-12:
        return math.inf if abs(delta_yaw) > 1e-12 else 0.0
    return 2.0 * math.sin(abs(delta_yaw) / 2.0) / chord_m


def _direction_label(a: PathPoint, b: PathPoint) -> str:
    if a.direction == b.direction:
        return a.direction
    if {a.direction, b.direction} == {"F", "R"}:
        return f"{a.direction}/{b.direction}"
    return b.direction if b.direction != "UNKNOWN" else a.direction


def compute_path_metrics(
    points: Sequence[PathPoint],
    *,
    curvature_limit_1pm: float | None = None,
    min_clearance_m: float | None = None,
    footprint_collision_count: int | None = None,
    semantic_violation_count: int | None = None,
) -> dict:
    length = 0.0
    reverse_distance = 0.0
    reverse_segments = 0
    max_curvature = 0.0
    zero_length_segments = 0
    in_reverse = False
    direction_transition_count = 0
    valid_cusp_count = 0
    ambiguous_direction_transition_count = 0
    worst: dict[str, object] | None = None
    for a, b in zip(points, points[1:]):
        ds = _distance(a, b)
        length += ds
        is_reverse = b.direction == "R"
        if is_reverse:
            reverse_distance += ds
            if not in_reverse:
                reverse_segments += 1
        in_reverse = is_reverse
        if ds <= 1e-12:
            zero_length_segments += 1
    # Keep the diagnostic pass index-based so repeated/equal PathPoints are safe.
    worst = None
    for segment_index, (a, b) in enumerate(zip(points, points[1:])):
        chord = _distance(a, b)
        delta_yaw = _angle_delta(a.yaw_rad, b.yaw_rad)
        direction_transition = {a.direction, b.direction} == {"F", "R"}
        if direction_transition:
            direction_transition_count += 1
            if chord <= _POSITION_CONTINUITY_EPS and abs(delta_yaw) <= _HEADING_CONTINUITY_EPS:
                valid_cusp_count += 1
            else:
                ambiguous_direction_transition_count += 1
            # A cusp is a velocity-sign event, not a continuous motion
            # primitive. Never fold its chord into curvature evaluation.
            continue
        curvature = _chord_curvature(delta_yaw, chord)
        if curvature > max_curvature:
            max_curvature = curvature
        if worst is None or curvature > float(worst["curvature"]):
            worst = {
                "curvature": curvature,
                "segment_index": segment_index,
                "x_m": b.x_m,
                "y_m": b.y_m,
                "direction": _direction_label(a, b),
                "delta_yaw_rad": delta_yaw,
                "chord_m": chord,
                "is_direction_transition": a.direction != b.direction,
            }
    if worst is None:
        worst = {
            "curvature": 0.0, "segment_index": None, "x_m": None, "y_m": None,
            "direction": "UNKNOWN", "delta_yaw_rad": 0.0, "chord_m": 0.0,
            "is_direction_transition": False,
        }
    ratio = None
    if curvature_limit_1pm is not None:
        ratio = (float(worst["curvature"]) / curvature_limit_1pm
                 if curvature_limit_1pm > 0.0 else math.inf)
    return {
        "path_length_m": length,
        "reverse_distance_m": reverse_distance,
        "reverse_segment_count": reverse_segments,
        "max_abs_curvature_1pm": max_curvature,
        "worst_curvature_segment_index": worst["segment_index"],
        "worst_curvature_x_m": worst["x_m"],
        "worst_curvature_y_m": worst["y_m"],
        "worst_curvature_direction": worst["direction"],
        "worst_curvature_delta_yaw_rad": worst["delta_yaw_rad"],
        "worst_curvature_chord_m": worst["chord_m"],
        "worst_curvature_ratio_to_limit": ratio,
        "worst_curvature_is_direction_transition": worst["is_direction_transition"],
        "direction_transition_count": direction_transition_count,
        "valid_cusp_count": valid_cusp_count,
        "ambiguous_direction_transition_count": ambiguous_direction_transition_count,
        "direction_transition_feasible": ambiguous_direction_transition_count == 0,
        "direction_transition_status": (
            "AMBIGUOUS_DIRECTION_TRANSITION"
            if ambiguous_direction_transition_count
            else ("VALID_CUSP" if valid_cusp_count else "NONE")
        ),
        "validation_error_codes": (
            ["ambiguous_direction_transition"]
            if ambiguous_direction_transition_count else []
        ),
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
