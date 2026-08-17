from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .contracts import PathPoint


@dataclass(frozen=True)
class ExecutedPose:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _reference_segments(reference: Sequence[PathPoint]):
    segments = []
    accumulated = 0.0
    for first, second in zip(reference, reference[1:]):
        dx = second.x_m - first.x_m
        dy = second.y_m - first.y_m
        length = math.hypot(dx, dy)
        if length <= 1e-12:
            continue
        segments.append((first, second, dx, dy, length, accumulated))
        accumulated += length
    return segments, accumulated


def _project_pose(x: float, y: float, segments):
    best = None
    for first, second, dx, dy, length, accumulated in segments:
        length_sq = length * length
        ratio = ((x - first.x_m) * dx + (y - first.y_m) * dy) / length_sq
        ratio = min(1.0, max(0.0, ratio))
        px = first.x_m + ratio * dx
        py = first.y_m + ratio * dy
        distance = math.hypot(x - px, y - py)
        yaw = first.yaw_rad + ratio * _angle_delta(first.yaw_rad, second.yaw_rad)
        along = accumulated + ratio * length
        candidate = (distance, along, yaw, px, py)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best


def _rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def compute_tracking_metrics(
    reference: Sequence[PathPoint],
    executed: Sequence[ExecutedPose],
) -> dict:
    """Compare one executed trajectory against an immutable Paper I reference path.

    Cross-track error is the Euclidean distance to the closest point on the
    piecewise-linear reference path, not merely to the closest stored vertex.
    Completion is the furthest along-path projection reached by any executed pose,
    which is robust to local reverse maneuvers and controller backtracking.
    """
    if len(reference) < 2:
        raise ValueError("reference path requires at least two points")
    if not executed:
        raise ValueError("executed trajectory requires at least one pose")
    for pose in executed:
        values = (pose.stamp_s, pose.x_m, pose.y_m, pose.yaw_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("executed trajectory contains non-finite values")

    segments, total_length = _reference_segments(reference)
    if not segments or total_length <= 1e-12:
        raise ValueError("reference path must have non-zero length")

    lateral_errors: list[float] = []
    heading_errors: list[float] = []
    along_values: list[float] = []
    for pose in executed:
        projection = _project_pose(pose.x_m, pose.y_m, segments)
        if projection is None:  # pragma: no cover - guarded by non-empty segments
            raise RuntimeError("reference path projection failed")
        distance, along, reference_yaw, _, _ = projection
        lateral_errors.append(distance)
        heading_errors.append(abs(_angle_delta(reference_yaw, pose.yaw_rad)))
        along_values.append(along)

    final = executed[-1]
    goal = reference[-1]
    final_position_error = math.hypot(final.x_m - goal.x_m, final.y_m - goal.y_m)
    final_yaw_error = abs(_angle_delta(goal.yaw_rad, final.yaw_rad))
    stamps = [pose.stamp_s for pose in executed]
    duration = max(stamps) - min(stamps)
    completion = min(1.0, max(0.0, max(along_values) / total_length))

    return {
        "reference_path_length_m": total_length,
        "executed_sample_count": len(executed),
        "executed_duration_s": duration,
        "lateral_rmse_m": _rmse(lateral_errors),
        "lateral_mean_abs_m": sum(lateral_errors) / len(lateral_errors),
        "lateral_max_abs_m": max(lateral_errors),
        "heading_rmse_rad": _rmse(heading_errors),
        "heading_mean_abs_rad": sum(heading_errors) / len(heading_errors),
        "heading_max_abs_rad": max(heading_errors),
        "path_completion_ratio": completion,
        "final_position_error_m": final_position_error,
        "final_yaw_error_rad": final_yaw_error,
    }
