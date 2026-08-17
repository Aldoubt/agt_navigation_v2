from __future__ import annotations

import math
from collections.abc import Sequence

from .contracts import PathPoint, Pose2D


def _wrapped_angle_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def compute_endpoint_deviation(
    requested_start: Pose2D,
    requested_goal: Pose2D,
    points: Sequence[PathPoint],
) -> dict[str, float]:
    if not points:
        raise ValueError("endpoint deviation requires a non-empty path")

    first = points[0]
    last = points[-1]
    return {
        "start_pose_deviation_m": math.hypot(
            first.x_m - requested_start[0],
            first.y_m - requested_start[1],
        ),
        "goal_pose_deviation_m": math.hypot(
            last.x_m - requested_goal[0],
            last.y_m - requested_goal[1],
        ),
        "start_heading_deviation_rad": _wrapped_angle_error(
            first.yaw_rad,
            requested_start[2],
        ),
        "goal_heading_deviation_rad": _wrapped_angle_error(
            last.yaw_rad,
            requested_goal[2],
        ),
    }
