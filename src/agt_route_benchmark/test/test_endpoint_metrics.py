from __future__ import annotations

import math

import pytest

from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.endpoint_metrics import compute_endpoint_deviation


def test_endpoint_deviation_reports_grid_center_quantization():
    points = (
        PathPoint(0.25, 0.25, 0.0, "F", "P2P", ""),
        PathPoint(4.25, 0.25, 0.0, "F", "P2P", ""),
    )

    metrics = compute_endpoint_deviation(
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        points,
    )

    assert metrics["start_pose_deviation_m"] == pytest.approx(math.sqrt(0.125))
    assert metrics["goal_pose_deviation_m"] == pytest.approx(math.sqrt(0.125))
    assert metrics["start_heading_deviation_rad"] == pytest.approx(0.0)
    assert metrics["goal_heading_deviation_rad"] == pytest.approx(0.0)


def test_endpoint_heading_deviation_wraps_across_pi_boundary():
    points = (
        PathPoint(0.0, 0.0, -math.pi + 0.01, "F", "P2P", ""),
        PathPoint(1.0, 0.0, -math.pi + 0.01, "F", "P2P", ""),
    )

    metrics = compute_endpoint_deviation(
        (0.0, 0.0, math.pi - 0.01),
        (1.0, 0.0, math.pi - 0.01),
        points,
    )

    assert metrics["start_heading_deviation_rad"] == pytest.approx(0.02)
    assert metrics["goal_heading_deviation_rad"] == pytest.approx(0.02)


def test_endpoint_deviation_rejects_empty_path():
    with pytest.raises(ValueError, match="non-empty"):
        compute_endpoint_deviation((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), ())
