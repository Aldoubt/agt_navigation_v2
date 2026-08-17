import math

from agt_route_benchmark.contracts import PathPoint
from agt_route_benchmark.tracking_metrics import ExecutedPose, compute_tracking_metrics


def _straight_reference():
    return (
        PathPoint(0.0, 0.0, 0.0, "F", "SWATH", "row_01"),
        PathPoint(5.0, 0.0, 0.0, "F", "SWATH", "row_01"),
        PathPoint(10.0, 0.0, 0.0, "F", "SWATH", "row_01"),
    )


def test_constant_lateral_offset_has_expected_rmse_and_completion():
    executed = tuple(
        ExecutedPose(float(x), float(x), 0.20, 0.0)
        for x in range(11)
    )
    metrics = compute_tracking_metrics(_straight_reference(), executed)
    assert abs(metrics["lateral_rmse_m"] - 0.20) < 1e-9
    assert abs(metrics["lateral_mean_abs_m"] - 0.20) < 1e-9
    assert abs(metrics["lateral_max_abs_m"] - 0.20) < 1e-9
    assert metrics["path_completion_ratio"] == 1.0
    assert abs(metrics["final_position_error_m"] - 0.20) < 1e-9
    assert metrics["final_yaw_error_rad"] == 0.0


def test_completion_uses_along_path_projection_not_nearest_vertex():
    executed = (
        ExecutedPose(0.0, 0.0, 0.0, 0.0),
        ExecutedPose(1.0, 2.5, 0.0, 0.0),
    )
    metrics = compute_tracking_metrics(_straight_reference(), executed)
    assert abs(metrics["path_completion_ratio"] - 0.25) < 1e-9
    assert abs(metrics["executed_duration_s"] - 1.0) < 1e-9


def test_heading_and_final_error_wrap_angles():
    reference = (
        PathPoint(0.0, 0.0, math.pi - 0.05, "F", "SWATH", "row_01"),
        PathPoint(-1.0, 0.0, -math.pi + 0.05, "F", "SWATH", "row_01"),
    )
    executed = (
        ExecutedPose(0.0, 0.0, 0.0, -math.pi + 0.05),
        ExecutedPose(1.0, -1.0, 0.0, math.pi - 0.05),
    )
    metrics = compute_tracking_metrics(reference, executed)
    assert metrics["heading_rmse_rad"] < 0.11
    assert metrics["final_yaw_error_rad"] < 0.11


def test_empty_or_degenerate_inputs_fail_closed():
    import pytest

    with pytest.raises(ValueError, match="reference path"):
        compute_tracking_metrics((), (ExecutedPose(0.0, 0.0, 0.0, 0.0),))
    with pytest.raises(ValueError, match="executed trajectory"):
        compute_tracking_metrics(_straight_reference(), ())
    degenerate = (
        PathPoint(0.0, 0.0, 0.0, "F", "SWATH", ""),
        PathPoint(0.0, 0.0, 0.0, "F", "SWATH", ""),
    )
    with pytest.raises(ValueError, match="non-zero length"):
        compute_tracking_metrics(degenerate, (ExecutedPose(0.0, 0.0, 0.0, 0.0),))
