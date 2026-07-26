import math

import pytest

from agt_teach_repeat.path_processing import deterministic_percentile, nearest_segment_metrics
from agt_teach_repeat.path_types import PathPose, quaternion_from_yaw
from agt_teach_repeat.repeatability_metrics import repeatability_metrics


def pose(stamp, x, y=0.0, yaw=0.0):
    return PathPose(stamp, x, y, 0.0, *quaternion_from_yaw(yaw), frame_id="map")


def test_nearest_segment_cross_track_and_wrapped_heading():
    reference = (pose(1, 0.0), pose(2, 2.0))
    result = nearest_segment_metrics(reference, 0.5, 0.2, 2.0 * math.pi - 0.1)
    assert result["cross_track_error"] == pytest.approx(0.2)
    assert result["along_track_progress"] == pytest.approx(0.5)
    assert result["heading_error"] == pytest.approx(-0.1)


def test_p95_uses_deterministic_linear_interpolation():
    assert deterministic_percentile([0.0, 1.0, 2.0, 3.0, 4.0], 95) == pytest.approx(3.8)
    assert deterministic_percentile([4.0], 95) == 4.0


def test_repeatability_summary_uses_segments_and_declares_internal_basis():
    reference = (pose(1, 0.0), pose(2, 1.0), pose(3, 2.0))
    executed = (pose(1, 0.0, 0.1), pose(2, 1.0, 0.1), pose(3, 2.0, 0.1))
    metrics = repeatability_metrics(reference, executed, duration_s=4.0)
    assert metrics["completion_ratio"] == pytest.approx(1.0)
    assert metrics["lateral_rmse_m"] == pytest.approx(0.1)
    assert metrics["ground_truth_independent"] is False
    assert "onboard localization" in metrics["notice"]
