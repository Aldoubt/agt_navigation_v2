import math

import pytest

from agt_teach_repeat.path_processing import (
    path_length,
    process_path,
    resample_by_arclength,
    smooth_positions,
)
from agt_teach_repeat.path_types import (
    PathPose,
    ProcessingConfig,
    TeachRepeatError,
    normalize_quaternion,
    quaternion_from_yaw,
    wrap_angle,
)


def pose(stamp, x, y=0.0, yaw=0.0, speed=0.2):
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    return PathPose(stamp, x, y, 0.0, qx, qy, qz, qw, speed, 0.0, 0.0)


def test_quaternion_normalization_and_yaw_wrap():
    assert normalize_quaternion(0.0, 0.0, 0.0, 2.0) == (0.0, 0.0, 0.0, 1.0)
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)
    with pytest.raises(TeachRepeatError, match="non-zero"):
        normalize_quaternion(0.0, 0.0, 0.0, 0.0)


def test_duplicate_removal_resampling_and_endpoints():
    raw = [
        pose(1, 0.0),
        pose(1, 0.1),
        pose(2, 0.0),
        pose(3, 0.01),
        pose(4, 1.0),
        pose(5, 1.01),
    ]
    result = process_path(raw, ProcessingConfig(resample_distance_m=0.25))
    assert result.report.valid_count == 3
    assert result.poses[0].x == pytest.approx(0.0)
    assert result.poses[-1].x == pytest.approx(1.01)
    assert path_length(result.poses) == pytest.approx(1.01)


def test_arclength_resampling_keeps_first_and_last():
    raw = [pose(1, 0.0), pose(2, 0.35), pose(3, 1.0)]
    output = resample_by_arclength(raw, 0.30)
    assert output[0] == raw[0]
    assert output[-1] == raw[-1]
    assert [item.x for item in output] == pytest.approx([0.0, 0.3, 0.6, 0.9, 1.0])


def test_smoothing_is_bounded_and_preserves_endpoints():
    raw = [pose(index, index * 0.1, 0.1 if index == 5 else 0.0) for index in range(11)]
    smoothed, maximum = smooth_positions(raw, 5, 0.02)
    assert smoothed[0] == raw[0]
    assert smoothed[-1] == raw[-1]
    assert maximum <= 0.02
    assert math.hypot(smoothed[5].x - raw[5].x, smoothed[5].y - raw[5].y) == pytest.approx(0.02)


def test_in_place_rotation_preserves_original_yaw_before_translation():
    raw = [
        pose(1, 0.0, yaw=0.0, speed=0.0),
        pose(2, 0.0, yaw=math.pi / 2, speed=0.0),
        pose(3, 1.0, yaw=0.0),
    ]
    result = process_path(raw, ProcessingConfig(resample_distance_m=0.25))
    assert result.poses[0].yaw == pytest.approx(0.0)
    assert result.poses[1].x == pytest.approx(0.0)
    assert result.poses[1].yaw == pytest.approx(math.pi / 2)
    assert result.control_points[0]["theta"] == pytest.approx(0.0)


@pytest.mark.parametrize("raw", [[], [pose(1, 0.0)]])
def test_empty_and_single_point_paths_are_rejected(raw):
    with pytest.raises(TeachRepeatError, match="at least two"):
        process_path(raw)


def test_non_finite_points_cannot_produce_a_valid_path():
    with pytest.raises(TeachRepeatError):
        process_path([pose(1, 0.0), pose(2, math.nan)])
