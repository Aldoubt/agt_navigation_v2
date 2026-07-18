from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from agt_coverage_planning.path_validator import (  # noqa: E402
    GridMap,
    PathValidationError,
    Pose2D,
)
from agt_coverage_planning.preview_audit import (  # noqa: E402
    audit_preview_path,
    merge_preview_grids,
)


FOOTPRINT = ((-0.4, -0.2), (0.4, -0.2), (0.4, 0.2), (-0.4, 0.2))


def _grid(cells=None, fill=0, origin_x=0.0):
    width, height = 100, 30
    data = [fill] * (width * height)
    for column, row, value in cells or []:
        data[row * width + column] = value
    return GridMap(
        width=width,
        height=height,
        resolution=0.1,
        origin_x=origin_x,
        origin_y=0.0,
        origin_yaw=0.0,
        data=tuple(data),
    )


def test_merge_preserves_unknown_and_semantic_obstacle_precedence():
    base = _grid(cells=[(10, 10, -1), (20, 10, -1)])
    mask = _grid(cells=[(20, 10, 100)])

    merged = merge_preview_grids(base, mask)

    assert merged.data[10 * 100 + 10] == -1
    assert merged.data[10 * 100 + 20] == 100


def test_preview_audit_classifies_base_and_keepout_collisions():
    poses = [Pose2D(1.0, 1.0, 0.0), Pose2D(8.0, 1.0, 0.0)]
    result = audit_preview_path(
        poses,
        _grid(cells=[(30, 10, 100)]),
        _grid(cells=[(60, 10, 100)]),
        FOOTPRINT,
        min_turning_radius=1.5,
    )

    assert result.report["status"] == "CONFLICT"
    assert result.report["advisory_only"] is True
    assert result.report["eligible_for_execution"] is False
    assert result.report["base_collision_pose_count"] > 0
    assert result.report["keepout_collision_pose_count"] > 0
    assert result.report["collision_pose_count"] > 0
    assert result.collision_samples


def test_preview_audit_never_marks_clear_candidate_executable():
    result = audit_preview_path(
        [Pose2D(1.0, 1.0, 0.0), Pose2D(8.0, 1.0, 0.0)],
        _grid(),
        _grid(),
        FOOTPRINT,
        min_turning_radius=1.5,
    )

    assert result.report["status"] == "CLEAR"
    assert result.report["eligible_for_execution"] is False


def test_mismatched_mask_metadata_is_rejected():
    with pytest.raises(PathValidationError) as error:
        merge_preview_grids(_grid(), _grid(origin_x=0.1))

    assert error.value.code == "preview_grid_metadata_mismatch"
