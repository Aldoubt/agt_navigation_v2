import numpy as np
import pytest

from agt_offline_assets import (
    FREE,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    derive_navigation_structure,
)


def _result(
    *,
    width=80,
    height=50,
    resolution=0.10,
    slope_x=0.0,
    obstacle_rows=(),
    sparse_patch=False,
):
    columns = np.arange(width, dtype=np.float64)
    rows = np.arange(height, dtype=np.float64)
    xx, yy = np.meshgrid((columns + 0.5) * resolution, (rows + 0.5) * resolution)
    ground = slope_x * xx
    # One sharp cell should not dominate the scene-scale plane estimate.
    ground[height // 2, width // 2] += 0.12
    valid = np.ones((height, width), dtype=bool)
    point_count = np.full((height, width), 8, dtype=np.int32)
    ground_support = np.full((height, width), 6, dtype=np.int32)
    if sparse_patch:
        ground_support[10:18, 10:18] = 1
    obstacle_count = np.zeros((height, width), dtype=np.int32)
    for row in obstacle_rows:
        obstacle_count[row, 5:75] = 4
        # Short gaps should be structurally repairable.
        obstacle_count[row, 28:31] = 0
    occupancy = np.full((height, width), FREE, dtype=np.uint8)
    return NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=ground,
        ground_valid=valid,
        point_count=point_count,
        ground_support_count=ground_support,
        obstacle_count=obstacle_count,
        slope_deg=np.zeros_like(ground),
        step_m=np.zeros_like(ground),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            maximum_slope_deg=10.0,
            minimum_obstacle_points=2,
        ),
    )


def test_robust_local_plane_slope_tracks_scene_scale_plane():
    slope = np.tan(np.deg2rad(5.0))
    result = _result(slope_x=slope)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            robust_slope_window_m=0.50,
            row_direction_mode="provided",
        ),
        row_direction_xy=(1.0, 0.0),
    )
    interior = structure.robust_slope_deg[5:-5, 5:-5]
    finite = interior[np.isfinite(interior)]
    assert np.median(finite) == pytest.approx(5.0, abs=0.35)
    assert np.nanpercentile(finite, 95) < 7.0


def test_ground_confidence_penalizes_sparse_support():
    result = _result(sparse_patch=True)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(row_direction_mode="provided"),
        row_direction_xy=(1.0, 0.0),
    )
    dense = float(np.nanmedian(structure.ground_confidence[25:35, 25:35]))
    sparse = float(np.nanmedian(structure.ground_confidence[10:18, 10:18]))
    assert dense > 0.70
    assert sparse < dense * 0.40


def test_row_regularization_repairs_short_gaps_without_filling_whole_scene():
    result = _result(obstacle_rows=(15, 30))
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_half_width_m=0.16,
            row_max_gap_m=0.50,
            row_minimum_segment_length_m=1.50,
            row_minimum_spacing_m=0.80,
            row_minimum_prominence_ratio=0.05,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    assert len(structure.row_model.centers_v_m) >= 2
    # The synthetic three-cell gap lies inside a detected row and should close.
    assert np.any(structure.row_regularized_obstacle[14:17, 28:31])
    assert np.any(structure.row_regularized_obstacle[29:32, 28:31])
    # Space midway between the rows remains outside the regularized crop rows.
    assert not np.any(structure.row_regularized_obstacle[21:25, 20:60])


def test_aisle_candidate_requires_confident_ground_and_no_row_or_raw_obstacle():
    result = _result(obstacle_rows=(15, 30), sparse_patch=True)
    structure = derive_navigation_structure(
        result,
        NavigationStructureConfig(
            row_direction_mode="provided",
            row_half_width_m=0.16,
            row_minimum_prominence_ratio=0.05,
            aisle_minimum_ground_confidence=0.30,
        ),
        row_direction_xy=(1.0, 0.0),
    )
    assert np.any(structure.aisle_candidate[20:25, 35:55])
    assert not np.any(structure.aisle_candidate[14:17, 20:60])
    assert not np.any(structure.aisle_candidate[10:18, 10:18])
