import numpy as np

from agt_offline_assets import (
    FREE,
    UNKNOWN,
    CorridorRefinementConfig,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
    derive_corridor_refinement,
)


def _fixture():
    resolution = 0.10
    height, width = 50, 60
    shape = (height, width)
    point_count = np.full(shape, 8, dtype=np.int32)
    ground_support = np.full(shape, 6, dtype=np.int32)
    obstacle_count = np.zeros(shape, dtype=np.int32)

    # Two wall-like elongated obstacle bands at the outer Y boundary plus three
    # interior crop rows.  Vegetation is intentionally wider than the nominal
    # structural row width.
    y_centers = [0.20, 1.00, 2.00, 3.00, 4.80]
    for center in y_centers:
        row = int(center / resolution)
        obstacle_count[max(0, row - 2):min(height, row + 3), 5:55] = 5

    # Independent obstacle inside the aisle between y=1 and y=2.
    obstacle_count[14:17, 28:31] = 8

    nav_config = GroundRelativeNavigationConfig(
        resolution_m=resolution,
        minimum_obstacle_points=2,
        maximum_slope_deg=15.0,
    )
    navigation = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=np.zeros(shape, dtype=np.float64),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=point_count,
        ground_support_count=ground_support,
        obstacle_count=obstacle_count,
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=np.full(shape, UNKNOWN, dtype=np.uint8),
        config=nav_config,
    )

    row_regularized = np.zeros(shape, dtype=bool)
    for center in y_centers:
        row = int(center / resolution)
        row_regularized[max(0, row - 2):min(height, row + 3), 5:55] = True
    structure = NavigationStructureResult(
        ground_confidence=np.ones(shape, dtype=np.float64),
        robust_slope_deg=np.zeros(shape, dtype=np.float64),
        robust_plane_residual_m=np.zeros(shape, dtype=np.float64),
        row_support=row_regularized.astype(np.float64),
        row_regularized_obstacle=row_regularized,
        aisle_candidate=~row_regularized,
        row_model=RowModel(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64),
            angle_deg=0.0,
            centers_v_m=tuple(y_centers),
            half_width_m=0.30,
            support_fraction=tuple(0.8 for _ in y_centers),
        ),
        config=NavigationStructureConfig(row_half_width_m=0.30),
    )
    return navigation, structure


def test_boundary_wall_peaks_are_not_crop_rows():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        CorridorRefinementConfig(boundary_exclusion_m=0.45),
    )
    assert result.accepted_row_centers_v_m == (1.0, 2.0, 3.0)
    assert 0.20 in result.rejected_row_centers_v_m
    assert 4.80 in result.rejected_row_centers_v_m


def test_structural_row_band_is_independent_from_vegetation_envelope_width():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        CorridorRefinementConfig(row_structural_half_width_m=0.15),
    )
    # Vegetation spans roughly five 10-cm rows while the structural band is
    # intentionally narrower and should therefore occupy fewer cells.
    assert np.count_nonzero(result.vegetation_envelope) > np.count_nonzero(
        result.row_structural_band
    )
    assert np.count_nonzero(result.row_centerline) > 0


def test_refined_aisle_exists_only_between_adjacent_crop_rows():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        CorridorRefinementConfig(
            row_structural_half_width_m=0.15,
            aisle_side_clearance_m=0.05,
            raw_obstacle_clearance_m=0.05,
            aisle_minimum_width_m=0.35,
        ),
    )
    # Interior aisle y=1.5 has candidates, but wall-row exterior y=0.6 and
    # outside the last row y=3.8 do not.
    assert np.count_nonzero(result.aisle_candidate[15, :]) > 0
    assert np.count_nonzero(result.aisle_candidate[6, :]) == 0
    assert np.count_nonzero(result.aisle_candidate[38, :]) == 0


def test_raw_obstacle_clearance_cuts_hole_in_aisle_candidate():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        CorridorRefinementConfig(
            row_structural_half_width_m=0.15,
            aisle_side_clearance_m=0.05,
            raw_obstacle_clearance_m=0.15,
            aisle_minimum_width_m=0.35,
        ),
    )
    assert not result.aisle_candidate[15, 29]
    assert np.count_nonzero(result.aisle_candidate[15, 10:20]) > 0


def test_default_safety_width_rejects_fixture_aisle_that_is_too_narrow():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure)
    # Default geometry leaves only 0.36 m between adjacent 1.00 m rows:
    # 1.00 - 2*0.20 structural half-width - 2*0.12 side clearance.
    # That is intentionally below the default 0.45 m minimum aisle width.
    assert np.count_nonzero(result.aisle_candidate) == 0
    assert np.count_nonzero(result.aisle_centerline) == 0


def test_aisle_centerline_is_subset_of_refined_aisle_when_corridor_is_feasible():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        CorridorRefinementConfig(
            row_structural_half_width_m=0.15,
            aisle_side_clearance_m=0.05,
            raw_obstacle_clearance_m=0.05,
            aisle_minimum_width_m=0.35,
        ),
    )
    assert np.all(~result.aisle_centerline | result.aisle_candidate)
    assert np.count_nonzero(result.aisle_candidate) > 0
    assert np.count_nonzero(result.aisle_centerline) > 0
