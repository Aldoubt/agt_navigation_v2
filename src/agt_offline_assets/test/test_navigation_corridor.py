import numpy as np

from agt_offline_assets import (
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
    # interior crop rows. Vegetation is intentionally wider than the nominal
    # structural row width.
    y_centers = [0.20, 1.00, 2.00, 3.00, 4.80]
    for center in y_centers:
        row = int(center / resolution)
        obstacle_count[max(0, row - 2):min(height, row + 3), 5:55] = 5

    # Independent obstacle inside the aisle between y=1 and y=2. It blocks the
    # exact mathematical midpoint but leaves safe cells on either side.
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


def _feasible_config(**kwargs):
    values = dict(
        row_structural_half_width_m=0.15,
        aisle_side_clearance_m=0.05,
        raw_obstacle_clearance_m=0.05,
        aisle_minimum_width_m=0.35,
    )
    values.update(kwargs)
    return CorridorRefinementConfig(**values)


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
    assert np.count_nonzero(result.vegetation_envelope) > np.count_nonzero(
        result.row_structural_band
    )
    assert np.count_nonzero(result.row_centerline) > 0


def test_refined_aisle_exists_only_between_adjacent_crop_rows_by_default():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure, _feasible_config())
    assert np.count_nonzero(result.aisle_candidate[15, :]) > 0
    assert np.count_nonzero(result.aisle_candidate[6, :]) == 0
    assert np.count_nonzero(result.aisle_candidate[38, :]) == 0
    assert np.count_nonzero(result.boundary_aisle_candidate) == 0
    assert np.count_nonzero(result.boundary_aisle_centerline) == 0


def test_raw_obstacle_clearance_cuts_hole_in_aisle_candidate():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(
        navigation,
        structure,
        _feasible_config(raw_obstacle_clearance_m=0.15),
    )
    assert not result.aisle_candidate[15, 29]
    assert np.count_nonzero(result.aisle_candidate[15, 10:20]) > 0


def test_default_safety_width_rejects_fixture_aisle_that_is_too_narrow():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure)
    assert np.count_nonzero(result.aisle_candidate) == 0
    assert np.count_nonzero(result.aisle_centerline) == 0
    assert len(result.aisle_pair_diagnostics) == 2
    for diagnostic in result.aisle_pair_diagnostics:
        assert diagnostic.status == "REJECTED_TOO_NARROW"
        assert diagnostic.pair_kind == "ROW_ROW"
        assert np.isclose(diagnostic.center_distance_m, 1.0)
        assert np.isclose(diagnostic.structural_reserved_m, 0.40)
        assert np.isclose(diagnostic.side_clearance_reserved_m, 0.24)
        assert np.isclose(diagnostic.geometric_available_width_m, 0.36)
        assert np.isclose(diagnostic.minimum_required_width_m, 0.45)
        assert diagnostic.safe_cell_count == 0
        assert diagnostic.centerline_cell_count == 0


def test_aisle_centerline_is_subset_of_refined_aisle_when_corridor_is_feasible():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure, _feasible_config())
    assert np.all(~result.aisle_centerline | result.aisle_candidate)
    assert np.count_nonzero(result.aisle_candidate) > 0
    assert np.count_nonzero(result.aisle_centerline) > 0
    assert len(result.aisle_pair_diagnostics) == 2
    for diagnostic in result.aisle_pair_diagnostics:
        assert diagnostic.status == "ACCEPTED"
        assert diagnostic.geometric_cell_count > 0
        assert diagnostic.safe_cell_count > 0
        assert diagnostic.centerline_cell_count > 0
        assert diagnostic.longitudinal_overlap_m is not None
        assert diagnostic.longitudinal_overlap_m >= 1.5


def test_aisle_centerline_shifts_around_midpoint_obstacle_when_side_clearance_exists():
    navigation, structure = _fixture()
    result = derive_corridor_refinement(navigation, structure, _feasible_config())
    assert not np.any(result.aisle_centerline[14:17, 28:31])
    assert np.any(result.aisle_centerline[12:19, 28:31])
    assert np.all(~result.aisle_centerline | result.aisle_candidate)


def test_explicit_boundary_aisles_connect_wall_anchor_to_nearest_crop_row():
    navigation, structure = _fixture()
    config = _feasible_config(
        enable_boundary_aisles=True,
        boundary_anchor_max_distance_m=0.80,
        boundary_wall_half_width_m=0.10,
        boundary_wall_clearance_m=0.05,
    )
    result = derive_corridor_refinement(navigation, structure, config)

    assert np.count_nonzero(result.boundary_aisle_candidate) > 0
    assert np.count_nonzero(result.boundary_aisle_centerline) > 0
    assert np.all(~result.boundary_aisle_candidate | result.aisle_candidate)
    assert np.all(~result.boundary_aisle_centerline | result.aisle_centerline)

    kinds = [diagnostic.pair_kind for diagnostic in result.aisle_pair_diagnostics]
    assert kinds[:2] == ["ROW_ROW", "ROW_ROW"]
    assert "BOUNDARY_LOW" in kinds
    assert "BOUNDARY_HIGH" in kinds
    boundary_diagnostics = [
        diagnostic
        for diagnostic in result.aisle_pair_diagnostics
        if diagnostic.pair_kind.startswith("BOUNDARY")
    ]
    assert len(boundary_diagnostics) == 2
    assert all(diagnostic.status == "ACCEPTED" for diagnostic in boundary_diagnostics)
    assert all(diagnostic.safe_cell_count > 0 for diagnostic in boundary_diagnostics)
    assert all(diagnostic.centerline_cell_count > 0 for diagnostic in boundary_diagnostics)
