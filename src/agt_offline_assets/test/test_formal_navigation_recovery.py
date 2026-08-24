from types import SimpleNamespace

import numpy as np

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.formal_navigation_map import (
    StructureAwareNavigationConfig,
    derive_hard_occupancy_provenance,
    materialize_structure_aware_navigation_map,
)


def _navigation(
    occupancy,
    *,
    obstacle_count=None,
    point_count=None,
    ground_support_count=None,
    slope_deg=None,
    step_m=None,
    resolution=0.1,
    obstacle_padding_m=0.0,
):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    if obstacle_count is None:
        obstacle_count = zeros_i.copy()
    if point_count is None:
        point_count = np.full(shape, 100, dtype=np.int32)
    if ground_support_count is None:
        ground_support_count = np.full(shape, 20, dtype=np.int32)
    if slope_deg is None:
        slope_deg = zeros_f.copy()
    if step_m is None:
        step_m = zeros_f.copy()
    return NavigationMapResult(
        resolution_m=float(resolution),
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.asarray(point_count, dtype=np.int32),
        ground_support_count=np.asarray(ground_support_count, dtype=np.int32),
        obstacle_count=np.asarray(obstacle_count, dtype=np.int32),
        slope_deg=np.asarray(slope_deg, dtype=np.float64),
        step_m=np.asarray(step_m, dtype=np.float64),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(
            resolution_m=float(resolution),
            minimum_obstacle_points=2,
            maximum_slope_deg=10.0,
            maximum_step_m=0.12,
            obstacle_padding_m=float(obstacle_padding_m),
        ),
    )


def _corridor(shape):
    return SimpleNamespace(
        aisle_geometric_envelope=np.ones(shape, dtype=bool),
        row_structural_band=np.zeros(shape, dtype=bool),
    )


def _boundary(navigation):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=(
            (0.0, 0.0),
            (navigation.width * navigation.resolution_m, 0.0),
            (
                navigation.width * navigation.resolution_m,
                navigation.height * navigation.resolution_m,
            ),
            (0.0, navigation.height * navigation.resolution_m),
        ),
    )


def _config():
    return StructureAwareNavigationConfig(
        soft_obstacle_max_count=4,
        soft_obstacle_max_ratio=0.05,
        soft_recovery_max_gap_m=0.30,
    )


def test_short_low_ratio_sensor_obstacle_gap_is_recovered_along_row_direction():
    navigation = _navigation(
        [[FREE, OCCUPIED, FREE, FREE, FREE]],
        obstacle_count=[[0, 2, 0, 0, 0]],
        point_count=[[100, 100, 100, 100, 100]],
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        _corridor(navigation.occupancy.shape),
        _boundary(navigation),
        row_direction_xy=(1.0, 0.0),
        config=_config(),
    )

    assert result.navigation.occupancy[0, 1] == FREE
    assert result.base_soft_occupied_mask[0, 1]
    assert result.structure_recovered_soft_occupied_mask[0, 1]
    assert not result.base_hard_occupied_mask[0, 1]


def test_strong_sensor_obstacle_stays_hard_occupied():
    navigation = _navigation(
        [[FREE, OCCUPIED, FREE]],
        obstacle_count=[[0, 12, 0]],
        point_count=[[100, 100, 100]],
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        _corridor(navigation.occupancy.shape),
        _boundary(navigation),
        row_direction_xy=(1.0, 0.0),
        config=_config(),
    )

    assert result.navigation.occupancy[0, 1] == OCCUPIED
    assert result.base_hard_occupied_mask[0, 1]
    assert not result.structure_recovered_soft_occupied_mask[0, 1]


def test_geometry_failure_stays_hard_even_with_weak_obstacle_count():
    navigation = _navigation(
        [[FREE, OCCUPIED, FREE]],
        obstacle_count=[[0, 2, 0]],
        point_count=[[100, 100, 100]],
        slope_deg=[[0.0, 25.0, 0.0]],
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        _corridor(navigation.occupancy.shape),
        _boundary(navigation),
        row_direction_xy=(1.0, 0.0),
        config=_config(),
    )

    assert result.navigation.occupancy[0, 1] == OCCUPIED
    assert result.base_hard_occupied_mask[0, 1]


def test_soft_obstacle_is_not_recovered_from_lateral_free_neighbors_only():
    navigation = _navigation(
        [
            [OCCUPIED, FREE, OCCUPIED],
            [OCCUPIED, OCCUPIED, OCCUPIED],
            [OCCUPIED, FREE, OCCUPIED],
        ],
        obstacle_count=[
            [12, 0, 12],
            [12, 2, 12],
            [12, 0, 12],
        ],
        point_count=np.full((3, 3), 100, dtype=np.int32),
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        _corridor(navigation.occupancy.shape),
        _boundary(navigation),
        row_direction_xy=(1.0, 0.0),
        config=_config(),
    )

    assert result.navigation.occupancy[1, 1] == OCCUPIED
    assert result.base_soft_occupied_mask[1, 1]
    assert not result.structure_recovered_soft_occupied_mask[1, 1]


def test_unknown_recovery_contract_is_preserved():
    navigation = _navigation([[FREE, UNKNOWN, FREE]])

    result = materialize_structure_aware_navigation_map(
        navigation,
        _corridor(navigation.occupancy.shape),
        _boundary(navigation),
        row_direction_xy=(1.0, 0.0),
        config=_config(),
    )

    assert result.navigation.occupancy[0, 1] == FREE
    assert result.structure_inferred_unknown_free_mask[0, 1]
    assert result.structure_inferred_free_mask[0, 1]


def test_hard_occupancy_provenance_separates_sensor_slope_and_step_causes():
    navigation = _navigation(
        [[OCCUPIED, OCCUPIED, OCCUPIED, OCCUPIED]],
        obstacle_count=[[12, 2, 0, 0]],
        point_count=[[100, 100, 100, 100]],
        slope_deg=[[0.0, 0.0, 15.0, 0.0]],
        step_m=[[0.0, 0.0, 0.0, 0.20]],
    )

    provenance = derive_hard_occupancy_provenance(navigation, _config())

    assert provenance.direct_obstacle_mask.tolist() == [[True, True, False, False]]
    assert provenance.strong_sensor_obstacle_mask.tolist() == [[True, False, False, False]]
    assert provenance.slope_hard_mask.tolist() == [[False, False, True, False]]
    assert provenance.step_hard_mask.tolist() == [[False, False, False, True]]
    assert provenance.hard_before_padding_mask.tolist() == [[True, False, True, True]]
    assert provenance.soft_occupied_mask.tolist() == [[False, True, False, False]]


def test_hard_occupancy_provenance_exposes_padding_added_cells_without_changing_policy():
    navigation = _navigation(
        [
            [FREE, OCCUPIED, OCCUPIED],
            [FREE, OCCUPIED, FREE],
            [FREE, FREE, FREE],
        ],
        obstacle_count=[
            [0, 0, 0],
            [0, 12, 0],
            [0, 0, 0],
        ],
        point_count=np.full((3, 3), 100, dtype=np.int32),
        obstacle_padding_m=0.05,
    )

    provenance = derive_hard_occupancy_provenance(navigation, _config())

    assert provenance.padding_cells == 1
    assert provenance.hard_before_padding_mask[1, 1]
    assert provenance.hard_after_padding_mask[0, 1]
    assert provenance.hard_after_padding_mask[0, 2]
    assert provenance.padding_added_hard_mask[0, 1]
    assert provenance.padding_added_hard_mask[0, 2]
    assert not provenance.padding_added_hard_mask[1, 1]
