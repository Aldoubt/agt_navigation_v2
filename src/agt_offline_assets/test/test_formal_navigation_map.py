from types import SimpleNamespace

import numpy as np
import pytest

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.formal_navigation_map import (
    materialize_structure_aware_navigation_map,
)


def make_navigation(occupancy):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    height, width = occupancy.shape
    zeros_f = np.zeros((height, width), dtype=np.float64)
    zeros_i = np.zeros((height, width), dtype=np.int32)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=width,
        height=height,
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones((height, width), dtype=bool),
        point_count=np.ones((height, width), dtype=np.int32),
        ground_support_count=np.ones((height, width), dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def make_corridor(aisle, row_band):
    return SimpleNamespace(
        aisle_geometric_envelope=np.asarray(aisle, dtype=bool),
        row_structural_band=np.asarray(row_band, dtype=bool),
    )


def full_boundary(navigation, *, frame_id="map"):
    return SiteBoundary(
        frame_id=frame_id,
        outer_boundary_xy=(
            (0.0, 0.0),
            (float(navigation.width), 0.0),
            (float(navigation.width), float(navigation.height)),
            (0.0, float(navigation.height)),
        ),
    )


def test_materializer_only_promotes_unknown_inside_aisle():
    navigation = make_navigation(
        [
            [FREE, UNKNOWN, UNKNOWN, OCCUPIED, UNKNOWN, FREE],
            [FREE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, FREE],
            [FREE, FREE, FREE, FREE, FREE, FREE],
        ]
    )
    corridor = make_corridor(
        aisle=[
            [False, True, True, True, False, False],
            [False, True, True, True, True, False],
            [False, False, False, False, False, False],
        ],
        row_band=[
            [False, False, True, False, False, False],
            [False, False, False, False, True, False],
            [False, False, False, False, False, False],
        ],
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        corridor,
        full_boundary(navigation),
    )
    occupancy = result.navigation.occupancy

    assert occupancy[0, 1] == FREE
    assert result.structure_inferred_free_mask[0, 1]
    assert occupancy[0, 2] == OCCUPIED
    assert occupancy[0, 3] == OCCUPIED
    assert occupancy[0, 4] == UNKNOWN
    assert result.navigation.resolution_m == navigation.resolution_m
    assert result.navigation.origin_x_m == navigation.origin_x_m
    assert result.navigation.origin_y_m == navigation.origin_y_m
    assert result.navigation.occupancy.shape == navigation.occupancy.shape


def test_materializer_hard_blocks_row_band_even_when_ground_is_free():
    navigation = make_navigation([[FREE, FREE, FREE]])
    corridor = make_corridor(
        aisle=[[False, False, False]],
        row_band=[[False, True, False]],
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        corridor,
        full_boundary(navigation),
    )

    assert result.navigation.occupancy[0, 1] == OCCUPIED
    assert result.row_structural_blocked_mask[0, 1]


def test_materializer_hard_blocks_cells_outside_site_boundary():
    navigation = make_navigation([[FREE, FREE, FREE, FREE]])
    corridor = make_corridor(
        aisle=[[False, False, False, False]],
        row_band=[[False, False, False, False]],
    )
    boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)),
    )

    result = materialize_structure_aware_navigation_map(
        navigation,
        corridor,
        boundary,
    )

    assert np.all(result.navigation.occupancy[0, 2:] == OCCUPIED)
    assert np.all(result.site_boundary_blocked_mask[0, 2:])


def test_materializer_rejects_mismatched_corridor_shape():
    navigation = make_navigation([[UNKNOWN, UNKNOWN]])
    corridor = make_corridor(
        aisle=[[True]],
        row_band=[[False]],
    )

    with pytest.raises(ValueError, match="grid shape mismatch"):
        materialize_structure_aware_navigation_map(
            navigation,
            corridor,
            full_boundary(navigation),
        )


def test_materializer_rejects_wrong_site_boundary_frame():
    navigation = make_navigation([[UNKNOWN, UNKNOWN]])
    corridor = make_corridor(
        aisle=[[True, True]],
        row_band=[[False, False]],
    )

    with pytest.raises(ValueError, match="frame_id mismatch"):
        materialize_structure_aware_navigation_map(
            navigation,
            corridor,
            full_boundary(navigation, frame_id="odom"),
        )
