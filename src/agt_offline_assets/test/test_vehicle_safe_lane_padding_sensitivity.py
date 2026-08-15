import numpy as np
import pytest

from agt_offline_assets.navigation_grid import NavigationGridEvidence
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED
from agt_offline_assets.vehicle_safe_lane_padding_sensitivity import (
    _counterfactual_navigation,
)


def _navigation():
    occupancy = np.full((7, 7), FREE, dtype=np.uint8)
    return NavigationGridEvidence(
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=7,
        height=7,
        occupancy=occupancy,
        frame_id="map",
    )


def test_counterfactual_padding_reconstruction_is_monotonic_and_immutable():
    navigation = _navigation()
    original = navigation.occupancy.copy()
    direct = np.zeros((7, 7), dtype=bool)
    direct[3, 3] = True
    ground_valid = np.ones((7, 7), dtype=bool)
    ground_support = np.full((7, 7), 3, dtype=np.int32)

    map0 = _counterfactual_navigation(
        navigation,
        direct,
        ground_valid,
        ground_support,
        2,
        0,
    )
    map1 = _counterfactual_navigation(
        navigation,
        direct,
        ground_valid,
        ground_support,
        2,
        1,
    )
    map2 = _counterfactual_navigation(
        navigation,
        direct,
        ground_valid,
        ground_support,
        2,
        2,
    )

    assert np.array_equal(navigation.occupancy, original)
    assert np.count_nonzero(map0.occupancy == OCCUPIED) == 1
    assert np.count_nonzero(map1.occupancy == OCCUPIED) == 9
    assert np.count_nonzero(map2.occupancy == OCCUPIED) == 25
    assert np.count_nonzero(map0.occupancy == FREE) > np.count_nonzero(map1.occupancy == FREE)
    assert np.count_nonzero(map1.occupancy == FREE) > np.count_nonzero(map2.occupancy == FREE)


def test_counterfactual_padding_rejects_negative_cell_radius():
    navigation = _navigation()
    direct = np.zeros((7, 7), dtype=bool)
    ground_valid = np.ones((7, 7), dtype=bool)
    ground_support = np.full((7, 7), 3, dtype=np.int32)

    with pytest.raises(ValueError, match="map_padding_cells"):
        _counterfactual_navigation(
            navigation,
            direct,
            ground_valid,
            ground_support,
            2,
            -1,
        )
