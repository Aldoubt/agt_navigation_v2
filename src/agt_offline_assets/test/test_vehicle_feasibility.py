from types import SimpleNamespace

import numpy as np

from agt_offline_assets.navigation_map_derivation import FREE
from agt_offline_assets.vehicle_feasibility import build_vehicle_feasible_aisle_audit


def _fixture():
    occupancy = np.full((5, 9), FREE, dtype=np.uint8)
    navigation = SimpleNamespace(
        occupancy=occupancy,
        resolution_m=0.10,
        origin_x_m=0.0,
        origin_y_m=0.0,
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.asarray([1.0, 0.0], dtype=np.float64))
    )
    diagnostic = SimpleNamespace(
        pair_kind="ROW_ROW",
        status="ACCEPTED",
        pair_index=1,
        left_row_center_v_m=0.0,
        right_row_center_v_m=0.5,
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.ones_like(occupancy, dtype=bool),
        aisle_pair_diagnostics=[diagnostic],
    )
    return navigation, structure, corridor, occupancy


def test_d11_connectivity_scope_is_explicit_and_compatibility_alias_matches():
    navigation, structure, corridor, occupancy = _fixture()

    report = build_vehicle_feasible_aisle_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        clearance_radius_m=0.10,
        terminal_inset_m=0.20,
    )[0]

    assert report["connectivity_scope"] == "INTERIOR_TERMINAL_BANDS"
    assert report["interior_terminal_raster_connectivity"] is True
    assert report["raster_grid_connectivity"] == report["interior_terminal_raster_connectivity"]
