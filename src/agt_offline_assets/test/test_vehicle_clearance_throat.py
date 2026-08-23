from types import SimpleNamespace

import numpy as np
import pytest

from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED


def _materialization_fixture(occupancy):
    shape = occupancy.shape
    blocked = np.asarray(occupancy, dtype=np.uint8) != FREE
    empty = np.zeros(shape, dtype=bool)
    materialized = SimpleNamespace(
        base_hard_occupied_mask=blocked.copy(),
        base_soft_occupied_mask=empty.copy(),
        row_structural_blocked_mask=empty.copy(),
        site_boundary_blocked_mask=empty.copy(),
        unresolved_unknown_mask=empty.copy(),
    )
    provenance = SimpleNamespace(
        direct_obstacle_mask=blocked.copy(),
        strong_sensor_obstacle_mask=blocked.copy(),
        slope_hard_mask=empty.copy(),
        step_hard_mask=empty.copy(),
        soft_occupied_mask=empty.copy(),
    )
    return materialized, provenance


def _fixture(occupancy, geometric, *, left_v, right_v):
    navigation = SimpleNamespace(
        occupancy=np.asarray(occupancy, dtype=np.uint8),
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
        left_row_center_v_m=float(left_v),
        right_row_center_v_m=float(right_v),
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.asarray(geometric, dtype=bool),
        aisle_pair_diagnostics=[diagnostic],
    )
    materialized, provenance = _materialization_fixture(occupancy)
    return navigation, structure, corridor, materialized, provenance


def test_d32_environment_distance_indices_resolve_diagonal_nearest_non_free_cell():
    from agt_offline_assets.vehicle_clearance_throat import _environment_clearance_with_nearest

    occupancy = np.full((5, 5), FREE, dtype=np.uint8)
    occupancy[1, 1] = OCCUPIED

    clearance, nearest_row, nearest_col = _environment_clearance_with_nearest(
        occupancy,
        0.10,
    )

    assert clearance[2, 2] == pytest.approx(np.sqrt(2.0) * 0.10 - 0.05)
    assert int(nearest_row[2, 2]) == 1
    assert int(nearest_col[2, 2]) == 1


def test_d31_environment_throat_reports_world_location_and_constraint_sources():
    from agt_offline_assets.vehicle_clearance_throat import build_vehicle_clearance_throat_audit

    occupancy = np.full((7, 11), FREE, dtype=np.uint8)
    occupancy[:3, 5] = OCCUPIED
    occupancy[4:, 5] = OCCUPIED
    geometric = np.ones_like(occupancy, dtype=bool)
    navigation, structure, corridor, materialized, provenance = _fixture(
        occupancy,
        geometric,
        left_v=0.0,
        right_v=0.7,
    )

    audit = build_vehicle_clearance_throat_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        clearance_radius_m=0.20,
        terminal_inset_m=0.20,
        materialized=materialized,
        provenance=provenance,
    )

    assert audit["schema"] == "agt_vehicle_clearance_throat_audit/v2"
    assert audit["clearance_throat_aisles"] == 1
    assert audit["nearest_environment_constraint_causes"] == {
        "STRONG_SENSOR_OBSTACLE": 1,
    }
    report = audit["aisles"][0]
    assert report["status"] == "CLEARANCE_THROAT"
    assert report["interior_terminal_raster_connectivity"] is True
    assert report["vehicle_feasible_connectivity"] is False
    assert report["bottleneck_clearance_m"] == pytest.approx(0.05)
    assert report["clearance_deficit_m"] == pytest.approx(0.15)
    assert report["limiting_constraint"] == "ENVIRONMENT_NON_FREE"

    throat = report["primary_throat"]
    assert throat["row"] == 3
    assert throat["col"] == 5
    assert throat["world_x_m"] == pytest.approx(0.55)
    assert throat["world_y_m"] == pytest.approx(0.35)
    assert throat["environment_clearance_m"] == pytest.approx(0.05)
    assert throat["lateral_clearance_m"] > throat["environment_clearance_m"]
    assert throat["nearest_environment_constraint"]["source"] == "STRONG_SENSOR_OBSTACLE"
    assert throat["nearest_environment_constraint"]["distance_m"] == pytest.approx(0.05)
    assert throat["lower_v_constraint"]["source"] == "STRONG_SENSOR_OBSTACLE"
    assert throat["upper_v_constraint"]["source"] == "STRONG_SENSOR_OBSTACLE"


def test_d31_lateral_boundary_throat_is_not_misreported_as_environment_obstacle():
    from agt_offline_assets.vehicle_clearance_throat import build_vehicle_clearance_throat_audit

    occupancy = np.full((9, 11), FREE, dtype=np.uint8)
    geometric = np.zeros_like(occupancy, dtype=bool)
    geometric[3:6, :] = True
    navigation, structure, corridor, materialized, provenance = _fixture(
        occupancy,
        geometric,
        left_v=0.30,
        right_v=0.60,
    )

    audit = build_vehicle_clearance_throat_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        clearance_radius_m=0.20,
        terminal_inset_m=0.20,
        materialized=materialized,
        provenance=provenance,
    )

    assert audit["nearest_environment_constraint_causes"] == {}
    report = audit["aisles"][0]
    assert report["status"] == "CLEARANCE_THROAT"
    assert report["bottleneck_clearance_m"] == pytest.approx(0.15)
    assert report["limiting_constraint"] == "LATERAL_AISLE_BOUNDARY"
    throat = report["primary_throat"]
    assert throat["nearest_environment_constraint"] is None
    assert throat["lower_v_constraint"]["source"] == "LATERAL_AISLE_BOUNDARY"
    assert throat["upper_v_constraint"]["source"] == "LATERAL_AISLE_BOUNDARY"
