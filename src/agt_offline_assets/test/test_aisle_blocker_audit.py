from types import SimpleNamespace

import numpy as np

from agt_offline_assets.aisle_blocker_audit import build_aisle_blocker_audit
from agt_offline_assets.navigation_map_derivation import FREE, OCCUPIED


def _fixture(occupancy: np.ndarray, *, cause: str | None = None):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    shape = occupancy.shape
    navigation = SimpleNamespace(
        occupancy=occupancy,
        origin_x_m=0.0,
        origin_y_m=0.0,
        resolution_m=1.0,
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0]))
    )
    diagnostic = SimpleNamespace(
        pair_index=1,
        pair_kind="ROW_ROW",
        status="ACCEPTED",
        left_row_center_v_m=0.0,
        right_row_center_v_m=float(shape[0]),
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.ones(shape, dtype=bool),
        aisle_pair_diagnostics=(diagnostic,),
    )

    empty = np.zeros(shape, dtype=bool)
    masks = {
        "strong_sensor_obstacle_mask": empty.copy(),
        "slope_hard_mask": empty.copy(),
        "step_hard_mask": empty.copy(),
        "soft_occupied_mask": empty.copy(),
    }
    if cause is not None:
        key = {
            "STRONG_SENSOR_OBSTACLE": "strong_sensor_obstacle_mask",
            "SLOPE_HARD": "slope_hard_mask",
            "STEP_HARD": "step_hard_mask",
        }[cause]
        masks[key] = occupancy == OCCUPIED
    provenance = SimpleNamespace(**masks)
    materialized = SimpleNamespace(
        row_structural_blocked_mask=empty.copy(),
        site_boundary_blocked_mask=empty.copy(),
    )
    return navigation, structure, corridor, provenance, materialized


def test_connected_aisle_reports_no_blocker():
    occupancy = np.full((3, 7), FREE, dtype=np.uint8)
    navigation, structure, corridor, provenance, materialized = _fixture(occupancy)

    reports = build_aisle_blocker_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        provenance,
        materialized=materialized,
    )

    assert len(reports) == 1
    report = reports[0]
    assert report["grid_connectivity"] is True
    assert report["failure_mode"] == "CONNECTED"
    assert report["minimum_blocker_cell_count"] == 0
    assert report["dominant_blocker_cause"] == "NONE"


def test_full_cross_section_sensor_barrier_is_localized_and_classified():
    occupancy = np.full((3, 7), FREE, dtype=np.uint8)
    occupancy[:, 3] = OCCUPIED
    navigation, structure, corridor, provenance, materialized = _fixture(
        occupancy, cause="STRONG_SENSOR_OBSTACLE"
    )

    report = build_aisle_blocker_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        provenance,
        materialized=materialized,
    )[0]

    assert report["grid_connectivity"] is False
    assert report["failure_mode"] == "NO_END_TO_END_COMPONENT"
    assert report["minimum_blocker_cell_count"] == 1
    assert report["dominant_blocker_cause"] == "STRONG_SENSOR_OBSTACLE"
    assert report["critical_blocker_cause_counts"]["STRONG_SENSOR_OBSTACLE"] >= 1


def test_missing_start_free_reports_start_block_and_slope_cause():
    occupancy = np.full((3, 7), FREE, dtype=np.uint8)
    occupancy[:, 0] = OCCUPIED
    navigation, structure, corridor, provenance, materialized = _fixture(
        occupancy, cause="SLOPE_HARD"
    )

    report = build_aisle_blocker_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        provenance,
        materialized=materialized,
    )[0]

    assert report["grid_connectivity"] is False
    assert report["failure_mode"] == "NO_START_FREE"
    assert report["minimum_blocker_cell_count"] >= 1
    assert report["dominant_blocker_cause"] == "SLOPE_HARD"


def test_unknown_blocker_is_reported_without_inventing_obstacle_evidence():
    unknown = np.uint8(205)
    occupancy = np.full((3, 7), FREE, dtype=np.uint8)
    occupancy[:, 3] = unknown
    navigation, structure, corridor, provenance, materialized = _fixture(occupancy)

    report = build_aisle_blocker_audit(
        navigation,
        structure,
        corridor,
        occupancy,
        provenance,
        materialized=materialized,
    )[0]

    assert report["grid_connectivity"] is False
    assert report["dominant_blocker_cause"] == "UNKNOWN"
    assert report["critical_blocker_cause_counts"]["UNKNOWN"] >= 1
