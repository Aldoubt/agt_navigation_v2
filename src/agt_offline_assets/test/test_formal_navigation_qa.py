from pathlib import Path
from types import SimpleNamespace

import json
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
    materialize_structure_aware_navigation_map,
)
from agt_offline_assets.formal_navigation_override import (
    FormalOverrideReplayResult,
    replay_formal_navigation_overrides,
)
from agt_offline_assets.formal_navigation_qa import (
    FORMAL_NAVIGATION_QA_SCHEMA,
    evaluate_formal_navigation_qa,
    write_formal_navigation_qa,
)


def _navigation(occupancy):
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
        point_count=np.full((height, width), 4, dtype=np.int32),
        ground_support_count=np.full((height, width), 3, dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0),
    )


def _boundary(width=5.0, height=2.0):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (width, 0.0), (width, height), (0.0, height)),
    )


def _diagnostic():
    return SimpleNamespace(
        pair_index=1,
        pair_kind="ROW_ROW",
        left_row_center_v_m=0.0,
        right_row_center_v_m=2.0,
        status="ACCEPTED",
    )


def _corridor(*, row_band=None, aisle=None):
    if row_band is None:
        row_band = np.zeros((2, 5), dtype=bool)
    if aisle is None:
        aisle = np.ones((2, 5), dtype=bool)
    return SimpleNamespace(
        row_structural_band=np.asarray(row_band, dtype=bool),
        aisle_geometric_envelope=np.asarray(aisle, dtype=bool),
        aisle_pair_diagnostics=(_diagnostic(),),
    )


def _structure():
    return SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0], dtype=np.float64))
    )


def _fixture(ground_occupancy, *, row_band=None, aisle=None, overrides=()):
    ground = _navigation(ground_occupancy)
    corridor = _corridor(row_band=row_band, aisle=aisle)
    boundary = _boundary()
    materialized = materialize_structure_aware_navigation_map(
        ground, corridor, boundary
    )
    accepted = replay_formal_navigation_overrides(
        materialized.navigation,
        corridor,
        boundary,
        overrides,
    )
    return ground, corridor, boundary, materialized, accepted


def _report(ground, corridor, boundary, materialized, accepted, overrides=()):
    return evaluate_formal_navigation_qa(
        ground_evidence=ground,
        materialized=materialized,
        accepted=accepted,
        overrides=overrides,
        structure=_structure(),
        corridor=corridor,
        site_boundary=boundary,
    )


def test_valid_formal_map_passes_hard_invariants():
    fixture = _fixture([[FREE] * 5, [FREE] * 5])
    report = _report(*fixture)

    assert report["schema"] == FORMAL_NAVIGATION_QA_SCHEMA
    assert report["status"] == "PASS"
    assert report["hard_failures"] == []
    assert report["outside_site_boundary_free_count"] == 0
    assert report["row_structural_band_free_leak_count"] == 0
    assert report["accepted_matches_replay"] is True


def test_outside_boundary_free_is_hard_failure():
    ground, corridor, boundary, materialized, accepted = _fixture(
        [[FREE] * 5, [FREE] * 5]
    )
    tampered = accepted.navigation.occupancy.copy()
    # Shrink the QA boundary after materialization so column 4 is outside.
    boundary = _boundary(width=4.0)
    tampered[:, 4] = FREE
    accepted = FormalOverrideReplayResult(
        navigation=NavigationMapResult(
            **{**accepted.navigation.__dict__, "occupancy": tampered}
        ),
        force_free_changed_cell_count=0,
        force_occupied_changed_cell_count=0,
        force_free_area_m2=0.0,
        force_occupied_area_m2=0.0,
    )

    report = _report(ground, corridor, boundary, materialized, accepted)
    assert report["status"] == "FAIL"
    assert "OUTSIDE_SITE_BOUNDARY_FREE" in report["hard_failures"]


def test_row_structural_free_leak_is_hard_failure():
    row_band = np.zeros((2, 5), dtype=bool)
    row_band[0, 2] = True
    ground, corridor, boundary, materialized, accepted = _fixture(
        [[FREE] * 5, [FREE] * 5], row_band=row_band
    )
    tampered = accepted.navigation.occupancy.copy()
    tampered[0, 2] = FREE
    accepted = FormalOverrideReplayResult(
        navigation=NavigationMapResult(
            **{**accepted.navigation.__dict__, "occupancy": tampered}
        ),
        force_free_changed_cell_count=0,
        force_occupied_changed_cell_count=0,
        force_free_area_m2=0.0,
        force_occupied_area_m2=0.0,
    )

    report = _report(ground, corridor, boundary, materialized, accepted)
    assert report["status"] == "FAIL"
    assert "ROW_STRUCTURAL_FREE_LEAK" in report["hard_failures"]


def test_tampered_accepted_map_fails_replay_check():
    ground, corridor, boundary, materialized, accepted = _fixture(
        [[FREE] * 5, [FREE] * 5]
    )
    tampered = accepted.navigation.occupancy.copy()
    tampered[0, 2] = OCCUPIED
    accepted = FormalOverrideReplayResult(
        navigation=NavigationMapResult(
            **{**accepted.navigation.__dict__, "occupancy": tampered}
        ),
        force_free_changed_cell_count=0,
        force_occupied_changed_cell_count=0,
        force_free_area_m2=0.0,
        force_occupied_area_m2=0.0,
    )

    report = _report(ground, corridor, boundary, materialized, accepted)
    assert report["accepted_matches_replay"] is False
    assert "ACCEPTED_REPLAY_MISMATCH" in report["hard_failures"]


def test_per_aisle_fractions_and_connectivity_are_reported():
    ground = [[FREE, FREE, UNKNOWN, FREE, FREE], [FREE, FREE, UNKNOWN, FREE, FREE]]
    aisle = np.ones((2, 5), dtype=bool)
    # Keep the two UNKNOWN cells unresolved by removing them from structure inference.
    aisle[:, 2] = False
    fixture = _fixture(ground, aisle=aisle)
    report = _report(*fixture)
    aisle_report = report["aisles"][0]

    assert aisle_report["free_fraction"] == 0.8
    assert aisle_report["unknown_fraction"] == 0.2
    assert aisle_report["occupied_conflict_fraction"] == 0.0
    assert aisle_report["grid_connectivity"] is False


def test_qa_json_writer_is_deterministic(tmp_path: Path):
    fixture = _fixture([[FREE] * 5, [FREE] * 5])
    report = _report(*fixture)
    first = write_formal_navigation_qa(report, tmp_path / "first.json")
    second = write_formal_navigation_qa(report, tmp_path / "second.json")

    assert first.read_bytes() == second.read_bytes()
    assert json.loads(first.read_text())["status"] == "PASS"
