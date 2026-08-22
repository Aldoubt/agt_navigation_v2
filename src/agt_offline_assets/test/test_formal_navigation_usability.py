from types import SimpleNamespace

import numpy as np

from agt_offline_assets import (
    FREE,
    OCCUPIED,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_offline_assets.formal_navigation_map import materialize_structure_aware_navigation_map
from agt_offline_assets.formal_navigation_override import replay_formal_navigation_overrides
from agt_offline_assets.formal_navigation_qa import evaluate_formal_navigation_qa


def _navigation(occupancy):
    occupancy = np.asarray(occupancy, dtype=np.uint8)
    shape = occupancy.shape
    zeros_f = np.zeros(shape, dtype=np.float64)
    zeros_i = np.zeros(shape, dtype=np.int32)
    return NavigationMapResult(
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=shape[1],
        height=shape[0],
        ground_height_m=zeros_f.copy(),
        ground_valid=np.ones(shape, dtype=bool),
        point_count=np.full(shape, 20, dtype=np.int32),
        ground_support_count=np.full(shape, 10, dtype=np.int32),
        obstacle_count=zeros_i.copy(),
        slope_deg=zeros_f.copy(),
        step_m=zeros_f.copy(),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(resolution_m=1.0, obstacle_padding_m=0.0),
    )


def _boundary(width, height):
    return SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (width, 0.0), (width, height), (0.0, height)),
    )


def _fixture(occupancy):
    navigation = _navigation(occupancy)
    shape = navigation.occupancy.shape
    diagnostic = SimpleNamespace(
        pair_index=1,
        pair_kind="ROW_ROW",
        left_row_center_v_m=0.0,
        right_row_center_v_m=float(navigation.height),
        status="ACCEPTED",
    )
    corridor = SimpleNamespace(
        aisle_geometric_envelope=np.ones(shape, dtype=bool),
        row_structural_band=np.zeros(shape, dtype=bool),
        aisle_pair_diagnostics=(diagnostic,),
    )
    structure = SimpleNamespace(
        row_model=SimpleNamespace(direction_xy=np.array([1.0, 0.0], dtype=np.float64))
    )
    boundary = _boundary(navigation.width, navigation.height)
    materialized = materialize_structure_aware_navigation_map(
        navigation, corridor, boundary, row_direction_xy=(1.0, 0.0)
    )
    accepted = replay_formal_navigation_overrides(
        materialized.navigation, corridor, boundary, []
    )
    report = evaluate_formal_navigation_qa(
        ground_evidence=navigation,
        materialized=materialized,
        accepted=accepted,
        overrides=[],
        structure=structure,
        corridor=corridor,
        site_boundary=boundary,
    )
    return report


def test_structurally_safe_but_disconnected_map_requires_navigation_review():
    report = _fixture(
        [
            [FREE, FREE, OCCUPIED, FREE, FREE],
            [FREE, FREE, OCCUPIED, FREE, FREE],
        ]
    )

    assert report["status"] == "PASS"
    assert report["structural_safety_status"] == "PASS"
    assert report["accepted_aisle_count"] == 1
    assert report["connected_aisle_count"] == 0
    assert report["navigation_usability_status"] == "REVIEW_REQUIRED"
    assert report["largest_free_component_fraction"] == 0.5


def test_fully_connected_map_reports_navigation_usability_pass():
    report = _fixture([[FREE] * 5, [FREE] * 5])

    assert report["status"] == "PASS"
    assert report["connected_aisle_count"] == 1
    assert report["navigation_usability_status"] == "PASS"
    assert report["largest_free_component_fraction"] == 1.0
