import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PyQt5.QtWidgets import QApplication

from agt_offline_assets import (
    FREE,
    UNKNOWN,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    SiteBoundary,
)
from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def qapp():
    return QApplication.instance() or QApplication([])


def _install_candidate_fixture(window):
    resolution = 0.10
    shape = (3, 20)
    occupancy = np.full(shape, FREE, dtype=np.uint8)
    ground_valid = np.ones(shape, dtype=bool)
    ground_height = np.zeros(shape, dtype=np.float64)
    occupancy[1, 6:10] = UNKNOWN
    ground_valid[1, 6:10] = False
    ground_height[1, 6:10] = np.nan

    window._navigation_result = NavigationMapResult(
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=0.0,
        width=20,
        height=3,
        ground_height_m=ground_height,
        ground_valid=ground_valid,
        point_count=np.full(shape, 6, dtype=np.int32),
        ground_support_count=np.full(shape, 4, dtype=np.int32),
        obstacle_count=np.zeros(shape, dtype=np.int32),
        slope_deg=np.zeros(shape, dtype=np.float64),
        step_m=np.zeros(shape, dtype=np.float64),
        occupancy=occupancy,
        config=GroundRelativeNavigationConfig(
            resolution_m=resolution,
            maximum_slope_deg=15.0,
            maximum_step_m=0.12,
        ),
    )
    window._navigation_structure_result = SimpleNamespace(
        row_model=SimpleNamespace(
            direction_xy=np.array([1.0, 0.0], dtype=np.float64)
        )
    )
    geometric = np.zeros(shape, dtype=bool)
    geometric[1, 1:19] = True
    window._corridor_refinement_result = SimpleNamespace(
        aisle_geometric_envelope=geometric,
        row_structural_band=np.zeros(shape, dtype=bool),
    )
    window._site_boundary = SiteBoundary(
        frame_id="map",
        outer_boundary_xy=((0.0, 0.0), (2.0, 0.0), (2.0, 0.30), (0.0, 0.30)),
    )


def test_finish_site_boundary_creates_ready_asset():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._site_boundary_vertices = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 4.0),
            (0.0, 4.0),
        ]
        assert window._finish_site_boundary_authoring(show_errors=False)
        assert window._site_boundary is not None
        assert window._site_boundary.status == "READY"
        assert window._site_boundary.boundary_semantics == "VEHICLE_PERMITTED_INNER_BOUNDARY"
        app.processEvents()
    finally:
        window.close()


def test_clear_boundary_does_not_clear_navigation_overrides():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        window._navigation_overrides = [
            {
                "mode": "no_go",
                "polygon_xy": [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]],
            }
        ]
        window._site_boundary_vertices = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0)]
        window._clear_site_boundary()
        assert len(window._navigation_overrides) == 1
        assert window._site_boundary is None
        assert window._site_boundary_vertices == []
    finally:
        window.close()


def test_generate_12f_candidate_does_not_mutate_current_navigation_result():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        _install_candidate_fixture(window)
        current = window._navigation_result
        before = current.occupancy.copy()

        assert window._generate_12f_candidate(show_errors=False)
        assert window._navigation_result is current
        assert np.array_equal(window._navigation_result.occupancy, before)
        assert window._navigation_12f_result is not None
        assert window._traversability_evidence is not None
        assert np.all(window._navigation_12f_result.occupancy[1, 6:10] == FREE)
        app.processEvents()
    finally:
        window.close()


def test_generate_12f_candidate_requires_ready_site_boundary():
    app = qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        _install_candidate_fixture(window)
        window._site_boundary = None
        assert not window._generate_12f_candidate(show_errors=False)
        assert window._navigation_12f_result is None
        assert window._traversability_evidence is None
    finally:
        window.close()
