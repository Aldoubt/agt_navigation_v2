import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def qapp():
    return QApplication.instance() or QApplication([])


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
