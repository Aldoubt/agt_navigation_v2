import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow
from agt_map_workbench.route_debug_panel import RouteDebugPanel


def _qapp():
    return QApplication.instance() or QApplication([])


def test_review_workbench_contains_route_debug_control_tab():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        labels = [
            window._control_tabs.tabText(index)
            for index in range(window._control_tabs.count())
        ]
        assert labels == ["点云编辑", "坐标系标定", "导航地图", "路径调试"]
        assert isinstance(window._route_debug_panel, RouteDebugPanel)
        assert window._route_debug_tab_index == 3
        app.processEvents()
    finally:
        window.close()


def test_route_debug_activation_does_not_leave_overlay_on_authoring_tabs():
    app = _qapp()
    window = ReviewMapWorkbenchWindow()
    try:
        route_index = window._route_debug_tab_index
        window._control_tabs.setCurrentIndex(route_index)
        app.processEvents()
        assert window._route_debug_panel.controller.is_active()

        window._control_tabs.setCurrentIndex(0)
        app.processEvents()
        assert not window._route_debug_panel.controller.is_active()
    finally:
        window.close()
