import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from agt_map_workbench.review_workbench import ReviewMapWorkbenchWindow


def qapp():
    return QApplication.instance() or QApplication([])


def test_missing_vehicle_feasible_segment_sibling_is_quiet_and_clears_state(tmp_path):
    app = qapp()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    source = run_dir / "map.pcd"
    source.write_text("placeholder\n", encoding="utf-8")

    window = ReviewMapWorkbenchWindow()
    try:
        window._source_path = source
        window._load_vehicle_feasible_segment_sibling()

        assert window._vehicle_feasible_segment_plan is None
        assert window._vehicle_feasible_segment_last_error == ""
        assert window._vehicle_feasible_segment_preview.active_item_count == 0
        app.processEvents()
    finally:
        window.close()
