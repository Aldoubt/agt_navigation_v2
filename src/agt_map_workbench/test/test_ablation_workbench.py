from PyQt5.QtWidgets import QApplication

from agt_map_workbench.ablation_workbench import AblationMapWorkbenchWindow


def test_ablation_workbench_exposes_ordered_a0_a3_profiles():
    app = QApplication.instance() or QApplication([])
    window = AblationMapWorkbenchWindow()
    try:
        combo = window._ablation_profile_combo
        keys = [str(combo.itemData(index)) for index in range(combo.count())]
        assert keys == ["A0", "A1", "A2", "A3"]
        assert window._selected_ablation_profile() == "A0"
        combo.setCurrentIndex(3)
        assert window._selected_ablation_profile() == "A3"
        assert window._ablation_profile_applied == "A0"
    finally:
        window.close()
        app.processEvents()
